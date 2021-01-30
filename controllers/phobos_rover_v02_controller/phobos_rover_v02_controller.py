from controller import Robot, Camera, Motor, Supervisor
import sys
import math
import json
import toml
import zmq
import concurrent.futures
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image
import io
import time
import base64
import multiprocessing as mp
from multiprocessing import Pipe, Process

# Toggle to disble/enable specific servers
MECH_SERVER = True
CAM_SERVER = True
SIM_SERVER = True

# Constants
str_motor_order = ['fl_gimbal', 'ml_gimbal', 'rl_gimbal', 'fr_gimbal', 'mr_gimbal', 'rr_gimbal']
drv_motor_order = ['fl_drive', 'ml_drive', 'rl_drive', 'fr_drive', 'mr_drive', 'rr_drive']
act_id_motor_group_index_map = {
    'DrvFL': 0,
	'DrvML': 1,
	'DrvRL': 2,
	'DrvFR': 3,
	'DrvMR': 4,
	'DrvRR': 5,
	'StrFL': 0,
	'StrML': 1,
	'StrRL': 2,
	'StrFR': 3,
	'StrMR': 4,
	'StrRR': 5,
	'ArmBase': 0,
	'ArmShoulder': 1,
	'ArmElbow': 2,
	'ArmWrist': 3,
	'ArmGrabber': 4
}

class PhobosRoverController(Supervisor):
    '''
    Controller interface for the phobos rover
    '''

    def __init__(self, params_path):
        '''
        Main constructor, initialises the controller.
        '''
        
        # Run the standard Robot class setup
        super(PhobosRoverController, self).__init__()

        # Set the supervisor node for the rover
        self.sup_node = self.getFromDef('PHOBOS')

        # Load the params from the params path
        self.params = toml.load(params_path)

        # Setup equipment
        self.init_eqpt()

    def init_eqpt(self):
        '''
        Initialise the equipment of the rover.

        This function will find all equipment in the simulation and set 
        endpoints in self to be able to access them
        '''

        # Get camera endpoints
        self.cameras = {}
        self.cameras['LeftNav'] = self.getCamera('l_cam')
        self.cameras['RightNav'] = self.getCamera('r_cam')

        # Enable cameras at the specified frequencies
        self.cameras['LeftNav'].enable(self.params['left_nav_cam_timestep_ms'])
        self.cameras['RightNav'].enable(self.params['right_nav_cam_timestep_ms'])
        
        # Get and enable the depth images
        self.cameras['LeftDepth'] = self.getRangeFinder('l_depth')
        self.cameras['LeftDepth'].enable(self.params['left_depth_cam_timestep_ms'])

        # Get steer motors
        self.str_motors = [self.getMotor(name) for name in str_motor_order]

        # Get drive motors
        self.drv_motors = [self.getMotor(name) for name in drv_motor_order]

    def actuate_mech_dems(self, dems):
        '''
        Actuate the given mechanisms demands.

        Demands must have been validated already using `self.validate_mech_dems`.
        '''

        # Actuate position demands
        for act_id, position_rad in dems['pos_rad'].items():
            # Get the motor group
            group = act_id[:3]

            if group == 'Str':
                self.str_motors[act_id_motor_group_index_map[act_id]] \
                    .setPosition(position_rad)

        # Actuate speed demands
        for act_id, speed_rads in dems['speed_rads'].items():
            # Get motor group
            group = act_id[:3]

            if group == 'Drv':
                drv = self.drv_motors[act_id_motor_group_index_map[act_id]]
                drv.setPosition(float('inf'))
                drv.setVelocity(speed_rads)

    def stop(self):
        '''
        Bring the rover to a complete stop.
        '''
        for drv in self.drv_motors:
            drv.setVelocity(0.0)

    def pose(self):
        '''
        Get the pose of the rover
        '''
        pass


def step(phobos):
    '''
    Step the rover simulation.

    Returns true if the controller should keep running.
    '''
    return phobos.step(phobos.params['controller_timestep_ms']) != -1

def run(phobos):
    '''
    Run the controller.
    '''

    # Create the cam server bg process and queue
    if CAM_SERVER:
        (cam_pipe, cam_child_pipe) = Pipe()
        cam_proc = Process(target=cam_process, args=(
            phobos.params['cam_rep_endpoint'], cam_child_pipe, 
        ))
        cam_proc.daemon = True
        cam_proc.start()
        
        print('CamServer started')
    else:
        print('CamServer disabled')

    # Create zmq context
    context = zmq.Context()

    # Open mechanisms server
    if MECH_SERVER:
        mech_rep = context.socket(zmq.REP)
        mech_rep.bind(phobos.params['mech_rep_endpoint'])
        mech_pub = context.socket(zmq.PUB)
        mech_pub.bind(phobos.params['mech_pub_endpoint'])

        print('MechServer started')
    else:
        print('MechServer disabled')

    # Open sim server
    if SIM_SERVER:
        sim_pub = context.socket(zmq.PUB)
        sim_pub.bind(phobos.params['sim_pub_endpoint'])

        print('SimServer started')
    else:
        print('SimServer disabled')

    # Run flag
    run_controller = True

    print('Starting main control loop')
    while run_controller:
        # Run mechanisms task
        if MECH_SERVER:
            run_controller &= handle_mech(phobos, mech_rep, mech_pub)

        # Handle any request from the camera process
        if CAM_SERVER:
            run_controller &= handle_cam_req(phobos, cam_pipe)

        # handle the simulation data
        if SIM_SERVER:
            run_controller &= handle_sim_data(phobos, sim_pub)

        # Step the rover
        run_controller &= step(phobos)

        sys.stdout.flush()

    # Close sockets

    if CAM_SERVER:
        # Send stop to cam process
        cam_pipe.send('STOP')
        # Join the cam process
        cam_proc.join()

    if MECH_SERVER:
        mech_rep.close()
        mech_pub.close()

    if SIM_SERVER:
        sim_pub.close()

    # Destroy context
    context.destroy()

    # Tell webots we've exited
    sys.exit(0)
        

def handle_mech(phobos, mech_rep, mech_pub):
    '''
    Handle mechanisms commands and publish mech data
    '''
    # Flag indicating whether or not to stop the rover
    stop = False

    # Get mechanisms demands from the rep socket
    try: 
        dems_str = mech_rep.recv_string(flags=zmq.NOBLOCK)
        mech_dems = json.loads(dems_str)
    except zmq.Again:
        mech_dems = None
    except zmq.ZMQError as e:
        print(f'MechServer: Error - {e}, ({e.errno})')
        stop = True
        mech_dems = None
    except Exception as e:
        print(f'MechServer Exception: {e}')
        stop = True
        mech_dems = None

    # If no demand
    if mech_dems is None:
        # If an error occured stop the rover
        if stop:
            phobos.stop()
    else:
        # TODO: vaidate demands

        # Send response to client
        mech_rep.send_string('"DemsOk"')

        # print(f'MechDems: {mech_dems}')

        # Actuate
        phobos.actuate_mech_dems(mech_dems)

    return True

def handle_cam_req(phobos, cam_pipe):
    '''
    Handle a possible camera request from the camera process, then send data 
    back to the cam process for sending.
    '''

    cam_req = None

    # Data to send back to cam process
    cam_data = {}

    # If the pipe is closed the sender has quit, so need to return false so
    # webots knows the server is shutdown
    if cam_pipe.closed:
        return False

    # Poll for data from the camera process
    if cam_pipe.poll():

        cam_req = cam_pipe.recv()

        # If a frame request unpack the request to build data to send back
        if cam_req['FrameRequest'] is not None:
            cam_req = cam_req['FrameRequest']
            cam_data['has_frames'] = True
        # Otherwise don't unpack and send back a stream settings rejected
        # packet. The simulation doesn't support streaming data
        elif cam_req['StreamSettingsRequest'] is not None:
            cam_data['has_frames'] = False
        
    # If no request return now
    else:
        return True

    # If it was a frames request
    if cam_data is not None:

        cam_data['format'] = cam_req['format']

        # For each camera in the request acquire an image
        for cam_id in cam_req['cameras']:
            # Get the raw data
            cam_data[cam_id] = {}
            cam_data[cam_id]['raw'] = phobos.cameras[cam_id].getImage();
            cam_data[cam_id]['timestamp'] = int(round(time.time() * 1000))
            cam_data[cam_id]['height'] = phobos.cameras[cam_id].getHeight()
            cam_data[cam_id]['width'] = phobos.cameras[cam_id].getWidth()

    # Send data to cam process
    cam_pipe.send(cam_data)

    return True

def handle_cam_send(cam_rep, cam_data):
    '''
    Send data via the zmq socket to the client, formatting in the correct way.
    '''

    res = None

    # If the data doesn't contain frames
    if not cam_data['has_frames']:
        # Make res the rejected response
        res = 'StreamSettingsRejected'

    # Or if we have frames data
    else:
        # Create frames component of response
        res = {'Frames': {}}

        # iterate over the raw data and cam IDs
        for cam_id, data in cam_data.items():
            if cam_id in ['format', 'has_frames']:
                continue
            
            res['Frames'][cam_id] = {};

            # Convert the raw data into a numpy array
            np_array = np.frombuffer(data['raw'], np.uint8)\
                .reshape((data['height'], data['width'], 4))

            # Rearrange from BRGA to RGBA
            np_array = np_array[...,[2,1,0,3]]

            # Convert to a PIL image
            image = Image.fromarray(np_array)

            # Create a byte array to write into
            img_bytes = io.BytesIO()

            # Save the image into this array
            if isinstance(cam_data['format'], str):
                image.save(img_bytes, format=cam_data['format'])
            else:
                image.save(img_bytes, format=list(cam_data['format'].keys())[0])

            # Get the raw byte value out
            img_bytes = img_bytes.getvalue()

            res['Frames'][cam_id]['timestamp'] = data['timestamp']
            res['Frames'][cam_id]['format'] = cam_data['format']
            res['Frames'][cam_id]['b64_data'] = base64.b64encode(img_bytes).decode('ascii')


    # Get the response as a JSON string
    res_str = json.dumps(res)

    # print(f'Sending camera response ({len(res_str)} long)')

    # Send the string to the client
    cam_rep.send_string(res_str)

def cam_process(cam_endpoint, cam_pipe):
    '''
    Handle camera-related networking in a separate process.
    '''
    print('Starting camera process')

    # Create new zmq context
    context = zmq.Context()

    # Open camera server
    cam_rep = context.socket(zmq.REP)
    cam_rep.bind(cam_endpoint)

    # Flag keeping the process running
    run_process = True

    # Flag indicating if we're still handling a request
    handling_req = False

    print('Camera process started')

    while run_process:

        msg = None

        # First poll the pipe to see if there's any data
        if cam_pipe.poll():
            # Recieve message from main process
            msg = cam_pipe.recv()

            # If got a stop message exit the loop
            if isinstance(msg, str):
                if msg == 'STOP':
                    break
        else:
            # If no data, pass
            pass

        # If we got data from the main process send it
        if isinstance(msg, dict):
            handle_cam_send(cam_rep, msg)

            # Unset the handling req flag
            handling_req = False

        # If still handling a request don't try to recieve data form the client
        if handling_req:
            continue

        # Get request from the rep socket
        try:
            req_str = cam_rep.recv_string(flags=zmq.NOBLOCK)
            cam_req = json.loads(req_str)
        except zmq.Again:
            cam_req = None
        except zmq.ZMQError as e:
            print(f'CamServer: Error - {e}, ({e.errno}')
            cam_req = None
        except Exception as e:
            print(f'CamServer Exception: {e}')
            cam_req = None

        if cam_req is None:
            continue

        # Send request to main process
        cam_pipe.send(cam_req)

        # Raise handling req flag
        handling_req = True

    # Close the pipe
    cam_pipe.close()

    # Kill the socket
    cam_rep.close()

    # Destroy the context
    context.destroy()

def handle_sim_data(phobos, sim_pub):
    '''
    Acquire and send simulation data to the sim_client.
    '''

    # Get the position of the rover
    pos_m_lm = phobos.sup_node.getPosition()

    # Get the rotation matrix describing the orientation of the rover
    att_mat = np.array(phobos.sup_node.getOrientation())
    att_mat = np.reshape(att_mat, (3, 3))

    # Convert the matrix into a quaternion
    rot = Rotation.from_matrix(att_mat)
    att_q_lm = rot.as_quat()

    # Build the data object
    data = {
        'Pose': {
            'position_m_lm': pos_m_lm, 
            'attitude_q_lm': att_q_lm.tolist()
        }
    }

    # Send the pose
    sim_pub.send_json(data)

    return True

def main():

    # Create phobos and run controller
    phobos = PhobosRoverController('../../params/phobos_rover_v02_controller.toml')

    run(phobos)

if __name__ == '__main__':

    main()