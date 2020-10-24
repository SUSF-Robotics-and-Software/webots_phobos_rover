from controller import Robot, Camera, Motor
import math
import zmq
import json

# Create zmq context
context = zmq.Context()

# From loco_ctrl.toml
gmb_limits = [math.radians(-90), math.radians(90)]
gmb_motors = ['fl_gimbal', 'ml_gimbal', 'rl_gimbal', 'fr_gimbal', 'mr_gimbal', 'rr_gimbal']

# From loco_ctrl.toml
drv_limits = [-3.6458, 3.6458]
drv_motors = ['fl_drive', 'ml_drive', 'rl_drive', 'fr_drive', 'mr_drive', 'rr_drive']

# Motor maps from MechDems
act_id_types = {
    'positional': [
        'StrFL',
        'StrML',
        'StrRL',
        'StrFR',
        'StrMR',
        'StrRR',
        'ArmBase',
        'ArmShoulder',
        'ArmElbow',
        'ArmWrist',
        'ArmGrabber'
    ],
    'continuous': [
        'DrvFL',
        'DrvML',
        'DrvRL',
        'DrvFR',
        'DrvMR',
        'DrvRR'
    ]
}
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

class PhobosRoverController(Robot):
    def __init__(self):
        super(PhobosRoverController, self).__init__()
        # Timestep in ms, must be longer than the send/recv timeout in the
        # mech_client.
        self.timestep = 20

        # Get and enable the cameras
        self.left_cam = self.getCamera('l_cam')
        self.right_cam = self.getCamera('r_cam')
        # 10 Hz cameras
        self.left_cam.enable(100)
        self.right_cam.enable(100)

        # Get and enable the depth images
        self.left_depth = self.getRangeFinder('l_depth')
        self.left_depth.enable(100)

        # Get and set motors
        self.gmb_motors = [self.getMotor(name) for name in gmb_motors]
        self.drv_motors = [self.getMotor(name) for name in drv_motors]

        # Set gimbal positions to left hand turn
        # corner_angle_rad = math.radians(45.0)
        # self.gmb_motors[0].setPosition(corner_angle_rad)
        # self.gmb_motors[2].setPosition(-corner_angle_rad)
        # self.gmb_motors[3].setPosition(corner_angle_rad)
        # self.gmb_motors[4].setPosition(-corner_angle_rad)

        # Make all drives move
        # for drv in self.drv_motors:
        #     drv.setPosition(float('inf'))
        #     drv.setVelocity(3.0)

        # Open the mechanisms server
        self.mech_rep = context.socket(zmq.REP)
        self.mech_rep.setsockopt(zmq.RCVTIMEO, 20)
        self.mech_rep.bind('tcp://*:5000')
        self.mech_pub = context.socket(zmq.PUB)
        self.mech_pub.bind('tcp://*:5001')

        print('MechServer started')

    def run(self):
        while self.step(self.timestep) != -1:
            # Recieve data from the client
            try:
                dems = json.loads(self.mech_rep.recv_string())
            except zmq.Again as e:
                continue
            except zmq.ZMQError as e:
                print(f'Error: {e}, ({e.errno})')
                continue
            finally:
                self.stop()

            print('MechServer - got demands')

            # Send resposnse back to client
            self.mech_rep.send_string('"DemsOk"')

            print(dems)

            # Actuate position demands
            for act_id, position_rad in dems['pos_rad'].items():
                # If the motor is positional
                if act_id in act_id_types['positional']:
                    # Get the motor group
                    group = act_id[:3]

                    if group == 'Str':
                        self.gmb_motors[act_id_motor_group_index_map[act_id]] \
                            .setPosition(position_rad)

            # Actuate speed demands
            for act_id, speed_rads in dems['speed_rads'].items():
                # If motor is continuous
                if act_id in act_id_types['continuous']:
                    # Get motor group
                    group = act_id[:3]

                    if group == 'Drv':
                        drv = self.drv_motors[act_id_motor_group_index_map[act_id]]
                        drv.setPosition(float('inf'))
                        drv.setVelocity(speed_rads)

    def stop(self):
        for drv in self.drv_motors:
            drv.setVelocity(0.0)



controller = PhobosRoverController()
controller.run()