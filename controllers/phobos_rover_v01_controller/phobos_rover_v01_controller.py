from controller import Robot, Camera, Motor
import math

# From loco_ctrl.toml
gmb_limits = [math.radians(-90), math.radians(90)]
gmb_motors = ["fl_gimbal", "ml_gimbal", "rl_gimbal", "fr_gimbal", "mr_gimbal", "rr_gimbal"]

# From loco_ctrl.toml
drv_limits = [-3.6458, 3.6458]
drv_motors = ["fl_drive", "ml_drive", "rl_drive", "fr_drive", "mr_drive", "rr_drive"]

class PhobosRoverController(Robot):
    def __init__(self):
        super(PhobosRoverController, self).__init__()
        # Timestep of 10 ms
        self.timestep = 10

        # Get and enable the cameras
        self.left_cam = self.getCamera("l_cam")
        self.right_cam = self.getCamera("r_cam")
        # 10 Hz cameras
        self.left_cam.enable(100)
        self.right_cam.enable(100)

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
        for drv in self.drv_motors:
            drv.setPosition(float('inf'))
            drv.setVelocity(3.0)

    def run(self):
        while self.step(self.timestep) != -1:
            pass

controller = PhobosRoverController()
controller.run()