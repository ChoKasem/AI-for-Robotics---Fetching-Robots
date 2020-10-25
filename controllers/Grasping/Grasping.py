"""Grasping controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Camera, TouchSensor, Device

MAX_SPEED = 6.28
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
# timestep = 64

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
# ds.enable(timestep)

wheel_motors = {}
wheel_motors["FLL_WHEEL"] = robot.getMotor("fl_caster_l_wheel_joint")
wheel_motors["FLR_WHEEL"] = robot.getMotor("fl_caster_r_wheel_joint")
wheel_motors["FRL_WHEEL"] = robot.getMotor("fr_caster_l_wheel_joint")
wheel_motors["FRR_WHEEL"] = robot.getMotor("fr_caster_r_wheel_joint")
wheel_motors["BLL_WHEEL"] = robot.getMotor("bl_caster_l_wheel_joint")
wheel_motors["BLR_WHEEL"] = robot.getMotor("bl_caster_r_wheel_joint")
wheel_motors["BRL_WHEEL"] = robot.getMotor("br_caster_l_wheel_joint")
wheel_motors["BRR_WHEEL"] = robot.getMotor("br_caster_r_wheel_joint")

#Left arm and Gripper
left_arm_motors = {}
left_arm_motors["ELBOW_LIFT"] = robot.getMotor("l_elbow_flex_joint")
left_arm_motors["SHOULDER_LIFT"] = robot.getMotor("l_shoulder_lift_joint")
left_arm_motors["SHOULDER_ROLL"] = robot.getMotor("l_shoulder_pan_joint")
left_arm_motors["UPPER_ARM_ROLL"] = robot.getMotor("l_upper_arm_roll_joint")
left_arm_motors["WRIST_ROLL"] = robot.getMotor("l_wrist_roll_joint")

for key in left_arm_motors:
    left_arm_motors[key].setPosition(0.0)

left_gripper_motors = {}
left_gripper_motors["LEFT_FINGER"] = robot.getMotor("r_gripper_l_finger_joint")
left_gripper_motors["RIGHT_FINGER"] = robot.getMotor("r_gripper_r_finger_joint")
left_gripper_motors["LEFT_TIP"] = robot.getMotor("r_gripper_l_finger_tip_joint")
left_gripper_motors["RIGHT_TIP"] = robot.getMotor("r_gripper_r_finger_tip_joint")
#Left Gripper Touch Sensor
left_gripper_sensors = {}
left_gripper_sensors["LEFT_FINGER"] = robot.getTouchSensor("l_gripper_l_finger_tip_contact_sensor")
left_gripper_sensors["RIGHT_FINGER"] = robot.getTouchSensor("l_gripper_r_finger_tip_contact_sensor")
for key in left_gripper_sensors:
    left_gripper_sensors[key].enable(timestep)
    print(left_gripper_sensors[key].getValue())
    # print(left_gripper_sensors[key])

#Right Arm
right_arm_motors = {}
right_arm_motors["ELBOW_LIFT"] = robot.getMotor("r_elbow_flex_joint")
right_arm_motors["SHOULDER_LIFT"] = robot.getMotor("r_shoulder_lift_joint")
right_arm_motors["SHOULDER_ROLL"] = robot.getMotor("r_shoulder_pan_joint")
right_arm_motors["UPPER_ARM_ROLL"] = robot.getMotor("r_upper_arm_roll_joint")
right_arm_motors["WRIST_ROLL"] = robot.getMotor("r_wrist_roll_joint")

right_arm_motors["SHOULDER_ROLL"].setPosition(-2)
right_arm_motors["SHOULDER_LIFT"].setPosition(0)
right_arm_motors["ELBOW_LIFT"].setPosition(-2)

#Camera
wide_stereo_l_stereo_camera_sensor = robot.getCamera("wide_stereo_l_stereo_camera_sensor")
wide_stereo_l_stereo_camera_sensor.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
# while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    # left_arm_motor["elbow_lift"].setPosition(0.0)
    # left_arm_motor["elbow_lift"].setVelocity(0.1*6.28)
    
    # left_arm.setVelocity(-0.5 * MAX_SPEED)
    # pass
# Enter here exit cleanup code.
