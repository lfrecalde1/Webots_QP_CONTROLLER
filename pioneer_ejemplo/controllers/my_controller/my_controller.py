"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import GPS
from controller import InertialUnit

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
# Gps definition
gps = GPS("gps")
gps.enable(timestep)

# Imu defintion
imu = InertialUnit("inertial unit")
imu.enable(timestep)


# Motor defintion
motor_l = robot.getDevice("left wheel")
motor_r = robot.getDevice("right wheel")

motor_l.setPosition(float('inf'))
motor_r.setPosition(float('inf'))

motor_l.setVelocity(0.0)
motor_r.setVelocity(0.0)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read gps values
    data_gps = gps.getValues()
    data_imu = imu.getRollPitchYaw()

    print(data_imu)

    motor_l.setVelocity(1.0)
    motor_r.setVelocity(-1.0)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
