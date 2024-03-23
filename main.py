#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
gyro = GyroSensor(Port.S2)
ultrasomic_sensor = UltrasonicSensor(Port.S1)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

wheel_diameter = 56
circumference = 3.14159 * wheel_diameter
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=147)

stopwatch = StopWatch()

# def degps_to_rps(deg_per_sec):
#     per 360 degrees there is 1 circumference
#     360/150 = 

def gyro_set_up():
    gyro.reset_angle(0)
    while True:
        if gyro.angle() == 0:
            break

    while True:
        if gyro.speed() == 0:
            break

def drive_straight_with_gyro(speed):
    target = gyro.angle()
    gain = 2

    while True:
        correction = target - gyro.angle() #might have to flip
        turn_power = correction * gain

        robot.drive(speed, turn_power)

def cm_to_mm(cm):
    return cm * 10

#probably won't work
def turn_to_gyro_angle(target_angle, speed):
    robot.stop()
    while (gyro.angle() != target_angle):
        if (gyro.angle() > target_angle):
            right_motor.run(speed)
            left_motor.run(-speed)
        elif (gyro.angle() < target_angle):
            left_motor.run(speed)
            right_motor.run(-speed)
        else:
            pass


#=========Notes===============#
#Field is 200cm by 200 cm
#Squares are 50cm by 50cm
#Centers are 25cm


# wait(time)
# Pauses the user time program for a specified amount of time in ms

# robot.straight(1000)
# Moves the robot straight for 1000mm or 1 m

# robot.turn(360)
# Turns the robot clockwide by 360 degrees

# robot.turn(-360)
# Turns the robot counterclockwise by 360 degrees

#run(speed)
#Runs the motor at a constant speed

    
# class DataLog(*headers, name='log', timestamp=True, extension='csv', append=False)
#     Create a file and log values
#     log(*values)
#     Saves one or more values on a new line in the File
#     Parameters: values(object, object,...)
        

#         If your robot turns not far enough, increase the axle_track value slightly.
#         If your robot turns too far, decrease the axle_track value slightly.

# When making these adjustments, always adjust the wheel_diameter first, as done above.
# uppose you make a DriveBase object using two Motor objects called left_motor and right_motor. You cannot use these motors individually while the DriveBase is active.

# The DriveBase is active if it is driving, but also when it is actively holding the wheels in place after a straight() or turn() command. To deactivate the DriveBase, call stop().

# Write your program here.
gyro.reset_angle(0)
robot.reset()
time = 0
dowel_to_robot_offset = 0
turn_offset = 0

stopwatch.reset()
robot.turn(360)
stopwatch.pause()
time = stopwatch.time()
ev3.speaker.beep()

print(time)
