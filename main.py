#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Create your objects here.
ev3 = EV3Brick()
gyro = GyroSensor(Port.S2)

gyro.mode = 'GYRO-ANG'

# ultrasomic_sensor = UltrasonicSensor(Port.S1)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

wheel_diameter = 56
circumference = 3.14159 * wheel_diameter
robot = DriveBase(left_motor, right_motor, wheel_diameter=57, axle_track=120)
robot.settings(200, 200, 360, 200)

# Track properties
# numOfStraights = 1
numOfTurns = 0

# Robot speed properties
targetTime = 0
# straightSpeed = (targetTime - (3 * numOfTurns)) / numOfStraights
#robot.settings(straightSpeed, 0, 30, 0)

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
# def turn_to_gyro_angle(target_angle, speed):
#     robot.stop()
#     while (gyro.angle() != target_angle):
#         if (gyro.angle() > target_angle):
#             right_motor.run(speed)
#             left_motor.run(-speed)
#         elif (gyro.angle() < target_angle):
#             left_motor.run(speed)
#             right_motor.run(-speed)
#         else:
#             pass


#=========Notes===============#
#Field is 250cm by 200 cm
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
        
#         If your robot turns not far enough, increase the axle_track value slightly.
#         If your robot turns too far, decrease the axle_track value slightly.

# When making these adjustments, always adjust the wheel_diameter first, as done above.
# suppose you make a DriveBase object using two Motor objects called left_motor and right_motor. You cannot use these motors individually while the DriveBase is active.

# The DriveBase is active if it is driving, but also when it is actively holding the wheels in place after a straight() or turn() command. To deactivate the DriveBase, call stop().

# def cycle(straights, turn_angle):
#     i = 0
#     while i < straights:
#         robot.straight(250)
#         i += 1
#     robot.turn(turn_angle)
#     total_angle += turn_angle
#     robot.turn(total_angle - gyro.angle) 

# Write your program here.
gyro.reset_angle(0)
robot.reset()
time = 0
dowel_to_robot_offset = 58
total_angle = 0

# Drive Functions
def driveForwardFourths(numberOfTimes):
    robot.straight(250 * numberOfTimes)

def driveBackwardFourths(numberOfTimes):
    robot.straight(-250 * numberOfTimes)

def turnLeft():
    global total_angle
    robot.turn(-90)
    total_angle += -90
    robot.turn(total_angle - gyro.angle())

def turnRight():
    global total_angle
    robot.turn(90)
    total_angle += 90
    robot.turn(total_angle - gyro.angle())

#let it go vroom vroom
ev3.speaker.beep()
robot.straight(250 + dowel_to_robot_offset)
turnLeft()
driveForwardFourths(2)
turnRight()
driveForwardFourths(6)
turnRight()
driveForwardFourths(4)
turnRight()
driveForwardFourths(2)
turnLeft()
driveForwardFourths(2)
turnRight()
driveForwardFourths(2)
turnLeft()
driveForwardFourths(2)
turnLeft()
driveForwardFourths(1.75)
driveBackwardFourths(3.75)
turnLeft()
driveForwardFourths(0.75)
driveBackwardFourths(0.75)
turnRight()
driveForwardFourths(6)
turnLeft()
driveForwardFourths(2)
turnLeft()
driveForwardFourths(2)
turnRight()
driveForwardFourths(2)
turnRight()
driveForwardFourths(2)
turnLeft()
driveForwardFourths(4)

ev3.speaker.beep()

# stopwatch.reset()
# robot.turn(90)
# around 985 ms
# robot.straight(250)
# print(stopwatch.time())

# robot.straight(250 + dowel_to_robot_offset)
# robot.turn(-90)
# total_angle -= 90
# robot.turn(total_angle - gyro.angle())

# robot.straight(250*4 - dowel_to_robot_offset)
# robot.turn(90)
# total_angle += 90
# robot.turn(total_angle - gyro.angle())

# robot.straight(250*3 + dowel_to_robot_offset)
# robot.straight(100)

# robot.straight(-250*3 - dowel_to_robot_offset)
# robot.straight(-100)

# robot.turn(90)
# total_angle += 90
# robot.turn(total_angle - gyro.angle())

# robot.straight(250*2 + dowel_to_robot_offset)
# robot.turn(-90)
# total_angle -= 90
# robot.turn(total_angle - gyro.angle())

# robot.straight(250*6)
# robot.turn(-90)
# total_angle -= 90
# robot.turn(total_angle - gyro.angle())

# robot.straight(350 - dowel_to_robot_offset)
# robot.straight(-350 + dowel_to_robot_offset)

# robot.straight(-250*4)
# robot.turn(-90)
# total_angle -= 90
# robot.turn(total_angle - gyro.angle())

# robot.straight(350 - dowel_to_robot_offset)
# robot.straight(-350 + dowel_to_robot_offset)


# robot.turn(90)
# total_angle += 90
# robot.turn(total_angle - gyro.angle())
# robot.straight(250*4 + dowel_to_robot_offset)

# robot.turn(-90)
# total_angle -= 90
# robot.turn(total_angle - gyro.angle())

# robot.straight(250*2 -  dowel_to_robot_offset)


# robot.turn(-90)
# total_angle -= 90
# robot.turn(total_angle - gyro.angle())


# robot.straight(250*2 - dowel_to_robot_offset)


