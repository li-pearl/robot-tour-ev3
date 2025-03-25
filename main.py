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

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# setup
dowel_to_robot_offset = 66
total_angle = 0
speedInMMPerSecond = 130
total_distance = 316

# robot parameters
wheel_diameter = 55
circumference = 3.14159 * wheel_diameter
axle_track = 117
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
robot.settings(speedInMMPerSecond, 200, 360, 200)


# Drive Functions
def upSquare(numberOfSquares):
    global total_distance 
    total_distance += 500 * numberOfSquares
    if gyro.angle != 0:
        robot.turn(-gyro.angle())
        robot.turn(-gyro.angle())
        robot.turn(-gyro.angle())
    while robot.distance() < total_distance:
        correction = (0 - gyro.angle()) * 1.5
        robot.drive(speedInMMPerSecond, correction)
    robot.stop()
    wait(250)
    if gyro.angle != 0:
        robot.turn(-gyro.angle())
        robot.turn(-gyro.angle())
        robot.turn(-gyro.angle())
        robot.turn(-gyro.angle())

def leftSquare(numberOfSquares):
    global total_distance
    total_distance += 500 * numberOfSquares
    if gyro.angle != -90:
        robot.turn(-90 - gyro.angle())
        robot.turn(-90 - gyro.angle())
        robot.turn(-90 - gyro.angle())
    while robot.distance() < total_distance:
        correction = (-90 - gyro.angle()) * 1.5
        robot.drive(speedInMMPerSecond, correction)
    robot.stop()
    wait(250)
    if gyro.angle != -90:
        robot.turn(-90 - gyro.angle())
        robot.turn(-90 - gyro.angle())
        robot.turn(-90 - gyro.angle())
        robot.turn(-90 - gyro.angle())

def rightSquare(numberOfSquares):
    global total_distance
    total_distance += 500 * numberOfSquares
    if gyro.angle != 90:
        robot.turn(90 - gyro.angle())
        robot.turn(90 - gyro.angle())
        robot.turn(90 - gyro.angle())
    while robot.distance() < total_distance:
        correction = (90 - gyro.angle()) * 1.5
        robot.drive(speedInMMPerSecond, correction)
    robot.stop()
    wait(250)
    if gyro.angle != 90:
        robot.turn(90 - gyro.angle())
        robot.turn(90 - gyro.angle())
        robot.turn(90 - gyro.angle())
        robot.turn(90 - gyro.angle())

def downSquare(numberOfSquares):    
    global total_distance
    total_distance -= 500 * numberOfSquares
    if gyro.angle != 0:
        robot.turn(-gyro.angle())
        robot.turn(-gyro.angle())
        robot.turn(-gyro.angle())
    while robot.distance() > total_distance:
        correction = (0 - gyro.angle()) * 1.5
        robot.drive(-speedInMMPerSecond, correction)
    robot.stop()
    wait(250)
    if gyro.angle != 0:
        robot.turn(0 - gyro.angle())
        robot.turn(0 - gyro.angle())
        robot.turn(0 - gyro.angle())
        robot.turn(0 - gyro.angle())


### let it go vroom vroom
ev3.speaker.beep()
robot.reset()
gyro.reset_angle(0)

robot.straight(250 + dowel_to_robot_offset) # initial move

leftSquare(2)
upSquare(3)
rightSquare(2)
downSquare(2)
rightSquare(2)
downSquare(1)
upSquare(1)
leftSquare(1)
upSquare(1)

robot.straight(500 + dowel_to_robot_offset) # final move

###

# def driveForwardFourths(numberOfTimes):
#     robot.straight(250 * numberOfTimes)

# def driveBackwardFourths(numberOfTimes):
#     robot.straight(-250 * numberOfTimes)

# def turnLeft():
#     global total_angle
#     robot.turn(-90)
#     total_angle += -90
#     robot.turn(total_angle - gyro.angle())

# def turnRight():
#     global total_angle
#     robot.turn(90)
#     total_angle += 90
#     robot.turn(total_angle - gyro.angle())

# ev3.speaker.beep()
# robot.straight(250 + dowel_to_robot_offset)
# turnLeft()
# driveForwardFourths(2)
# turnRight()
# driveForwardFourths(6)
# turnRight()
# driveForwardFourths(4)
# turnRight()
# driveForwardFourths(2)
# turnLeft()
# driveForwardFourths(2)
# turnRight()
# driveForwardFourths(2)
# turnLeft()
# driveForwardFourths(2)
# turnLeft()
# driveForwardFourths(1.75)
# driveBackwardFourths(3.75)
# turnLeft()
# driveForwardFourths(0.75)
# driveBackwardFourths(0.75)
# turnRight()
# driveForwardFourths(6)
# turnLeft()
# driveForwardFourths(2)
# turnLeft()
# driveForwardFourths(2)
# turnRight()
# driveForwardFourths(2)
# turnRight()
# driveForwardFourths(2)
# turnLeft()
# driveForwardFourths(4)

# ev3.speaker.beep()

# #probably won't work
# # def turn_to_gyro_angle(target_angle, speed):
# #     robot.stop()
# #     while (gyro.angle() != target_angle):
# #         if (gyro.angle() > target_angle):
# #             right_motor.run(speed)
# #             left_motor.run(-speed)
# #         elif (gyro.angle() < target_angle):
# #             left_motor.run(speed)
# #             right_motor.run(-speed)
# #         else:
# #             pass


# #=========Notes===============#
# #Field is 250cm by 200 cm
# #Squares are 50cm by 50cm
# #Centers are 25cm


# # wait(time)
# # Pauses the user time program for a specified amount of time in ms

# # robot.straight(1000)
# # Moves the robot straight for 1000mm or 1 m

# # robot.turn(360)
# # Turns the robot clockwide by 360 degrees

# # robot.turn(-360)
# # Turns the robot counterclockwise by 360 degrees

# #run(speed)
# #Runs the motor at a constant speed
        
# #         If your robot turns not far enough, increase the axle_track value slightly.
# #         If your robot turns too far, decrease the axle_track value slightly.

# # When making these adjustments, always adjust the wheel_diameter first, as done above.
# # suppose you make a DriveBase object using two Motor objects called left_motor and right_motor. You cannot use these motors individually while the DriveBase is active.

# # The DriveBase is active if it is driving, but also when it is actively holding the wheels in place after a straight() or turn() command. To deactivate the DriveBase, call stop().

# def cycle(straights, turn_angle):
#     i = 0
#     while i < straights:
#         robot.straight(250)
#         i += 1
#     robot.turn(turn_angle)
#     total_angle += turn_angle
#     robot.turn(total_angle - gyro.angle) 

