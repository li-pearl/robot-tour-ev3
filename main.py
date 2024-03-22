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
gyro = GyroSensor(2)
left_motor = Motor(2)
right_motor = Motor(3)

robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=147)

# Track properties
numOfStraights = 0
numOfTurns = 0

# Robot speed properties
targetTime = 0
straightSpeed = (targetTime - (3 * numOfTurns)) / numOfStraights
robot.settings(straightSpeed, 0, 30, 0)

gyro.reset_angle(0)
intendedAngle = 0

# Write your path here.
ev3.speaker.beep()

# Drive Functions
def driveHalfBlock():
    robot.straight(500)
    robot.turn

def turnLeft():
    robot.turn(-90)
    intendedAngle += -90

def turnRight():
    robot.turn(90)
    intendedAngle += -90