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
def spinTurn(direction):
    if direction == "left":
        while(gyro.angle() < -90):
            left_motor.run(-speedInMMPerSecond)
            right_motor.run(speedInMMPerSecond)
    elif direction == "right":
        while(gyro.angle() < 90):
            left_motor.run(speedInMMPerSecond)
            right_motor.run(-speedInMMPerSecond)
    else:
        if(gyro.angle() > 0):
            while(gyro.angle() > 0):
                left_motor.run(-speedInMMPerSecond)
                right_motor.run(speedInMMPerSecond)
        else:
            while(gyro.angle() < 0):
                left_motor.run(speedInMMPerSecond)
                right_motor.run(-speedInMMPerSecond)
    robot.stop()

def upSquare(numberOfSquares):
    global total_distance 
    total_distance += 100 * numberOfSquares
    if gyro.angle != 0:
        spinTurn("up")
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
        spinTurn("left")
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
        spinTurn("right")
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
        spinTurn("down")
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

rightSquare(1)

