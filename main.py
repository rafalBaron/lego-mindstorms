#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor, InfraredSensor)
from pybricks.parameters import Button, Port, Stop, Direction, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import math


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
#color_sensor = ColorSensor(Port.S4)
#left_motor = Motor(Port.B)
#right_motor = Motor(Port.C)
#robot = DriveBase(left_motor, right_motor, wheel_diameter=29, axle_track=117)
DRIVE_SPEED = 70

PROPORTIONAL_GAIN = 1.8


# Write your program here.
ev3.speaker.beep()


class robocik:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.fi = 0
        self.r = 29
        self.L = 117
        self.left_motor = Motor(Port.B)
        self.right_motor = Motor(Port.C)
        self.nr = self.left_motor.angle()
        self.nl = self.right_motor.angle()
        self.center_sensor = UltrasonicSensor(Port.S2)
        self.left_sensor = InfraredSensor(Port.S1)
        self.right_sensor = InfraredSensor(Port.S4)

    def droga(self):

        self.nl_current = self.left_motor.angle()
        self.nr_current = self.right_motor.angle()
        self.delta_nl = self.nl_current - self.nl
        self.delta_nr = self.nr_current - self.nr
        self.Dl = 2 * math.pi * self.r * (self.delta_nl/360)
        self.Dr = 2 * math.pi * self.r * (self.delta_nr/360)
        self.nr = self.nr_current
        self.nl =  self.nl_current

        self.DC = (self.Dl + self.Dr)/2

        return [self.DC, self.Dl, self.Dr]
        
    def update_position(self):
        self.x = self.x + self.droga()[0] * math.cos(self.position()[2])
        self.y = self.y + self.droga()[0] * math.sin(self.position()[2])
        self.fi = self.fi + ((self.droga()[2] - self.droga()[1])/self.L)

    def position(self):
        return [self.x,self.y,self.fi]
    
    def run(self, v, omega):
        self.v_right = v + omega
        self.v_left = v - omega
        #ev3.screen.print((round(self.x),round(self.y),self.fi))
        self.left_motor.run(self.v_left)
        self.right_motor.run(self.v_right)

    def unikaj(self):
        dist_L, dist_R, dist_C = self.distance_przeszkoda()
        x_c = dist_C*math.cos(self.position()[2]) + 5*math.cos(self.position()[2]) + self.position()[0]
        y_c = dist_C*math.sin(self.position()[2]) + 5*math.sin(self.position()[2]) + self.position()[1]
        x_l = dist_L*(0.5*math.cos(self.position()[2])-math.sqrt(3/2)*math.sin(self.position()[2]))-8*math.sin(self.position()[2])+8*math.cos(self.position()[2])+self.position()[0]
        y_l = dist_L*(0.5*math.sin(self.position()[2])+math.sqrt(3/2)*math.cos(self.position()[2]))+8*math.sin(self.position()[2])+8*math.cos(self.position()[2])+self.position()[1]
        x_r = dist_R*(math.sqrt(3/2)*math.sin(self.position()[2])+0.5*math.cos(self.position()[2]))-8*math.sin(self.position()[2])+8*math.cos(self.position()[2])+self.position()[0]
        y_r = dist_R*(0.5*math.sin(self.position()[2])-math.sqrt(3/2)*math.cos(self.position()[2]))+8*math.sin(self.position()[2])+8*math.cos(self.position()[2])+self.position()[1]
        x_obst = (x_c - self.position()[0]) + (x_l - self.position()[0]) + (x_r - self.position()[0])
        y_obst = (y_c - self.position()[1]) + (y_l - self.position()[1]) + (y_r - self.position()[1])
        return x_obst, y_obst
        
    def distance_przeszkoda(self):
        dist_L = self.left_sensor.distance()*0.7
        dist_R = self.right_sensor.distance()*0.7
        dist_C = self.center_sensor.distance()/10

        return dist_L, dist_R, dist_C

    def clear(self):
        dist_L, dist_R, dist_C = self.distance_przeszkoda()
        if dist_L>30 and dist_R>30 and dist_C>50:
            return True
        else:
            return False

    def omin(self):
        x_obst, y_obst = self.unikaj()
        dist_L, dist_R, dist_C = self.distance_przeszkoda()
        ksi = math.atan2(y_obst, x_obst)
        e = ksi - self.position()[2]
        Kp = 150
        v = 100
        if(dist_L>40 and dist_R>40 and dist_C>70):
            Kp = 0
        omega = Kp*e
        self.run(v, omega)
            
    def go_to_goal(self, goal):
        xg, yg = goal
        while (math.sqrt((xg-self.position()[0])**2+(yg-self.position()[1])**2) > 100):
            ksi = math.atan2(yg - self.position()[1], xg - self.position()[0])
            e = ksi - self.position()[2]
            Kp = 215
            omega = Kp*e
            v = 250
            #if self.clear():
            self.run(v,omega)
            #else:
            #self.omin()
            print(round(self.position()[0]),round(self.position()[1]),self.position()[2])
            #ev3.screen.print((round(self.x),round(self.y),self.fi))
            self.update_position()
        self.stop()
            
    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        

while not Button.LEFT in ev3.buttons.pressed():
    pass

waypoints = [[2000, 0], [1250, 1000], [1250, 2000]]
robot = robocik()
for index, goal in enumerate(waypoints):
    ev3.screen.print(goal, index)
    robot.go_to_goal(goal)


    
        
        







    