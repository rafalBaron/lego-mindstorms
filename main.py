#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor, InfraredSensor)
from pybricks.parameters import Button, Port, Stop, Direction, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import math

ev3 = EV3Brick()
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
        self.center_sensor = UltrasonicSensor(Port.S1)
        self.left_sensor = InfraredSensor(Port.S3)
        self.right_sensor = InfraredSensor(Port.S4)

    
    def update_position(self):
        self.nl_current = self.left_motor.angle()
        self.nr_current = self.right_motor.angle()
        self.delta_nl = self.nl_current - self.nl
        self.delta_nr = self.nr_current - self.nr
        self.Dl = 2 * math.pi * self.r * (self.delta_nl/360)
        self.Dr = 2 * math.pi * self.r * (self.delta_nr/360)
        self.nr = self.nr_current
        self.nl =  self.nl_current
        
        self.DC = (self.Dl + self.Dr)/2
        
        self.x = self.x + self.DC * math.cos(self.fi)
        self.y = self.y + self.DC * math.sin(self.fi)
        self.fi = self.fi + ((self.Dr - self.Dl)/self.L)
        
    def run(self, v, omega):
        self.v_right = v + omega
        self.v_left = v - omega
        ev3.screen.print((round(self.x/10),round(self.y/10),self.fi))
        self.left_motor.run(self.v_left)
        self.right_motor.run(self.v_right)

    def unikaj(self):
        dist_L, dist_R, dist_C = self.distance_przeszkoda()
        x_c = dist_C*math.cos(self.fi) + 5*math.cos(self.fi) + self.x
        y_c = dist_C*math.sin(self.fi) + 5*math.sin(self.fi) + self.y
        x_l = dist_L*(0.5*math.cos(self.fi)-math.sqrt(3/2)*math.sin(self.fi))-8*math.sin(self.fi)+8*math.cos(self.fi)+self.x
        y_l = dist_L*(0.5*math.sin(self.fi)+math.sqrt(3/2)*math.cos(self.fi))+8*math.sin(self.fi)+8*math.cos(self.fi)+self.y
        x_r = dist_R*(math.sqrt(3/2)*math.sin(self.fi)+0.5*math.cos(self.fi))-8*math.sin(self.fi)+8*math.cos(self.fi)+self.x
        y_r = dist_R*(0.5*math.sin(self.fi)-math.sqrt(3/2)*math.cos(self.fi))+8*math.sin(self.fi)+8*math.cos(self.fi)+self.y
        x_obst = (x_c - self.x) + (x_l - self.x) + (x_r - self.x)
        y_obst = (y_c - self.y) + (y_l - self.y) + (y_r - self.y)
        return x_obst, y_obst
        
    def distance_przeszkoda(self):
        dist_L = self.left_sensor.distance()*7
        dist_R = self.right_sensor.distance()*7
        dist_C = self.center_sensor.distance()

        return dist_L, dist_R, dist_C

    def clear(self):
        dist_L, dist_R, dist_C = self.distance_przeszkoda()
        if dist_L>300 and dist_R>300 and dist_C>500:
            return True
        else:
            return False

    def omin(self):
        x_obst, y_obst = self.unikaj()
        dist_L, dist_R, dist_C = self.distance_przeszkoda()
        ksi = math.atan2(y_obst, x_obst)
        e = ksi - self.fi
        Kp = 150
        v = 150
        if(dist_L>300 and dist_R>300 and dist_C>500):
            Kp = 0
            v = 300
        omega = Kp*e
        self.run(v, omega)
            
    def go_to_goal(self, goal):
        xg, yg = goal
        while (math.sqrt((xg-self.x)**2+(yg-self.y)**2) > 250):
            ksi = math.atan2(yg - self.y, xg - self.x)
            e = ksi - self.fi
            Kp = 110  
            omega = Kp*e
            v = 300
            if self.clear():
                self.run(v,omega)
            else:
                self.omin()
            print(round(self.x/10),round(self.y/10),self.fi)
            #ev3.screen.print((round(self.x),round(self.y),self.fi))
            self.update_position()
        self.stop()
            
    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        

while not Button.LEFT in ev3.buttons.pressed():
    pass

#waypoints = [[2000, 0], [1250, 1000], [1250, 2000]]
waypoints = [[0,-3500]]
robot = robocik()
for index, goal in enumerate(waypoints):
    robot.go_to_goal(goal)


    
        
        







    