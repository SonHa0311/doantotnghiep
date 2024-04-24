#!/usr/bin/enable_motorv python3
# This Code controls Dc motor and servo motor through pwm
# We defined all gpio pins in BCM mode and create a switch statment to test all
#DC motor pwm -> 0-100 (direct duty cycle)
# servo motor angle -> 0-90 ( 35 is straight , 65 is full right , 0 is full left)

import sys
from numpy import interp
from time import sleep
import RPi.GPIO as GPIO
import wiringpi


from collections import deque

angle_queue = deque(maxlen=3)



wiringpi.wiringPiSetup()
wiringpi.pinMode(21,1)
wiringpi.pinMode(22,1)
wiringpi.pinMode(23,1)
wiringpi.pinMode(24,1)

#dc_pwm,servo_pwm,servo_motor=0,0,18
#Motors Pins
motor_a = 21 
motor_b = 20
enable_motor = 16
servo_motor=18
run_car=True
prev_Mode = "Detection"
GPIO.setmode(GPIO.BCM)
    #Motors Setup
GPIO.setup(motor_a,GPIO.OUT)
GPIO.setup(motor_b,GPIO.OUT)
GPIO.setup(enable_motor,GPIO.OUT)
GPIO.setup(servo_motor, GPIO.OUT)
    #Pwm setup
dc_pwm=GPIO.PWM(enable_motor,1000)
dc_pwm.start(0)
servo_pwm=GPIO.PWM(servo_motor, 50)
servo_pwm.start(0)
if run_car:
    car_speed=90
else:
    car_speed=0
dc_pwm.ChangeDutyCycle(car_speed)
## function names are self representing 
def setServoAngle(angle):
    duty = angle / 18 + 2
    GPIO.output(servo_motor, True)
    servo_pwm.ChangeDutyCycle(duty)
    #sleep(1)
    GPIO.output(servo_motor, False)
    servo_pwm.ChangeDutyCycle(duty)
def forward():
    GPIO.output(motor_a,GPIO.HIGH)
    GPIO.output(motor_b,GPIO.LOW)
    print("forward main hun")
def backward():
    GPIO.output(motor_a,GPIO.LOW)
    GPIO.output(motor_b,GPIO.HIGH)
def stop():
    GPIO.output(motor_a,GPIO.LOW)
    GPIO.output(motor_b,GPIO.LOW)
def changePwm(x):
    dc_pwm.ChangeDutyCycle(x)
def turnOfCar():
    GPIO.cleanup()
    dc_pwm.stop()
    servo_pwm.stop()
    
def On_Cam():
    wiringpi.digitalWrite(21,0)
    wiringpi.digitalWrite(22,1)
    wiringpi.digitalWrite(23,0)
    wiringpi.digitalWrite(24,0)
def Off_Cam():
    wiringpi.digitalWrite(21,1)
    wiringpi.digitalWrite(22,0)
    wiringpi.digitalWrite(23,0)
    wiringpi.digitalWrite(24,0)

    

def beInLane(Max_Sane_dist,distance,curvature , Mode , Tracked_class):

    IncreaseTireSpeedInTurns = False
    global car_speed , prev_Mode
    if((Tracked_class!=0) and (prev_Mode == "Tracking") and (Mode == "Detection")):
        if  (Tracked_class =="speed_sign_70"):
            car_speed = 70
        elif(Tracked_class =="speed_sign_80"):
            car_speed = 80
        elif(Tracked_class =="stop"):
            car_speed = 0
        
    prev_Mode = Mode # Set prevMode to current Mode
    
    Max_turn_angle = 90  # 90
    Max_turn_angle_neg = -90   #-90

    CarTurn_angle = 0

    if( (distance > Max_Sane_dist) or (distance < (-1 * Max_Sane_dist) ) ):
        # Max sane distance reached ---> Max penalize (Max turn Tires)
        if(distance > Max_Sane_dist):
            #Car offseted left --> Turn full wheels right
            CarTurn_angle = Max_turn_angle + curvature
        else:
            #Car Offseted right--> Turn full wheels left
            CarTurn_angle = Max_turn_angle_neg + curvature
    else:
        # Within allowed distance limits for car and lane
        # Interpolate distance to Angle Range
        Turn_angle_interpolated = interp(distance,[-Max_Sane_dist,Max_Sane_dist],[-90,90])
        print("Turn_angle_interpolated = ", Turn_angle_interpolated)
        CarTurn_angle = Turn_angle_interpolated + curvature

    # Handle Max Limit [if (greater then either limits) --> set to max limit]
    if( (CarTurn_angle > Max_turn_angle) or (CarTurn_angle < (-1 *Max_turn_angle) ) ):
        if(CarTurn_angle > Max_turn_angle):
            CarTurn_angle = Max_turn_angle
        else:
            CarTurn_angle = -Max_turn_angle

    angle = interp(CarTurn_angle,[-Max_turn_angle,Max_turn_angle],[5,65])

    curr_speed = car_speed
    
    if IncreaseTireSpeedInTurns:
        if(angle>95):
            car_speed_turn = interp(angle,[95,120],[80,100])
            if run_car:
                dc_pwm.ChangeDutyCycle(car_speed_turn)
            curr_speed = car_speed_turn
        elif(angle<55):
            car_speed_turn = interp(angle,[30,55],[100,80])
            if run_car:
                dc_pwm.ChangeDutyCycle(car_speed_turn)
            curr_speed = car_speed_turn
        else:
            dc_pwm.ChangeDutyCycle(car_speed)
            
    # Rolling average applied to get smoother steering angles for robot
    global angle_queue
    angle_queue.append(angle)
    angle = (sum(angle_queue)/len(angle_queue))
    
    print("goc_servo = ", angle)
    setServoAngle(int(angle))
    
    return angle , curr_speed
#turnOfCar() // to disconnect all channels of pwm

    