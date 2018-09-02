'''This file combines all the other methods needed for the bot
(camera06ColorTest,motorTester,servotester, and IRProximitySensor)'''
#from camera06_detectcolortest
import cv2
import picamera
import picamera.array
import time        #used in cam,IRProximity,motorTesterNew,servoTester
import numpy as np
#from IRProximityTestNew
import RPi.GPIO as GPIO     #used in IRProximity,motorTesterNew,servoTester

#GPIO Mode(BOARD/BCM)
GPIO.setmode(GPIO.BOARD)

'''set GPIO Pins'''
    #Servo Pins
ServoPin = 7     #GPIO Pin for servo
    #Motor Pins
GPIO_Ain1 = 11
GPIO_Ain2 = 13
GPIO_Apwm = 15
GPIO_Bin1 = 29
GPIO_Bin2 = 31
GPIO_Bpwm = 33
    #IRProximitySensor Pins
IR_PIN=16

'''Setting up GPIO direction(IN/OUT)'''
    #Setup for servo, pin mode to output
GPIO.setup(ServoPin,GPIO.OUT)
    #Setup for motors to be all outputs
GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)
GPIO.setup(GPIO_Bin1, GPIO.OUT)
GPIO.setup(GPIO_Bin2, GPIO.OUT)
GPIO.setup(GPIO_Bpwm, GPIO.OUT)

#Servo,motorTester,IRProximity,camera06
