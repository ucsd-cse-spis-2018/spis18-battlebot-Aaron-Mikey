'''This file combines all the other methods needed for the bot
(camera06ColorTest,motorTester,servotester, and IRProximitySensor)'''
#Order of files Servo,motorTester,IRProximity,camera06
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
GPIO.setwarnings(False)    #-----for when I get those errors
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


'''SERVO METHOD'''
# The servo is controlled using Pulse Width Modulation (PWM)
# --- Start of the PWM setup ---
    # Set PWM parameters
pwm_frequency = 50
duty_min = 2.5 * float(pwm_frequency) / 50.0
duty_max = 12.5 * float(pwm_frequency) / 50.0

    # Helper function to set the duty cycle
def set_duty_cycle(angle):
    return ((duty_max - duty_min) * float(angle) / 180.0 + duty_min)

    # Create a PWM instance
pwm_servo = GPIO.PWM(ServoPin, pwm_frequency)
# --- End of the PWM setup ---




'''MOTOR METHOD'''
# Both motors are stopped 
GPIO.output(GPIO_Ain1, False)
GPIO.output(GPIO_Ain2, False)
GPIO.output(GPIO_Bin1, False)
GPIO.output(GPIO_Bin2, False)
# Set PWM parameters
pwm_frequency1 = 50

# Create the PWM instances  
pwmA = GPIO.PWM(GPIO_Apwm, pwm_frequency1)
pwmB = GPIO.PWM(GPIO_Bpwm, pwm_frequency1)

# Set the duty cycle (between 0 and 100)
# The duty cycle determines the speed of the wheels
pwmA.start(100)
pwmB.start(100)



'''IR PROXIMITY METHOD'''
def readir(pin):
    GPIO.setup(pin,GPIO.OUT)
    GPIO.output(pin,False)
    time.sleep(0.1)
    GPIO.setup(pin,GPIO.IN)
    i=GPIO.input(pin)
    if i==False:
        print("No intruders",i)
    elif i==True:
        print("Intruders identified",i)



'''CAMERA METHODS'''
# Define the range colors to filter; these numbers represent HSV
lowerColorThreshold = np.array([120, 57, 0])
upperColorThreshold = np.array([255, 255, 255])

# Initialize the camera and grab a reference to the frame
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
camera.vflip = False                            # Flip upside down or not
camera.hflip = True                             # Flip left-right or not

# Create an array to store a frame
rawframe = picamera.array.PiRGBArray(camera, size=(640, 480))

# Allow the camera to warm up
time.sleep(0.1)











'''MAAAAAIIIIINNNN METHOD'''
