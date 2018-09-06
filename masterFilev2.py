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
#from AnalogTester
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

#GPIO Mode(BOARD/BCM)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)    #-----for when I get those errors
'''set GPIO Pins'''
    #Servo Pins
ServoPin =4 #7     #GPIO Pin for servo
    #Motor Pins
GPIO_Ain1 =17 #11
GPIO_Ain2 =27 #13
GPIO_Apwm =22 #15
GPIO_Bin1 =5 #29
GPIO_Bin2 =6 #31
GPIO_Bpwm =13 #33
    #IRProximitySensor Pins
IR_PIN=23 #16
    # Analog Sensor
'''#Software SPI
# The library interface uses BCM labeling
# Connect the chip to the following pins
#       CLK to GPIO 18 (physical pin 12)
#       MISO to GPIO 23 (physical pin 16)
#       MOSI to GPIO 24 (physical pin 18)
#       CS to GPIO 25 (physical pin 22) '''
mcp = Adafruit_MCP3008.MCP3008(clk=18,cs=25,miso=23,mosi=24)

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

'''Set up for motor movement'''
# Both motors are stopped 
GPIO.output(GPIO_Ain1, False)
GPIO.output(GPIO_Ain2, False)
GPIO.output(GPIO_Bin1, False)
GPIO.output(GPIO_Bin2, False)
#Set PWM parameters
pwm_frequency1 = 50
# Create the PWM instances  
pwmA = GPIO.PWM(GPIO_Apwm, pwm_frequency1)
pwmB = GPIO.PWM(GPIO_Bpwm, pwm_frequency1)
pwmA.start(0)
pwmB.start(0)
    
    
def servoRunner():
    '''SERVO METHOD'''
    # The servo is controlled using Pulse Width Modulation (PWM)
    # --- Start of the PWM setup ---
    # Set PWM parameters
    pwm_frequency = 50
    duty_min = 2.5 * float(pwm_frequency) / 50.0
    duty_max = 12.5 * float(pwm_frequency) / 50.0
    
    # Create a PWM instance
    pwm_servo = GPIO.PWM(ServoPin, pwm_frequency)
    # --- End of the PWM setup --
    #sets the duty cycle/Move servo
    angle=0
    setDutyCycle= ((duty_max - duty_min) * float(angle) / 180.0 + duty_min)
    pwm_servo.start(setDutyCycle)
    print ("Moving to angle 0")
    time.sleep(1)
                       
    angle = 180
    setDutyCycle= ((duty_max - duty_min) * float(angle) / 180.0 + duty_min)
    pwm_servo.start(setDutyCycle)
    print ("Moving to angle 180")
    time.sleep(1)

    angle=0
    setDutyCycle= ((duty_max - duty_min) * float(angle) / 180.0 + duty_min)
    pwm_servo.start(setDutyCycle)
    print ("Moving back to angle 0")
    time.sleep(1)
   
    # Create a PWM instance
    pwm_servo = GPIO.PWM(ServoPin, pwm_frequency)
    # --- End of the PWM setup --'''
    




'''MOTOR METHOD'''
def moveForward(speed):
    #Move Forward
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, True)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, True)
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)
    print("Forward")
    time.sleep(0.3)

def moveBackward():
    GPIO.output(GPIO_Ain1, True)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, True)
    GPIO.output(GPIO_Bin2, False)
    pwmA.ChangeDutyCycle(33)
    pwmB.ChangeDutyCycle(33)
    print("Backward")
    time.sleep(0.4)

def turnLeft():
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, True)
    GPIO.output(GPIO_Bin1, True)
    GPIO.output(GPIO_Bin2, False)
    pwmA.ChangeDutyCycle(75)               # right wheel faster
    pwmB.ChangeDutyCycle(50)               # left wheel slower
    print ("Turning Left")
    
    time.sleep(0.01)
    
def turnRight():
    #Turn Right
    GPIO.output(GPIO_Ain1, True)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, True) #Makes other wheel move forward too
    pwmA.ChangeDutyCycle(50)               # right wheel slower
    pwmB.ChangeDutyCycle(75)               # left wheel faster
    print ("Turning Right")
    time.sleep(0.01)
def stop():
    #Stop
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, False)
    print("Stopping")
    time.sleep(3)

def analogSensor():
    val = mcp.read_adc(0)
    print(val)
    return val
    time.sleep(0.5)
    

'''CAMERA METHODS'''
def colorDetect():
    # Define the range colors to filter; these numbers represent HSV
    lowerColorThreshold = np.array([0, 100, 100])
    upperColorThreshold = np.array([18, 255, 255])

    # Initialize the camera and grab a reference to the frame
    camera = picamera.PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    camera.vflip = False                            # Flip upside down or not
    camera.hflip = True                             # Flip left-right or not


    '''1/3:[ (0:480)(0,213)] left
    2/3:[ (0:480)(213,426) ] middle
    3/3:[ (0:480)(426,640) ] right'''
    # Create an array to store a frame
    rawframe = picamera.array.PiRGBArray(camera, size=(640, 480))

    # Allow the camera to warm up
    time.sleep(0.1)
    # Continuously capture frames from the camera
    # Note that the format is BGR instead of RGB because we want to use openCV later on and it only supports BGR
    for frame in camera.capture_continuous(rawframe, format = 'bgr', use_video_port = True):

        # Clear the stream in preparation for the next frame
        rawframe.truncate(0)  
        # Create a numpy array representing the image
        image = frame.array     

        #-----------------------------------------------------
        # We will use numpy and OpenCV for image manipulations
        #-----------------------------------------------------

        # Convert for BGR to HSV color space, using openCV
        # The reason is that it is easier to extract colors in the HSV space
        # Note: the fact that we are using openCV is why the format for the camera.capture was chosen to be BGR
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only colors in a range
        # The colors in range are set to white (255), while the colors not in range are set to black (0)
        ourmask = cv2.inRange(image_hsv, lowerColorThreshold, upperColorThreshold)

                   
        # Bitwise AND of the mask and the original image
        image_masked = cv2.bitwise_and(image, image, mask = ourmask)

       #counts pixels in left, mid, and right pixels
        maskLeft=ourmask[0 : 480, 0 : 213]
        maskMid=ourmask[0 : 480, 213 : 426]
        maskRight=ourmask[0 : 480, 426 : 640]
        numPixLeft=cv2.countNonZero(maskLeft)
        numPixMid=cv2.countNonZero(maskMid)
        numPixRight=cv2.countNonZero(maskRight)
        print("Number of pixels in the color range on the left part of the image:", numPixLeft)
        print("Number of pixels in the color range in the center part of the image:", numPixMid)
        print("Number of pixels in the color range on the right part of the image:", numPixRight)
        maxWhite=max(numPixLeft,numPixMid)
        maxWhite=max(maxWhite,numPixRight)
        isZero=maxWhite==0
        nonZero= maxWhite!=0
        if maxWhite==numPixLeft: #and nonZero:
            print("It's on the left")
            turnLeft()
            #moveForward(50)
            '''elif maxWhite==0:
            print("Not on screen! Scanning room...")
            turnLeft()
            time.sleep(1)
            if isZero:
                turnRight()
                time.sleep(0.5)
                turnRight()
                    if isZero:
                        turnLeft()'''
        elif maxWhite==numPixMid:
            print("It's in the middle")
            moveForward(80)
        else:
            print("It's on the right")
            turnRight()
            #moveForward(50)
        if analogSensor()>250 and ( (maxWhite==numPixMid and nonZero) or (maxWhite==numPixRight and nonZero) ):
            servoRunner()
        # Show the frames
        # The waitKey command is needed to force openCV to show the image
        cv2.imshow("Frame", image)
        cv2.imshow("Mask", ourmask)
        cv2.imshow("Masked image", image_masked)  
        cv2.waitKey(1)




'''MAAAAAIIIIINNNN METHOD'''
if __name__ == '__main__':
    try:
        # This code repeats forever
        while True:
            #servoRunner()
            #moveForward()
            #moveBackward()
            #turnLeft()
            #turnRight()
            colorDetect()
            #analogSensor()
            #time.sleep(2)
            #stop()
            #time.sleep(2)
            

          
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        GPIO.cleanup()

