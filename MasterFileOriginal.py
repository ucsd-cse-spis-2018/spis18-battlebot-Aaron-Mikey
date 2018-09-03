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
    time.sleep(0.5)
                       
    angle = 100
    setDutyCycle= ((duty_max - duty_min) * float(angle) / 180.0 + duty_min)
    pwm_servo.start(setDutyCycle)
    print ("Moving to angle 100")
    time.sleep(0.5)
   
    # Create a PWM instance
    pwm_servo = GPIO.PWM(ServoPin, pwm_frequency)
    # --- End of the PWM setup --

'''# Helper function to set the duty cycle
def set_duty_cycle(angle):
    return ((duty_max - duty_min) * float(angle) / 180.0 + duty_min)

    # Create a PWM instance
    pwm_servo = GPIO.PWM(ServoPin, pwm_frequency)
    # --- End of the PWM setup ---'''


'''MOTOR METHOD'''
def motorRunner():
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
    #Move Forward
    GPIO.output(GPIO_Ain1, True)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, True)            #Makes other wheel(L) move forward too
    pwmA.ChangeDutyCycle(100)               # duty cycle between 0 and 100
    pwmB.ChangeDutyCycle(100)               # duty cycle between 0 and 100
    print ("Forward full speed")
    time.sleep(1)

    #Move Backward
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, True)
    GPIO.output(GPIO_Bin1, True)
    GPIO.output(GPIO_Bin2, False)
    pwmA.ChangeDutyCycle(33)                # duty cycle between 0 and 100
    pwmB.ChangeDutyCycle(33)                # duty cycle between 0 and 100
    print ("Backward third speed")
    time.sleep(1)

    #Turn Left
    GPIO.output(GPIO_Ain1, True)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, True)    #Makes other wheel move forward too
    pwmA.ChangeDutyCycle(75)               # right wheel faster
    pwmB.ChangeDutyCycle(0)               # left wheel slower
    print ("Turning Left")
    Time.sleep(1)

    #Turn Right
    GPIO.output(GPIO_Ain1, True)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, True) #Makes other wheel move forward too
    pwmA.ChangeDutyCycle(0)               # right wheel slower
    pwmB.ChangeDutyCycle(75)               # left wheel faster
    print ("Turning Right")
    time.sleep(1)

            
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, False)
    print ("Stop")
    time.sleep(1)


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
def colorDetect():
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

        # Count the number of white pixels in the mask
        numpixels = cv2.countNonZero(ourmask)
        print("Number of pixels in the color range:", numpixels)
       
        # Get the size of the array (the mask is of type 'numpy')
        # This should be 640 x 480 as defined earlier
        numx, numy = ourmask.shape

        # Select a part of the image and count the number of white pixels
        ourmask_center = ourmask[ numx//4 : 3*numx//4 , numy//4 : 3*numy//4 ]
        numpixels_center = cv2.countNonZero(ourmask_center)
        print("Number of pixels in the color range in the center part of the image:", numpixels_center)
           
        # Bitwise AND of the mask and the original image
        image_masked = cv2.bitwise_and(image, image, mask = ourmask)


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
            servoRunner()
            motorRunner()
            #IR PROXIMITY METHOD
            readir(IR_PIN)
            time.sleep(0.5)
            colorDetect()

        '''END OF MAIN METHOD RESULTS IN RUNNING SEQUENTIALLY
        CAUSES SERVO TO TWITCH AT PICTURE DISPLAY'''   
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        GPIO.cleanup()

