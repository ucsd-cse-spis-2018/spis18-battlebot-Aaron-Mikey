# Libraries
import RPi.GPIO as GPIO
import time

 
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD)
 
# set GPIO Pins
GPIO_Ain1 = 11
GPIO_Ain2 = 13
GPIO_Apwm = 15
GPIO_Bin1 = 29
GPIO_Bin2 = 31
GPIO_Bpwm = 33

# Set GPIO direction (IN / OUT)
GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)
GPIO.setup(GPIO_Bin1, GPIO.OUT)
GPIO.setup(GPIO_Bin2, GPIO.OUT)
GPIO.setup(GPIO_Bpwm, GPIO.OUT)

# Both motors are stopped 
GPIO.output(GPIO_Ain1, False)
GPIO.output(GPIO_Ain2, False)
GPIO.output(GPIO_Bin1, False)
GPIO.output(GPIO_Bin2, False)

# Set PWM parameters
pwm_frequency = 50

# Create the PWM instances  
pwmA = GPIO.PWM(GPIO_Apwm, pwm_frequency)
pwmB = GPIO.PWM(GPIO_Bpwm, pwm_frequency)

# Set the duty cycle (between 0 and 100)
# The duty cycle determines the speed of the wheels
pwmA.start(100)
pwmB.start(100)

 
# Main program
if __name__ == '__main__':
    try:
        
        while True:

            '''GPIO.output(GPIO_Ain1, True)
            GPIO.output(GPIO_Ain2, False)
            GPIO.output(GPIO_Bin1, True)
            GPIO.output(GPIO_Bin2, False)
            pwmA.ChangeDutyCycle(50)                # duty cycle between 0 and 100
            pwmB.ChangeDutyCycle(50)                # duty cycle between 0 and 100
            print ("Forward half speed")
            time.sleep(1)'''

            GPIO.output(GPIO_Ain1, True)
            GPIO.output(GPIO_Ain2, False)
            GPIO.output(GPIO_Bin1, False)
            GPIO.output(GPIO_Bin2, True) #Makes other wheel(L) move forward too
            pwmA.ChangeDutyCycle(50)               # duty cycle between 0 and 100
            pwmB.ChangeDutyCycle(50)               # duty cycle between 0 and 100
            print ("Forward full speed")
            time.sleep(1)
            
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
            GPIO.output(GPIO_Bin2, True) #Makes other wheel move forward too
            pwmA.ChangeDutyCycle(75)               # left wheel slower
            pwmB.ChangeDutyCycle(20)               # left wheel slower
            print ("Forward full speed")
            time.sleep(1)

            
            GPIO.output(GPIO_Ain1, False)
            GPIO.output(GPIO_Ain2, False)
            GPIO.output(GPIO_Bin1, False)
            GPIO.output(GPIO_Bin2, False)
            print ("Stop")
            time.sleep(1)
            

    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        pwmA.stop()
        pwmB.stop()
        print("Program stopped by User")
        GPIO.cleanup()
