import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
IR_PIN=16
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
try:
    while True:
        readir(IR_PIN)
        time.sleep(0.5)
except KeyboardInterrupt:
    GPIO.cleanup()
    exit()
