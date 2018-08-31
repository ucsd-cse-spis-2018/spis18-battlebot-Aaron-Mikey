'''/***********************************************************************************************
    Tutorial Motor

    Control two motors using the separate motor driver board. Note that if the motor is forced to
    run too slow, it may stall (resulting in no motion and a whining noise).     
*************************************************************************************************/'''

#pins used
Ain1Pin= 7
Ain2Pin= 8
PwmAPin= 9
Bin1Pin= 4
Bin2Pin= 5
PwmBPin= 6


def setup():
    '''Configure the pins
    For the motor to move forward:  pin 1 = HIGH and pin 2 = LOW
    For the motor to move backward: pin 1 = LOW  and pin 2 = HIGH
    For the motor to stop or break: pin 1 = LOW  and pin 2 = LOW
    The PWM pin determines the speed of the motor'''
    pinMode(Ain2Pin,OUTPUT)
    pinMode(Ain2Pin,OUTPUT)
    pinMode(PwmAPin,OUTPUT)
    pinMode(Bin1Pin,OUTPUT)
    pinMode(Bin2Pin,OUTPUT)
    pinMode(PwmBPin,OUTPUT)
def loop():
    
    delay(2000)

    #Move motor A forward at full speed
    digitalWrite(Ain1Pin, LOW)
    digitalWrite(Ain2Pin, HIGH)
    analogWrite(PwmAPin, 255)

    #Move motor B forward at full speed
    digitalWrite(Bin1Pin, LOW)
    digitalWrite(Bin2Pin, HIGH)
    analogWrite(PwmBPin, 255)
        
    delay(2000)

    #Move motor A forward at half speed
    digitalWrite(Ain1Pin, LOW)
    digitalWrite(Ain2Pin, HIGH)
    analogWrite(PwmAPin, 123)

    #Move motor B backward at full speed
    digitalWrite(Bin1Pin, HIGH)
    digitalWrite(Bin2Pin, LOW)
    analogWrite(PwmBPin, 255)

    delay(2000);

    #Move motor A backward at half speed
    digitalWrite(Ain1Pin, HIGH)
    digitalWrite(Ain2Pin, LOW)
    analogWrite(PwmAPin, 123)

    #Stop motor B
    digitalWrite(Bin1Pin, LOW)
    digitalWrite(Bin2Pin, LOW)
    analogWrite(PwmBPin, 0)

    delay(2000)

    #Stop motor A 
    digitalWrite(Ain1Pin, LOW)
    digitalWrite(Ain2Pin, LOW)
    analogWrite(PwmAPin, 0)

    #Move motor B forward at half speed
    digitalWrite(Bin1Pin, LOW)
    digitalWrite(Bin2Pin, HIGH)
    analogWrite(PwmBPin, 123)
