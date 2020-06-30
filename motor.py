#!/usr/bin/python3
import threading
import RPi.GPIO as GPIO
import time
from guizero import App, Box, Text, TextBox, PushButton, Slider
import matplotlib.pyplot as plt

""" Input Output  configaration """
pinEncA = 23
pinEncB = 24
pinPwm1 = 7
pinPwm2 = 8
pinEn = 1
GPIO.setmode(GPIO.BCM)
GPIO.setup(pinEncA, GPIO.IN, GPIO.PUD_DOWN)  # Encoder 23
GPIO.setup(pinEncB, GPIO.IN, GPIO.PUD_DOWN)  # Encoder 24
GPIO.setup(pinPwm1, GPIO.OUT)  # PWM1
GPIO.setup(pinPwm2, GPIO.OUT)  # PWM2
GPIO.setup(pinEn, GPIO.OUT)  # enable

""" PID Const """
Kp = 1
Ki = 1
Kd = 1
previousError = 0
integral = 0
derivative = 0
setRPM = 500
counter = 0
runRPM = 0
output = 0
"""time var"""
previousTime = 0
currentTime = 0
timestamp = 0
samplingTime = 1
"""control var"""
PWM1 = 0
PWM2 = 0
"""plot var"""
pltData = []
"""calc PID"""


def PID():
    global Kp
    global Ki
    global Kd
    global previousError
    global integral
    global derivative
    global setRPM
    global output
    error = setRPM - runRPM

    integral = integral + (error*timestamp)
    derivative = (error - previousError)/timestamp

    output = (Kp*error) + (Ki*integral) + (Kd*derivative)

    previousError = error
    return output


"""config motor"""
GPIO.output(pinEn, True)
pwm1 = GPIO.PWM(pinPwm1, 2500)
pwm2 = GPIO.PWM(pinPwm2, 2500)
pwm1.start(PWM1)
pwm2.start(PWM2)
# p1.ChangeDutyCycle(0)

try:
    t1 = threading.Thread(target=plt.show)
    t1.start()
    if (previousTime == 0):
        previousTime = time.time()
    lastStateA = GPIO.input(pinEncA)

    """main loop"""
    while True:
        currentTime = time.time()
        if(currentTime - previousTime >= samplingTime):
            timestamp = currentTime-previousTime  # calc dtime
            previousTime = currentTime

            runRPM = 60*counter/timestamp/22
            counter = 0

            PID()

            if(setRPM > 0):
                PWM2 = 0
                PWM1 += output/200
                # constraint value pwm between 0 and 100
                if(PWM1 > 100):
                    PWM1 = 100
                if(PWM1 < 0):
                    PWM1 = 0
                # update pwm
                pwm1.ChangeDutyCycle(PWM1)
                pwm2.ChangeDutyCycle(PWM2)
            elif(setRPM < 0):
                PWM1 = 0
                PWM2 += output/200
                # constraint value pwm between 0 and 100
                if(PWM2 > 100):
                    PWM2 = 100
                if(PWM2 < 0):
                    PWM2 = 0
                # update pwm
                pwm1.ChangeDutyCycle(PWM1)
                pwm2.ChangeDutyCycle(PWM2)
            else:
                PWM1 = 0
                PWM2 = 0
                pwm1.ChangeDutyCycle(PWM1)
                pwm2.ChangeDutyCycle(PWM2)

            print("speed = ", runRPM, " RPM in ", timestamp, " s")
            print("PID = ", output)

            pltData.append(runRPM)
            plt.plot(pltData)
            plt.pause(0.05)

        encA = GPIO.input(pinEncA)
        encB = GPIO.input(pinEncB)
        if encA != lastStateA:
            if encA != encB:
                counter += 1
                # print(counter)
            else:
                counter -= 1
                # print(counter)
        lastStateA = encA

except KeyboardInterrupt:
    GPIO.cleanup()
finally:
    GPIO.cleanup()
