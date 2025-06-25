import Jetson.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

class DCMotor:
    # the min_duty and max_duty are defined for 15000Hz frequency
    # you can pass as arguments
    def __init__(
        self,
        pin1: int,
        pin2: int,
        pwm_pin: int,
        min_duty: int = 0,
        max_duty: int = 100,
    ) -> None:
        GPIO.setup(pin1, GPIO.OUT, initial=GPIO.LOW)
        self.__pin1 = pin1

        GPIO.setup(pin2, GPIO.OUT, initial=GPIO.LOW)
        self.__pin2 = pin2

        GPIO.setup(pwm_pin, GPIO.OUT, initial=GPIO.HIGH)
        self.__pwm = GPIO.PWM(pwm_pin, 1000)
        self.pwm.start(0)

        self.__min_duty = min_duty
        self.__max_duty = max_duty
        # self.speed: int = 0

    @property
    def pwm(self) -> GPIO.PWM:
        return self.__pwm

    # speed value can be between 0 and 100
    def forward(self, speed: int = 50) -> None:
        if speed < self.__min_duty or speed > self.__max_duty:
            print(f"Invalid speed: {speed}. Setting to 0.")
            speed = 0
        GPIO.output(self.__pin1, GPIO.HIGH)
        GPIO.output(self.__pin2, GPIO.LOW)
        self.duty_cycle(speed)

    def backward(self, speed: int = 50) -> None:
        if speed < self.__min_duty or speed > self.__max_duty:
            print(f"Invalid speed: {speed}. Setting to 0.")
            speed = 0
        GPIO.output(self.__pin1, GPIO.LOW)
        GPIO.output(self.__pin2, GPIO.HIGH)
        self.duty_cycle(speed)

    def stop(self):
        GPIO.output(self.__pin1, GPIO.LOW)
        GPIO.output(self.__pin2, GPIO.LOW)
        self.duty_cycle(0)

    def duty_cycle(self, speed: int = 50) -> None:
        if speed < self.__min_duty or speed > self.__max_duty:
            print(f"Invalid speed: {speed}. Setting to 0.")
            speed = 0
        # self.speed = speed
        # self.pwm.start(0)
        self.pwm.ChangeDutyCycle(speed)
        # if self.speed <= 0 or self.speed > 100:
        #     duty_cycle = 0
        # else:
        #     duty_cycle = int(
        #         self.min_duty
        #         + (self.max_duty - self.min_duty) * ((self.speed - 1) / (100 - 1))
        #     )
        # return duty_cycle


m1 = DCMotor(
    11, 13, 32
)
m2 = DCMotor(
    16, 18, 33
)


import time
try:
    print("Motor Control Test")
    print("Motor 1 Forward at 50% speed...")
    m1.forward(50)
    time.sleep(3)

    print("Motor 1 Backward at 75% speed...")
    m2.backward(75)
    time.sleep(3)

    print("Motor 1 Stop...")
    m1.stop()
    time.sleep(1)

    print("Motor 2 Forward at 60% speed...")
    m2.backward(60)
    time.sleep(3)

    print("Motor 2 Backward at 80% speed...")
    m2.backward(80)
    time.sleep(3)

    print("Motor 2 Stop...")
    m2.stop()
    time.sleep(1)

    print("Both motors forward at 40% speed...")
    m1.forward(40)
    m2.forward(40)
    time.sleep(3)

    print("Stopping all motors and cleaning up GPIO...")

except KeyboardInterrupt:
    print("\nExiting program.")

finally:
    m1.pwm.stop()
    m2.pwm.stop()
    GPIO.cleanup() # This resets all GPIO pins used by the script to their default state
    print("GPIO cleaned up.")
