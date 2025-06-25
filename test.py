import Jetson.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

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
        GPIO.setup(pin1, GPIO.OUT)
        self.__pin1 = pin1

        GPIO.setup(pin2, GPIO.OUT)
        self.__pin2 = pin2

        GPIO.setup(pwm_pin, GPIO.OUT)
        self.__pwm = GPIO.PWM(pwm_pin, 1000)

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
        self.duty_cycle(speed)
        GPIO.output(self.__pin1, GPIO.HIGH)
        GPIO.output(self.__pin2, GPIO.LOW)

    def backward(self, speed: int = 50) -> None:
        if speed < self.__min_duty or speed > self.__max_duty:
            print(f"Invalid speed: {speed}. Setting to 0.")
            speed = 0
        self.duty_cycle(speed)
        GPIO.output(self.__pin1, GPIO.LOW)
        GPIO.output(self.__pin2, GPIO.HIGH)

    def stop(self):
        GPIO.output(self.__pin1, GPIO.LOW)
        GPIO.output(self.__pin2, GPIO.LOW)
        # self.duty_cycle(0)

    def duty_cycle(self, speed: int = 50) -> None:
        if speed < self.__min_duty or speed > self.__max_duty:
            print(f"Invalid speed: {speed}. Setting to 0.")
            speed = 0
        # self.speed = speed
        self.pwm.start(0)
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


try:
    while (True):
        m1.forward()
        m2.forward()
except KeyboardInterrupt:
    m1.stop()
    m2.stop()
finally:
    m1.pwm.stop()
    m2.pwm.stop()
    GPIO.cleanup()
