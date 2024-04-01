from machine import Pin

class Controller:
    def __init__(self):
        # Define the pins connected to the L298N
        # Replace with the actual pins you're using
        self.IN1 = Pin(0, Pin.OUT)
        self.IN2 = Pin(1, Pin.OUT)
        self.IN3 = Pin(2, Pin.OUT)
        self.IN4 = Pin(3, Pin.OUT)

    def move_forward(self):
        self.IN1.high()
        self.IN2.low()
        self.IN3.high()
        self.IN4.low()

    def move_backward(self):
        self.IN1.low()
        self.IN2.high()
        self.IN3.low()
        self.IN4.high()

    def turn_left(self):
        self.IN1.low()
        self.IN2.high()
        self.IN3.high()
        self.IN4.low()

    def turn_right(self):
        self.IN1.high()
        self.IN2.low()
        self.IN3.low()
        self.IN4.high()

    def brake(self):
        self.stop()

    def stop(self):
        self.IN1.low()
        self.IN2.low()
        self.IN3.low()
        self.IN4.low()