import time
from machine import Pin, PWM # type: ignore
import time

# Controller class for motor and servo control
class Controller:
    SERVO_MIN_DUTY = int(1000 / 20000 * 65535)  # 0 degrees
    SERVO_MAX_DUTY = int(2000 / 20000 * 65535)  # 180 degrees
    SERVO_MID_DUTY = int(1500 / 20000 * 65535)  # 90 degrees

    def __init__(self):
        self.IN1 = Pin(2, Pin.OUT)  # Motor control pin 1
        self.IN2 = Pin(4, Pin.OUT)  # Motor control pin 2
        
        self.servo = PWM(Pin(21))  # Servo control on GPIO 21
        self.servo.freq(50)
        self.set_servo_to_zero()

    def set_servo_to_zero(self):
        print('Servo set to neutral (90°)')
        self.servo.duty_u16(self.SERVO_MID_DUTY)

    def move_forward(self):
        print('Moving forward')
        self.IN1.value(1)
        self.IN2.value(0)

    def move_backward(self):
        print('Moving backward')
        self.IN1.value(0)
        self.IN2.value(1)

    def turn_left(self):
        print('Turning left (servo to 0°)')
        self.servo.duty_u16(self.SERVO_MIN_DUTY)

    def turn_right(self):
        print('Turning right (servo to 180°)')
        self.servo.duty_u16(self.SERVO_MAX_DUTY)

    def stop(self):
        print('Stopping motors')
        self.IN1.value(0)
        self.IN2.value(0)
        self.servo.duty_u16(self.SERVO_MID_DUTY)

    def brake_motor(self):
        print('Braking motor')
        self.IN1.value(1)
        self.IN2.value(1)

    def stop_and_brake(self):
        self.stop()
        time.sleep(0.1)
        self.brake_motor()

# Line Follower class with PID and smooth obstacle avoidance

class LineFollower(Controller):
    def __init__(self):
        super().__init__()

        # Line sensor pins
        self.S1 = Pin(13, Pin.IN)
        self.S2 = Pin(12, Pin.IN)
        self.S3 = Pin(14, Pin.IN)
        self.S4 = Pin(27, Pin.IN)
        self.S5 = Pin(26, Pin.IN)
        self.NEAR = Pin(33, Pin.IN)  # NEAR IR sensor for obstacles

        # Ultrasonic sensors for obstacle avoidance
        self.front_left_trigger = Pin(5, Pin.OUT)
        self.front_left_echo = Pin(15, Pin.IN)
        self.front_right_trigger = Pin(23, Pin.OUT)
        self.front_right_echo = Pin(22, Pin.IN)

        # PID control constants
        self.kp = 0.5
        self.ki = 0.1
        self.kd = 0.2

        # Memory for path tracking
        self.path_memory = []

        # Track errors
        self.previous_error = 0
        self.integral = 0

        # Speed control
        self.base_speed = 32768  # Base speed (mid-range)
        
    def read_sensors(self):
        return [self.S1.value(), self.S2.value(), self.S3.value(), self.S4.value(), self.S5.value()]

    def calculate_pid(self, error):
        self.integral += error
        derivative = error - self.previous_error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output

    def adjust_speed(self, error):
        adjustment = self.calculate_pid(error)
        left_speed = self.base_speed + adjustment
        right_speed = self.base_speed - adjustment

        # Ensure speeds are within valid range
        left_speed = max(0, min(65535, left_speed))
        right_speed = max(0, min(65535, right_speed))

        # Adjust servo based on error
        if left_speed > right_speed:
            self.turn_left()
        elif right_speed > left_speed:
            self.turn_right()
        else:
            self.set_servo_to_zero()

        self.move_forward()

    def detect_intersection(self, sensors):
        # Simple detection for intersections (where multiple sensors read black)
        return sensors == [1, 1, 1, 1, 1]  # Can be tweaked based on track setup

    def read_ultrasonic(self, trigger_pin, echo_pin):
        trigger_pin.value(0)
        time.sleep_us(2) # type: ignore
        trigger_pin.value(1)
        time.sleep_us(10) # type: ignore
        trigger_pin.value(0)
        pulse_time = time_pulse_us(echo_pin, 1)  # type: ignore
        distance = (pulse_time * 0.0343) / 2  # Convert to distance
        return distance

    def avoid_obstacle(self):
        left_distance = self.read_ultrasonic(self.front_left_trigger, self.front_left_echo)
        right_distance = self.read_ultrasonic(self.front_right_trigger, self.front_right_echo)

        if left_distance < 30 and right_distance >= 30:
            self.turn_right()
            time.sleep(0.5)
        elif right_distance < 30 and left_distance >= 30:
            self.turn_left()
            time.sleep(0.5)
        else:
            self.stop()

    def follow_line(self):
        while True:
            sensors = self.read_sensors()

            # Detect obstacles using NEAR IR sensor
            if self.NEAR.value() == 0:
                print('Obstacle detected! Stopping.')
                self.stop_and_brake()
                self.avoid_obstacle()
                continue

            # Calculate error for line following
            error = 0
            if sensors == [1, 0, 0, 0, 0]:
                error = -2
            elif sensors == [1, 1, 0, 0, 0]:
                error = -1
            elif sensors == [0, 1, 0, 0, 0]:
                error = 0
            elif sensors == [0, 1, 1, 0, 0]:
                error = 1
            elif sensors == [0, 0, 1, 1, 0]:
                error = 2
            elif sensors == [0, 0, 1, 0, 0]:
                error = 0
            else:
                print('Line lost, stopping.')
                self.stop_and_brake()
                time.sleep(0.5)
                continue

            # Handle intersections (e.g., decision-making on new paths)
            if self.detect_intersection(sensors):
                print("Intersection detected!")
                if self.path_memory:  # If there's memory data from a previous run
                    self.adjust_to_memory()
                else:
                    self.path_memory.append('Intersection')
                continue

            # Adjust speed based on PID control
            self.adjust_speed(error)
            time.sleep(0.1)

    def adjust_to_memory(self):
        # Adjust movement based on stored memory
        # This would be expanded based on your logic (e.g., choosing specific paths)
        print("Adjusting to past memory...")
        # Example: after an intersection, follow the last successful path
        last_action = self.path_memory[-1]
        if last_action == "Left":
            self.turn_left()
        elif last_action == "Right":
            self.turn_right()

    def stop(self):
        super().stop()
        
car = LineFollower()
print('starting following line')
car.follow_line()
