import time
from machine import Pin, PWM
from collections import deque

class Controller:

    # Adjusted servo duty calculations
    SERVO_MIN_DUTY = int(1000 / 20000 * 65535)  # 0 degrees
    SERVO_MAX_DUTY = int(2000 / 20000 * 65535)  # 180 degrees
    SERVO_MID_DUTY = int(1500 / 20000 * 65535)  # 90 degrees

    def __init__(self, logger):
        self.logger = logger
        self.IN1 = Pin(2, Pin.OUT)  # Motor control pin 1
        self.IN2 = Pin(4, Pin.OUT)  # Motor control pin 2

        self.servo = PWM(Pin(18))  # Servo control on GPIO 19
        self.servo.freq(50)

        # Line sensor pins
        self.S1 = Pin(13, Pin.IN)
        self.S2 = Pin(12, Pin.IN)
        self.S3 = Pin(14, Pin.IN)
        self.S4 = Pin(27, Pin.IN)
        self.S5 = Pin(26, Pin.IN)
        self.NEAR = Pin(25, Pin.IN)  # NEAR IR sensor for obstacles

        self.set_servo_to_zero()

    def set_servo_to_zero(self):
        self.logger.info('Servo set to neutral (90°)')
        self.servo.duty_u16(self.SERVO_MID_DUTY)

    def move_forward(self):
        self.logger.info('Moving forward')
        self.IN1.value(1)
        self.IN2.value(0)

    def move_backward(self):
        self.logger.info('Moving backward')
        self.IN1.value(0)
        self.IN2.value(1)

    def stop(self):
        self.logger.info('Stopping motors')
        self.IN1.value(0)
        self.IN2.value(0)
        self.servo.duty_u16(self.SERVO_MID_DUTY)

    def brake_motor(self):
        self.logger.info('Braking motor')
        self.IN1.value(1)
        self.IN2.value(1)

    def stop_and_brake(self):
        self.stop()
        time.sleep_ms(100)  # 100 ms delay
        self.brake_motor()

class SensorArray:
    def __init__(self, sensors):
        self.sensors = sensors

    def read(self):
        raw_values = []
        # Take multiple readings for debouncing
        for _ in range(5):
            readings = [sensor.value() for sensor in self.sensors]
            raw_values.append(readings)
            time.sleep_ms(2)
        # Average the readings
        averaged_values = [sum(col) / len(col) for col in zip(*raw_values)]
        # Convert to binary (0 or 1)
        binary_values = [1 if val > 0.5 else 0 for val in averaged_values]
        return binary_values

class PIDController:
    def __init__(self):
        # Set initial PID values
        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.1
        self.last_error = 0.0
        self.integral = 0.0
        self.error_history = deque(maxlen=50)
        self.tuning = True

    def compute(self, error, delta_time):
        if delta_time <= 0:
            delta_time = 1e-3  # Prevent division by zero

        self.integral += error * delta_time
        derivative = (error - self.last_error) / delta_time
        self.last_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Auto-tune PID parameters if tuning is enabled
        if self.tuning:
            self.error_history.append(abs(error))
            if len(self.error_history) >= self.error_history.maxlen:
                self.auto_tune()

        return output

    def auto_tune(self):
        # Enhanced PID auto-tuning algorithm
        avg_error = sum(self.error_history) / len(self.error_history)
        self.error_history.clear()

        # Adjust Kp, Ki, Kd based on average error
        if avg_error > 1.0:
            self.kp += 0.05
            self.kd += 0.01
            self.ki = max(self.ki - 0.001, 0.0)
        elif avg_error < 0.5:
            self.kp = max(self.kp - 0.05, 0.1)
            self.kd = max(self.kd - 0.01, 0.01)
            self.ki += 0.001
        else:
            # Stop tuning when average error is acceptable
            self.tuning = False
            self.logger.info("PID tuning completed with Kp: {:.2f}, Ki: {:.4f}, Kd: {:.2f}".format(
                self.kp, self.ki, self.kd))

class EnhancedLineFollower(Controller):
    def __init__(self, logger):
        super().__init__(logger)
        self.sensor_array = SensorArray([self.S1, self.S2, self.S3, self.S4, self.S5])
        self.pid = PIDController()
        self.last_time = time.ticks_ms()
        self.logger.info("Enhanced Line Follower initialized")

    def follow_line(self):
        self.logger.info("Starting line following")
        try:
            while True:
                # Check NEAR sensor
                if self.NEAR.value() == 0:
                    self.logger.info("Obstacle detected. Stopping.")
                    self.stop_and_brake()
                    while self.NEAR.value() == 0:
                        # Wait until obstacle is removed
                        time.sleep_ms(100)
                    self.logger.info("Obstacle cleared. Resuming.")

                sensors = self.sensor_array.read()

                # Calculate line position error
                positions = [-2, -1, 0, 1, 2]
                error = 0
                active_sensors = 0
                for pos, val in zip(positions, sensors):
                    if val == 1:
                        error += pos
                        active_sensors += 1

                if active_sensors > 0:
                    error /= active_sensors
                else:
                    # Line is lost
                    self.logger.error("Line lost. Stopping.")
                    self.stop_and_brake()
                    break  # Exit the loop immediately

                current_time = time.ticks_ms()
                delta_time = (time.ticks_diff(current_time, self.last_time)) / 1000.0
                self.last_time = current_time

                adjustment = self.pid.compute(error, delta_time)

                # Convert adjustment to servo duty cycle
                max_adjustment = (self.SERVO_MAX_DUTY - self.SERVO_MID_DUTY) * 0.5
                adjustment = max(min(adjustment, max_adjustment), -max_adjustment)
                target_duty = int(self.SERVO_MID_DUTY + adjustment)

                # Clamp target_duty within servo limits
                target_duty = max(min(target_duty, self.SERVO_MAX_DUTY), self.SERVO_MIN_DUTY)

                self.servo.duty_u16(target_duty)

                self.move_forward()
                time.sleep_ms(10)
        except Exception as e:
            self.logger.error(f"Error during line following: {e}")
            self.stop_and_brake()
            self.shutdown()
        finally:
            # Ensure the robot stops in any case
            self.stop_and_brake()
            self.shutdown()

    def shutdown(self):
        self.logger.info("Shutting down.")
        self.stop_and_brake()
        self.servo.deinit()
        # Deinitialize motor pins
        self.IN1.init(Pin.IN)
        self.IN2.init(Pin.IN)
        self.logger.info("Shutdown complete.")

class Logger:
    def __init__(self, filename):
        self.filename = filename
        self.buffer = []
        self.buffer_limit = 10

    def log(self, level, message):
        current_time = time.localtime()
        timestamp = "{:04}-{:02}-{:02} {:02}:{:02}:{:02}".format(
            current_time[0], current_time[1], current_time[2],
            current_time[3], current_time[4], current_time[5]
        )
        log_entry = f"{timestamp} - {level} - {message}\n"
        self.buffer.append(log_entry)
        if len(self.buffer) >= self.buffer_limit:
            self.flush()

    def flush(self):
        try:
            with open(self.filename, 'a') as log_file:
                log_file.writelines(self.buffer)
            self.buffer = []
        except Exception as e:
            print(f"Failed to write to log file: {e}")

    def error(self, message):
        self.log('ERROR', message)
        print(f'ERROR: {message}')

    def debug(self, message):
        self.log('DEBUG', message)
        print(f'DEBUG: {message}')

    def info(self, message):
        self.log('INFO', message)
        print(f'INFO: {message}')

    def __del__(self):
        self.flush()

# Initialize Logger and EnhancedLineFollower
llogger = Logger('car_log.txt')
llogger.info('Starting enhanced line follower')
car = EnhancedLineFollower(logger=llogger)

try:
    car.follow_line()
except KeyboardInterrupt:
    car.shutdown()
except Exception as e:
    llogger.error(f"Unhandled exception: {e}")
    car.shutdown()
finally:
    # Ensure the robot stops in any case
    car.shutdown()