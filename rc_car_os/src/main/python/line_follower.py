import time
from machine import Pin, PWM
import math
import random
from collections import deque

class Controller:

    # Adjusted servo duty calculations
    SERVO_MIN_DUTY = int((0.5 / 20) * 65535)   # 0.5 ms pulse width for 0 degrees
    SERVO_MAX_DUTY = int((2.5 / 20) * 65535)   # 2.5 ms pulse width for 180 degrees
    SERVO_MID_DUTY = int((1.5 / 20) * 65535)   # 1.5 ms pulse width for 90 degrees
    
    def __init__(self, logger):
        self.logger = logger
        self.IN1 = Pin(2, Pin.OUT)  # Motor control pin 1
        self.IN2 = Pin(4, Pin.OUT)  # Motor control pin 2

        self.servo = PWM(Pin(21))  # Servo control on GPIO 21
        self.servo.freq(50)
        # Line sensor pins
        self.S1 = Pin(13, Pin.IN)
        self.S2 = Pin(12, Pin.IN)
        self.S3 = Pin(14, Pin.IN)
        self.S4 = Pin(27, Pin.IN)
        self.S5 = Pin(26, Pin.IN)
        self.NEAR = Pin(25, Pin.IN)  # NEAR IR sensor for obstacles

        # Ultrasonic sensors for obstacle avoidance
        self.front_left_trigger = Pin(5, Pin.OUT)
        self.front_left_echo = Pin(15, Pin.IN)
        self.front_right_trigger = Pin(23, Pin.OUT)
        self.front_right_echo = Pin(22, Pin.IN)
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

    def turn_left(self):
        self.logger.info('Turning left (servo to 0°)')
        self.servo.duty_u16(self.SERVO_MIN_DUTY)

    def turn_right(self):
        self.logger.info('Turning right (servo to 180°)')
        self.servo.duty_u16(self.SERVO_MAX_DUTY)

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
        time.sleep_ms(100)  # 100 ms instead of 0.1 seconds
        self.brake_motor()

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, initial_value=0):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.estimate_error = 1

    def update(self, measurement):
        # Prediction
        prediction_error = self.estimate_error + self.process_variance
        # Update
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        return self.estimate

class SensorArray:
    def __init__(self, sensors):
        self.sensors = sensors
        self.kalman_filters = [KalmanFilter(0.1, 1) for _ in range(len(sensors))]
        self.calibration_values = {'black': [], 'white': []}
        self.thresholds = []

    def calibrate(self):
        # Collect calibration data
        for color in ['black', 'white']:
            values = []
            for _ in range(100):  # Take 100 readings
                readings = [sensor.value() for sensor in self.sensors]
                values.append(readings)
                time.sleep_ms(10)
            self.calibration_values[color] = [sum(x)/len(x) for x in zip(*values)]

        # Calculate thresholds
        self.thresholds = [(b + w)/2 for b, w in 
                           zip(self.calibration_values['black'], 
                               self.calibration_values['white'])]

    def read(self):
        raw_values = []
        # Take multiple readings for debouncing
        for _ in range(5):
            readings = [sensor.value() for sensor in self.sensors]
            raw_values.append(readings)
            time.sleep_ms(2)
        # Average the readings
        averaged_values = [sum(col) / len(col) for col in zip(*raw_values)]
        filtered_values = [kf.update(val) for kf, val in
                           zip(self.kalman_filters, averaged_values)]
        return [1 if val > thresh else 0 for val, thresh in
                zip(filtered_values, self.thresholds)]

class Graph:
    def __init__(self, logger):
        self.nodes = {}
        self.edges = {}
        self.logger = logger

    def add_node(self, node, pos):
        self.nodes[node] = pos
        if node not in self.edges:
            self.edges[node] = []

    def add_edge(self, from_node, to_node, weight):
        if from_node in self.nodes and to_node in self.nodes:
            self.edges[from_node].append((to_node, weight))
        else:
            self.logger.error(f"Attempted to add edge with undefined nodes: {from_node}, {to_node}")

    def h(self, node, goal):
        # Manhattan distance heuristic
        x1, y1 = self.nodes[node]
        x2, y2 = self.nodes[goal]
        return abs(x1 - x2) + abs(y1 - y2)

    def a_star(self, start, goal):
        frontier = [(0, start)]
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            current = frontier.pop(0)[1]

            if current == goal:
                break

            for next_node, weight in self.edges.get(current, []):
                new_cost = cost_so_far[current] + weight
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.h(next_node, goal)
                    frontier.append((priority, next_node))
                    frontier.sort()
                    came_from[next_node] = current

        if goal not in came_from:
            self.logger.error("No path found to the goal.")
            return []

        # Reconstruct path
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
            if current is None:
                self.logger.error("Path reconstruction failed.")
                return []
        path.append(start)
        path.reverse()
        return path

class PIDOptimizer:
    def __init__(self):
        self.best_params = {'kp': 0.5, 'ki': 0.1, 'kd': 0.2}
        self.scores = []

    def evaluate_performance(self, error_history):
        # Calculate performance score based on error history
        return -sum(abs(e) for e in error_history)

    def optimize(self, error_history):
        score = self.evaluate_performance(error_history)
        self.scores.append(score)

        # Simple hill climbing
        if len(self.scores) > 1 and score > self.scores[-2]:
            # Randomly adjust parameters
            param = random.choice(['kp', 'ki', 'kd'])
            adjustment = 1 + random.uniform(-0.1, 0.1)
            self.best_params[param] *= adjustment
            self.best_params[param] = max(0, min(self.best_params[param], 10))  # Clamp values

        return self.best_params

class DecisionMemory:
    def __init__(self):
        self.decisions = {}  # {intersection_pattern: (decision, success_count)}
        self.current_decision = None
        self.decision_time = None

    def store_decision(self, pattern, decision):
        self.current_decision = (pattern, decision)
        self.decision_time = time.ticks_ms()

    def evaluate_decision(self, success):
        if self.current_decision:
            pattern, decision = self.current_decision
            if pattern not in self.decisions:
                self.decisions[pattern] = (decision, 1 if success else -1)
            else:
                old_decision, count = self.decisions[pattern]
                if success:
                    self.decisions[pattern] = (decision, count + 1)
                else:
                    new_decision = "right" if old_decision == "left" else "left"
                    self.decisions[pattern] = (new_decision, count - 1)

    def get_decision(self, pattern):
        if pattern in self.decisions:
            return self.decisions[pattern][0]
        return "right"  # Default decision

class ObjectDetector:
    def __init__(self):
        self.current_object = None
        self.object_width = 0
        self.detection_count = 0

    def detect_object(self, near, us_left, us_right, current_position):
        if near == 0:  # Object detected by NEAR sensor
            left_dist = us_left
            right_dist = us_right

            # Calculate object width
            self.object_width = 4 - left_dist - right_dist
            self.current_object = {
                'center': (current_position[0], current_position[1] + 8),  # Relative positioning
                'width': max(0.5, self.object_width),
                'detection_time': time.ticks_ms()
            }
            self.detection_count += 1
            return True
        return False

    def object_fading(self, near, us_left, us_right):
        if self.current_object:
            new_width = 4 - us_left - us_right
            if new_width < self.object_width:
                self.current_object = None
                return True
        return False

class EnhancedLineFollower(Controller):
    def __init__(self, logger):
        super().__init__(logger)

        # Enhanced sensor processing
        self.sensor_array = SensorArray([self.S1, self.S2, self.S3, self.S4, self.S5])

        # Track mapping
        self.graph = Graph(logger)
        self.current_position = (0, 0)
        self.intersections = []

        # PID optimization
        self.pid_optimizer = PIDOptimizer()
        self.error_history = deque(maxlen=100)

        # Motion smoothing
        self.last_steering = self.SERVO_MID_DUTY
        self.steering_smoothing = 0.7

        # Decision memory
        self.decision_memory = DecisionMemory()

        # Object detection
        self.object_detector = ObjectDetector()
        self.avoiding_obstacle = False
        self.return_to_line = False
        self.last_line_position = None

        # Initialize PID parameters
        self.kp = self.pid_optimizer.best_params['kp']
        self.ki = self.pid_optimizer.best_params['ki']
        self.kd = self.pid_optimizer.best_params['kd']

    def system_check(self):
        self.logger.info("Starting system check...")

        # Test motors
        try:
            self.move_forward()
            time.sleep_ms(100)
            self.stop()
            self.logger.info("Motor check: OK")
        except Exception as e:
            self.logger.error(f"Motor check failed: {e}")
            return False

        # Test servo
        try:
            self.turn_left()
            time.sleep_ms(100)
            self.set_servo_to_zero()
            self.turn_right()
            time.sleep_ms(100)
            self.set_servo_to_zero()
            self.logger.info("Servo check: OK")
        except Exception as e:
            self.logger.error(f"Servo check failed: {e}")
            return False

        # Test sensors
        try:
            self.sensor_array.calibrate()
            self.logger.info("Sensor calibration: OK")
        except Exception as e:
            self.logger.error(f"Sensor calibration failed: {e}")
            return False

        return True

    def read_ultrasonic(self, trigger_pin, echo_pin):
        # Send a pulse to trigger pin
        trigger_pin.value(0)
        time.sleep_us(2)
        trigger_pin.value(1)
        time.sleep_us(10)
        trigger_pin.value(0)

        # Measure the duration of echo pin HIGH
        start = time.ticks_us()
        while echo_pin.value() == 0:
            if time.ticks_diff(time.ticks_us(), start) > 30000:
                # Timeout after 30 ms
                return None
        start = time.ticks_us()
        while echo_pin.value() == 1:
            if time.ticks_diff(time.ticks_us(), start) > 30000:
                # Timeout after 30 ms
                return None
        end = time.ticks_us()

        # Calculate distance in cm (speed of sound ~34300 cm/s)
        duration = time.ticks_diff(end, start)
        distance = (duration * 34300) / 1e6 / 2  # Divide by 2 for round trip
        return distance

    def detect_intersection(self, sensors):
        # Define intersection detection logic
        # Example: All sensors detect the line
        return all(sensors)

    def calculate_pid(self, error):
        current_time = time.ticks_ms()
        delta_time = (time.ticks_diff(current_time, self.last_time)) / 1000  # Convert ms to seconds
        self.last_time = current_time

        if delta_time <= 0:
            delta_time = 1e-3  # Prevent division by zero

        # Reset integral if line is lost
        if error == 0:
            self.integral = 0
        else:
            self.integral += error * delta_time
            # Clamp the integral term
            integral_limit = 100
            self.integral = max(min(self.integral, integral_limit), -integral_limit)

        derivative = (error - self.previous_error) / delta_time
        self.previous_error = error

        adjustment = self.kp * error + self.ki * self.integral + self.kd * derivative
        return adjustment

    def smooth_turn(self, adjustment):
        # Clamp the adjustment to the servo's range
        max_adjustment = self.SERVO_MAX_DUTY - self.SERVO_MID_DUTY
        adjustment = max(min(adjustment, max_adjustment), -max_adjustment)
    
        target_angle = self.SERVO_MID_DUTY + int(adjustment)
        # Clamp the target_angle within servo limits
        target_angle = max(self.SERVO_MIN_DUTY, min(target_angle, self.SERVO_MAX_DUTY))
        # Smooth the turn
        new_angle = (target_angle * (1 - self.steering_smoothing) +
                     self.last_steering * self.steering_smoothing)
        self.servo.duty_u16(int(new_angle))
        self.last_steering = new_angle

    def update_position_forward(self):
        x, y = self.current_position
        self.current_position = (x, y + 1)

    def update_position_backward(self):
        x, y = self.current_position
        self.current_position = (x, y - 1)

    def update_map(self, sensors):
        if self.detect_intersection(sensors):
            self.intersections.append(self.current_position)
            node_id = len(self.intersections) - 1
            self.graph.add_node(node_id, self.current_position)
            # Connect nodes if there are at least two
            if node_id > 0:
                self.graph.add_edge(node_id - 1, node_id, weight=1)  # Example weight
                self.graph.add_edge(node_id, node_id - 1, weight=1)  # Bidirectional

    def optimize_pid(self):
        params = self.pid_optimizer.optimize(list(self.error_history))
        self.kp = params['kp']
        self.ki = params['ki']
        self.kd = params['kd']
        self.logger.info(f"Optimized PID parameters: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")

    def avoid_obstacle(self):
        if not self.avoiding_obstacle:
            self.last_line_position = self.current_position
            self.avoiding_obstacle = True

        left_dist = self.read_ultrasonic(self.front_left_trigger, self.front_left_echo)
        right_dist = self.read_ultrasonic(self.front_right_trigger, self.front_right_echo)

        if left_dist is None or right_dist is None:
            self.logger.error("Ultrasonic sensor timeout.")
            return

        if self.object_detector.detect_object(self.NEAR.value(), left_dist, right_dist, self.current_position):
            # Choose direction based on more space
            if left_dist > right_dist:
                self.turn_left()
            else:
                self.turn_right()
            self.move_forward()
            self.update_position_forward()
        elif self.object_detector.object_fading(self.NEAR.value(), left_dist, right_dist):
            self.return_to_line = True
            self.avoiding_obstacle = False

        if self.return_to_line:
            if self.find_line():
                self.return_to_line = False

    def find_line(self):
        sensors = self.sensor_array.read()
        if any(sensors):  # If any sensor detects line
            # Calculate error for PID
            error = sum([(i - 2) * val for i, val in enumerate(sensors)])
            self.error_history.append(error)
            adjustment = self.calculate_pid(error)
            self.smooth_turn(adjustment)
            self.move_forward()

            if abs(error) < 0.1:  # Centered
                return True
        else:
            # If no line is detected, search pattern
            self.turn_left()
            self.move_forward()
        return False

    def follow_line(self):
        if not self.system_check():
            self.logger.error("System check failed. Aborting.")
            return

        try:
            while True:
                sensors = self.sensor_array.read()

                if self.avoiding_obstacle or self.return_to_line:
                    self.avoid_obstacle()
                    time.sleep_ms(10)
                    continue

                if self.NEAR.value() == 0:
                    self.logger.info('Obstacle detected! Starting avoidance.')
                    self.avoid_obstacle()
                    time.sleep_ms(10)
                    continue

                # Normal line following
                error = sum([(i - 2) * val for i, val in enumerate(sensors)])
                self.error_history.append(error)

                adjustment = self.calculate_pid(error)
                self.smooth_turn(adjustment)

                if self.detect_intersection(sensors):
                    pattern = tuple(sensors)
                    decision = self.decision_memory.get_decision(pattern)
                    if decision == "left":
                        self.turn_left()
                    else:
                        self.turn_right()
                    self.decision_memory.store_decision(pattern, decision)
                    self.logger.info(f"Intersection detected. Decision: {decision}")

                # Evaluate previous decision after 5 seconds
                if self.decision_memory.current_decision:
                    if time.ticks_diff(time.ticks_ms(), 
                                       self.decision_memory.decision_time) > 5000:
                        success = not self.avoiding_obstacle and not self.return_to_line
                        self.decision_memory.evaluate_decision(success)
                        self.logger.info(f"Decision evaluation: {'Success' if success else 'Failure'}")

                self.move_forward()
                self.update_position_forward()

                # Periodically optimize PID
                if len(self.error_history) >= 100:
                    self.optimize_pid()

                time.sleep_ms(10)
        except Exception as e:
            self.logger.error(f"Error during follow_line: {e}")
            self.stop_and_brake()
            self.shutdown()

    def follow_path(self, path):
        for node in path:
            if node not in self.graph.nodes:
                self.logger.error(f"Node {node} does not exist in the graph.")
                continue

            target_pos = self.graph.nodes[node]
            while self.current_position != target_pos:
                dx = target_pos[0] - self.current_position[0]
                dy = target_pos[1] - self.current_position[1]
                angle = math.atan2(dy, dx)

                # Convert angle to servo duty adjustment
                adjustment = int((angle / math.pi) * (self.SERVO_MAX_DUTY - self.SERVO_MID_DUTY))
                self.smooth_turn(adjustment)
                self.move_forward()
                self.update_position_forward()
                time.sleep_ms(10)

    def shutdown(self):
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
        self.buffer_limit = 10  # Adjust as needed

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
            # In a real scenario, consider how to handle logging failures
            pass

    def error(self, message):
        self.log('ERROR', message)

    def debug(self, message):
        self.log('DEBUG', message)

    def info(self, message):
        self.log('INFO', message)

    def __del__(self):
        # Ensure all logs are flushed on deletion
        self.flush()

# Initialize Logger and EnhancedLineFollower
logger = Logger('car_log.txt')
car = EnhancedLineFollower(logger)
logger.info('Starting enhanced line following')

try:
    car.follow_line()
except KeyboardInterrupt:
    car.shutdown()
except Exception as e:
    logger.error(f"Unhandled exception: {e}")
    car.shutdown()