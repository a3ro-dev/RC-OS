import time
from machine import Pin, PWM  #type: ignore
import math
import random
from collections import deque

class Controller:
    SERVO_MIN_DUTY = int(1000 / 20000 * 65535)  # 0 degrees
    SERVO_MAX_DUTY = int(2000 / 20000 * 65535)  # 180 degrees
    SERVO_MID_DUTY = int(1500 / 20000 * 65535)  # 90 degrees

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
        self.NEAR = Pin(33, Pin.IN)  # NEAR IR sensor for obstacles

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
        time.sleep(0.1)
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
        raw_values = [sensor.value() for sensor in self.sensors]
        filtered_values = [kf.update(val) for kf, val in 
                         zip(self.kalman_filters, raw_values)]
        return [1 if val > thresh else 0 for val, thresh in 
                zip(filtered_values, self.thresholds)]

class Graph:
    def __init__(self):
        self.nodes = {}
        self.edges = {}
        
    def add_node(self, node, pos):
        self.nodes[node] = pos
        if node not in self.edges:
            self.edges[node] = []
            
    def add_edge(self, from_node, to_node, weight):
        self.edges[from_node].append((to_node, weight))
        
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
                
            for next_node, weight in self.edges[current]:
                new_cost = cost_so_far[current] + weight
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.h(next_node, goal)
                    frontier.append((priority, next_node))
                    frontier.sort()
                    came_from[next_node] = current
        
        # Reconstruct path
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
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
            self.best_params[param] *= (1 + random.uniform(-0.1, 0.1))
            
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
                    self.decisions[pattern] = (
                        "right" if old_decision == "left" else "left", 
                        -1
                    )
                    
    def get_decision(self, pattern):
        if pattern in self.decisions:
            return self.decisions[pattern][0]
        return "right"  # Default decision

class ObjectDetector:
    def __init__(self):
        self.current_object = None
        self.object_width = 0
        self.detection_count = 0
        
    def detect_object(self, near, us_left, us_right):
        if near == 0:  # Object detected by NEAR sensor
            left_dist = us_left
            right_dist = us_right
            
            # Calculate object width
            self.object_width = 4 - left_dist - right_dist
            self.current_object = {
                'center': (0, 8),
                'width': max(0.5, self.object_width),
                'detection_time': time.ticks_ms()
            }
            self.detection_count += 1
            return True
        return False
        
    def object_fading(self, near, us_left, us_right):
        if self.current_object:
            new_width = 4 - us_left - us_right
            return new_width < self.object_width
        return False

class EnhancedLineFollower(Controller):
    def __init__(self, logger):
        super().__init__(logger)
        
        # Enhanced sensor processing
        self.sensor_array = SensorArray([self.S1, self.S2, self.S3, self.S4, self.S5])
        
        # Track mapping
        self.graph = Graph()
        self.current_position = (0, 0)
        self.intersections = []
        
        # PID optimization
        self.pid_optimizer = PIDOptimizer()
        self.error_history = []
        
        # Motion smoothing
        self.last_steering = 0
        self.steering_smoothing = 0.7
        
        # Decision memory
        self.decision_memory = DecisionMemory()
        
        # Object detection
        self.object_detector = ObjectDetector()
        self.avoiding_obstacle = False
        self.return_to_line = False
        self.last_line_position = None
        
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
        
    def smooth_turn(self, target_angle):
        current_angle = self.last_steering
        new_angle = (target_angle * (1 - self.steering_smoothing) + 
                    current_angle * self.steering_smoothing)
        self.servo.duty_u16(int(new_angle))
        self.last_steering = new_angle
        
    def update_map(self, sensors):
        if self.detect_intersection(sensors):
            self.intersections.append(self.current_position)
            # Add node to graph
            self.graph.add_node(len(self.intersections) - 1, self.current_position)
            
    def optimize_pid(self):
        params = self.pid_optimizer.optimize(self.error_history)
        self.kp = params['kp']
        self.ki = params['ki']
        self.kd = params['kd']
        
    def avoid_obstacle(self):
        if not self.avoiding_obstacle:
            self.last_line_position = self.current_position
            self.avoiding_obstacle = True
            
        left_dist = self.read_ultrasonic(self.front_left_trigger, self.front_left_echo)
        right_dist = self.read_ultrasonic(self.front_right_trigger, self.front_right_echo)
        
        if self.object_detector.detect_object(self.NEAR.value(), left_dist, right_dist):
            # Choose direction based on more space
            if left_dist > right_dist:
                self.turn_left()
            else:
                self.turn_right()
            self.move_forward()
            
        elif self.object_detector.object_fading(self.NEAR.value(), left_dist, right_dist):
            self.return_to_line = True
            self.avoiding_obstacle = False
            
        if self.return_to_line:
            if self.find_line():
                self.return_to_line = False
                
    def find_line(self):
        sensors = self.sensor_array.read()
        if any(sensors):  # If any sensor detects line
            # Center on line
            error = sum([(i - 2) * val for i, val in enumerate(sensors)])
            if abs(error) < 0.1:  # Centered
                return True
            elif error < 0:
                self.turn_left()
            else:
                self.turn_right()
        return False
        
    def follow_line(self):
        if not self.system_check():
            self.logger.error("System check failed. Aborting.")
            return
            
        while True:
            sensors = self.sensor_array.read()
            
            if self.avoiding_obstacle or self.return_to_line:
                self.avoid_obstacle()
                continue
                
            if self.NEAR.value() == 0:
                self.logger.info('Obstacle detected! Starting avoidance.')
                self.avoid_obstacle()
                continue
                
            # Normal line following
            error = sum([(i - 2) * val for i, val in enumerate(sensors)])
            self.error_history.append(error)
            
            if self.detect_intersection(sensors):
                pattern = tuple(sensors)
                decision = self.decision_memory.get_decision(pattern)
                if decision == "left":
                    self.turn_left()
                else:
                    self.turn_right()
                self.decision_memory.store_decision(pattern, decision)
                
            # Evaluate previous decision after 5 seconds
            if self.decision_memory.current_decision:
                if time.ticks_diff(time.ticks_ms(), 
                                 self.decision_memory.decision_time) > 5000:
                    self.decision_memory.evaluate_decision(
                        not self.avoiding_obstacle and not self.return_to_line
                    )
                    
            self.move_forward()
            time.sleep_ms(10)
            
    def follow_path(self, path):
        for node in path:
            target_pos = self.graph.nodes[node]
            while self.current_position != target_pos:
                # Calculate required turn
                dx = target_pos[0] - self.current_position[0]
                dy = target_pos[1] - self.current_position[1]
                angle = math.atan2(dy, dx)
                
                # Smooth turn to target
                self.smooth_turn(self.SERVO_MID_DUTY + angle * 
                               (self.SERVO_MAX_DUTY - self.SERVO_MID_DUTY) / math.pi)
                self.move_forward()
                time.sleep_ms(10)

class Logger:
    def __init__(self, filename):
        self.filename = filename

    def log(self, level, message):
        with open(self.filename, 'a') as log_file:
            log_file.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {level} - {message}\n")

    def error(self, message):
        self.log('ERROR', message)

    def debug(self, message):
        self.log('DEBUG', message)

    def info(self, message):
        self.log('INFO', message)

logger = Logger('car_log.txt')
car = EnhancedLineFollower(logger)
logger.info('Starting enhanced line following')
car.follow_line()