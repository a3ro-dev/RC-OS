html = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Remote Control</title>
    <style>
        body {
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            margin: 0;
            background-color: #1a1a1a;
            color: #fff;
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        }

        .dashboard {
            display: flex;
            flex-direction: column;
            align-items: center;
            background-color: #2c2c2c;
            padding: 20px;
            border-radius: 15px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.5);
        }

        .button-container {
            display: grid;
            grid-template-areas:
                ". forward ."
                "left stop right"
                ". backward .";
            gap: 15px;
            margin-bottom: 30px;
        }

        button {
            padding: 15px 30px;
            font-size: 18px;
            border: none;
            border-radius: 10px;
            color: white;
            background-color: #4CAF50;
            cursor: pointer;
            box-shadow: 0 5px 15px rgba(0, 0, 0, 0.2);
            transition: all 0.3s ease;
        }

        button:hover {
            background-color: #45a049;
        }

        button:active {
            transform: scale(0.95);
            box-shadow: 0 3px 10px rgba(0, 0, 0, 0.2);
        }

        #forward {
            grid-area: forward;
        }

        #backward {
            grid-area: backward;
        }

        #left {
            grid-area: left;
        }

        #right {
            grid-area: right;
        }

        #stop {
            grid-area: stop;
            background-color: #e74c3c;
        }

        .stunts-container {
            display: flex;
            justify-content: center;
            gap: 15px;
        }

        .stunts-container button {
            background-color: #3498db;
        }

        .stunts-container button:hover {
            background-color: #2980b9;
        }

        .stunts-container button:active {
            transform: scale(0.95);
            box-shadow: 0 3px 10px rgba(0, 0, 0, 0.2);
        }

        h2 {
            margin: 20px 0;
            font-size: 24px;
            color: #f39c12;
        }
    </style>
</head>
<body>
    <div class="dashboard">
        <div class="button-container">
            <button id="forward" onmousedown="buttonDown('forward')" onmouseup="buttonUp()">Forward</button>
            <button id="left" onmousedown="buttonDown('left')" onmouseup="buttonUp()">Left</button>
            <button id="stop" onmousedown="buttonDown('stop')" onmouseup="buttonUp()">Stop</button>
            <button id="right" onmousedown="buttonDown('right')" onmouseup="buttonUp()">Right</button>
            <button id="backward" onmousedown="buttonDown('backward')" onmouseup="buttonUp()">Backward</button>
        </div>
        <h2>Stunts 🚗💨</h2>
        <div class="stunts-container">
            <button id="s-drift" onmousedown="buttonDown('s-drift')" onmouseup="buttonUp()">🌀 S-Drift</button>
            <button id="donut" onmousedown="buttonDown('donut')" onmouseup="buttonUp()">🍩 Donut</button>
            <button id="reverse-donut" onmousedown="buttonDown('reverse-donut')" onmouseup="buttonUp()">↩️ Reverse Donut</button>
            <button id="normal-drift" onmousedown="buttonDown('normal-drift')" onmouseup="buttonUp()">⏱️ Normal Drift</button>
            <button id="line-following" onclick="toggleLineFollowing()">🚗 Line Following</button>
        </div>
    </div>
    <script>
        function buttonDown(direction) {
            fetch(`/${direction}`, { method: 'POST' });
        }

        function buttonUp() {
            fetch('/stop', { method: 'POST' });
        }

        function toggleLineFollowing() {
            const button = document.getElementById('line-following');
            if (button.textContent.includes('LF-On')) {
                button.textContent = '🚗 Line Following';
                fetch('/LF-Off', { method: 'POST' });
            } else {
                button.textContent = '🚗 Line Following (LF-On)';
                fetch('/LF-On', { method: 'POST' });
            }
        }

        window.addEventListener('keydown', function(event) {
            switch(event.key) {
                case 'w':
                case 'W':
                    buttonDown('forward');
                    break;
                case 's':
                case 'S':
                    buttonDown('backward');
                    break;
                case 'a':
                case 'A':
                    buttonDown('left');
                    break;
                case 'd':
                case 'D':
                    buttonDown('right');
                    break;
            }
        });

        window.addEventListener('keyup', function(event) {
            switch(event.key) {
                case 'w':
                case 'W':
                case 's':
                case 'S':
                case 'a':
                case 'A':
                case 'd':
                case 'D':
                    buttonUp();
                    break;
            }
        });
    </script>
</body>
</html>
"""

import socket
from time import sleep
from machine import Pin, PWM, time_pulse_us  # MicroPython hardware control
import network  # Network control for WiFi
import time  # Time functions
from neopixel import NeoPixel  # Control for WS2812 LED strip

# Pin setup for NeoPixel (WS2812) LED ring
LED_PIN = 13  # GPIO pin for the WS2812 LEDs
NUM_Px = 8  # Number of pixels on the NeoPixel LED strip
led_ring = NeoPixel(Pin(LED_PIN), NUM_Px)

# WiFi setup
WLAN = network.WLAN(network.STA_IF)  # Station interface (client mode)

# Logging levels
LOG_LEVELS = {
    'err': 'ERROR',
    'deb': 'DEBUG',
    'inf': 'INFO'
}

def log(level, msg):
    """Logs a message with a specified log level."""
    log_type = LOG_LEVELS.get(level, 'UNKNOWN')
    print(f"{log_type} - [{msg}]")

# WiFi connection logic
def connect_to_wifi(ssid, password):
    """Connects to the specified WiFi network using static IP settings."""
    WLAN.active(True)
    if not WLAN.isconnected():
        print('Connecting to network...')
        WLAN.ifconfig(('192.168.1.69', '255.255.255.0', '192.168.1.1', '8.8.8.8'))  # Static IP configuration
        WLAN.connect(ssid, password)
        
        # Wait for connection or timeout after 20 seconds
        start_time = time.time()
        while not WLAN.isconnected():
            elapsed_time = time.time() - start_time
            if elapsed_time > 20:
                print('Failed to connect to WiFi. Starting hotspot...')
                start_hotspot()
                return
            
            loading_animation(elapsed_time)  # Show loading animation while connecting
            time.sleep(0.1)
        
        led_ring.fill((0, 255, 0))  # All LEDs turn green when connected
        led_ring.write()

    print('Network config:', WLAN.ifconfig())

# LED loading animation
def loading_animation(elapsed_time):
    """Displays a rotating LED animation during the WiFi connection process."""
    num_leds = len(led_ring)
    position = int((elapsed_time * 10) % num_leds)
    for i in range(num_leds):
        # Adjust brightness based on distance from the current position
        distance = (i - position) % num_leds
        brightness = max(0, 255 - (distance * 50))  # Fading effect for LEDs
        led_ring[i] = (0, 0, brightness) if brightness > 0 else (0, 0, 0)
    
    led_ring.write()

# Hotspot creation logic
def start_hotspot():
    """Starts an access point (hotspot) with the given SSID and password."""
    ap = network.WLAN(network.AP_IF)  # Access point interface (AP mode)
    ap.active(True)
    ap.config(essid='RC-OSv2.7', password='GodComplex', max_clients=1)
    ap.ifconfig(('192.168.1.69', '255.255.255.0', '192.168.1.1', '8.8.8.8'))  # Static IP configuration for AP
    
    # Set LEDs to red to indicate hotspot mode
    led_ring.fill((255, 0, 0))
    led_ring.write()

    print('Hotspot started with SSID: RC-OSv2.7 and password: GodComplex')

# Define pins for the ultrasonic sensor
TRIGGER_PIN = 5  # GPIO pin for trigger
ECHO_PIN = 18    # GPIO pin for echo

# Setup pins
trigger = Pin(TRIGGER_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)

# Constants for speed calculation
DISTANCE_TO_GROUND_CM = 5  # Measured distance from the sensor to ground when stationary
CM_TO_MS_CONVERSION = 0.01  # Convert cm/s to m/s (or adjust for other units)
FILTER_WINDOW = 5  # Moving average window size

# Time and history tracking
distance_history = []
speed_history = []
last_time = time.ticks_ms()

# Kalman filter class for noise reduction
class KalmanFilter:
    """Kalman filter for reducing noise in distance measurements."""
    def __init__(self, process_variance, measurement_variance, initial_value=0):
        self.process_variance = process_variance  # Process (movement) variance
        self.measurement_variance = measurement_variance  # Sensor noise variance
        self.value = initial_value
        self.estimate_error = 1.0
        self.kalman_gain = 0.0

    def update(self, measurement):
        """Update the Kalman estimate with the new measurement."""
        # Update Kalman gain
        self.kalman_gain = self.estimate_error / (self.estimate_error + self.measurement_variance)

        # Update estimate based on the measurement
        self.value += self.kalman_gain * (measurement - self.value)

        # Update estimate error
        self.estimate_error = (1 - self.kalman_gain) * self.estimate_error + abs(self.value - measurement) * self.process_variance

        return self.value

# Exponential moving average filter for smoothing
class ExpSmoothingFilter:
    """Exponential smoothing filter to further reduce noise in sensor data."""
    def __init__(self, alpha=0.1, initial_value=0):
        self.alpha = alpha
        self.value = initial_value

    def update(self, new_value):
        """Update the filtered value using exponential smoothing."""
        self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value

# Initialize the filters
kf = KalmanFilter(process_variance=1e-5, measurement_variance=0.1)
exp_filter = ExpSmoothingFilter(alpha=0.2)

# Function to read ultrasonic distance
def read_distance():
    """Reads distance from the ultrasonic sensor in centimeters."""
    # Send a 10µs pulse to trigger
    trigger.value(0)
    time.sleep_us(2)
    trigger.value(1)
    time.sleep_us(10)
    trigger.value(0)

    # Measure the time for the echo
    pulse_duration = time_pulse_us(echo, 1, 30000)  # Timeout after 30ms
    if pulse_duration > 0:
        # Distance calculation based on the speed of sound
        distance_cm = (pulse_duration / 2) * 0.0343
        return distance_cm
    else:
        return None  # Return None if no valid echo

# Moving average filter
def moving_average(data, window_size):
    """Calculates the moving average for the given data and window size."""
    if len(data) < window_size:
        return sum(data) / len(data)
    else:
        return sum(data[-window_size:]) / window_size

# Outlier detection function
def reject_outliers(data, m=2):
    """Removes outliers from the data based on the specified threshold."""
    mean = sum(data) / len(data)
    std_dev = (sum([(x - mean) ** 2 for x in data]) / len(data)) ** 0.5
    return [x for x in data if abs(x - mean) < m * std_dev]

# Main loop to calculate speed
def calculate_speed():
    """Main loop for calculating speed based on distance changes."""
    global last_time
    while True:
        current_time = time.ticks_ms()
        time_delta = time.ticks_diff(current_time, last_time) / 1000  # Time difference in seconds
        last_time = current_time

        # Read distance from the sensor
        distance = read_distance()
        if distance is not None:
            # Apply Kalman filter to the distance reading
            filtered_distance = kf.update(distance)

            # Further smooth the data using exponential smoothing
            smooth_distance = exp_filter.update(filtered_distance)

            # Store distance for speed calculation and filtering
            distance_history.append(smooth_distance)

            # Apply moving average filter to smooth the distance readings
            avg_distance = moving_average(distance_history, FILTER_WINDOW)

            # Remove outliers from distance history for better accuracy (periodically)
            if len(distance_history) % FILTER_WINDOW == 0:
                cleaned_data = reject_outliers(distance_history)

                # Calculate speed based on the change in distance
                if len(cleaned_data) > 1:
                    delta_distance = cleaned_data[-1] - cleaned_data[-2]
                    if abs(delta_distance) > 0.1:  # Ignore small noise variations
                        speed = abs(delta_distance / time_delta)  # Speed in cm/s
                        speed_history.append(speed)

                        # Apply moving average to speed to reduce noise
                        avg_speed = moving_average(speed_history, FILTER_WINDOW)

                        # Convert cm/s to m/s
                        speed_m_s = avg_speed * CM_TO_MS_CONVERSION

                        # Return the filtered speed in m/s
                        return speed_m_s

        # Adjust delay based on the speed
        time.sleep(0.05 if len(speed_history) > 0 and speed_history[-1] > 0 else 0.1)

class Controller:
    # Servo duty cycle values for different angles
    SERVO_MIN_DUTY = int(1000 / 20000 * 65535)  # 0 degrees
    SERVO_MAX_DUTY = int(2000 / 20000 * 65535)  # 180 degrees
    SERVO_MID_DUTY = int(1500 / 20000 * 65535)  # 90 degrees (Neutral)

    def __init__(self):
        # Initialize motor control pins
        self.IN1 = Pin(2, Pin.OUT)  # Motor control pin 1
        self.IN2 = Pin(4, Pin.OUT)  # Motor control pin 2
        
        # Initialize servo motor
        self.servo = PWM(Pin(21))  # GPIO pin for servo control
        self.servo.freq(50)  # Set servo frequency to 50Hz
        self.set_servo_to_zero()  # Initialize servo to 0 degrees

    def set_servo_to_zero(self):
        """Sets the servo to 0 degrees."""
        print('Setting servo to 0 degrees')
        self.servo.duty_u16(self.SERVO_MIN_DUTY)

    def move_forward(self):
        """Activates motors to move forward."""
        print('Moving forward')
        self.IN1.high()
        self.IN2.low()

    def move_backward(self):
        """Activates motors to move backward."""
        print('Moving backward')
        self.IN1.low()
        self.IN2.high()

    def turn_left(self):
        """Turns the servo to the left (0 degrees)."""
        print('Turning left')
        self.servo.duty_u16(self.SERVO_MIN_DUTY)

    def turn_right(self):
        """Turns the servo to the right (180 degrees)."""
        print('Turning right')
        self.servo.duty_u16(self.SERVO_MAX_DUTY)

    def stop(self):
        """Stops the motors and centers the servo (90 degrees)."""
        print('Stopping')
        self.IN1.low()
        self.IN2.low()  # Stop both motors
        self.servo.duty_u16(self.SERVO_MID_DUTY)  # Center the servo

    def brake_motor(self):
        """Applies braking by energizing both motor pins."""
        print('Braking motor')
        self.IN1.high()
        self.IN2.high()  # Apply braking to the motor

    def stop_and_brake(self):
        """Stops the motor and then applies braking."""
        print('Stopping and braking')
        self.stop()  # Stop the motor first
        time.sleep(0.1)  # Delay to allow the motor to slow down
        self.brake_motor()  # Apply braking
class Stunts(Controller):

    def perform_donut(self):
        print('Performing donut')
        self.set_servo_to_zero()  # Set the steering to the center position
        time.sleep(0.5)  # Wait for stability

        self.move_forward()  # Begin driving forward
        time.sleep(0.5)  # Wait for stability

        # Initiate the donut maneuver
        self.turn_left()  # Turn the steering to the left
        time.sleep(0.5)  # Adjust as needed for timing

        # Maintain the donut for 5-8 seconds
        start_time = time.time()
        while time.time() - start_time < 6:  # Keep going for 6 seconds
            self.servo.duty_u16(int(1200 / 20000 * 65535))  # Adjust steering angle
            time.sleep(0.1)  # Adjust as needed for timing

        # End the donut
        self.set_servo_to_zero()  # Return the steering to the center position
        self.stop()  # Stop the car

    def perform_s_drift(self):
        print('Performing S-drift')
        self.set_servo_to_zero()  # Set the steering to the center position
        time.sleep(0.5)  # Wait for stability

        self.move_forward()  # Begin driving forward
        time.sleep(0.5)  # Wait for stability

        # Perform the S-drift for 5-8 seconds
        start_time = time.time()
        while time.time() - start_time < 6:  # Keep going for 6 seconds
            self.turn_left()  # Turn the steering to the left
            time.sleep(0.5)  # Adjust as needed for timing
            self.turn_right()  # Turn the steering to the right
            time.sleep(0.5)  # Adjust as needed for timing

        # End the S-drift
        self.set_servo_to_zero()  # Return the steering to the center position
        self.stop()  # Stop the car

class LineFollower(Controller):
    def __init__(self):
        super().__init__()
        # Initialize line sensors
        self.S1 = Pin(13, Pin.IN)
        self.S2 = Pin(12, Pin.IN)
        self.S3 = Pin(14, Pin.IN)
        self.S4 = Pin(27, Pin.IN)
        self.S5 = Pin(26, Pin.IN)
        self.NEAR = Pin(33, Pin.IN)  # NEAR IR sensor

        # Ultrasonic sensor pins
        self.front_left_trigger = Pin(15, Pin.OUT)
        self.front_left_echo = Pin(4, Pin.IN)
        self.front_right_trigger = Pin(23, Pin.OUT)
        self.front_right_echo = Pin(22, Pin.IN)

        # PID control parameters
        self.kp = 0.5  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.2  # Derivative gain

        # PID state
        self.previous_error = 0
        self.integral = 0

        # Speed control
        self.base_speed = 32768  # Base speed for motors (50% of max PWM)

        # Speedometer for measuring speed (if needed)
        self.speedometer = Speedometer(encoder_pin=25)

    def read_sensors(self):
        """Reads the values of the line sensors."""
        return [self.S1.value(), self.S2.value(), self.S3.value(), self.S4.value(), self.S5.value()]

    def calculate_pid(self, error):
        """Calculates the PID control value."""
        self.integral += error
        derivative = error - self.previous_error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output

    def adjust_speed(self, error):
        """Adjusts motor speeds based on the PID error."""
        adjustment = self.calculate_pid(error)
        left_speed = self.base_speed + adjustment
        right_speed = self.base_speed - adjustment
    
        # Steering logic based on speed differences
        if left_speed > right_speed:
            self.turn_left()
        elif right_speed > left_speed:
            self.turn_right()
        else:
            self.set_servo_to_zero()
    
        # Move forward at the base speed
        self.move_forward()
        
    def read_ultrasonic(self, trigger_pin, echo_pin):
        """Reads distance from the ultrasonic sensor."""
        trigger_pin.low()
        time.sleep_us(2)
        trigger_pin.high()
        time.sleep_us(10)
        trigger_pin.low()
        pulse_time = time_pulse_us(echo_pin, 1)
        distance = (pulse_time * 0.0343) / 2  # Speed of sound is 343 m/s
        return distance

    def avoid_obstacle(self):
        """Detects and avoids obstacles using ultrasonic sensors."""
        left_distance = self.read_ultrasonic(self.front_left_trigger, self.front_left_echo)
        right_distance = self.read_ultrasonic(self.front_right_trigger, self.front_right_echo)

        if left_distance < 30 and right_distance >= 30:
            print('Obstacle on left! Turning right.')
            self.turn_right()
            time.sleep(0.5)
        elif right_distance < 30 and left_distance >= 30:
            print('Obstacle on right! Turning left.')
            self.turn_left()
            time.sleep(0.5)
        else:
            print('Obstacle ahead! Stopping.')
            self.stop()

    def follow_line(self):
        """Main loop for line-following behavior."""
        while True:
            sensors = self.read_sensors()

            # Obstacle detection with NEAR IR sensor
            if self.NEAR.value() == 0:
                print('Obstacle detected! Stopping.')
                self.stop_and_brake()
                self.avoid_obstacle()
                continue

            # Calculate error based on sensor readings
            error = self.calculate_error(sensors)
            if error is None:
                print('Lost the line! Stopping.')
                self.stop_and_brake()
                time.sleep(0.5)  # Pause before retrying
                continue

            # Adjust speed based on PID controller output
            self.adjust_speed(error)
            time.sleep(0.1)  # Delay to prevent overwhelming the system

    def calculate_error(self, sensors):
        """Calculates the error based on the sensor readings."""
        # Map sensor states to error values
        if sensors == [1, 0, 0, 0, 0]:
            return -2
        elif sensors == [1, 1, 0, 0, 0]:
            return -1
        elif sensors == [0, 1, 0, 0, 0]:
            return 0
        elif sensors == [0, 1, 1, 0, 0]:
            return 1
        elif sensors == [0, 0, 1, 1, 0]:
            return 2
        elif sensors == [0, 0, 1, 0, 0]:
            return 0
        else:
            return None  # Indicates the robot has lost the line

    def stop(self):
        """Stops the robot by calling the parent stop method."""
        super().stop()

def handle_request(request):
    status_code = '200 OK'
    message = ''
    lines = request.split('\n')
    first_line = lines[0].split(' ')
    if len(first_line) >= 3:
        method, path, _ = first_line
    else:
        method, path = first_line[0], ''

    if method == 'POST':
        try:
            if path == '/forward':
                controller.move_forward()
                message = 'Moving forward'
            elif path == '/backward':
                controller.move_backward()
                message = 'Moving backward'
            elif path == '/left':
                controller.turn_left()
                message = 'Turning left'
            elif path == '/right':
                controller.turn_right()
                message = 'Turning right'
            elif path == '/stop':
                controller.stop()
                message = 'Stopped'
            elif path == '/s-drift':
                stunts.perform_s_drift()
                message = 'Performing S-Drift'
            elif path == '/donut':
                stunts.perform_donut()
                message = 'Performing Donut'
            elif path == '/LF-On':  # Start line following feature
                line_follower.follow_line()
                message = 'Line following mode enabled'
            elif path == '/LF-Off':  # Stop line following feature
                line_follower.stop()
                message = 'Line following mode disabled'
        except ValueError as e:
            status_code = '400 Bad Request'
            message = f'Value Error: {str(e)}'
        except Exception as e:
            status_code = '500 Internal Server Error'
            message = f'Error: {str(e)}'
    return f'HTTP/1.1 {status_code}\n\n{message}\n{html}'


controller = Controller()
stunts = Stunts()
line_follower = LineFollower()

s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('0.0.0.0', 8000))
connect_to_wifi('vijayprakashsinghkushwaha2.4ghz', '9696949718')
s.listen(1)  # Listen for connections
print("Server Up and Running")
ip_address = WLAN.ifconfig()[0]
print('IP:', ip_address)
import _thread

def handle_connection(conn, addr):
    request = conn.recv(1024).decode('utf-8')
    response = handle_request(request)
    conn.send(response.encode('utf-8'))
    conn.close()

while True:
    conn, addr = s.accept()
    _thread.start_new_thread(handle_connection, (conn, addr))  # Use threading for non-blocking
    time.sleep(0.5)