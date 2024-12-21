from machine import Pin, PWM, Timer
import socket
import ujson  # Use ujson consistently, not json
import gc
import time
import network

class Controller:
    def __init__(self):
        try:
            # Motor control pins
            self.IN1 = Pin(15, Pin.OUT)
            self.IN2 = Pin(2, Pin.OUT)
            self.ENA = PWM(Pin(4))  # Enable A for PWM speed control
            
            # Servo setup
            self.servo = PWM(Pin(18))
            
            # Configure PWM
            self.ENA.freq(1000)  # 1KHz frequency for motor
            self.servo.freq(50)  # 50Hz for servo
            
            # Initialize settings
            self._speed = 0
            self._direction = 'stop'
            self._steering_angle = 90  # Center position
            
            # Set initial states
            self.stop()
            self.set_servo_angle(90)
            
            print("Controller initialized successfully")
            
            # ML-related attributes
            self.performance_history = []
            self.learning_rate = 0.1
            self.drift_params = {
                'angle': 60,
                'duration': 2,
                'speed': 100
            }
            
            # Load previous learning data if available
            try:
                self.load_learning_data()
            except:
                print("No previous learning data found")
                
            self.successful_maneuvers = 0
            self.total_maneuvers = 0
            
        except Exception as e:
            print(f"Error initializing controller: {e}")
            raise

        # Add system monitoring
        self.last_error = None
        self.request_count = 0
        self.last_request_time = 0
        self.rate_limit = 50  # requests per second
        self.system_status = {
            'healthy': True,
            'last_error': None,
            'memory_free': 0,
            'uptime': 0
        }
        
        # Start system monitoring
        self.monitor_timer = Timer(0)
        self.monitor_timer.init(period=5000, mode=Timer.PERIODIC, callback=self.update_system_status)
        
        # Wi-Fi connection setup
        self.connect_wifi("vijayprakashsingh", "9696949718")
        
        # Start server with error handling
        try:
            self.start_server()
        except Exception as e:
            self.system_status['healthy'] = False
            self.system_status['last_error'] = str(e)
            raise

    def connect_wifi(self, ssid, password):
        """Connect to WiFi network"""
        try:
            sta_if = network.WLAN(network.STA_IF)
            if not sta_if.active():
                sta_if.active(True)
            if not sta_if.isconnected():
                print('Connecting to WiFi...')
                sta_if.connect(ssid, password)
                while not sta_if.isconnected():
                    time.sleep(1)
            print('WiFi connected:', sta_if.ifconfig()[0])
        except Exception as e:
            print('WiFi connection failed:', str(e))
            raise

    def load_learning_data(self):
        """Load learned parameters from file"""
        try:
            with open('learning_data.json', 'r') as f:
                data = ujson.load(f)  # Use ujson instead of json
                self.drift_params = data.get('drift_params', self.drift_params)
                self.successful_maneuvers = data.get('successful_maneuvers', 0)
                self.total_maneuvers = data.get('total_maneuvers', 0)
        except:
            print("No learning data found")

    def save_learning_data(self):
        """Save learned parameters to file"""
        try:
            data = {
                'drift_params': self.drift_params,
                'successful_maneuvers': self.successful_maneuvers,
                'total_maneuvers': self.total_maneuvers
            }
            with open('learning_data.json', 'w') as f:
                ujson.dump(data, f)  # Use ujson instead of json
        except:
            print("Could not save learning data")

    def record_performance(self, maneuver_type, success, metrics):
        """Record performance metrics for learning"""
        self.performance_history.append({
            'type': maneuver_type,
            'success': success,
            'metrics': metrics,
            'timestamp': time.time()
        })
        
        if len(self.performance_history) > 100:
            self.performance_history.pop(0)

    def optimize_parameters(self, maneuver_type):
        """Optimize parameters based on performance history using a lightweight algorithm"""
        if not self.performance_history:
            return
            
        successful_attempts = [p for p in self.performance_history 
                             if p['type'] == maneuver_type and p['success']]
        
        if successful_attempts:
            # Calculate simple averages using integer math
            avg_angle = sum(a['metrics']['angle'] for a in successful_attempts) // len(successful_attempts)
            avg_speed = sum(a['metrics']['speed'] for a in successful_attempts) // len(successful_attempts)
            avg_duration = sum(a['metrics']['duration'] for a in successful_attempts) // len(successful_attempts)
            
            # Apply adjustments
            self.drift_params['angle'] = (self.drift_params['angle'] + avg_angle) // 2
            self.drift_params['speed'] = (self.drift_params['speed'] + avg_speed) // 2
            self.drift_params['duration'] = (self.drift_params['duration'] + avg_duration) // 2

    def perform_drift(self, direction='right', duration=None):
        """Perform an optimized drift maneuver with reduced latency"""
        try:
            # Use learned parameters with minimal random variation
            angle = self.drift_params['angle']  # Removed random for lower latency
            speed = self.drift_params['speed']
            duration = duration or self.drift_params['duration']
            
            start_time = time.time()
            self.move_forward(speed)
            time.sleep(0.5)
            
            if direction == 'right':
                self.turn_right(angle)
            else:
                self.turn_left(angle)
                
            time.sleep(duration)
            
            # Record performance metrics
            end_time = time.time()
            success = True  # Could be determined by sensors in a real implementation
            
            metrics = {
                'angle': angle,
                'speed': speed,
                'duration': duration,
                'execution_time': end_time - start_time
            }
            
            self.record_performance('drift', success, metrics)
            self.optimize_parameters('drift')
            
            if success:
                self.successful_maneuvers += 1
            self.total_maneuvers += 1
            
            # Save learning data periodically
            if self.total_maneuvers % 10 == 0:
                self.save_learning_data()
                
        except Exception as e:
            print(f"Error performing drift: {e}")
            self.stop()
            return False
        finally:
            self.stop()
        
        return True

    def adaptive_speed_control(self, target_speed):
        """Implement adaptive speed control based on performance"""
        try:
            current_speed = self._speed
            speed_diff = target_speed - current_speed
            
            # Apply PID-like control with learned parameters
            Kp = 1.0  # Proportional gain
            Ki = 0.1  # Integral gain
            
            # Calculate control signal
            control = Kp * speed_diff + Ki * sum(speed_diff)
            
            # Apply speed with smoothing
            new_speed = min(100, max(0, current_speed + control))
            self._set_speed(new_speed)
            
            return new_speed
            
        except Exception as e:
            print(f"Error in adaptive speed control: {e}")
            return current_speed

    def get_performance_stats(self):
        """Get performance statistics"""
        success_rate = (self.successful_maneuvers / self.total_maneuvers 
                       if self.total_maneuvers > 0 else 0)
        return {
            'success_rate': success_rate,
            'total_maneuvers': self.total_maneuvers,
            'successful_maneuvers': self.successful_maneuvers,
            'current_params': self.drift_params
        }

    def _set_speed(self, speed):
        """Set motor speed (0-100) using integer arithmetic for faster computation"""
        if not 0 <= speed <= 100:
            raise ValueError("Speed must be between 0 and 100")
        self._speed = speed
        duty = (speed * 65535) // 100  # Use integer division
        self.ENA.duty_u16(duty)

    def set_servo_angle(self, angle):
        """Set servo angle (0-180 degrees)"""
        try:
            if not 0 <= angle <= 180:
                raise ValueError("Angle must be between 0 and 180")
            
            # Convert angle to duty cycle (1000µs to 2000µs)
            duty = int(((angle / 180) * (2000 - 1000) + 1000) / 20000 * 65535)
            self.servo.duty_u16(duty)
            self._steering_angle = angle
            
        except Exception as e:
            print(f"Error setting servo angle: {e}")
            self.set_servo_angle(90)  # Return to safe center position

    def move_forward(self, speed=100):
        """Move forward with specified speed"""
        try:
            self._set_speed(speed)
            self.IN1.value(1)
            self.IN2.value(0)
            self._direction = 'forward'
        except Exception as e:
            print(f"Error moving forward: {e}")
            self.stop()

    def move_backward(self, speed=100):
        """Move backward with specified speed"""
        try:
            self._set_speed(speed)
            self.IN1.value(0)
            self.IN2.value(1)
            self._direction = 'backward'
        except Exception as e:
            print(f"Error moving backward: {e}")
            self.stop()

    def turn_left(self, angle=45):
        """Turn left to specified angle"""
        try:
            center = 90
            target_angle = center - angle
            self.set_servo_angle(max(0, target_angle))
        except Exception as e:
            print(f"Error turning left: {e}")
            self.set_servo_angle(90)

    def turn_right(self, angle=45):
        """Turn right to specified angle"""
        try:
            center = 90
            target_angle = center + angle
            self.set_servo_angle(min(180, target_angle))
        except Exception as e:
            print(f"Error turning right: {e}")
            self.set_servo_angle(90)

    def stop(self):
        """Stop motors and center steering"""
        try:
            self.IN1.value(0)
            self.IN2.value(0)
            self._set_speed(0)
            self.set_servo_angle(90)
            self._direction = 'stop'
        except Exception as e:
            print(f"Error stopping: {e}")
            # Emergency stop - direct pin control
            self.IN1.value(0)
            self.IN2.value(0)

    def get_status(self):
        """Return current controller status"""
        return {
            'direction': self._direction,
            'speed': self._speed,
            'steering_angle': self._steering_angle
        }

    def emergency_stop(self):
        """Enhanced emergency stop with system status update"""
        try:
            # Cut power to motors immediately
            self.IN1.value(0)
            self.IN2.value(0)
            self.ENA.duty_u16(0)
            # Center steering
            self.set_servo_angle(90)
            self._direction = 'emergency_stop'
            self.system_status['healthy'] = False
            self.system_status['last_error'] = "Emergency stop activated"
            print("Emergency stop executed")
        except Exception as e:
            print(f"Critical error during emergency stop: {e}")
            # Last resort emergency stop
            try:
                self.IN1.value(0)
                self.IN2.value(0)
                self.ENA.duty_u16(0)
            except:
                pass

    def update_system_status(self, timer):
        """Update system health status"""
        try:
            gc.collect()
            self.system_status.update({
                'memory_free': gc.mem_free(),
                'uptime': time.time(),
                'request_count': self.request_count
            })
        except Exception as e:
            self.system_status['healthy'] = False
            self.system_status['last_error'] = str(e)

    def start_server(self, host='0.0.0.0', port=80):
        """Start HTTP server with improved error handling"""
        addr = socket.getaddrinfo(host, port)[0][-1]
        sock = socket.socket()
        sock.setblocking(False)  # Non-blocking socket
        sock.bind(addr)
        sock.listen(5)
        print(f"Server listening on {host}:{port}")

        while True:
            try:
                # Rate limiting
                if self.is_rate_limited():
                    time.sleep(0.1)
                    continue
                    
                try:
                    cl, addr = sock.accept()
                    cl.settimeout(3.0)  # 3 second timeout
                except OSError as e:
                    # No connection available
                    time.sleep(0.1)
                    continue

                try:
                    request = cl.recv(1024)
                    if not request:
                        cl.close()
                        continue
                        
                    request_str = request.decode('utf-8')
                    if not request_str:
                        self.send_error_response(cl, 400, "Invalid request")
                        continue

                    # Parse request with error handling
                    try:
                        method, path, _ = self.parse_request(request_str)
                    except ValueError:
                        self.send_error_response(cl, 400, "Malformed request")
                        continue

                    # Handle request
                    response = self.handle_request(path)
                    cl.send(response)
                    
                    # Update metrics
                    self.request_count += 1
                    self.last_request_time = time.time()
                    
                except Exception as e:
                    self.send_error_response(cl, 500, f"Internal error: {str(e)}")
                finally:
                    try:
                        cl.close()
                    except:
                        pass
                    
            except Exception as e:
                self.system_status['healthy'] = False
                self.system_status['last_error'] = str(e)
                print(f"Server error: {e}")
                time.sleep(1)  # Prevent tight error loop
                
            # Periodic cleanup
            if self.request_count % 100 == 0:
                gc.collect()

    def parse_request(self, request_str):
        """Safely parse HTTP request"""
        try:
            lines = request_str.split('\r\n')
            if not lines:
                raise ValueError("Empty request")
            parts = lines[0].split(' ')
            if len(parts) < 3:
                raise ValueError("Invalid request line")
            return parts[0], parts[1], parts[2]
        except Exception as e:
            raise ValueError(f"Request parsing error: {str(e)}")

    def is_rate_limited(self):
        """Check if requests are coming too fast"""
        current_time = time.time()
        if current_time - self.last_request_time < (1.0 / self.rate_limit):
            return True
        return False

    def send_error_response(self, client, status_code, message):
        """Send error response to client"""
        try:
            response = self.http_response(status_code, message)
            client.send(response)
        except:
            pass

    def handle_request(self, path):
        """Enhanced request handler with better error handling"""
        try:
            # Check system health
            if not self.system_status['healthy']:
                return self.http_response(503, "System unhealthy", 
                                          content_type='application/json')

            # Handle requests with validation
            if path.startswith('/forward'):
                speed = self.validate_speed(self.extract_speed(path))
                if not self.is_safe_to_move():
                    return self.http_response(409, "Unsafe to move")
                self.move_forward(speed)
                return self.http_response(200, 'Moving forward')
            elif path.startswith('/backward'):
                speed = self.validate_speed(self.extract_speed(path))
                if not self.is_safe_to_move():
                    return self.http_response(409, "Unsafe to move")
                self.move_backward(speed)
                return self.http_response(200, 'Moving backward')
            elif path.startswith('/perform_drift'):
                direction = self.extract_direction(path)
                success = self.perform_drift(direction)
                return self.http_response(200, 'Drift performed' if success else 'Drift failed')
            elif path.startswith('/get_performance_stats'):
                stats = self.get_performance_stats()
                return self.http_response(200, ujson.dumps(stats), content_type='application/json')
            elif path.startswith('/turn_left'):
                angle = self.extract_angle(path)
                self.turn_left(angle)
                return self.http_response(200, f'Turning left {angle}°')
            elif path.startswith('/turn_right'):
                angle = self.extract_angle(path)
                self.turn_right(angle)
                return self.http_response(200, f'Turning right {angle}°')
            elif path.startswith('/stop'):
                self.stop()
                return self.http_response(200, 'Stopped')
            elif path.startswith('/emergency_stop'):
                self.emergency_stop()
                return self.http_response(200, 'Emergency stop activated')
            elif path.startswith('/status'):
                status = {self.get_status(), self.system_status}
                return self.http_response(200, ujson.dumps(status), 
                                          content_type='application/json')
            else:
                return self.http_response(404, 'Not Found')
                
        except ValueError as e:
            return self.http_response(400, f"Invalid parameter: {str(e)}")
        except Exception as e:
            self.last_error = str(e)
            return self.http_response(500, f"Internal error: {str(e)}")

    def validate_speed(self, speed):
        """Validate speed parameter"""
        if not isinstance(speed, (int, float)):
            raise ValueError("Speed must be a number")
        if not 0 <= speed <= 100:
            raise ValueError("Speed must be between 0 and 100")
        return speed

    def is_safe_to_move(self):
        """Check if it's safe to move"""
        return (self.system_status['healthy'] and 
                self._direction != 'emergency_stop' and
                self.system_status['memory_free'] > 1000)

    def extract_speed(self, path):
        """Extract speed parameter from URL"""
        try:
            parts = path.split('?')
            if len(parts) > 1:
                params = parts[1].split('=')
                if params[0] == 'speed' and len(params) > 1:
                    return int(params[1])
            return 100  # Default speed
        except:
            return 100

    def extract_angle(self, path):
        """Extract angle parameter from URL"""
        try:
            parts = path.split('?')
            if len(parts) > 1:
                params = parts[1].split('=')
                if params[0] == 'angle' and len(params) > 1:
                    return int(params[1])
            return 45  # Default angle
        except:
            return 45

    def extract_direction(self, path):
        """Extract direction parameter from URL for drift"""
        try:
            parts = path.split('?')
            if len(parts) > 1:
                params = parts[1].split('=')
                if params[0] == 'direction' and len(params) > 1:
                    return params[1]
            return 'right'  # Default direction
        except:
            return 'right'

    def http_response(self, status_code, body, content_type='text/plain'):
        """Create an HTTP response"""
        return f"HTTP/1.1 {status_code} OK\r\nContent-Type: {content_type}\r\n\r\n{body}"

if __name__ == "__main__":
    try:
        controller = Controller()
    except Exception as e:
        print(f"Failed to initialize Controller: {e}")

