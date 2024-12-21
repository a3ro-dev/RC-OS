import cv2
import numpy as np
import logging
import time
from typing import Dict, Tuple, Optional
import requests
import os

class CarAPI:
    def __init__(self):
        self.base_url = "http://192.168.31.89:80"  # Set default IP
        logging.info(f"Car API initialized with default IP: {self.base_url}")
        
    def set_ip(self, ip=None):
        if ip:
            self.base_url = f"http://{ip}:80"
        logging.info(f"Car API using: {self.base_url}")
        
    def backward(self, speed=50):
        if self.base_url:
            logging.info(f"Car moving forward at speed {speed}")
            response = requests.get(f"{self.base_url}/forward?speed={speed}")
            logging.debug(f"API Response: {response.status_code}")
            
    def forward(self, speed=50):
        if self.base_url:
            logging.info(f"Car moving backward at speed {speed}")
            response = requests.get(f"{self.base_url}/backward?speed={speed}")
            logging.debug(f"API Response: {response.status_code}")
            
    def turn_left(self, angle=45):
        if self.base_url:
            logging.info(f"Car turning left at angle {angle}")
            response = requests.get(f"{self.base_url}/turn_left?angle={angle}")
            logging.debug(f"API Response: {response.status_code}")
            
    def turn_right(self, angle=45):
        if self.base_url:
            logging.info(f"Car turning right at angle {angle}")
            response = requests.get(f"{self.base_url}/turn_right?angle={angle}")
            logging.debug(f"API Response: {response.status_code}")
            
    def stop(self):
        if self.base_url:
            logging.info("Car stopping")
            response = requests.get(f"{self.base_url}/stop")
            logging.debug(f"API Response: {response.status_code}")
            
    def emergency_stop(self):
        if self.base_url:
            logging.warning("Emergency stop activated!")
            response = requests.get(f"{self.base_url}/emergency_stop")
            logging.debug(f"API Response: {response.status_code}")

class BackEnd:
    def __init__(self):
        # Initialize logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # Initialize Car API
        try:
            self.car = CarAPI()
            self.car.stop()  # Ensure car is stopped on startup
            self.logger.info("Car API initialized successfully")
        except Exception as e:
            self.logger.error(f"Failed to initialize Car API: {e}")
            raise

        # Camera settings
        self.camera_index = 0
        self.frame_width = 640
        self.frame_height = 480
        self.cap = None

        # Human following parameters
        self.max_speed = 30  # Safe maximum speed
        self.min_speed = 15  # Minimum speed while moving
        self.max_steering = 30  # Maximum steering angle
        self.turning_threshold = 15  # Minimum angle to trigger a turn
        
        # Safety parameters
        self.min_follow_distance = 200  # Stop if closer than this
        self.max_follow_distance = 400  # Full speed if further than this
        self.target_distance = 300  # Ideal following distance
        
        # Detection parameters
        self.frame_skip = 2  # Process every nth frame
        self.frame_counter = 0
        
        # Adjust ROI to focus on lower body
        self.roi_height = self.frame_height // 2  # Only look at bottom half
        self.roi_y = self.frame_height - self.roi_height
        
        # Adjust following parameters
        self.min_follow_distance = 150  # Closer following
        self.max_follow_distance = 300  # Reduced max distance
        self.target_distance = 200  # Ideal following distance
        
        # More responsive PID values
        self.kp = 0.3
        self.ki = 0.0
        self.kd = 0.1
        self.integral = 0
        self.last_error = 0
        self.last_steering = 0

        # Initialize camera and model
        self._setup_camera()
        self._setup_model()

    def _setup_model(self):
        """Initialize Haar Cascade model"""
        try:
            model_path = os.path.join(os.path.dirname(__file__), 'haarcascade_fullbody.xml')
            self.cascade = cv2.CascadeClassifier(model_path)
            if self.cascade.empty():
                raise RuntimeError("Failed to load Haar Cascade model")
            self.logger.info("Haar Cascade model loaded successfully")
        except Exception as e:
            self.logger.error(f"Failed to load Haar Cascade model: {e}")
            raise

    def _setup_camera(self) -> bool:
        """Initialize the camera with specified settings."""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            
            if not self.cap.isOpened():
                raise RuntimeError("Failed to open camera")
                
            return True
        except Exception as e:
            self.logger.error(f"Camera initialization failed: {str(e)}")
            return False

    def get_frame(self) -> Optional[np.ndarray]:
        """Optimized frame capture with ROI"""
        if self.cap is None or not self.cap.isOpened():
            self._setup_camera()
            
        ret, full_frame = self.cap.read()
        if not ret or full_frame is None:
            self.logger.error("Failed to capture frame")
            return None
            
        # Only process bottom half of frame
        frame = full_frame[self.roi_y:, :]
        return frame

    def detect_humans(self, frame):
        """Optimized human detection using Haar Cascade"""
        # Skip frames to reduce lag
        self.frame_counter += 1
        if self.frame_counter % self.frame_skip != 0:
            return []

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect humans
        humans = self.cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        
        return [{'box': (x, y, w, h), 'confidence': 1.0} for (x, y, w, h) in humans]

    def follow_human(self, frame) -> Dict:
        """Optimized human following logic"""
        humans = self.detect_humans(frame)
        
        if not humans:
            self.car.stop()
            self.integral = 0
            return {"speed": 0, "steering": 0, "confidence": 0}
        
        # Get largest (closest) human
        largest_human = max(humans, key=lambda h: h['box'][2] * h['box'][3])
        x, y, w, h = largest_human['box']
        
        # Draw detection box (if needed)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Calculate center offset from bottom of detection box
        frame_center = self.frame_width // 2
        human_center = x + (w // 2)
        
        # Simplified steering control
        error = frame_center - human_center
        self.integral = max(min(self.integral + error, 50), -50)  # Reduced integral limit
        derivative = error - self.last_error
        
        steering = -(self.kp * error + self.ki * self.integral + self.kd * derivative)
        steering = max(-self.max_steering, min(self.max_steering, steering))
        
        # Simplified speed control based on width of detection
        if w > self.min_follow_distance:  # Too close
            speed = 0
            self.car.stop()
        else:
            speed = self.max_speed  # Move forward at max speed
            
            # Apply controls immediately
            self.car.forward(speed)
            if abs(steering) > self.turning_threshold:
                if steering > 0:
                    self.car.turn_left(abs(steering))
                else:
                    self.car.turn_right(abs(steering))
        
        self.last_error = error
        
        return {
            "speed": speed,
            "steering": steering,
            "confidence": largest_human['confidence']
        }

    def process_frame(self) -> Dict:
        """Optimized processing loop"""
        frame = self.get_frame()
        if frame is None:
            return {"speed": 0, "steering": 0, "confidence": 0}
            
        try:
            result = self.follow_human(frame)
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)
            return result
        except Exception as e:
            self.logger.error(f"Error processing frame: {str(e)}")
            self.car.stop()
            return {"speed": 0, "steering": 0, "confidence": 0}

    def cleanup(self):
        """Enhanced cleanup with car stop"""
        self.car.stop()  # Ensure car stops when program ends
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

    def __del__(self):
        self.cleanup()


backend = BackEnd()
while True:
    backend.process_frame()