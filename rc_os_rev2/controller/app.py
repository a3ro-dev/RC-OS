from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6.QtGui import *
import sys
import cv2
import numpy as np
import requests
import socket
import threading
from backEND import BackEnd
import logging
import time

# Setup logging configuration after imports
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Add API control functions
class CarAPI:
    def __init__(self):
        self.base_url = "http://192.168.31.89:80"  # Set default IP
        logging.info(f"Car API initialized with default IP: {self.base_url}")
        
    def set_ip(self, ip=None):
        if ip:
            self.base_url = f"http://{ip}:80"
        logging.info(f"Car API using: {self.base_url}")
        
    def forward(self, speed=50):
        if self.base_url:
            logging.info(f"Car moving forward at speed {speed}")
            response = requests.get(f"{self.base_url}/forward?speed={speed}")
            logging.debug(f"API Response: {response.status_code}")
            
    def backward(self, speed=50):
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

class VideoThread(QThread):
    frame_ready = pyqtSignal(np.ndarray)
    
    def __init__(self, backend, car_api):
        super().__init__()
        self.backend = backend
        self.car_api = car_api
        self.running = True
        self._last_frame_time = time.time()
        self._fps = 0
        self.video_label = None  # Will be set from MainWindow
        logging.info("VideoThread initialized")

    def run(self):
        while self.running:
            try:
                frame = self.backend.get_frame()
                if frame is not None and frame.size > 0:  # Verify frame is valid
                    # Debug frame info
                    logging.debug(f"Received frame: shape={frame.shape}, dtype={frame.dtype}")
                    
                    # Process frame
                    if self.backend.autopilot_enabled:
                        command = self.backend.autonomous_drive(frame)
                        # Execute command
                        action = command.get("action", "stop")
                        speed = command.get("speed", 0)
                        angle = command.get("angle", 0)
                        
                        logging.info(f"Executing command: {action} speed={speed} angle={angle}")
                        
                        if action == "forward":
                            self.car_api.forward(speed)
                            if abs(angle) > 5:  # Only turn if angle is significant
                                if angle > 0:
                                    self.car_api.turn_right(min(abs(angle), 45))
                                else:
                                    self.car_api.turn_left(min(abs(angle), 45))
                        elif action == "stop":
                            self.car_api.stop()

                    # Always show detection visualization
                    boxes, confidences, class_ids, human_boxes = self.backend.detect(frame)
                    lanes = self.backend.detect_lanes(frame)
                    
                    # Draw detections
                    self.draw_visualizations(frame, boxes, confidences, lanes, human_boxes)
                    
                    # Add FPS counter
                    cv2.putText(frame, f"FPS: {self._fps:.1f}", (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                    # Convert and emit
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    self.frame_ready.emit(frame_rgb)
                    
                    # Update FPS
                    current_time = time.time()
                    self._fps = 1.0 / (current_time - self._last_frame_time)
                    self._last_frame_time = current_time
                    
                else:
                    logging.warning("Received invalid frame")
                    time.sleep(0.1)  # Add delay to prevent rapid retries
                    
            except Exception as e:
                logging.error(f"Error in video thread: {str(e)}")
                time.sleep(0.1)

    def draw_visualizations(self, frame, boxes, confidences, lanes, human_boxes):
        """Draw all detections and visualizations on frame"""
        # Draw lanes if detected
        if lanes is not None:
            for line in lanes:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        
        # Draw detected objects
        if len(boxes) > 0:
            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
            for i in indexes:
                box = boxes[i]
                x, y, w, h = box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    def update_frame(self, frame):
        try:
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(
                self.video_label.size(), 
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            self.video_label.setPixmap(scaled_pixmap)
        except Exception as e:
            logging.error(f"Error updating frame: {str(e)}")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RC-OS Control Panel")
        self.backend = BackEnd()
        self.car_api = CarAPI()  # Will use default IP
        self.setup_ui()
        self.setup_autopilot_ui()
        
    def setup_ui(self):
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        self.main_layout = QVBoxLayout(main_widget)
        self.main_layout.setContentsMargins(10, 10, 10, 10)
        self.main_layout.setSpacing(10)
        
        # Video display
        self.video_label = QLabel()
        self.video_label.setMinimumSize(800, 600)
        self.video_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.video_label.setScaledContents(True)
        self.video_label.setStyleSheet("""
            QLabel {
                background: rgba(255, 255, 255, 0.1);
                border-radius: 15px;
            }
        """)
        
        # Control buttons
        btn_layout = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        
        for btn in (self.connect_btn, self.disconnect_btn):
            btn.setStyleSheet("""
                QPushButton {
                    background: rgba(255, 255, 255, 0.2);
                    border: none;
                    border-radius: 10px;
                    color: white;
                    padding: 10px 20px;
                }
                QPushButton:hover {
                    background: rgba(255, 255, 255, 0.3);
                }
            """)
            btn_layout.addWidget(btn)
        
        # Key status indicators
        keys_layout = QHBoxLayout()
        self.key_labels = {}
        for key in 'WASD':
            label = QLabel(key)
            label.setStyleSheet("""
                QLabel {
                    background: rgba(255, 255, 255, 0.1);
                    border-radius: 5px;
                    color: white;
                    padding: 5px 15px;
                    min-width: 30px;
                    text-align: center;
                }
            """)
            keys_layout.addWidget(label)
            self.key_labels[key] = label
        
        # Devices combo box, refresh button, and autopilot checkbox
        self.devices_combo = QComboBox()
        self.refresh_devices_btn = QPushButton("Refresh Devices")
        self.autopilot_checkbox = QCheckBox("Autopilot")

        devices_layout = QHBoxLayout()
        devices_layout.addWidget(self.devices_combo)
        devices_layout.addWidget(self.refresh_devices_btn)
        devices_layout.addWidget(self.autopilot_checkbox)

        # Add widgets to main layout
        self.main_layout.addWidget(self.video_label)
        self.main_layout.addLayout(btn_layout)
        self.main_layout.addLayout(keys_layout)
        self.main_layout.addLayout(devices_layout)
        
        # Setup video thread with reference to video_label
        self.video_thread = VideoThread(self.backend, self.car_api)
        self.video_thread.video_label = self.video_label  # Pass reference
        self.video_thread.frame_ready.connect(self.update_frame)
        
        # Start video automatically
        self.video_thread.running = True
        self.video_thread.start()
        
        # Connect signals
        self.connect_btn.clicked.connect(self.start_video)
        self.disconnect_btn.clicked.connect(self.stop_video)
        self.refresh_devices_btn.clicked.connect(self.refresh_devices)
        self.autopilot_checkbox.stateChanged.connect(self.toggle_autopilot)
        
        # Window styling
        self.setStyleSheet("""
            QMainWindow {
                background: #1f1f1f;
            }
        """)
        
    def setup_autopilot_ui(self):
        """Add enhanced autopilot controls"""
        # Create autopilot control group
        autopilot_group = QGroupBox("Autopilot Controls")
        autopilot_layout = QVBoxLayout()
        
        # Mode selection
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Lane Following", "Human Following"])
        self.mode_combo.currentTextChanged.connect(self.change_autonomous_mode)
        
        # Status indicator
        self.status_label = QLabel("Autopilot: Disabled")
        self.status_label.setStyleSheet("color: red;")
        
        # Add to layout
        autopilot_layout.addWidget(self.mode_combo)
        autopilot_layout.addWidget(self.status_label)
        autopilot_group.setLayout(autopilot_layout)
        
        # Add to main layout after devices_layout
        self.main_layout.addWidget(autopilot_group)
        
    def change_autonomous_mode(self, mode):
        """Handle autopilot mode changes"""
        mode_map = {
            "Lane Following": "lane_following",
            "Human Following": "human_following"
        }
        self.backend.autonomous_mode = mode_map[mode]
        logging.info(f"Autopilot mode changed to: {mode}")
        
    def toggle_autopilot(self, state):
        """Enhanced autopilot toggle with verification"""
        try:
            enable = (state == Qt.CheckState.Checked)
            result = self.backend.toggle_autopilot(enable)
            
            if result:  # If backend toggle was successful
                self.status_label.setText(f"Autopilot: {'Enabled' if enable else 'Disabled'}")
                self.status_label.setStyleSheet(f"color: {'green' if enable else 'red'};")
                self.mode_combo.setEnabled(enable)
                logging.info(f"Autopilot {'enabled' if enable else 'disabled'}")
                
                # Don't override the checkbox state since the toggle was successful
                return
            else:
                logging.error("Failed to toggle autopilot")
                
        except Exception as e:
            logging.error(f"Error toggling autopilot: {e}")
            
        # Only set checkbox to False if we get here (meaning there was an error)
        self.autopilot_checkbox.setChecked(False)
        
    def update_frame(self, frame):
        try:
            if frame is None:
                logging.warning("Received None frame")
                return
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(
                self.video_label.size(), 
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            self.video_label.setPixmap(scaled_pixmap)
        except Exception as e:
            logging.error(f"Error updating frame: {str(e)}")
        
    def keyPressEvent(self, event):
        key = event.text().upper()
        if self.backend.autopilot_enabled:
            self.autopilot_checkbox.setChecked(False)  # This will trigger toggle_autopilot
            self.car_api.emergency_stop()
            logging.info("Manual control override - Autopilot disabled")
            
        if key in self.key_labels:
            self.key_labels[key].setStyleSheet("""
                QLabel {
                    background: rgba(0, 255, 0, 0.3);
                    border-radius: 5px;
                    color: white;
                    padding: 5px 15px;
                }
            """)
            # Direct mapping of keys to CarAPI commands
            if key == 'W':
                logging.info("Manual control: Forward")
                self.car_api.forward(30)
            elif key == 'S':
                logging.info("Manual control: Backward")
                self.car_api.backward(30)
            elif key == 'A':
                logging.info("Manual control: Left turn")
                self.car_api.turn_left(30)
            elif key == 'D':
                logging.info("Manual control: Right turn")
                self.car_api.turn_right(30)
                
        if event.key() == Qt.Key.Key_Space:
            logging.warning("Manual emergency stop triggered")
            self.car_api.emergency_stop()
    
    def keyReleaseEvent(self, event):
        key = event.text().upper()
        if key in self.key_labels:
            self.key_labels[key].setStyleSheet("""
                QLabel {
                    background: rgba(255, 255, 255, 0.1);
                    border-radius: 5px;
                    color: white;
                    padding: 5px 15px;
                }
            """)
            # Stop on key release
            if key in 'WASD':
                self.car_api.stop()
    
    def start_video(self):
        """Simplified start video - no need to get IP from combo box"""
        self.video_thread.running = True
        self.video_thread.start()
        
    def stop_video(self):
        self.video_thread.running = False
        self.video_thread.wait()
        self.video_label.clear()
        
    def closeEvent(self, event):
        self.stop_video()
        event.accept()

    def refresh_devices(self):
        logging.info("Scanning for devices...")
        self.devices_combo.clear()
        devices = self.backend.get_devices()
        for device in devices:
            logging.info(f"Found device: {device['ip']} ({device['hostname']})")
            self.devices_combo.addItem(f"{device['ip']} ({device['hostname']})")

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()