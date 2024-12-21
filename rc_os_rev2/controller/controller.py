import tkinter as tk
from tkinter import messagebox, simpledialog
import logging
import requests

# GUI for Car API
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

# Create GUI
class CarControllerGUI:
    def __init__(self, root):
        self.api = CarAPI()

        # Configure root window
        root.title("Car Controller GUI")
        root.geometry("600x500")
        root.configure(bg="#d3d3d3")

        # Title label
        tk.Label(root, text="Car Controller", font=("Segoe UI", 20, "bold"), bg="#d3d3d3", fg="#000").pack(pady=15)

        # IP configuration
        tk.Button(root, text="Set IP", font=("Segoe UI", 12), command=self.set_ip, bg="#0078d7", fg="white", relief="flat").pack(pady=10)

        # Key bindings
        root.bind("<w>", lambda event: self.api.forward(speed=50))
        root.bind("<s>", lambda event: self.api.backward(speed=50))
        root.bind("<a>", lambda event: self.api.turn_left(angle=45))
        root.bind("<d>", lambda event: self.api.turn_right(angle=45))
        root.bind("<space>", lambda event: self.api.emergency_stop())

        # Instructions
        tk.Label(root, text="Use W/A/S/D to control the car", font=("Segoe UI", 12), bg="#d3d3d3", fg="#000").pack(pady=5)
        tk.Label(root, text="Press SPACE for Emergency Stop", font=("Segoe UI", 12), bg="#d3d3d3", fg="#000").pack(pady=5)

        # Logging frame
        tk.Label(root, text="Logs", font=("Segoe UI", 12, "bold"), bg="#d3d3d3", fg="#000").pack()
        self.log_text = tk.Text(root, height=15, width=70, state=tk.DISABLED, bg="#f0f0f0", fg="#000")
        self.log_text.pack(pady=10)

        # Logging configuration
        logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")
        logging.getLogger().addHandler(self.TextHandler(self.log_text))

    def set_ip(self):
        ip = simpledialog.askstring("Set IP", "Enter the IP address of the car:")
        if ip:
            self.api.set_ip(ip)

    class TextHandler(logging.Handler):
        def __init__(self, text_widget):
            super().__init__()
            self.text_widget = text_widget

        def emit(self, record):
            self.text_widget.config(state=tk.NORMAL)
            self.text_widget.insert(tk.END, self.format(record) + "\n")
            self.text_widget.see(tk.END)
            self.text_widget.config(state=tk.DISABLED)

if __name__ == "__main__":
    root = tk.Tk()
    app = CarControllerGUI(root)
    root.mainloop()
