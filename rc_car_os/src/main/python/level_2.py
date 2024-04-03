import socket
from time import sleep
from machine import Pin, PWM

import network

def connect_to_wifi(ssid, password):
    # Initialize the WiFi station interface
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    # If not already connected, then connect
    if not wlan.isconnected():
        print('Connecting to network...')
        wlan.connect(ssid, password)

        # Wait for the board to connect to the WiFi
        while not wlan.isconnected():
            pass

    print('Network config:', wlan.ifconfig())

# Replace with your network's SSID and password
connect_to_wifi('a3rodev', 'a3rodev')

class Controller:
    def __init__(self):
        # Define the pins connected to the L298N
        # Replace with the actual pins you're using
        self.IN1 = Pin(0, Pin.OUT)
        self.IN2 = Pin(1, Pin.OUT)
        self.servo = PWM(Pin(2))

        # Set the frequency and duty cycle of the servo motor
        self.servo.freq(50)
        self.servo.duty_u16(32767)  # Set to middle position

    def move_forward(self):
        self.IN1.high()
        self.IN2.low()

    def move_backward(self):
        self.IN1.low()
        self.IN2.high()

    def turn_left(self):
        # Set the duty cycle to move the servo to the left
        self.servo.duty_u16(13107)  # 20% duty cycle

    def turn_right(self):
        # Set the duty cycle to move the servo to the right
        self.servo.duty_u16(52428)  # 80% duty cycle

    def stop(self):
        self.IN1.low()
        self.IN2.low()
        self.servo.duty_u16(32767)  # Set to middle position

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
        }

        .button-container {
            display: grid;
            grid-template-areas:
                ". forward ."
                "left stop right"
                ". backward .";
            gap: 10px;
        }

        button {
            padding: 10px 20px;
            font-size: 16px;
            border: none;
            border-radius: 50px;
            color: white;
            background-color: #4CAF50;
            cursor: pointer;
            box-shadow: 0px 8px 15px rgba(0, 0, 0, 0.1);
            transition: all 0.3s ease 0s;
        }

        button:hover {
            background-color: #45a049;
        }

        button:active {
            transform: scale(0.95);
            box-shadow: 0px 5px 10px rgba(0, 0, 0, 0.1);
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
            background-color: red;
        }
    </style>
</head>
<body>
    <div class="button-container">
        <button id="forward" onmousedown="buttonDown('forward')" onmouseup="buttonUp()">Forward</button>
        <button id="left" onmousedown="buttonDown('left')" onmouseup="buttonUp()">Left</button>
        <button id="stop" onmousedown="buttonDown('stop')" onmouseup="buttonUp()">Stop</button>
        <button id="right" onmousedown="buttonDown('right')" onmouseup="buttonUp()">Right</button>
        <button id="backward" onmousedown="buttonDown('backward')" onmouseup="buttonUp()">Backward</button>
    </div>
    <script>
        function buttonDown(direction) {
            fetch(`/${direction}`, { method: 'POST' });
        }

        function buttonUp() {
            fetch('/stop', { method: 'POST' });
        }
    </script>
</body>
</html>
"""

def handle_request(request):
    lines = request.split('\n')
    method, path, _ = lines[0].split(' ')

    if method == 'POST':
        if path == '/forward':
            controller.move_forward()
        elif path == '/backward':
            controller.move_backward()
        elif path == '/left':
            controller.turn_left()
        elif path == '/right':
            controller.turn_right()
        elif path == '/stop':
            controller.stop()

    return 'HTTP/1.1 200 OK\n\n' + html

controller = Controller()
s = socket.socket()

# Bind the socket to your local IP address and a port
s.bind(('0.0.0.0', 80))
s.listen(1)

while True:
    conn, addr = s.accept()
    request = conn.recv(1024).decode('utf-8')
    response = handle_request(request)
    conn.send(response.encode('utf-8'))
    conn.close()