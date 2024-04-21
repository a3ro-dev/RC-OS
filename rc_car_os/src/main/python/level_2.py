import socket
from time import sleep
from machine import Pin, PWM
import network

WLAN = network.WLAN(network.STA_IF)

def connect_to_wifi(ssid, password):
    WLAN.active(True)
    if not WLAN.isconnected():
        print('Connecting to network...')
        WLAN.connect(ssid, password)
        while not WLAN.isconnected():
            sleep(0.1)  # Add a small delay to reduce CPU usage
    print('Network config:', WLAN.ifconfig())

class Controller:
    def __init__(self):
        self.IN1 = Pin(0, Pin.OUT)
        self.IN2 = Pin(1, Pin.OUT)
        self.IN3 = Pin(2, Pin.OUT)
        self.IN4 = Pin(3, Pin.OUT)
        self.servo = PWM(Pin(4))
        self.servo.freq(50)
        self.servo.duty_u16(int(1000 / 20000 * 65535))  # Set initial position to 0 degrees

    def move_forward(self):
        print('Moving forward')
        self.IN1.high()
        self.IN2.low()
        self.IN3.high()
        self.IN4.low()

    def move_backward(self):
        print('Moving backward')
        self.IN1.low()
        self.IN2.high()
        self.IN3.low()
        self.IN4.high()

    def turn_left(self):
        print('Turning left')
        self.servo.duty_u16(int(1000 / 20000 * 65535))  # Set position to 0 degrees

    def turn_right(self):
        print('Turning right')
        self.servo.duty_u16(int(2000 / 20000 * 65535))  # Set position to 180 degrees

    def stop(self):
        print('Stopped')
        self.IN1.low()
        self.IN2.low()
        self.IN3.low()
        self.IN4.low()
        self.servo.duty_u16(int(1500 / 20000 * 65535))  # Set position back to 90 degrees

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
            background-color: #333;
            color: #fff;
            font-family: Arial, sans-serif;
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
    first_line = lines[0].split(' ')
    if len(first_line) >= 3:
        method, path, _ = first_line
    else:
        method, path = first_line[0], ''
    status_code = '200 OK'
    message = ''
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
        except Exception as e:
            status_code = '500 Internal Server Error'
            message = str(e)
    return f'HTTP/1.1 {status_code}\n\n{message}\n{html}'

controller = Controller()
s = socket.socket()
s.bind(('0.0.0.0', 80))
connect_to_wifi('vijayprakashsinghkushwaha2.4ghz', '9696949718')
s.listen(1)  # Listen for connections
print("Server Up and Running")
ip_address = WLAN.ifconfig()[0]
print('IP:', ip_address)

while True:
    conn, addr = s.accept()
    request = conn.recv(1024).decode('utf-8')
    response = handle_request(request)
    conn.send(response.encode('utf-8'))
    conn.close()  # Add a small delay to reduce CPU usage