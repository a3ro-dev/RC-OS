import requests

BASE_URL = "http://192.168.31.89:80"

def test_forward(speed=25):
    response = requests.get(f"{BASE_URL}/forward?speed={speed}")
    print(f"Forward: {response.status_code}, {response.text}")

def test_backward(speed=25):
    response = requests.get(f"{BASE_URL}/backward?speed={speed}")
    print(f"Backward: {response.status_code}, {response.text}")

def test_perform_drift(direction='right'):
    response = requests.get(f"{BASE_URL}/perform_drift?direction={direction}")
    print(f"Perform Drift: {response.status_code}, {response.text}")

def test_get_performance_stats():
    response = requests.get(f"{BASE_URL}/get_performance_stats")
    print(f"Get Performance Stats: {response.status_code}, {response.text}")

def test_turn_left(angle=45):
    response = requests.get(f"{BASE_URL}/turn_left?angle={angle}")
    print(f"Turn Left: {response.status_code}, {response.text}")

def test_turn_right(angle=45):
    response = requests.get(f"{BASE_URL}/turn_right?angle={angle}")
    print(f"Turn Right: {response.status_code}, {response.text}")

def test_stop():
    response = requests.get(f"{BASE_URL}/stop")
    print(f"Stop: {response.status_code}, {response.text}")

def test_emergency_stop():
    response = requests.get(f"{BASE_URL}/emergency_stop")
    print(f"Emergency Stop: {response.status_code}, {response.text}")

def test_status():
    response = requests.get(f"{BASE_URL}/status")
    print(f"Status: {response.status_code}, {response.text}")



if __name__ == "__main__":
    test_forward()
    test_backward()
    # test_perform_drift()
    test_get_performance_stats()
    test_turn_left()
    test_turn_right()
    test_stop()
    test_emergency_stop()
    test_status()
