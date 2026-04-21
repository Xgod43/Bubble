from picamera2 import Picamera2
import time

# Initialize camera
picam2 = Picamera2()

# Configure camera for still capture
camera_config = picam2.create_still_configuration()
picam2.configure(camera_config)

# Start camera
picam2.start()

# Give the camera a moment to adjust
time.sleep(2)

# Capture image
picam2.capture_file("test_image222.jpg")

print("Image saved as test_image222.jpg")
