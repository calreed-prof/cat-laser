import time
from picamera2 import Picamera2
import libcamera

# Initialize the camera
picam2 = Picamera2()

# Configure camera parameters (optional)
camera_config = picam2.create_still_configuration(main={"size": (1920, 1080)})
camera_config["transform"] = libcamera.Transform(vflip=1, hflip=1)
picam2.configure(camera_config)

# Start camera and let sensor adjust to light
picam2.start()
time.sleep(2)

# Capture and save the image
picam2.capture_file("my_image.jpg")
print("Image saved successfully.")

# Stop camera
picam2.stop()