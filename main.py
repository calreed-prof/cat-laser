from gpiozero import LED
from time import sleep

laser = LED(17)
while True:
    laser.on()
    print("ON")
    sleep(2)
    laser.off()
    print("OFF")
    sleep(2)
