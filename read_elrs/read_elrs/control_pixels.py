
import board
import neopixel
import random
import time

DATA_PIN = board.D10
pixels = neopixel.NeoPixel(DATA_PIN, 8, auto_write=False)

while True:

    for i in range(8):
        # Set each pixel to a random color
        pixels[i] = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
    
        pixels.show()

        time.sleep(random.random()/10) #0.05

        pixels[i] = (0, 0, 0)  # Turn off the pixel after showing the color
        pixels.show()