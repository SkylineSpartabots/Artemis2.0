import RPi.GPIO as GPIO
import time
import neopixel
import serial

# led info
LED_COUNT = 300
LED_PIN = 18
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_INVERT = False
LED_BRIGHTNESS = 255
LED_CHANNEL = 0

strip = neopixel.Adafruit_NeoPixel(
    LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL
)
strip.begin()

OFF = (0, 0, 0)
RED = (0, 255, 255)
ORANGE = (15, 255, 255)
YELLOW = (55, 255, 255)
GREEN = (96, 255, 255)
BLUE = (160, 255, 255)
PURPLE = (192, 255, 255)
PINK = (224, 255, 255)
WHITE = (0, 0, 127) #elliot dim func

def set_solid(color):
    for i in range(LED_COUNT):
        strip.setPixelColorRGB(i, *color)
    strip.show()

def run_ant(color, primary_size, secondary_size, delay_ms):
    k = 0
    while True:
        for i in range(LED_COUNT):
            if (i - k) % primary_size == 0:
                for j in range(secondary_size):
                    if i - j >= 0:
                        strip.setPixelColorRGB(i - j, 0, 0, 0)
            else:
                strip.setPixelColorRGB(i, *color)
        strip.show()
        time.sleep(delay_ms / 1000)
        k = (k + 1) % primary_size

def flash_solid(color, on_ms, off_ms, counts):
    for _ in range(counts):
        set_solid(color)
        time.sleep(on_ms / 1000)
        set_solid(OFF)
        time.sleep(off_ms / 1000)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
 
try:
    ser = serial.Serial('/dev/ttyACM0', 9600)
except serial.SerialException:
    print("Serial connection not available")

if __name__ == "__main__":
    while True:
        if ser: 
            data = ser.readline().decode().strip()
            try:
                mode = int(data)
            except ValueError:
                print("Invalid mode received over serial")

        input0 = GPIO.input(17)
        input1 = GPIO.input(27)
        input2 = GPIO.input(22)
        input3 = GPIO.input(23)
        gpio_mode = input0 + (2 * input1) + (4 * input2) + (8 * input3)

        if ser:
            selected_mode = mode
        else:
            selected_mode = gpio_mode

        if selected_mode == 0:
            set_solid(OFF)
        elif selected_mode == 1:
            set_solid(RED)
        elif selected_mode == 2:
            set_solid(ORANGE)
        elif selected_mode == 3:
            set_solid(GREEN)
        elif selected_mode == 4:
            set_solid(BLUE)
        elif selected_mode == 5:
            set_solid(PURPLE)
        
        else:
            print("Unknown mode") 
