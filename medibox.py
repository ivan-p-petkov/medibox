import cv2
import subprocess
import time
import RPi.GPIO as GPIO
import smbus
from pyzbar.pyzbar import decode
import I2C_LCD_driver
import threading

myLcd = I2C_LCD_driver.lcd()

GPIO.setmode(GPIO.BCM)
But1 = 20
But2 = 21
GPIO.setup(But1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(But2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

I2C_ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09

bus = smbus.SMBus(1)

def write_byte(reg, value):
    bus.write_byte_data(I2C_ADDR, reg, value)

def read_byte(reg):
    return bus.read_byte_data(I2C_ADDR, reg)

def set_pwm_freq(freq):
    prescaleval = 25000000.0 / 4096 / freq - 1
    prescale = int(prescaleval + 0.5)
    old_mode = read_byte(MODE1)
    write_byte(MODE1, (old_mode & 0x7F) | 0x10)
    write_byte(PRESCALE, prescale)
    write_byte(MODE1, old_mode)
    time.sleep(0.005)
    write_byte(MODE1, old_mode | 0x80)

def set_pwm(channel, pulse):
    pulse_length = int(pulse * 4096 / 20000)
    write_byte(LED0_ON_L + 4 * channel, 0 & 0xFF)
    write_byte(LED0_ON_H + 4 * channel, 0 >> 8)
    write_byte(LED0_OFF_L + 4 * channel, pulse_length & 0xFF)
    write_byte(LED0_OFF_H + 4 * channel, pulse_length >> 8)

def angle_to_pulse(angle):
    return 500 + (angle / 180) * 2000

def setup():
    write_byte(MODE1, 0x00)
    set_pwm_freq(50)

def move_servo(channel, angle):
    pulse = angle_to_pulse(angle)
    set_pwm(channel, pulse)

SERVO_1 = 0
SERVO_2 = 1

def capture_qr_image():
    subprocess.run(["rpicam-jpeg", "-o", "QR.jpg"])

def scan_qr_code(image_path):
    while True:
        capture_qr_image()
        image = cv2.imread(image_path)
        if image is None:
            time.sleep(1)
            continue
        decoded_objects = decode(image)
        if not decoded_objects:
            time.sleep(1)
            continue
        for obj in decoded_objects:
            return obj.data.decode("utf-8")

def control_servo(channel, intake, hours, button):
    for _ in range(intake):
        move_servo(channel, 90)
        time.sleep(1)
        while GPIO.input(button) == GPIO.HIGH:
            time.sleep(0.1)
        move_servo(channel, 0)
        time.sleep(hours)

if __name__ == "__main__":
    qr_text = scan_qr_code("QR.jpg")
    lines = qr_text.splitlines()
    if len(lines) >= 2:
        line1 = lines[0].split()
        line2 = lines[1].split()
        if len(line1) >= 3 and len(line2) >= 3:
            name1, intake1, hours1 = line1[0], line1[1], line1[2]
            name2, intake2, hours2 = line2[0], line2[1], line2[2]
            intake1 = intake1.replace(",", "")
            hours1 = hours1.replace(",", "")
            intake2 = intake2.replace(",", "")
            hours2 = hours2.replace(",", "")
            intake1 = int(intake1)
            hours1 = int(hours1)
            intake2 = int(intake2)
            hours2 = int(hours2)
            myLcd.lcd_display_string(f"{name1}", 1)
            myLcd.lcd_display_string(f"{name2}", 2)
            setup()
            servo1_thread = threading.Thread(target=control_servo, args=(SERVO_1, intake1, hours1, But1))
            servo2_thread = threading.Thread(target=control_servo, args=(SERVO_2, intake2, hours2, But2))
            servo1_thread.start()
            servo2_thread.start()
            servo1_thread.join()
            servo2_thread.join()
