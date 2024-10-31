import machine
import time
import ustruct
import sys

from bno055 import *
from machine import Pin

# Pyboard hardware I2C
#i2c = machine.I2C(1, freq=100000)
# ESP32 and ESP8266 soft I2C
# i2c = machine.SoftI2C(scl=machine.Pin(2), sda=machine.Pin(0), timeout=100_000)

# Raspberry Pico RP2040 hardware I2C
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=4000, timeout=100_000)
imu = BNO055(i2c)
calibrated = False
led = Pin(25, Pin.OUT)
packetCount=0

#// packet structure for ROS part
# typedef struct {
#     char  char_0;   //'$'
#     char  char_1;   //'0x03'
#     float               float_s[10];  //quaternion[w,x,y,z], qyro, accel
#     unsigned long long  message_count;
#     char  char_2;   //'\r',
#     char  char_3;   //'\n'
# } custom_struct;

while True:
    if not calibrated:
        calibrated = imu.calibrated()
        #print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
    
    # Get size of each 'message' record
    # https://www.fredscave.com/45-micropython-ustruct-module.html
    format = ("<ss10fQss")
    size = ustruct.calcsize(format)
    #print("Message packed size: ", size)

    buffer = bytearray(size)
    offset=0
    
    #Message
    ustruct.pack_into(
        format,
        buffer,
        offset,
        "$", b"\x03",
        *imu.quaternion(),
        *imu.gyro(),
        *imu.accel(),
        packetCount,
        b"\r",
        b"\n"
        )
    
    packetCount+=1
    #print(buffer) #bad worked CR LF CRLF
    
    #https://github.com/orgs/micropython/discussions/12374#discussioncomment-6996634
    sys.stdout.buffer.write(buffer)

