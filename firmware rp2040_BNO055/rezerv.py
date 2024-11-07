import machine
import time
import ustruct
import sys
import os

from bno055 import *
from machine import Pin

#recomended for autostart script
#https://github.com/micropython-IMU/micropython-bno055?tab=readme-ov-file#24-basic-usage
time.sleep_ms(500)

# Pyboard hardware I2C
#i2c = machine.I2C(1, freq=100000)
# ESP32 and ESP8266 soft I2C
# i2c = machine.SoftI2C(scl=machine.Pin(2), sda=machine.Pin(0), timeout=100_000)

#RaspberryPico hardware I2C
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=4000, timeout=100_000)
imu = BNO055(i2c)
led = Pin(25, Pin.OUT)
led.off()
trigger=Pin(28, Pin.OUT)


#// packet structure for ROS part
# typedef struct {
#     char  char_0;   //'$'
#     char  char_1;   //'0x03'
#     float               float_s[10];  //quaternion[w,x,y,z], qyro, accel
#     unsigned long long  message_count;
#     char  char_2;   //'\r',
#     char  char_3;   //'\n'
# } custom_struct;


def save_sensor_offsets_in_file(sensor_offsets):
    # Path to this file
    dir_path = os.getcwd()

    try: 
        binary_file = open(str(dir_path) + "sensor_offsets", "wb")
        binary_file.write(sensor_offsets)
        binary_file.close()
        #print("save offsets ok")
    except: 
        print("Error while saving sensor_offsets in file 'calibration'")
        
        
def set_sensor_offsets_from_file():

    # Path to this file
    dir_path = os.getcwd()
    try:
        binary_file = open(str(dir_path) + "sensor_offsets", "rb")
        sensor_offsets = binary_file.read()
        binary_file.close()
        #print("The calibration read from the 'sensor_offsets' file is: ")
        #print(sensor_offsets)
        imu.set_offsets(sensor_offsets)
    
    except:
        sensor_offsets = 0
        print("The file does not exist or the file cannot be read")


def send_message_to_ros(trigger_counter):
    # Get size of each 'message' record
    # https://www.fredscave.com/45-micropython-ustruct-module.html
    format = ("ss10fLss")
    size = ustruct.calcsize(format)
    #print("Message packed size: ", size)

    buffer = bytearray(size)
    offset=0
    
    #Create message
    ustruct.pack_into(
        format, buffer, offset,
        "$", b"\x03", 
        *imu.quaternion(), *imu.gyro(), *imu.accel(), trigger_counter,
        b"\r", b"\n") 

    #print(buffer) #bad worked CR LF CRLF
    #https://github.com/orgs/micropython/discussions/12374#discussioncomment-6996634
    sys.stdout.buffer.write(buffer)
    
    
calibrated = False   
irq_counter=0
trigger_counter=0


set_sensor_offsets_from_file()
while True:
    if not calibrated:
        led.off()
        calibrated = imu.calibrated()
        print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
    else:
        if trigger_counter == 0:
            led.on()
        if trigger_counter!=0 and trigger_counter%(25*60+1) == 0:
            save_sensor_offsets_in_file(imu.sensor_offsets())
            #time.sleep_ms(25)
        else:
            irq_counter += 1
            #if irq_counter % 4 == 0:
            trigger.on()
            trigger.off()
            irq_timestamp=time.ticks_ms()
            send_message_to_ros(trigger_counter)
            #print(trigger_counter)
            #print(type(irq_timestamp))
            trigger_counter += 1
            irq_counter = 0
            

