#!/bin/python/

import serial
import time

dict_led_commands = {
'led_reset' : '\xCA\x00\x00\x00\x00\x00\xFE\x8C\xF0',
}

serialObject = serial.Serial("COM52", 9600)
list_command = [0xCA, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x8C, 0xF0]
for x in range(5):
    serialObject.write(dict_led_commands['led_reset'])
    time.sleep(2)

print "end"

# XOR Checksum based on a polynomial.