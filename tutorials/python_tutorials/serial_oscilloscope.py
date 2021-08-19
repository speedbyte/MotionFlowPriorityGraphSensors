#!/bin/python/

import serial
import random
import time
import struct

sendstring = raw_input("Enter ur choice    :   ")
try:
    counter = 0
    ser = serial.Serial("COM9", 38400)
    print ser
    while(ser!=None):
        if ( "string" in sendstring ):
            rand_string = "abraca#dsfsd#sdfdsfdsfsd#Run#"
            time.sleep(.1)    
            rand_val = random.randint(0,255) 
            rand_string = rand_string + "MF:" + str(rand_val) + ";"
            rand_val = random.randint(0,255) 
            rand_string = rand_string + str(rand_val) + ";"
            rand_val = random.randint(0,255) 
            rand_string = rand_string + str(rand_val) + '#further\r\n'
            #rand_string = "hello\r\n"
            ser.write(rand_string)
            print rand_string
        else:
            time.sleep(0.01)
            rand_val = random.randint(0,255) 
            val = struct.pack("B", (rand_val& 0xFF ))        
            print counter, val
            ser.write(val)
            if (counter == 7 ):
                ser.write('\n')
                counter = 0
            else:
                counter = counter + 1
finally:
    ser.close()
    ser = None
