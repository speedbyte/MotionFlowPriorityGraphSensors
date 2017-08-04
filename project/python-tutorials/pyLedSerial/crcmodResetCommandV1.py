'''
Created on 7 Nis 2014

@author: omer
'''

'''WORKING CRC + SERIAL'''

import serial
import time
import crcmod.predefined
from binascii import unhexlify

#without crc
dict_led_commands = {
'led_reset' :       '\xCA\x00\x00\x00\x00\x00\xFE',
}

crc_modbus = crcmod.predefined.mkCrcFun('modbus')
crcresult = hex(crc_modbus(dict_led_commands['led_reset']))
print crcresult
print unhexlify(crcresult[2:])                                  

finalresult = dict_led_commands['led_reset']+unhexlify((crcresult)[2:]) #removing 0x part and converting back
print finalresult

serialObject = serial.Serial("COM9", 9600)
serialObject.write(finalresult)