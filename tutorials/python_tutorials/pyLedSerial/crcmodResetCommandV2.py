'''
Created on 7 Nis 2014

@author: omer
'''

'''WORKING CRC + SERIAL'''
import serial
import time
import crcmod.predefined
from binascii import unhexlify

#FUNCTION
def crccalc(command):
    crc_modbus = crcmod.predefined.mkCrcFun('modbus')
    crcresult = hex(crc_modbus(command))
    print crcresult
    print unhexlify(crcresult[2:])  
    finalresult = dict_led_commands['led_reset']+unhexlify((crcresult)[2:]) #removing 0x part and converting back
    print finalresult 
    return finalresult;

def sendtochromoflex (finalresult):
    serialObject = serial.Serial("COM9", 9600)
    serialObject.write(finalresult)
    return;

#without crc
dict_led_commands = {
'led_reset' :       '\xCA\x00\x00\x00\x00\x00\xFE',

#REG_BITS
'led_start'              : '\xCA',
'led_broadcast'          : '\x00',
'led_res'                : '\xFE',
'led_write'              : '\x7E',

#REG_ADDRESS
'reg_red_level'          : '\x00',
'reg_green_level'        : '\x01',
'reg_blue_level'         : '\x02',
'reg_x_level'            : '\x03',

'reg_red_set'            : '\x04',
'reg_green_set'          : '\x05',
'reg_blue_set'           : '\x06',
'reg_x_set'              : '\x07',

'reg_red_incr'           : '\x08',
'reg_green_incr'         : '\x09',
'reg_blue_incr'          : '\x0A',
'reg_x_incr'             : '\x0B',

'reg_track'              : '\x11',
'reg_status'             : '\x12',

'reg_program_high'       : '\x15',
'reg_program_low'        : '\x16',        

}

#crc_modbus = crcmod.predefined.mkCrcFun('modbus')
#crcresult = hex(crc_modbus(dict_led_commands['led_reset']))
#print crcresult
#print unhexlify(crcresult[2:])                                  

a = crccalc(dict_led_commands['led_reset'])

#finalresult = dict_led_commands['led_reset']+unhexlify((crcresult)[2:]) #removing 0x part and converting back
#print finalresult

sendtochromoflex (a)
#serialObject = serial.Serial("COM9", 9600)
#serialObject.write(finalresult)