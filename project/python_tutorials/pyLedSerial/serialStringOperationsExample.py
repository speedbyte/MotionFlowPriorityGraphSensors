'''
Created on 4 Nis 2014

@author: omer
'''
import serial
import time
import crcmod.predefined

dict_led_commands = {
'led_start'              : '\xCA',
'led_broadcast'          : '\x00',
'led_res'                : '\xFE',
'led_write'              : '\x7E',
'led_reset'              : '\xCA\x00\x00\x00\x00\x00\xFE\x8C\xF0',

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

serialObject = serial.Serial("COM9", 9600)
list_command = [0xCA, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x8C, 0xF0]
#for x in range(5):
#serialObject.write(dict_led_commands['led_start']+(dict_led_commands['led_broadcast']*3))

serialObject.write(dict_led_commands['led_reset'])
#time.sleep(1)
serialObject.write('\xCA\x00\x00\x00\x00\x03\x7E\x11\x01\x01\x66\x99')
serialObject.write('\xCA\x00\x00\x00\x00\x05\x7E\x08\x01\x01\x01\x01\xE8\x51')
serialObject.write('\xCA\x00\x00\x00\x00\x05\x7E\x04\x00\x00\x00\x00\x85\xD1')
    #print(dict_led_commands['led_reset'])
time.sleep(1)

print "end"