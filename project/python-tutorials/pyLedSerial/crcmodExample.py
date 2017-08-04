'''
Created on 4 Nis 2014

@author: omer
'''
import crcmod.predefined
crc_modbus = crcmod.predefined.mkCrcFun('modbus')
crcresult = hex(crc_modbus("\xCA\x00\x00\x00\x00\x05\x7E\x08\x01\x01\x01\x01"))
print crcresult
#result is E851 correct!

crcresult = hex(crc_modbus("\xCA\x00\x00\x00\x00\x03\x7E\x11\x01\x01"))
print crcresult

crcresult = hex(crc_modbus("\xCA\x00\x00\x00\x00\x05\x7E\x04\x00\x00\x00\x00"))
print crcresult