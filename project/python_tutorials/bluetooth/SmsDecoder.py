
"""	  
    This file is part of DoomiN's Phone Manager program.
    Copyright 2003 by Dominik Pytlewski <d (dot) pytlewski (at) gazeta (dot) pl>

    DoomiN's Phone Manager is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 2 as published by
    the Free Software Foundation
    
    DoomiN's Phone Manager is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with DoomiN's Phone Manager; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

import re
import sys
import string
from string import *

hex2dec = {'0':0, '1':1, '2':2, '3':3,'4':4, '5':5, '6':6, '7':7,'8':8, '9':9, 'A':10, 'B':11, 'C':12, 'D':13, 'E':14, 'F':15 }

def hex2int( p_hex, debugMode=True ):
	if debugMode:
		print "hex2int(%s)" % p_hex

	c1 = p_hex[0]
	c2 = p_hex[1]

	c3 = (hex2dec[c1]*16)+(hex2dec[c2])
	return int("%s" % c3)	

def byteSwap( p_byte ):
	return "%c%c" % ( p_byte[1], p_byte[0] )

def char2bits( p_char, debugMode=True ):

	#print "char2bits(%s)" % p_char
	inputChar = hex2int( p_char, debugMode )
	mask = 1
	output = ''
	bitNo = 1

	while bitNo<=8 :

		if inputChar & mask > 0:
			output = '1'+output
		else:
			output = '0'+output

		#print "new mask == %d" % mask 
		mask = mask<<1
		bitNo+=1

	#print "char %s -> int %d -> bits %s" % ( p_char, inputChar, output )
	return output

def bits2int( p_bits ):
	mask=1

	i = 0	
	end = len(p_bits)-1

	result = 0

	while i<=end:
		if p_bits[end-i] == "1":
			result += mask

		mask = mask<<1
		i+=1

	#print "bit2int(%s) == %d" % ( p_bits, result )
	return result

def decodeText16Bit( p_src, debugMode=True ):

	chars = u''
	i=0
	while i<len(p_src):
		h1 = p_src[i:i+2]
		h2 = p_src[i+2:i+4]
		c1 = hex2int(h1,debugMode)
		c2 = hex2int(h2,debugMode)
		
		unicodeIntChar = (256*c1)+c2
		unicodeChar = chr( unicodeIntChar )
		
		chars += unicodeChar
		
		if debugMode:
			print "[%s,%s] ->[%d,%d] -> %d -> %c" %  (h1,h2,c1,c2,unicodeIntChar,unicodeChar)
			print "current decoded: %s" % chars
		i+=4
	
	if debugMode:
		print "decoded as unicode %s" % chars
	
	return chars

def decodeText8Bit( p_src,  debugMode=True ):
	return "8bit"

def decodeText7Bit( p_src, debugMode=True ):

	bits = ''

	i = 0
	l = len(p_src)-1

	while i<l:
		bits += char2bits( p_src[i:i+2] )
		i+=2

	if debugMode:
		print "message as bit stream: %s" % bits	

	# now decoding those pseudo-8bit octets

	char_nr = 0
	i = 1

	tmp_out = ''
	acumul = ''
	decoded = ''
	while char_nr <= len(bits):

		#print "while char_nr==%d, i==%d [%d:%d]" % (char_nr,i,char_nr+i,char_nr+8)
		byte = bits[char_nr+i:char_nr+8]

		tmp_out += byte +"+"+acumul+ " "

		byte+=acumul		
		c = chr( bits2int( byte ))

		if debugMode:
			print "i==%d chr==%d bits: [%s] decod: [%c]" % (i,char_nr,byte,c)
		decoded += c

		acumul = bits[char_nr:char_nr+i]

		i+=1
		char_nr+=8

		if i==8:
			i=1
			char_nr
			decoded+=chr(bits2int(acumul))
			acumul=''	
			tmp_out+="\n"

	#print "currently_decoded: [%s]" % decoded

	if debugMode:	
		print "tmp_out: %s " % tmp_out
		print "decoded: [%s]" % decoded

	return decoded

def parseTimeStamp( p_time ):

	y = byteSwap( p_time[0:2] )
	m = byteSwap( p_time[2:4] )
	d = byteSwap( p_time[4:6] )

	hour = byteSwap( p_time[6:8] )
	min = byteSwap( p_time[8:10] )
	sec = byteSwap( p_time[10:12] )

	return "%s/%s/%s %s:%s" % (y,m,d,hour,min)



#
## this class decodes sms stored in PDU format
#
class SmsDecoder:
	def __init__( self, p_encodedText ):
		self.errorStr = ''
		self.success = self.decodeSms( p_encodedText )
		return

	#
	# return decoded message
	#
	def getMessage( self ):
		if self.success:
			return self.decodedMsg

	#
	# return decoded phone number
	#
	def getPhoneNumber( self ):
		if self.success:
			return self.headers['sender_number']

	#
	# return time stamp
	#
	def getTimeStamp( self ):
		if self.success:
			return self.headers['time_stamp']


	#
	# decoding status
	#
	def isOk( self ):
		return self.success
		
	def __del__( self ):
		return

	def decodeHeader( self, p_encSms ):
		header = {}
		smsc_len = hex2int( p_encSms[0:2] )

		type_of_address = hex2int( p_encSms[2:4] )
		
		#
		## load SMS_CENTER number
		#
		
		smsc_number = ''

		i = 4

		while i/2 <= smsc_len:
	
			smsc_number += p_encSms[i+1]

			if i/2 != smsc_len:
				 smsc_number += p_encSms[i]

			i+=2
		
		header[ 'smsc_number' ] = smsc_number
		
		if __debug__:
			print "sms_center_len == %d" % smsc_len
			print "type_of_address == %d" % type_of_address
			print "sms_center_number == %s" % smsc_number

		#	
		## decoding sms sender information
		#
		if __debug__:
			print "decoding pdu_type flags: %s [%s]" % ( p_encSms[i:i+2],char2bits( p_encSms[i:i+2] ) )

		pdu_flags = hex2int(p_encSms[i:i+2],__debug__)

		if pdu_flags & 1 == 0 and pdu_flags & 2 == 0:
			pdu_is_sms_deliver = True
		else:
			pdu_is_sms_deliver = False

		if pdu_flags & 32 != 0:	
			status_report_present = True
		else:
			status_report_present = False

		if pdu_flags & 64 != 0:	
			user_data_present = True
		else:
			user_data_present = False

		if pdu_flags & 64 != 0:	
			reply_path_present = True
		else:
			reply_path_present = False

		if __debug__:
			print "isStatRep==%d isSmsDeliver==%d isUserData==%d isReplyPath==%d" % (status_report_present,pdu_is_sms_deliver,user_data_present,reply_path_present)

		if not pdu_is_sms_deliver:
			print "isSmsDeliver != True"
			self.errorStr = "isSmsDeliver != True"
			return False
			
		#
		## now load SENDER number
		#

		i += 2

		sender_len = hex2int( p_encSms[i:i+2],__debug__ )

		if __debug__:
			print "sender number length: %s == %s" % (p_encSms[i:i+2], sender_len)
		
		i += 2

		sender_address_type = p_encSms[i:i+2]

		if __debug__:
			print "sender address type: %s" % sender_address_type
			
		i += 2

		if sender_len % 2 == 1:
			sender_number = p_encSms[i:i+sender_len+1]
			i += sender_len+1
		else:
			sender_number = p_encSms[i:i+sender_len]
			i += sender_len		
			
		if __debug__:
			print "raw sender_number [%s]" % sender_number
			i+=2
			
		tmp_sender = ''

		s = 0
		e = len(sender_number)-1

		while s<e:		
			tmp_sender += sender_number[s+1]
			tmp_sender += sender_number[s]
				
			s+=2

		if __debug__:
			print "after while [%s]" % tmp_sender	

		if sender_len % 2 == 1:
			# need to cut of 'F'
			tmp_sender = tmp_sender[0:-1]

		if __debug__:
			print "sender number: %s" % tmp_sender

		header['sender_number'] = tmp_sender		

		#
		## load DATA CODING SCHEME and FLAGS
		#

		protocol_id = p_encSms[i:i+2]
		i+=2
		dcs_mode = p_encSms[i:i+2]
		i+=2

		dcs_bits = hex2int( dcs_mode,__debug__ )

		if __debug__:
			print "protocol id == [%s](%s) dcs == [%s](%s)==%d" % ( protocol_id, char2bits( protocol_id,__debug__ ), dcs_mode,char2bits( dcs_mode,__debug__ ),dcs_bits )	

		if dcs_bits & 128 == 1 and dcs_bits & 64 == 1 and dcs_bits & 32 == 1 and dcs_bits & 16 == 1:
			special_coding = True
			if __debug__:
				print "special coding = True"
		else:
			special_coding = False
			if __debug__:
				print "special coding = False"

		if (dcs_bits & 8 == 0 and dcs_bits & 4 == 0) or special_coding == True:
			user_data_coding = 7	
		elif dcs_bits & 8 >0 and dcs_bits & 4  == 0:
			user_data_coding = 16
			#ascii 8bit
		elif dcs_bits & 8 == 0 and dcs_bits & 4 > 0:
			user_data_coding = 8
			#UCS2
		else:
			print 'unknown encoding == %d', dcs_bits
			return None

		header['user_data_coding'] = user_data_coding

		if __debug__:
			print "dcs mode text encoding == %d" % user_data_coding

		send_time_stamp = p_encSms[i:i+14]

		header['time_stamp'] = parseTimeStamp( send_time_stamp )

		if __debug__:
			print "send_time == %s (time_stamp == %s)" % ( header['time_stamp'], send_time_stamp )

		i+=14

		header['message_len'] = hex2int( p_encSms[i:i+2] )

		if __debug__:
			print "message length: %d" % header['message_len']
		
		header['position']=i
		return header

	def decodeSms( self, p_encSms ):
		decoded = ''

		smsc_len = hex2int( p_encSms[0:2], __debug__ )

		#
		## check if stripped sms in nokia's outbox archive
		#
		
		if smsc_len > 0:
			headers = self.decodeHeader( p_encSms )			
		else:
			headers = { 'position':16, 'message_len':len( p_encSms )-2, 'user_data_coding':7, 'time_stamp':'', 'sender_number':'' }
		
		self.headers = headers
		
		if headers['message_len'] > 0:
			i=headers['position']
			message_enc = p_encSms[i:i+(headers['message_len']*2)-1]

			if __debug__:
				print "encoded message: %s" % message_enc

			if headers['user_data_coding'] == 7:
				message_dec = decodeText7Bit( message_enc )
			elif headers['user_data_coding'] == 8:
				message_dec = decodeText8Bit( message_enc )
			else:
				message_dec = decodeText16Bit( message_enc )			

			self.decodedMsg = message_dec

			if __debug__:
				print "decoded message: %s" % message_dec
				
		else:
			self.decodedMsg = ''
			return True

		return True
