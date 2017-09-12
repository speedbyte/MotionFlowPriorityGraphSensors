# -*- coding: utf-8 -*- #cp1252
import bluetooth
import string, sys
import re
#import constructor
from SmsDecoder import SmsDecoder

class at_commands(object):

    def __init__(self, bd_addr, port):
        self.s=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
        self.s.connect((bd_addr, port))
        
        #self.s.send('AT+CLAC\r')
        #print self.s.recv(1024),
        #print self.s.recv(1024),
        #print self.s.recv(1024),

    def __del__(self):
        self.s.close()

    def readPhoneBookNumber(self, index):
        self.s.send('AT+MODE=2\r')
        self.s.recv(1024),
        
        self.s.send('AT+CPBS="AD"\r')
        self.s.send('AT+CPBR='+str(index)+'\r')
        result=self.s.recv(1024),
        result=string.split(result[0], '\r')
        result=string.split(result[5], ',')
        print result[1]

        self.s.send('AT+MODE=0\r')
        self.s.recv(1024),

        return result[1]

    def readPhoneBookName(self, index):
        self.s.send('AT+MODE=2\r')
        self.s.recv(1024),
        
        self.s.send('AT+CPBS="AD"\r')
        self.s.recv(1024),
        
        self.s.send('AT+CPBR='+str(index)+'\r')
        result=self.s.recv(1024),
        #print result
        result=string.split(result[0], '\r')
        result=string.split(result[5], ',')
        print result[3]

        self.s.send('AT+MODE=0\r')
        self.s.recv(1024),

    def readPhoneBookIndex(self, contact):
        self.s.send('AT+CPBS="AD"\r')
        self.s.recv(1024),
        
        self.s.send('AT+CPBR=?\r')
        result=self.s.recv(1024),
        result=string.split(result[0], '\r')
        result=string.split(result[2], '(')
        result=string.split(result[1], ')')
        result=string.split(result[0], '-')

        for i in range(int(result[0]),int(result[1])):
            self.s.send('AT+CPBR='+str(i)+'\r')
            result=self.s.recv(1024),
            print 'result', result
            result2=result
            result_num=string.split(result[0], '\r')
            result_num=string.split(result_num[2], ',')
            
            result_name=string.split(result2[0], '\r')
            result_name=string.split(result_name[2], ',')
            print 'found', result_num[1], result_name[3]
            print 'contact', contact
            if result_num[1]=='"'+contact+'"' or result_name[3]=='"'+contact+'"':
                print 'contact index: ', i
                return i

    
    def readSms(self):
        #self.s.send('AT+MODE=2\r')
        #self.s.recv(1024),
        
        self.s.send('AT+CMGF=0\r')
        self.s.recv(1024),

        self.s.send('AT+CMGL=4\r')
        result=self.s.recv(1024),
        print result
        
        pattern1=re.compile(r'OK')
        pattern2=re.compile(r'ERROR')
        while not (re.search(pattern1, str(result), re.IGNORECASE) or re.search(pattern2, str(result), re.IGNORECASE)):
            result=self.s.recv(1024),
            print 'Ü',result
            sms=SmsDecoder('0791947101670000040D91946190306324F40011802150216352003EC8309BFD6681C2ECF91B941EA341E83D9D5E0695E5733A28DC')
            print 'decoded', sms.getMessage()
            #p=re.compile(r': ')
            #iterator=p.finditer(str(result))
            #for match in iterator:
                #print match.span()
                #print str(result)[match.span()[0]:match.span()[1]]
            

            
        #print result
        #result= string.split(result[0], '\n')
        #result=result[2]
        #sms=SmsDecoder(result)
        #print 'decoded', sms.getMessage()
        #print result
        #res=""
        #result=result[31:]
        #print 'ab 31',result
        #for i in range(len(result)/2):
        #    ind=i*2
        #    res+=chr(int(str(result[ind])+str(result[ind+1]),16))

        #print 'string',res
    
        #self.s.send('AT+MODE=0\r')
        #self.s.recv(1024),


    def sendSms(self, contact, message):
        if self.isPhoneNumber(contact):
            phonenumber=contact
        else:
            phonenumber=self.readPhoneBookNumber(self.readPhoneBookIndex(contact))
            
        #self.s.send('AT+MODE=2\r')
        #self.s.recv(1024),

        self.s.send('ATZ\r') #Reset the modem
        self.s.recv(1024),

        self.s.send('AT+CMGF=1\r')
        self.s.recv(1024),

        #self.s.send('AT+CSCA="+393359609600"\r') # Client TIM ITA
        #self.s.recv(1024),
        
        self.s.send('AT+CMGS="'+phonenumber+'"\r')
        self.s.recv(1024),

        self.s.send(message+'\n')
        self.s.recv(1024),

        self.s.send(chr(26)) # CTRL+Z
        self.s.recv(1024),
        
        #self.s.send('AT+MODE=0\r')
        #self.s.recv(1024),

    def isPhoneNumber(self, contact):
        regex=re.compile(r'[+ ]?(\d){6,}')
        if regex.match(contact):
            print 'isPhoneNumber', 'True'
            return True
        else:
            print 'isPhoneNumber', 'False'
            return False

    def getCurrentPhoneMemory( self ): # Preferred Message Storage
        self.s.send('at+cpms?\r')
        result = self.s.recv(1024),
        print result        

    def setCurrentPhoneMemory( self, memory ): # Preferred Message Storage
        self.s.send('at+cpms="%s"\r' % memory)
        result = self.s.recv(1024),
        print result

    def getSupportedPhoneMemory( self ): # Preferred Message Storage
        self.s.send('at+cpms=?\r')
        result = self.s.recv(1024),
        print result 


if __name__=='__main__':
    c=at_commands('00:0A:28:4B:0E:73', 1)
    try:
        choice=0
        while True:
            print """
                Options:
                0 - exit
                1 - read phone book number
                2 - read phone book name
                3 - read sms
                4 - send sms
                5 - read phone book index
                6 - is phone number ?
                7 - getCurrentPhoneMemory
                8 - setCurrentPhoneMemory
                9 - getSupportedPhoneMemory
                """
            choice=raw_input('option: ')
            if choice=='0':
                break
            elif choice=='1':
                i=raw_input('index: ')
                c.readPhoneBookNumber(i)
            elif choice=='2':
                i=raw_input('index: ')
                c.readPhoneBookName(i)
            elif choice=='3':
                #i=raw_input('index: ')
                c.readSms()
            elif choice=='4':
                m=raw_input('message: ')
                p=raw_input('phone book number or contact name: ')
                c.sendSms()
            elif choice=='5':
                contact=raw_input('phone number or contact name: ')
                c.readPhoneBookIndex(contact)
            elif choice=='6':
                contact=raw_input('phone number or contact name: ')
                c.isPhoneNumber(contact)
            elif choice=='7':
                c.getCurrentPhoneMemory()
            elif choice=='8':
                memory=raw_input('phone memory: ')
                c.setCurrentPhoneMemory(memory)
            elif choice=='9':
                c.getSupportedPhoneMemory()
            else:
                print 'invalid option'

        del c
        
    except Exception, e:
        print e
        del c
        
