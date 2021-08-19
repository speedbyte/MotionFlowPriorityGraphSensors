import socket
import time
import random 

UDP_IP = "127.0.0.1"
UDP_PORT = 9876
sock = socket.socket(socket.AF_INET, # Internet
             socket.SOCK_DGRAM,0) # UDP
n = 1000;
while(n>0):
	time.sleep(.01)    
	rand_string = "abraca#dsfsd#sdfdsfdsfsd#Run#"
	rand_val = random.randint(0,255) 
	rand_string = rand_string + "MF:" + str(rand_val) + ";"
	rand_val = random.randint(0,255) 
	rand_string = rand_string + str(rand_val) + ";"
	rand_val = random.randint(0,255) 
	rand_string = rand_string + str(rand_val) + '#further\r\n'
	print rand_string
	try:
		sock.sendto(rand_string, (UDP_IP, UDP_PORT))
	except:
		"cant send yet"
	n = n - 1