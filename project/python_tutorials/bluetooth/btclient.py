
from time import sleep
import time
import bluetooth
import threading

class btclient(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.lock = threading.Lock() 
        self.connected = False
        self.timeout = 1
    
    def run(self):
        while True:
            if self.connected:
                self.send_msg("1;")
                sleep(0.5)
    
    def connect(self, target=None, timeout=10.0):

        if not target:
            service = "Bluetooth Server"
            service_matches = bluetooth.find_service(service)
            for i in range(len(service_matches)):
                if service_matches[i]["name"]==service:
                    target = (service_matches[i]["host"], service_matches[i]["port"])
            print "target:", repr(target)
        
        if not target:
            print "no target to connect"
            return False

        self.connected = False
        t = time.time()
        while not self.connected and (timeout > time.time()-t):
            self.client_socket = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
            try:
                self.client_socket.connect(target)
                self.connected = True
            except bluetooth.BluetoothError, e:
                sleep(0.1)
                
        return self.connected
        
    
    def disconnect(self):
        self.connected = False
        if self.client_socket:
            self.client_socket.close()


    def send_msg(self, msg):
        self.lock.acquire()
        self.client_socket.send(msg)
        sleep(0.2)
        self.lock.release()


    def recv_msg(self, timeout=10, length=1024):
        result = None
        start = time.time()
        while (time.time()-start<timeout) and (result==None):
            try:
                result = self.client_socket.recv(length)
            except bluetooth.BluetoothError:
                pass
        return result
    

    def __del__(self):        
        pass


    def quit(self):
        pass



if __name__=="__main__":
    # ("00:1f:df:fc:2a:e4", 12) # N78

    c = btclient()
    #c.start()
    print("performing inquiry...")
    nearby_devices = bluetooth.discover_devices(lookup_names = True)
    print("found %d devices" % len(nearby_devices))
    for addr, name in nearby_devices:
        print("  %s - %s" % (addr, name))

    #if c.connect(("00:21:fe:f0:31:33", 12)):
    #    print "connection successful"
        #c.send_msg("AT+CNUM\r") AT+CPBS=MC
        #c.send_msg("AT+CPBS=MC\r") 
        #c.send_msg("AT+CPBR=1,10\r")
        #c.send_msg("0x1C;07119702314")
        #result = c.recv_msg(5)
        #print "response:", result
        #sleep(10)
        #c.send_msg("0x1D;")
        #result = c.recv_msg(5)
        #print "response:", result
        c.disconnect()

    #sleep(2.0)


