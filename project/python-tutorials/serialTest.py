##=========================================
# serialTest.py
# Read serial port where XBee is connected
# and write message to file
#
# Leander John @ fraunhofer IAO KEIM
# 14.06.2013
##==========================================

import serial, time, os
    
if __name__ == "__main__":

    s1 = serial.Serial("COM10", baudrate=9600, timeout=0)
    os.remove("C:\\Temp\\ausgabe.csv")
    time.sleep(0.1)
    #tempfile = open("C:\\Temp\\ausgabe.txt", "a")
    pre_value = 'x'
    try:
        
        while True:    
            #r = s1.read(14)
            r = s1.readline()
            if r != pre_value:
                with open('C:\\Temp\\ausgabe.csv', 'a') as tempfile:
                    tempfile.write(r + ',')

                #tempfile.close()
                print r
            
            pre_value = r
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        pass
    
    finally:
        s1.close()
        tempfile.close()
        print "done"

