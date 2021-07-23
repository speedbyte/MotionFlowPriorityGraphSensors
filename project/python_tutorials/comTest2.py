'''
Created on 28.05.2013

@author: Leander_John
'''
import serial, time

#sys.path.append("D:\\IT_Studium\\Abschlussarbeit\\PythonFiles")
    
if __name__ == "__main__":

#    s1 = serial.Serial("COM5", baudrate=9600, timeout=0)
    s1 = serial.Serial("COM5")
#    s1.open()
#    print s1.portstr # check which port was really used
#    s2 = serial.Serial("COM6", baudrate=9600, timeout=0)
    s2 = serial.Serial("COM6")
    
    for i in range(3000):
        
#        s1.write("hello")
        s1.write(0x55)
        r = s2.read(2)
        print r 
#        print "."
        
        time.sleep(1)
        
    s1.close()
    s2.close()
    print "done"
    
