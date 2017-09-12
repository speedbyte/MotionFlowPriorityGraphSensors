#!/usr/bin/env python
"""
Compute the cross spectral density of two signals
"""
import numpy as np
import matplotlib.pyplot as plt
import ctypes
import logging

def runAnalysis(filename = ""):
    # make a little extra space between the subplots
    logging.info("analysing %s", filename)
    plt.subplots_adjust(wspace=0.5)
    
    altitude1 = []
    speed1 = []
    strom = []
    speed = []
    time = []
    stopundgo = []
    
    fd_gps = open(filename, 'r') # ess -> iao, autobahn
    #fd_energy = open('f:/workspace_python/DeviceTracer/log/DATA_09_12_07_07_2014_log.txt', 'r') # ess -> iao, autobahn
    
    fileContent = fd_gps.readlines()
    #fileContentEnergy = fd_energy.readlines()
    
    fd_gps.close()
    #fd_energy.close()
    
    for line in fileContent:
        if ( "GPS" in line and "GPGGA" in line):
            splittedline = line.split(',')
            #print splittedline
            if len(splittedline ) > 7 :
                if ( splittedline[8] != ''):
                    altitude1.append(float(splittedline[9]))
        elif ( "GPS" in line and "GPRMC" in line):
            splittedline = line.split(',')
            #print splittedline
            if len(splittedline ) > 7 :
                if ( splittedline[8] != '\r'):
                    try:
                        if ( float(splittedline[7]) < 2):
                            #print "correct %f" % float(splittedline[8])
                            speed1.append(400)
                        else:
                            #print "incorrect %f" % float(splittedline[8])
                            speed1.append(0)
                    except:
                        print "skipping %d" % (len(speed1))
        elif ( "ENERGY" in line ):
            splittedline = line.split('\t')
            #print splittedline
            if len(splittedline ) > 2 :
                if ( splittedline[0] != ''):
                    strom.append((float(splittedline[2])))
                    speed.append(splittedline[9])
                    #print splittedline[9]
                    try:
                        if ( int(splittedline[9]) < 10):
                            stopundgo.append(50)
                        else:
                            stopundgo.append(-50)
                    except:
                        print "skipping %d" % (len(stopundgo))

                    time.append(float(splittedline[1]))
    #print stopundgo
    x1 = len(altitude1)
    x2 = len(speed1)
    x3 = len(strom)
    x4 = len(speed)
    x5 = len(stopundgo)

    dt = 1

    t3 = np.linspace(0, x3, x3)
    t4 = np.linspace(0, x4, x4)
    t5 = np.linspace(0, x5, x5)
    
    t = max(x4,x5)
    # nse1 = np.random.randn(len(t))                 # white noise 1
    # nse2 = np.random.randn(len(t))                 # white noise 2
    # r = np.exp(-t/0.05)
    # 
    # cnse1 = np.convolve(nse1, r, mode='same')*dt   # colored noise 1
    # cnse2 = np.convolve(nse2, r, mode='same')*dt   # colored noise 2
    
    # two signals with a coherent part and a random part
    # s1 = 0.01*np.sin(2*np.pi*10*t) + cnse1
    # s2 = 0.01*np.sin(2*np.pi*10*t) + cnse2
    
    s1 = altitude1
    s2 = speed1
    s3 = strom
    s4 = speed
    s5 = stopundgo

    
    #s4 = list(map(lambda x: x + 20, s3))
    
    plt.subplot(211)
    plt.plot(t5, s5, 'r-', t3, s3, 'b-')
    #plt.plot(t1, s1, 'b-')
    plt.xlim(0,t)
    plt.xlabel('time')
    plt.ylabel('strom')
    plt.grid(True)
    
    
    plt.subplot(212)
    plt.plot(t4, s4, 'g-')
    plt.xlim(0,t)
    #cxy, f = plt.csd(s1, s2, 256, 1./dt)
    plt.ylabel('speed')
    plt.show()


if __name__ == '__main__':
    try:
        logging.info("starting numpy")
        runAnalysis('f:\workspace_python\DeviceTracer\log\DATA_20_57_09_07_2014_log.txt')
    except:
        logging.error("analyse failed due to exception")
        str1 = "Ford Files nicht gleich !"
        str2 = "EEPROM_RamMirror" 
        ctypes.windll.user32.MessageBoxA(None, str1, str2, 0)
#         raise KeyError
