#!/usr/bin/env python
"""
authors: apullin

Contents of this file are copyright Andrew Pullin, 2014

"""
from lib import command
import time,sys
import serial
import shared

from or_helpers import *


###### Operation Flags ####
SAVE_DATA1 = True 
RESET_R1 = False

EXIT_WAIT   = False

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE, timeout = 0, rtscts = True)
    
    R1 = Robot('\x20\x52', xb)
    
    shared.ROBOTS = [R1] #This is neccesary so callbackfunc can reference robots
    shared.xb = xb           #This is neccesary so callbackfunc can halt before exit
    
   
    # Query
    R1.query( retries = 1 )
    #R1.reset()
    
    if EXIT_WAIT:  #Pause for a Ctrl + Cif specified
        while True:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                break

    print "Done"
    xb_safe_exit()


#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\nRecieved Ctrl+C, exiting."
        shared.xb.halt()
        shared.ser.close()
    except Exception as args:
        print "\nGeneral exception:",args
        print "Attemping to exit cleanly..."
        shared.xb.halt()
        shared.ser.close()
        sys.exit()
