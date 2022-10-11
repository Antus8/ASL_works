import sys
import os
import serial
import time

working_directory = os.getcwd()
print("Arguments are: {}".format(sys.argv))

def main():
    print("Process starting...")

    # ARDUINO CONFIG
    #self.port = '/dev/ttyACM0'
    port = "COM7"
    arduino = serial.Serial(port,9600,timeout=5)
    time.sleep(2)
    
    azimuth = sys.argv[1]
    elevation = sys.argv[2]

    print("Received: AZIMUTH {}, ELEVATION {}".format(azimuth, elevation))

    sum = azimuth + elevation
    arduino.write(str(sum))
    time.sleep(0.05)
    msg = arduino.read(arduino.inWaiting()) # read all characters in buffer
    print ("Message from Arduino: {}".format(msg))

    print("Ending program...")
    print("END")

if __name__ == "__main__":
    main()