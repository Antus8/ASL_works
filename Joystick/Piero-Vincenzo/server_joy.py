#!/usr/bin/python
import time
import serial
import syslog
import socket
import rospy
from sensor_msgs.msg import Joy


class JoyServer():

    def __init__(self):
        self.HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        self.PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

        # ARDUINO CONFIG
        self.port = '/dev/ttyACM0'
        self.arduino = serial.Serial(self.port,9600,timeout=5) # TODO: timeout? 
        time.sleep(2) # wait for Arduino

        #self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self.s.bind((HOST, PORT))
        #self.s.listen(3)
        #self.conn, self.addr = s.accept()
        self.value = 0

        self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
    


    def joy_callback(self, msg):
        self.arduino.flush()
        #print("BUTTONS")
        #print(msg.buttons)
        #print("AXES")
        print("MSG")
        print(msg)
        print("AXES")
        print(msg.axes)

        roll = msg.axes[0]  #left/right
        pitch = msg.axes[1]  #forward/backward
        speed = 0

        if(float(roll) == 0.0 and float(pitch) == 0.0):
            print("STOP")
            self.value = 0

        elif(abs(float(roll)) > abs(float(pitch))):
            if(float(roll) > 0.2):

                print("Moving LEFT")
                if(roll != 1):
                    speed = round(float(roll), 4)
                self.value = 4 + speed

            elif(float(roll) < -0.2):
                print("Moving RIGHT")
                if(roll != -1):
                    speed = round(float(roll), 4)
                self.value = 2 + (speed * -1)
        else:
            if(float(pitch) > 0.2):
                print("Moving FORWARD")
            
                if(pitch != 1):
                    speed = round(float(pitch), 4)
                self.value = 1 + speed


            elif(float(pitch) < -0.2):
                print("Moving BACKWARD")
                if(pitch != -1):
                    speed = round(float(pitch), 4)
                self.value = 3 + (speed * -1)

        #rospy.loginfo("Sending:")
        #rospy.loginfo(str(self.value))
        self.arduino.write(str(self.value))
        time.sleep(0.05)
        #msg = self.arduino.read(self.arduino.inWaiting()) # read all characters in buffer
        #print ("Message from Arduino: " + msg)

def main():
    rospy.init_node('joystick_reader',anonymous=True)

    server = JoyServer()
    rospy.loginfo("JOYSTICK READER READY!")
    
    rospy.spin()

if __name__ == "__main__":
    main()




"""
while True:
    arduino.flush()
    data = conn.recv(1024)
    
    #print(data)
    
    data_selected = data.split(";")[0]
    data_splitted = data_selected.split("/")
    if len(data_splitted) > 4:
        throttle = data_splitted[0]
        roll = data_splitted[1]
        pitch = data_splitted[2]
        yaw = data_splitted[3]
        aux = data_splitted[4]

        # print("Throttle: " + str(throttle) + " Roll: " + str(roll) + " Pitch: " + str(pitch) + " Yaw: " + str(yaw) + " Aux: " + str(aux))

        
        if(float(roll) == 0.0 and float(pitch) == 0.0):
            print("STOP")
            value = 0
        elif(abs(float(roll)) > abs(float(pitch))):
            if(float(roll) > 0.2):
                print("Moving RIGHT")
                value = 2
            elif(float(roll) < -0.2):
                print("Moving LEFT")
                value = 4
        else:
            if(float(pitch) > 0.2):
                print("Moving FORWARD")
                value = 1
            elif(float(pitch) < -0.2):
                print("Moving BACKWARD")
                value = 3

        start = time.time()
        arduino.write(str(value))
        time.sleep(0.05)
        end = time.time()
        print("Took: " + str(end-start))

        msg = arduino.read(arduino.inWaiting()) # read all characters in buffer
        print ("Message from Arduino: " + msg)

"""
