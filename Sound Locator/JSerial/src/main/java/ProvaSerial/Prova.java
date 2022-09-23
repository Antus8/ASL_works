package ProvaSerial;

import com.fazecast.jSerialComm.*;
import java.util.*;

public class Prova {
  public static void main(String[] args) { 
  Scanner console = new Scanner(System.in);
  System.out.println("List COM ports");
  SerialPort comPorts[] = SerialPort.getCommPorts();
  for (int i = 0; i < comPorts.length; i++)   
          System.out.println("comPorts[" + i + "] = " + comPorts[i].getDescriptivePortName());
  int port = 0;     // array index to select COM port
  comPorts[port].openPort();
  System.out.println("open port comPorts[" + port + "]  " + comPorts[port].getDescriptivePortName());
  comPorts[port].setBaudRate(115200);
  try {
    while (true)
    {
      // if keyboard token entered read it
      if(System.in.available() > 0)
           {
           //System.out.println("enter chars ");
           String s = console.nextLine() + "\n";                // read token
           byte[] writeBuffer=s.getBytes() ;
           comPorts[port].writeBytes(writeBuffer, writeBuffer.length);
           //System.out.println("write " + writeBuffer.length);
          }
     // read serial port  and display data
      while (comPorts[port].bytesAvailable() > 0)
          {
          byte[] readBuffer = new byte[comPorts[port].bytesAvailable()];
          int numRead = comPorts[port].readBytes(readBuffer, readBuffer.length);
          //System.out.print("Read " + numRead + " bytes from COM port: ");
          for (int i = 0; i < readBuffer.length; i++)   
             System.out.print((char)readBuffer[i]);
          //System.out.println();
          }
     }
  } catch (Exception e) { e.printStackTrace(); }
  comPorts[port].closePort();  
}
}

