#include <SabertoothSimplified.h>
#include <stdlib.h>

#define motor_full_speed_fw 127
#define motor_full_speed_bw -127

#define motor_smoothed_speed_fw -30
#define motor_smoothed_speed_bw 30
/*
#define slow_forward_right 65
#define slow_forward_left 193
#define slow_backward_right 63
#define slow_backward_left 191

#define fast_forward_right 127
#define fast_forward_left 255
#define fast_backward_right 1
#define fast_backward_left 128
*/

#define stop_motors 0

String readS;
float readF;  //the float convertion of the string
unsigned char buffer[6];

SabertoothSimplified ST;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13,OUTPUT);
  SabertoothTXPinSerial.begin(9600);
}


void loop()
{
  while (Serial.available()){
    delay(1);  //delay to allow buffer to fill 
    if (Serial.available() >0){
      if(Serial.readBytes(buffer, sizeof(float)) == sizeof(float)){
        memcpy(&readF, buffer, sizeof(float));
      } 
    }
  }
  
  Serial.println(readF);
    
  //if (readS.length() >0){
    if(readS == 0,00){
      stop();
    }
    else{
    if(readF >= 1 && readF < 2){
      digitalWrite(13,HIGH);
      delay(50); 
      digitalWrite(13,LOW);
      move_forward(readF);
    }
    else{
      if(readF >= 2 && readF < 3){
        digitalWrite(13,HIGH);
        delay(50);
        digitalWrite(13,LOW);
        move_right(readF);
        
      }
      else{
        if(readF >= 3 && readF < 4){
          digitalWrite(13,HIGH);
          delay(50);
          digitalWrite(13,LOW);
          move_backward(readF);
        }
        else{
          if(readF >= 4){
            digitalWrite(13,HIGH);
            delay(50);
            digitalWrite(13,LOW);
            move_left(readF);
          }
        }
      }
    }
    }
    //}
    
    
  readS = "";
  Serial.flush();
    /*
    if(readString == "1"){
      move_forward();
    }
    elif()
    Serial.print("Arduino received: ");
    Serial.print(readS); //see what was received*/
    
  }



void move_forward(float speed){

  speed -= 1;  //Ex. From 1.27 to 0.27
  
  ST.motor(1,motor_smoothed_speed_fw * speed);
  ST.motor(2,motor_smoothed_speed_fw * speed);

  long a=motor_smoothed_speed_fw * speed;
    
  Serial.write("Avanti :");
  Serial.write(a);

}

void move_right(float speed){
  
  speed -= 2;

  ST.motor(1,motor_smoothed_speed_fw * speed);
  ST.motor(2,motor_smoothed_speed_bw * speed);

  long a=motor_smoothed_speed_fw * speed;
  long b=motor_smoothed_speed_bw * speed;
  
  Serial.write("Destra 1 :");
  Serial.write(a);
  Serial.write("\nDestra 2:");
  Serial.write(b);
}

void move_left(float speed){

  speed -= 4;

  ST.motor(1,motor_smoothed_speed_bw * speed);
  ST.motor(2,motor_smoothed_speed_fw * speed);


  long a=motor_smoothed_speed_fw * speed;
  long b=motor_smoothed_speed_bw * speed;
  
  Serial.write("Sinistra 1 :");
  Serial.write(a);
  Serial.write("\nSinistra 2:");
  Serial.write(b);
}

void move_backward(float speed){

  speed -= 3;
  
  ST.motor(1,motor_smoothed_speed_bw * speed);
  ST.motor(2,motor_smoothed_speed_bw * speed);

  long a=motor_smoothed_speed_bw * speed;

  Serial.print(speed);
  Serial.write("Dietro :");
  Serial.write(a);
 }

void stop(){
  ST.motor(1,0);
  ST.motor(2,0);
}
