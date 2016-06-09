//echo serial 2 to computer usb serial
#include <Servo.h>

Servo esc;

int throttle = 0;
int cmd = 0;
int pos = 1000;

void setup() {
  Serial.begin(9600);
  pinMode(7, OUTPUT);
  esc.attach(7, 1000, 2000);
  esc.writeMicroseconds(1500);
  delay(50);
  esc.writeMicroseconds(1000);
  delay(50);
}

void loop() {
  //throttle hook
//  for(pos = 1000; pos <= 1200; pos += 1){ // in steps of 1 microsecond
//    esc.writeMicroseconds(pos);
//    delay(15);
//  } 
//  for(pos = 1200; pos>= 1000; pos-=1){  // goes from 1000 microseconds to 2000 microseconds 
//    esc.writeMicroseconds(pos);
//    delay(15);
//  }
  if (Serial.available())
  {
    cmd = Serial.parseFloat();
    
//    //ease cmd
//    if ((cmd - throttle)>0){
//     throttle++;
//     delay(10);
//    }else if((cmd-throttle)<0){
//     throttle--; 
//     delay(10);
//    }
//    //normalize and scale
//    pos = 1000+throttle/2;
//    
    Serial.println(pos+cmd*5);
    esc.writeMicroseconds(pos+cmd*5);
  }
}
