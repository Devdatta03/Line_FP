#include <AutoPID.h>

//Sensor input
//PINB input directly taken
//Motor OUTPUT
//A-Left Motor
//B-Right Motor
int mApin1=3;//HIGH-Forward
int mApin2=4;//HIGH-Backward
int mBpin1=7;//HIGH-Forward
int mBpin2=2;//HIGH-Backward
int mAspeed=5;
int mBspeed=6;// PWM speed pin
int pwm_speed1=100;
int pwm_speed2=100;
int sense4=8;
int sense3=9;
int sense1=11;
int sense0=12;

double input,output,setpoint=0,Kp,Ki,Kd;

AutoPID linePID(&input,&setpoint,&output,0,255,Kp,Ki,Kd);

void setup() {
  pinMode(mApin1,OUTPUT);
  pinMode(mApin2,OUTPUT);
  pinMode(mBpin1,OUTPUT);
  pinMode(mBpin2,OUTPUT);

  pinMode(sense4,OUTPUT);
  pinMode(sense3,OUTPUT);  
  pinMode(sense1,OUTPUT);
  pinMode(sense0,OUTPUT);
  
  Serial.begin(9600);
}

void inputVal()
{
    input=2*digitalRead(sense4)+digitalRead(sense3)-digitalRead(sense1)-2*digitalRead(sense0);
}
void motion()
{
  Serial.println("motion");
  digitalWrite(mApin1,HIGH);
  digitalWrite(mApin2,LOW);
  digitalWrite(mBpin1,HIGH);
  digitalWrite(mBpin2,LOW);

  analogWrite(mAspeed,pwm_speed1-output);
  analogWrite(mBspeed,pwm_speed2+output);

}

void loop() {
  Serial.println(PINB,BIN);
  //positionCheck();
  inputVal();
  linePID.run();
  Serial.println(output);
  motion();
}
