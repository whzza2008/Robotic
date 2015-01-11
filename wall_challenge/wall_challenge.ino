const int EN = 9;
const int EN2 = 11;
const int MC1 = 6;
const int MC2 = 7;
const int MC3 = 10;
const int MC4 = 12;
const int R_Ref = A1;//<700
const int L_Ref = A0;//<650
const int F_sensor = A2;
const int B_sensor = A3;
const int trigPin = 3;
const int echoPin = 2;
const int trigPinL=5;
const int echoPinL =4;
int val_F =0;
int val_B =0;
int val_LRef = 0;
int val_RRef = 0;
//int left = 0;
//int right = 0;
//int velocity =0;
void setup()
{
  Serial.begin(9600);
  pinMode(EN,OUTPUT);
  pinMode(EN2,OUTPUT);
  pinMode(MC1,OUTPUT);
  pinMode(MC2,OUTPUT);
  pinMode(MC3,OUTPUT);
  pinMode(MC4,OUTPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  pinMode(trigPinL,OUTPUT);
  pinMode(echoPinL,INPUT);
  brake();
  delay(5000);
}

void loop(){
  
 // Serial.println();
//  delay(500);
  
 val_F = analogRead(F_sensor);
 val_B = analogRead(B_sensor);
 val_LRef = analogRead(L_Ref);
 val_RRef = analogRead(R_Ref);
 Serial.println(echoLeft());
 //delay(500);
  if(val_F>240){
   MoveRight();
   delay(10);
   MoveRight();
  }
 else if (echoLeft() < 13){
   if (echoLeft()<8){
     MoveToRight();
   }
   else{
     forward(255);
   }
 }
 else{
   MoveToLeft();
 }
 

 //front of the car on the white line
// if(val_LRef < 650 && val_RRef < 700){
//     backward(255);
//   if(val_B < 80){
//     MoveLeft();
//     delay(1);
//     MoveLeft();
//   }

//   if(echoRight() < 15){
//     MoveRight();
//   }
//   if(echoLeft() < 15){
//     MoveLeft();
//   }else{
//     MoveLeft();
//   }
// }
 // main push
// if(val_F > 50 ){
//    forward(255);
//    val_F = analogRead(F_sensor);
//    
//    if(val_F > 80){
//      forward(255);
//      val_F = analogRead(F_sensor);
//      
//    } 
//    while(val_F > 100){
//      forward(255);
//      val_F = analogRead(F_sensor);
//    }
// }
// if(echoRight() < 50){
//   MoveToRight();
// }
// if(echoLeft() < 50){
//   MoveToLeft();
// }
// if(val_B > 70){
//   MoveLeft();
//   delay(1);
//   MoveLeft();
// }
}
void reverse(int rate)
{
  digitalWrite(EN,LOW);
  digitalWrite(EN2,LOW);
  digitalWrite(MC2,LOW);
  digitalWrite(MC1,HIGH);
  digitalWrite(MC3,LOW);
  digitalWrite(MC4,HIGH);
  analogWrite(EN,rate);
  analogWrite(EN2,rate);
}
void forward(int rate)
{
  digitalWrite(EN,LOW);
  digitalWrite(EN2,LOW);
  digitalWrite(MC1,LOW);
  digitalWrite(MC2,HIGH);
  digitalWrite(MC4,LOW);
  digitalWrite(MC3,HIGH);
  analogWrite(EN,rate);
  analogWrite(EN2,rate);
}
void brake()
{
  digitalWrite(EN,LOW);
  digitalWrite(EN2,LOW);
  digitalWrite(MC1,LOW);
  digitalWrite(MC2,LOW);
  digitalWrite(MC3,LOW);
  digitalWrite(MC4,LOW);
  digitalWrite(EN,HIGH);
  digitalWrite(EN2,HIGH);
}

void MoveLeft()
{
  digitalWrite(EN,LOW);
  digitalWrite(EN2,LOW);
  digitalWrite(MC1,LOW);
  digitalWrite(MC2,HIGH);
  digitalWrite(MC3,LOW);
  digitalWrite(MC4,HIGH);
  analogWrite(EN,255);
  analogWrite(EN2,255);
}

void MoveRight()
{
 digitalWrite(EN,LOW);
  digitalWrite(EN2,LOW);
  digitalWrite(MC2,LOW);
  digitalWrite(MC1,HIGH);
  digitalWrite(MC4,LOW);
  digitalWrite(MC3,HIGH);
  analogWrite(EN,255);
  analogWrite(EN2,255);
}
void MoveToLeft()
{
   digitalWrite(EN,LOW);
  digitalWrite(EN2,LOW);
  digitalWrite(MC1,LOW);
  digitalWrite(MC2,HIGH);
  digitalWrite(MC4,LOW);
  digitalWrite(MC3,HIGH);
  analogWrite(EN,255);
  analogWrite(EN2,90);
}
void MoveToRight()
{
  digitalWrite(EN,LOW);
  digitalWrite(EN2,LOW);
  digitalWrite(MC1,LOW);
  digitalWrite(MC2,HIGH);
  digitalWrite(MC4,LOW);
  digitalWrite(MC3,HIGH);
  analogWrite(EN,90);
  analogWrite(EN2,255);
}
double echoRight()
{
  double duration,distance;
  digitalWrite(trigPin,HIGH);
  delay(1);
  digitalWrite(trigPin,LOW);
  duration = pulseIn(echoPin,HIGH);
  distance = (duration/2)/29.1;
  return distance;
}
double echoLeft()
{
  double duration,distance;
  digitalWrite(trigPinL,HIGH);
  delay(1);
  digitalWrite(trigPinL,LOW);
  duration = pulseIn(echoPinL,HIGH);
  distance = (duration/2)/29.1;
  return distance;
}
