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
int val_R=0;
int val_L =0;
int left = 3;
int store=0;

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

void loop()
{
 val_F = analogRead(F_sensor);
 val_B = analogRead(B_sensor);
 val_R = analogRead(R_Ref);
 val_L = analogRead(L_Ref);
 Serial.println(val_L);
 delay(500);
 
 if ((val_R<650)||(val_L <500)){
   if(val_R<630){MoveLeft();delay(400);}
   else{MoveRight();delay(400);}
 }
 //---------------------------------------
// if(false){;}
 //else ----------------------------------------------------
 else{
  if(val_F>100){
       val_F=analogRead(F_sensor);
       brake();
       delay(300);
       store = analogRead(F_sensor);
       if((store - val_F)>20){
           MoveRight();
           delay(400);
           forward(255);
          delay(100);}
       else{ 
         while(val_F>100){
         if((val_R<630)||(val_L<590)){break;}
         forward(255);
         delay(100);
         val_F=analogRead(F_sensor);
         val_R = analogRead(R_Ref);
         val_L = analogRead(L_Ref);}
   }
 }
   else if(val_B>100){
     val_B=analogRead(B_sensor);
     brake();
     delay(100);
     store = analogRead(B_sensor);
     if((store - val_B)>20){
       
       MoveRight();
       delay(400);
       forward(255);}
     else{
       while(val_F>100){
         if((val_R<630)||(val_L<590)){break;}
         MoveLeft();
         delay(500);
         val_F=analogRead(F_sensor);
         val_R = analogRead(R_Ref);
         val_L = analogRead(L_Ref);}
     }
   } 
  
  //in else(for two side sensor)--------------------------------
   else if(echoLeft()<40){MoveToLeft();delay(400); left = 1;}
   else if(echoRight()<40){MoveToRight();delay(400);left = 0;}
   else{
     if(left ==1){MoveLeft();delay(400);}
     else if(left ==3){brake();}
     else{MoveRight();delay(400);}  
 }
 //two sensor end--------------------------------------------
}
 //else loop end-----------------------------------------------------
}
   

void reverse(int rate){
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
