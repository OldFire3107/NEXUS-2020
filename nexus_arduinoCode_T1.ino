void setup()
{
  pinMode(3,OUTPUT);//motor left +ve
  pinMode(4,OUTPUT);//motor left -ve
  pinMode(5,OUTPUT);//motor right +ve
  pinMode(6,OUTPUT);//motor right +ve
  pinMode(7,OUTPUT);//LED
  Serial.begin(9600);
  while(!Serial){}
}

int incomingData;


void loop()
{
  while(Serial.available()>0)
  {
    incomingData=Serial.read();
   
    if(incomingData=='w'){
      forward();
    }
   
    else if(incomingData=='a'){
      left();
    }
   
    else if(incomingData=='d'){
      right();
    }
/*
 *  else if(incomingData=='t'){
 *    digitalWrite(7,HIGH);
 *    delay(1000);
 *    digitalWrite(7,LOW);
 *    left();
 *  }
 *    
 *  else if(incomingData=='r'){
 *    digitalWrite(7,HIGH);
 *    delay(1000);
 *    digitalWrite(7,LOW);
 *    delay(1000);
 *    digitalWrite(7,HIGH);
 *    delay(1000);
 *    digitalWrite(7,LOW);
 *    right();
 *  }
*/
  }
}


void forward()
{
  digitalWrite(3,HIGH);
  digitalWrite(5,HIGH);
  digitalWrite(4,LOW);
  digitalWrite(6,LOW);
}

void left()
{
  digitalWrite(3,LOW);
  digitalWrite(5,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(6,LOW);
}

void right()
{
  digitalWrite(3,HIGH);
  digitalWrite(5,LOW);
  digitalWrite(4,LOW);
  digitalWrite(6,HIGH);
}
