#define UP 40 //五向按键
#define DOWN 42
#define LEFT 44
#define RIGHT 46
#define MID 48
#define SET 50
#define RST 52

void setup(){
  for(int i=40;i<=52;i+=2){pinMode(i,INPUT_PULLUP);}
  Serial.begin(9600);
}
void loop(){
  for(int i=40;i<=52;i+=2){
    if(!digitalRead(i))//被按下是低电平
    {
      if(i==40){Serial.println("RST");}
      if(i==42){Serial.println("SET");}
      if(i==44){Serial.println("MID");}
      if(i==46){Serial.println("RIGHT");}
      if(i==48){Serial.println("LEFT");}
      if(i==50){Serial.println("DOWN");}
      if(i==52){Serial.println("UP");}
    while(!digitalRead(i)){;}
  }
  delay(20);
  }
}
