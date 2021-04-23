#include <movingAvg.h>
#include <PID_v1.h>
//Arduino Mega 6DOF Controller

//pin definition

int pinmotor1f=30;
int pinmotor1r=31;
int pinmotor2f=32;
int pinmotor2r=33;
int pinmotor3f=34;
int pinmotor3r=35;
int pinmotor4f=36;
int pinmotor4r=37;
int pinmotor5f=38;
int pinmotor5r=39;
int pinmotor6f=40;
int pinmotor6r=41;
int pinmot1pwm=2;
int pinmot2pwm=3;
int pinmot3pwm=4;
int pinmot4pwm=5;
int pinmot5pwm=6;
int pinmot6pwm=7;

int pinpot1=A0;
int pinpot2=A1;
int pinpot3=A2;
int pinpot4=A3;
int pinpot5=A4;
int pinpot6=A5;


//current position variable
double pot1p;
double pot2p;
double pot3p;
double pot4p;
double pot5p;
double pot6p;
double pot2praw;
double pot4praw;
double pot6praw;


double initialposition=60;

//target position
double pot1setp=initialposition;
double pot2setp=initialposition;
double pot3setp=initialposition;
double pot4setp=initialposition;
double pot5setp=initialposition;
double pot6setp=initialposition;


//define moving average variable

double pot1pavg=initialposition;
double pot2pavg=initialposition;
double pot3pavg=initialposition;
double pot4pavg=initialposition;
double pot5pavg=initialposition;
double pot6pavg=initialposition;

double mot1output;
double mot2output;
double mot3output;
double mot4output;
double mot5output;
double mot6output;

double pot1direction;
double pot2direction;
double pot3direction;
double pot4direction;
double pot5direction;
double pot6direction;

int changedirection1=0;
int changedirection2=0;
int changedirection3=0;
int changedirection4=0;
int changedirection5=0;
int changedirection6=0;


char Snd;
int Smesg;
int trash;
int count=0;
int noserialcount=0;

double Kp=1, Ki=0, Kd=0.3;
double Kpup=1, Kiup=0, Kdup=0.3;

PID pot1PIDP(&pot1pavg, &mot1output, &pot1setp, Kpup, Kiup, Kdup, DIRECT);
PID pot1PIDR(&pot1pavg, &mot1output, &pot1setp, Kp, Ki, Kd, REVERSE);
PID pot2PIDP(&pot2pavg, &mot2output, &pot2setp, Kpup, Kiup, Kdup, DIRECT);
PID pot2PIDR(&pot2pavg, &mot2output, &pot2setp, Kp, Ki, Kd, REVERSE);
PID pot3PIDP(&pot3pavg, &mot3output, &pot3setp, Kpup, Kiup, Kdup, DIRECT);
PID pot3PIDR(&pot3pavg, &mot3output, &pot3setp, Kp, Ki, Kd, REVERSE);
PID pot4PIDP(&pot4pavg, &mot4output, &pot4setp, Kpup, Kiup, Kdup, DIRECT);
PID pot4PIDR(&pot4pavg, &mot4output, &pot4setp, Kp, Ki, Kd, REVERSE);
PID pot5PIDP(&pot5pavg, &mot5output, &pot5setp, Kpup, Kiup, Kdup, DIRECT);
PID pot5PIDR(&pot5pavg, &mot5output, &pot5setp, Kp, Ki, Kd, REVERSE);
PID pot6PIDP(&pot6pavg, &mot6output, &pot6setp, Kpup, Kiup, Kdup, DIRECT);
PID pot6PIDR(&pot6pavg, &mot6output, &pot6setp, Kp, Ki, Kd, REVERSE);

movingAvg pot1sensor(7);
movingAvg pot2sensor(7);
movingAvg pot3sensor(7);
movingAvg pot4sensor(7);
movingAvg pot5sensor(7);
movingAvg pot6sensor(7);

double testbyte1 = 0;
double testbyte2 = 0;
double tmpbyte = 0;


void setup() {

pinMode(pinmotor1f,OUTPUT);
pinMode(pinmotor1r,OUTPUT);
pinMode(pinmotor2f,OUTPUT);
pinMode(pinmotor2r,OUTPUT);
pinMode(pinmotor3f,OUTPUT);
pinMode(pinmotor3r,OUTPUT);
pinMode(pinmotor4f,OUTPUT);
pinMode(pinmotor4r,OUTPUT);
pinMode(pinmotor5f,OUTPUT);
pinMode(pinmotor5r,OUTPUT);
pinMode(pinmotor6f,OUTPUT);
pinMode(pinmotor6r,OUTPUT);

pinMode(pinmot1pwm,OUTPUT);
pinMode(pinmot2pwm,OUTPUT);
pinMode(pinmot3pwm,OUTPUT);
pinMode(pinmot4pwm,OUTPUT);
pinMode(pinmot5pwm,OUTPUT);
pinMode(pinmot6pwm,OUTPUT);

digitalWrite(pinmotor1f, LOW);
digitalWrite(pinmotor1r, LOW);
digitalWrite(pinmotor2f, LOW);
digitalWrite(pinmotor2r, LOW);
digitalWrite(pinmotor3f, LOW);
digitalWrite(pinmotor3r, LOW);
digitalWrite(pinmotor4f, LOW);
digitalWrite(pinmotor4r, LOW);
digitalWrite(pinmotor5f, LOW);
digitalWrite(pinmotor5r, LOW);
digitalWrite(pinmotor6f, LOW);
digitalWrite(pinmotor6r, LOW);
delay(100);

pot1PIDP.SetMode(AUTOMATIC);
pot1PIDR.SetMode(AUTOMATIC);
pot2PIDP.SetMode(AUTOMATIC);
pot2PIDR.SetMode(AUTOMATIC);
pot3PIDP.SetMode(AUTOMATIC);
pot3PIDR.SetMode(AUTOMATIC);
pot4PIDP.SetMode(AUTOMATIC);
pot4PIDR.SetMode(AUTOMATIC);
pot5PIDP.SetMode(AUTOMATIC);
pot5PIDR.SetMode(AUTOMATIC);
pot6PIDP.SetMode(AUTOMATIC);
pot6PIDR.SetMode(AUTOMATIC);

pot1sensor.begin();
pot2sensor.begin();
pot3sensor.begin();
pot4sensor.begin();
pot5sensor.begin();
pot6sensor.begin();

updatepotsensor();

Serial.begin(115200,SERIAL_8N1);

        
for (int i=1;i<60;i++){
    motcontroller();
    
}



}

void loop() {

//Serial.print(pot1pavg);  
//Serial.print(",");
//Serial.print(pot2pavg); 
//Serial.print(",");
//Serial.print(pot3pavg); 
//Serial.print(",");
//Serial.print(pot4pavg); 
//Serial.print(",");
//Serial.print(pot5pavg); 
//Serial.print(",");
//Serial.println(pot6pavg);    
//updatepotsensor();

    while (Serial.available() > 0){
      readserial();
      motcontroller();
      emergency();
      noserialcount=0;
     }

 
    noserialcount++;
    delay(10);

    if (noserialcount==200){
            pot1setp=45;
            pot2setp=45;
            pot3setp=45;
            pot4setp=45;
            pot5setp=45;
            pot6setp=45;
            for (int i=1;i<60;i++){
                motcontroller();
                emergency();
      }
    }

}
void updatepotsensor(){
  pot1p=analogRead(pinpot1)>>2;
  pot2praw=analogRead(pinpot2)>>2;
  pot3p=analogRead(pinpot3)>>2;
  pot4praw=analogRead(pinpot4)>>2;
  pot5p=analogRead(pinpot5)>>2;
  pot6praw=analogRead(pinpot6)>>2;

  pot2p=255-pot2praw;
  pot4p=255-pot4praw;
  pot6p=255-pot6praw;

  

  pot1pavg=pot1sensor.reading(pot1p);
  pot2pavg=pot2sensor.reading(pot2p);
  pot3pavg=pot3sensor.reading(pot3p);
  pot4pavg=pot4sensor.reading(pot4p);
  pot5pavg=pot5sensor.reading(pot5p);
  pot6pavg=pot6sensor.reading(pot6p);
  
}


void readserial(){
  testbyte1 = Serial.read();
  if (testbyte1 == 65){
    testbyte2 = Serial.read();
    if (testbyte2 ==66){
      Smesg=Serial.read();
      pot1setp=Serial.read();
      pot2setp=Serial.read();
      pot3setp=Serial.read();
      pot4setp=Serial.read();
      pot5setp=Serial.read();
      pot6setp=Serial.read();
        
        pot1setp=min(190,pot1setp);
        pot1setp=max(45,pot1setp);
        pot2setp=min(190,pot2setp);
        pot2setp=max(45,pot2setp);
        pot3setp=min(190,pot3setp);
        pot3setp=max(45,pot3setp);
        pot4setp=min(190,pot4setp);
        pot4setp=max(45,pot4setp);
        pot5setp=min(190,pot5setp);
        pot5setp=max(45,pot5setp);
        pot6setp=min(190,pot6setp);
        pot6setp=max(45,pot6setp);
      
      Snd=Serial.read();
      if (Snd== '\r'){
          Serial.print(pot3pavg);
          Serial.print(",");
          Serial.println(pot4pavg);
//        Serial.print("A");
//        Serial.print(pot1setp);
//        Serial.print("B");
//        Serial.print(pot2setp);
//        Serial.print("C");
//        Serial.print(pot3setp);
//        Serial.print("D");
//        Serial.print(pot4setp);
//        Serial.print("E");
//        Serial.print(pot5setp);
//        Serial.print("F");
//        Serial.println(pot6setp);
        testbyte1=0;
        testbyte2=0;
        Snd=0;
          while (Serial.available()){
            trash = Serial.read();
           }
        
      }
      
    }
  }
}
void testVFD(){
  
}

void pidcal() {
    if (pot1pavg<=pot1setp){
    pot1PIDP.Compute();
    pot1direction = 1;
  }else{
    if (pot1direction == 1){
    pot1PIDR.Compute();
    pot1direction = -1;
    changedirection1 = 1;
    }else{
    pot1PIDR.Compute();
    pot1direction = -1;
    changedirection1 = 0;
    }
  }
// !!!!!!! motor 2 is of a differnet brand and has the phase reversed
  if (pot2pavg>=pot2setp){
    if (pot2direction == -1){
    pot2PIDR.Compute();
    pot2direction = 1;
    changedirection2 = 1;
    }else{
    pot2PIDR.Compute();
    pot2direction = 1;
    changedirection2 = 0;
    }
  }else{
    pot2PIDP.Compute();
    pot2direction = -1;
  }
  
  if (pot3pavg<=pot3setp){
    pot3PIDP.Compute();
    pot3direction = 1;
  }else{
    if (pot3direction == 1){
    pot3PIDR.Compute();
    pot3direction = -1;
    changedirection3 = 1;
    }else {
    pot3PIDR.Compute();
    pot3direction = -1;
    changedirection3 = 0;      
    }
  }
  
  if (pot4pavg<=pot4setp){
    pot4PIDP.Compute();
    pot4direction = 1;
  }else{
    if (pot4direction == 1){
    pot4PIDR.Compute();
    pot4direction = -1;
    changedirection4 = 1;
    }else {
    pot4PIDR.Compute();
    pot4direction = -1;
    changedirection4 = 0;      
    }
  }
  
  if (pot5pavg<=pot5setp){
    pot5PIDP.Compute();
    pot5direction = 1;
  }else{
    if (pot5direction == 1){
    pot5PIDR.Compute();
    pot5direction = -1;
    changedirection5 = 1;
    }else {
    pot5PIDR.Compute();
    pot5direction = -1;
    changedirection5 = 0;      
    }
  }
  
  if (pot6pavg<=pot6setp){
    pot6PIDP.Compute();
    pot6direction = 1;
  }else{
    if (pot6direction == 1){
    pot6PIDR.Compute();
    pot6direction = -1;
    changedirection6 = 1;
    }else {
    pot6PIDR.Compute();
    pot6direction = -1;
    changedirection6 = 0;      
    }
  }
  
  if (pot1direction==-1){
    digitalWrite(pinmotor1r, HIGH);
  }else if (pot1direction==1){
    digitalWrite(pinmotor1f,HIGH);  
  }  
  if (pot2direction==-1){
    digitalWrite(pinmotor2r, HIGH);
  }else if (pot2direction==1){
    digitalWrite(pinmotor2f,HIGH);  
  }  
  if (pot3direction==-1){
    digitalWrite(pinmotor3r, HIGH);
  }else if (pot3direction==1){
    digitalWrite(pinmotor3f,HIGH);  
  }  
  if (pot4direction==-1){
    digitalWrite(pinmotor4r, HIGH);
  }else if (pot4direction==1){
    digitalWrite(pinmotor4f,HIGH);  
  }  
  if (pot5direction==-1){
    digitalWrite(pinmotor5r, HIGH);
  }else if (pot5direction==1){
    digitalWrite(pinmotor5f,HIGH);  
  }  
  if (pot6direction==-1){
    digitalWrite(pinmotor6r, HIGH);
  }else if (pot6direction==1){
    digitalWrite(pinmotor6f,HIGH);  
  }
}

void motcontroller(){

while (count<200){
  if (count % 10 ==0) {
    updatepotsensor();
  }
  pidcal();
//if (changedirection1 == 1){
//  analogWrite(pinmot1pwm, 2);
//}
//if (changedirection2 == 1){
//  analogWrite(pinmot2pwm, 2);
//}
//if (changedirection3 == 1){
//  analogWrite(pinmot3pwm, 2);
//}
//if (changedirection4 == 1){
//  analogWrite(pinmot4pwm, 2);
//}
//if (changedirection5 == 1){
//  analogWrite(pinmot5pwm, 2);
//}
//if (changedirection6 == 1){
//  analogWrite(pinmot6pwm, 2);
//}
//if ((changedirection1 == 1) || (changedirection2 == 1) || (changedirection3 == 1) || (changedirection4 == 1) || (changedirection5 == 1) || (changedirection6 == 1)){
//  changedirection1 = 0;
//  changedirection2 = 0;
//  changedirection3 = 0;
//  changedirection4 = 0;
//  changedirection5 = 0;
//  changedirection6 = 0;
//}
//mot1output=min(150,mot1output);
//mot2output=min(150,mot2output);
//mot3output=min(150,mot3output);
//mot4output=min(150,mot4output);
//mot5output=min(150,mot5output);
//mot6output=min(150,mot6output);
//
//if (mot1output<2){
//  digitalWrite(pinmotor1f, LOW);
//  digitalWrite(pinmotor1r, LOW);
//}
//if (mot2output<2){
//  digitalWrite(pinmotor2f, LOW);
//  digitalWrite(pinmotor2r, LOW);
//}
//if (mot3output<2){
//  digitalWrite(pinmotor3f, LOW);
//  digitalWrite(pinmotor3r, LOW);
//}
//if (mot4output<2){
//  digitalWrite(pinmotor4f, LOW);
//  digitalWrite(pinmotor4r, LOW);
//}
//if (mot5output<2){
//  digitalWrite(pinmotor5f, LOW);
//  digitalWrite(pinmotor5r, LOW);
//}
//if (mot6output<2){
//  digitalWrite(pinmotor6f, LOW);
//  digitalWrite(pinmotor6r, LOW);
//}
 

analogWrite(pinmot1pwm, mot1output);
analogWrite(pinmot2pwm, mot2output);
analogWrite(pinmot3pwm, mot3output);
analogWrite(pinmot4pwm, mot4output);
analogWrite(pinmot5pwm, mot5output);
analogWrite(pinmot6pwm, mot6output);
count++;
}
count=0;

digitalWrite(pinmotor1f, LOW);
digitalWrite(pinmotor1r, LOW);
digitalWrite(pinmotor2f, LOW);
digitalWrite(pinmotor2r, LOW);
digitalWrite(pinmotor3f, LOW);
digitalWrite(pinmotor3r, LOW);
digitalWrite(pinmotor4f, LOW);
digitalWrite(pinmotor4r, LOW);
digitalWrite(pinmotor5f, LOW);
digitalWrite(pinmotor5r, LOW);
digitalWrite(pinmotor6f, LOW);
digitalWrite(pinmotor6r, LOW);
  
}

void emergency(){
  if (pot1pavg<30 || pot2pavg<30 || pot3pavg<30 || pot4pavg<30 || pot5pavg<30 || pot6pavg<30){
    while (true){
      digitalWrite(pinmotor1f, LOW);
      digitalWrite(pinmotor1r, LOW);
      digitalWrite(pinmotor2f, LOW);
      digitalWrite(pinmotor2r, LOW);
      digitalWrite(pinmotor3f, LOW);
      digitalWrite(pinmotor3r, LOW);
      digitalWrite(pinmotor4f, LOW);
      digitalWrite(pinmotor4r, LOW);
      digitalWrite(pinmotor5f, LOW);
      digitalWrite(pinmotor5r, LOW);
      digitalWrite(pinmotor6f, LOW);
      digitalWrite(pinmotor6r, LOW);
    }
  }
  if (pot1pavg>230 || pot2pavg>230 || pot3pavg>230 || pot4pavg>230 || pot5pavg>230 || pot6pavg>230){
    while (true){
      digitalWrite(pinmotor1f, LOW);
      digitalWrite(pinmotor1r, LOW);
      digitalWrite(pinmotor2f, LOW);
      digitalWrite(pinmotor2r, LOW);
      digitalWrite(pinmotor3f, LOW);
      digitalWrite(pinmotor3r, LOW);
      digitalWrite(pinmotor4f, LOW);
      digitalWrite(pinmotor4r, LOW);
      digitalWrite(pinmotor5f, LOW);
      digitalWrite(pinmotor5r, LOW);
      digitalWrite(pinmotor6f, LOW);
      digitalWrite(pinmotor6r, LOW);    
    }
  }
}
