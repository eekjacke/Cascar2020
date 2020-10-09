//#define _CAR_DEBUG_ 

unsigned long RX_last;
const int Throttle_zero = 3000;
const int Steering_zero = 3000;
const int Throttle_endpoint = 600;
const int Steering_endpoint = 600;

const uint16_t RX_timeout = 1000;     //milliseconds

const byte interruptPin_Left = 2;   //Hallsensor left pin 2
const byte interruptPin_Right = 3;  //Hallsensor right pin 3

volatile unsigned long Left_time_last = 0;
volatile unsigned long Right_time_last = 0;

long Rticks = -1;
long Lticks = -1;

int Rcnt=0;
int Lcnt=0;

void setup()
{
  Serial.begin(115200);
  pinMode(9, OUTPUT);    //throttle
  pinMode(10, OUTPUT);   //steering
  pinMode(interruptPin_Left, INPUT_PULLUP);   
  pinMode(interruptPin_Right, INPUT_PULLUP);  
  
  attachInterrupt(digitalPinToInterrupt(interruptPin_Left), Interrupt_Left, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin_Right), Interrupt_Right, FALLING);
  
  // Setup for timers
  TCCR1A =((1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)); 
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11); 
  OCR1A = Throttle_zero; //D9 Throttle
  OCR1B = Steering_zero; //D10 Steering
  ICR1 = 40000; //50hz freq
  RX_last = millis();  
}

void loop() 
{
  if(millis() > RX_last+RX_timeout) {
    OCR1A = Throttle_zero; //D9 Throttle
    OCR1B = Steering_zero; //D10 Steering
  }
  
  if( Lticks > 0 ) {
    Serial.print("L;");
    #ifdef _CAR_DEBUG_
      Serial.print(Lticks);
      Serial.print(";");
      Serial.println(Lcnt);
      Lcnt = Lcnt + 1;
      if( Lcnt >= 100 ) {
        Lcnt = 0;
      }
    #else
      Serial.println(Lticks);
    #endif
    Lticks = -1;
  }
  if( Rticks > 0 ) {
    Serial.print("R;");
    #ifdef _CAR_DEBUG_
      Serial.print(Rticks);
      Serial.print(";");
      Serial.println(Rcnt);
      Rcnt = Rcnt + 1;
      if( Rcnt >= 100 ) {
        Rcnt = 0;
      }
    #else
      Serial.println(Rticks);
    #endif
    Rticks = -1;
  }
  
  if(Serial.available() > 0) {
    String CMD = Serial.readStringUntil(';');
    if( CMD == "PING" ) {
//      if(Serial.read() == '\r') {
        Serial.println("PONG;");
//     } else {
//        Serial.println("NO_CR;");
//     }
    }
    
    if (CMD == "T") { //Throttle
      int Throttle = Serial.parseInt();
      Throttle = constrain(Throttle, -100, 100);
      if(Serial.read() == '\r') { 
    	  // Serial.print("T;");
	      // Serial.println(Throttle);  
	      if (Throttle > 0) Throttle = Throttle+66;
	      if (Throttle < 0) Throttle = Throttle-95;

       // 
	      OCR1A = Throttle_zero + Throttle; 
	      RX_last = millis();
      }
    } 
    
    if(CMD == "S") {  //Steering
      int Steering = Serial.parseInt();    
      Steering = constrain( Steering, -100, 100);
      if(Serial.read() == '\r') {
	      //Serial.print("S;");
	      //Serial.println(Steering);
	      OCR1B = Steering_zero + (Steering*Steering_endpoint/100);
	      RX_last = millis();
      }
    } 
  }
}

void Interrupt_Left()
{
  unsigned long Left_time_now=micros();
  unsigned long Left_rotation=Left_time_now-Left_time_last;
   
  Left_time_last=Left_time_now;
  Lticks = Left_rotation;
}

void Interrupt_Right()
{
  unsigned long Right_time_now=micros(); 
  unsigned long Right_rotation=Right_time_now - Right_time_last;
  
  Right_time_last=Right_time_now;
  Rticks = Right_rotation;
}
