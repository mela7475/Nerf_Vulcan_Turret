#include <SPI.h>  
#include <Pixy.h>
#include <Servo.h>

Pixy pixy;
Servo servoX;
Servo servoY;

uint16_t blocks;
int newX, newY;
const byte ledPin = 4;
const byte interruptPin = 2;
volatile byte state = LOW;

void setup(){
  servoX.attach(5);//PWN digital pins
  servoY.attach(3);//PWN digital pins
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING);
  Serial.begin(9600);

  pixy.init();
}

void loop(){ 
  blocks = pixy.getBlocks();
  if (blocks){ //checks blcoks on pixycam
    newX = map(pixy.blocks[0].x,0,319,0,180);//0,180
    newY = map(pixy.blocks[0].y,0,199,556,2420);//556,2420
    servoX.write(newX);//Range 0,180
    servoY.write(newY);//Range 556,2420
    digitalWrite(ledPin, state);
    //Serial.println(newX);
    //Serial.println(newY);
    delay(1);// short delay for servos
  }
}

void blink() {
  static unsigned long last_interrupt_time2 = 0;
  unsigned long interrupt_time2 = millis();
  if(interrupt_time2 - last_interrupt_time2 > 200){
     state = !state;
     Serial.println("change");
  }
  last_interrupt_time2 = interrupt_time2;
}
