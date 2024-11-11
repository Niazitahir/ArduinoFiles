#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
#include "extra.h"
// extern float fh = 0, fp = 0, fr = 0;
// extern float IMUout[3];

Servo ESC;     // create servo object to control the ESC
Servo ESC2;
int redpin = 14; //select the pin for the red LED
int bluepin =15; // select the pin for the blue LED
int greenpin =16;// select the pin for the green LED


RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

void setup() {
  pinMode(2, OUTPUT);
  pinMode(redpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(10, OUTPUT);
  Wire.begin();

  Serial.begin(9600);
  delay(100);


  if(!radio.begin()){
    analogWrite(redpin, 255);
    analogWrite(greenpin, 128);
    analogWrite(bluepin, 0);
    Serial.println("Radio Failed. Womp Womp");
    while(true){}
  }
  radio.openReadingPipe(0, address);
  radio.setPALevel(HIGH);
  radio.startListening();
  radio.setAutoAck(0);
  radio.setDataRate(RF24_2MBPS); // Set the speed of the transmission to the quickest available
  radio.setChannel(124); // Use a channel unlikely to be used by Wifi, Microwave ovens etc
  radio.setPALevel(RF24_PA_MAX); // Set radio Tx power to MAX
  if(radio.isChipConnected()){
    Serial.println("Radio Reciever Module Active");
  }
  else{
    Serial.println("Failed to connect");
    while(1){}
  }
  //esc1 top two
  ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  //esc2 bottom two
  ESC2.attach(6,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 

  analogWrite(greenpin, 255);
  analogWrite(bluepin, 0);
  analogWrite(redpin, 0);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(2, HIGH);
  delay(10);
  digitalWrite(2, LOW);
  setupIMU();
  delay(10);
  calibrate();
}

void loop() {

  if (radio.available()) {
    Serial.println("Recieving...");
    uint8_t text[10];
    radio.read(&text, sizeof(text));
    //Serial.println(text[0]);
    m1 = map(text[0], 0, 127, 0, 180);
    m2 = map(text[2], 0, 127, 0, 180);
    //sscanf(text, "%d", &finalThrust);
    ESC.write(m1);    // Send the signal to the ESC
    ESC2.write(m2);    // Send the signal to the ESC
    Serial.println(m1);
    Serial.println(m2);
  }
  IMU();
  delay(10);
  Serial.print("=");
}