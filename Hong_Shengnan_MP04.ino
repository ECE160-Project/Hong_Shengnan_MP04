/*
  Hong_Shengnan_MP04.ino

  PROJECT NAME
  Wireless Communication
  
  PROGRAM DESCRIPTION
  This program uses IR sensor and sonar sensor to detect
  the object. It has has a wired and wireless connection
  with joystick to control the servo and LED with shift
  register.
  
  DATA CREATED
  X,Y,Z axis value of the joystick.
  The distance between sonar sensor and object.
  The distance between IR sensor and object.

  KEY FUNCTION
  setup();
  loop();
  read_sensor();
  wired_test();
  wireless_test();
  shiftWrite();
  LeftToRight();
  RightToLeft();

                            Created by Steven Hong
                                        10/02/2017
*/

#include <NewPing.h>
#include <RF24.h>   //wireless transmitter library
#include <Servo.h>  //servo library

//define the pins for all component
#define CE_PIN 7
#define CSN_PIN 8
#define JoyX A2  
#define JoyY A1
#define JoyZ A0
const int sonarPin = A4;
const int irPin = A3;
const int dataPin = 2;
const int clockPin = 3;
const int latchPin = 4;
const int motorPin = 9;
const int servoPin =5;

//Variables to hold the coordinates
int xAxis;
int yAxis;
int zAxis;
int remain;

int delayTime = 50;  // Time (milliseconds) to pause between LEDs
                     // Make this smaller for faster switching

byte data = 0;

#define MAX_DISTANCE 200
NewPing sonar(sonarPin, sonarPin, MAX_DISTANCE);
Servo servo1; //servo control object

const uint64_t pipe = 0xE8E8F0F0E1LL; //transmission pipe
const int CHAN = 32;                   //transmission channel
RF24 radio(CE_PIN, CSN_PIN);          //creating a radio object
int joystick [3]; 

void setup() {
  // put your setup code here, to run once
  pinMode(JoyX, INPUT); //set up pin as input
  pinMode(JoyY, INPUT); //set up pin as input
  pinMode(JoyZ, INPUT_PULLUP);  //set up pin as input_pullup
  pinMode(dataPin, OUTPUT); //set up pin as output
  pinMode(clockPin, OUTPUT);  //setup pin as output
  pinMode(latchPin, OUTPUT);  //setup pin as output
  pinMode(motorPin, OUTPUT);  //setup pin as output
  
  radio.begin();
  radio.setChannel(CHAN); //partners on same channel between 1 & 125
  radio.openReadingPipe(1, pipe); //start writing
  //radio.openWritingPipe(pipe);  //start transmitting
  radio.printDetails();
  radio.startListening(); //start receiving
  
  Serial.begin(9600); //Initialize serial port & set rate to 9600 bits per second (bps)
  //Serial.println("nRF24L01 Receiver Starting..");

  servo1.attach(servoPin, 900, 2100);  //connect the servo to servoPin
                                       //with a minimum pulse width of
                                       //900 and a maximum pulse width
                                       //2100.
}

void loop() {
  // put your main code here, to run repeatedly:
  //read_sensor();
  //wired_test();

  //receiver
  //check to see if there is a radio transmitting data
  if (radio.available()){
    //Read the data until we've received transmitting data
    while (radio.available()){
      //fetch the data that is being transmitted
      //note the usage of the read() function
      radio.read(joystick, sizeof(joystick));

      xAxis = joystick[0];
      yAxis = joystick[1];
      zAxis = joystick[2];
      Serial.print("X value: ");  //print the outcome of joystick
      Serial.print(joystick[0]);
      Serial.print("\tY value: ");
      Serial.print(joystick[1]);
      Serial.print("\tZ value: ");
      Serial.println(joystick[2]);
      wireless_test();
    }
  }
  else {    //nothing is transmitting
    Serial.println("No radio available");
  }
  
  //transmitter
  //radio.stopListening();
  /*
  joystick[0] = analogRead(JoyX); //read the value from joystick
  joystick[1] = analogRead(JoyY);
  joystick[2] = digitalRead(JoyZ);
  
  Serial.print("X value: ");  //print the result of joystick
  Serial.print(joystick[0]);
  Serial.print("\tY value: ");
  Serial.print(joystick[1]);
  Serial.print("\tZ value: ");
  Serial.println(joystick[2]);
  
  radio.write(joystick, sizeof(joystick));  //start transmitting
 */   
}

void read_sensor(){
  
  //print the result of sonar sensor
  Serial.println(sonar.ping_cm());  //read and print sonar in cm
  //Serial.print(" cm\t");          //print units to serial monitor
  delay(10);                      //wait 10 ms for ADC to reset
  //Serial.print(sonar.ping_in());  //read and print sonar in in
  //Serial.println(" in");          //print units to serial monitor
  //delay(10);                      //wait 10 ms for ADC to reset
  
  /*
  //print the result of infrared sensor
  int val = analogRead(irPin);    //read analog input
  //Serial.print("value ");         //print analog value to serial
  Serial.print(val);              //print analog value to serial
  int distance = (2914 / (val + 5)) - 1; //distance in cm
  //Serial.print("\tdistance ");    //print distance label
  Serial.println(distance);         //print distance on serial
  //Serial.print(" cm\t");          //print cm lab
  //Serial.print(distance * 0.39371);//print distance on serial in inches
  //Serial.println(" in");          //print cm lab
  delay(10);                      //wait 10 ms for ADC reset
  */
}

void wired_test(){
  
  xAxis = analogRead(JoyX); //read the value from joystick
  yAxis = analogRead(JoyY);
  zAxis = digitalRead(JoyZ);

  //Serial.println(xAxis);

  if (sonar.ping_cm() >= 25 ) //if sonar does not detect object, run the code
  {
  if (xAxis >= 800)
  {
    LeftToRight();  //illuminate LED from left to the right
    remain = 0;     //record the status
  }
  else if (xAxis <= 200)
  {
    RightToLeft();  //illuminate LED from right to the left
    remain = 1;     //record the status
  }
  else
  {
    if (remain == 0)
    {
      shiftWrite(7, OUTPUT);  //remains on the last LED lit
    }
    else
    {
      shiftWrite(0, OUTPUT);  //remains on the last LED lit
    }
  }

  //Serial.println(zAxis);  //debugging purpose
  if (zAxis == 0)
  {
    analogWrite(motorPin, 255); //run the motor
  }
  else
  {
    analogWrite(motorPin, 0); //stop the motor
  }

  if (yAxis >= 800)
  {
    servo1.write(180);  // Tell servo to go to 180 degree
  }
  else if (yAxis <= 200)
  {
    servo1.write(0);    // Tell servo to go to 0 degree
  }
  else
  {
    servo1.write(90);   // Tell servo to go to 90 degree
  }    
  }
}

void wireless_test(){

  if (sonar.ping_cm() >= 25 ) //if sonar does not detect object, run the code
  {
    if (xAxis >= 800)
  {
    LeftToRight();  //illuminate LEDs from left to the right
    remain = 0;     //record the status
  }
  else if (xAxis <= 200)
  {
    RightToLeft();  //illuminate LEDs from right to the left
    remain = 1;     //record the status
  }
  else
  {
    if (remain == 0)
    {
      shiftWrite(7, HIGH);  //remains on the last LED lit
    }
    else
    {
      shiftWrite(0, HIGH);  //remains on the last LED lit
    }
  }

  //Serial.println(zAxis);  //debugging purpose
  if (zAxis == 0)
  {
    analogWrite(motorPin, 255); //run the motor
  }
  else
  {
    analogWrite(motorPin, 0);   //stop the motor
  }

  if (yAxis >= 800)
  {
    servo1.write(180);  // Tell servo to go to 180 degree
  }
  else if (yAxis <= 200)
  {
    servo1.write(0);    // Tell servo to go to 0 degree
  }
  else
  {
    servo1.write(90);   // Tell servo to go to 90 degree
  }
  }
}

void shiftWrite(int desiredPin, boolean desiredState){
  // This function lets you make the shift register outputs
  // HIGH or LOW in exactly the same way that you use digitalWrite().

    bitWrite(data, desiredPin, desiredState); //Change desired pin to 0 or 1 in "data"

    // Now we'll actually send that data to the shift register.
    // The shiftOut() function does all the hard work of
    // manipulating the data and clock pins to move the data
    // into the shift register;

    shiftOut(dataPin, clockPin, MSBFIRST, data);  //send "data" to the shift register

    //Toggle the latchPin to make "data" appear at the outputs
    digitalWrite(latchPin, HIGH);
    digitalWrite(latchPin, LOW);
}


void LeftToRight(){
  //this function will turn the LEDs on and off, one-by-one, from left to right
  int index;

  // step through the LEDs, from 0 to 7
  for (index = 0; index <= 7; index++)
  {
    shiftWrite(index, HIGH);  //turn LED on
    delay(delayTime);   //pause to slow down the sequence
    shiftWrite(index, LOW);   //turn LED off
  }
}

void RightToLeft(){
  //this function will turn the LEDs on and off, one-by-one, from right to left
  int index;

  // step through the LEDs, from 0 to 7
  for (index = 7; index >= 0; index--)
  {
    shiftWrite(index, HIGH);  //turn LED on
    delay(delayTime);   //pause to slow down the sequence
    shiftWrite(index, LOW);   //turn LED off
  }
}

