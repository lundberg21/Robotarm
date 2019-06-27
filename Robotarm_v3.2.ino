/* Robot arm special course

*/

#include <Servo.h>
#include <Math.h>
/* Libary to help find the median of an array
https://github.com/luisllamasbinaburo/Arduino-MedianFilter */
#include "MedianFilterLib.h"


MedianFilter<int> medianFilter(5);

const int buttonPin = 9;        // The pin for the record button
const int pausePin = 10;        // THe pin for the pause button

int buttonState = 0;            // Push button value for the record button
int count = 0;                  // Count value used to position in the recorded degrees i.e. the posServoX[] array
int Measurement[10];            // Measurement array the length determines how many measurements are taken in the Measure function

// Pause function variables
int pauseState = 0;             // Push button value for the pause button
int state = LOW;                // The current state of the output pin
int reading= 0;                 // The current reading from the input pin
int previous = LOW;             // The previous reading from the input pin
long time = 0;                  // The last time the output pin was toggled
   
int rec=0;                      // Value to keep track if the last function was a the record function.

// Settings
const int moveincrement=1;
const int diffSensitivity=2;    // Varible to control when the measurement is a repeat
const int dlay=15;              // Sets the delay between each moved degree in milliseconds
const int recDlay=300;          // Sets the delay between each recorded positions in milliseconds
const long debounce = 200;      // Sets the pause delay time in milliseconds

// Servo 1 bottom
Servo myServo1; 
int Sensor1Pin=A5;              // Set pin for potentiometer
int Servo1Pin=2;                // Set pin for Servo
int servo1Min=55;               // Set minimum value for sensor i.e. potentiometer reading and 0 degree
int servo1Max=600;              // Set maximum value for sensor i.e. potentiometer reading and 180 degree
int posServo1[50] ;             // Variable to store the servo position

// Servo 2 white & grey
Servo myServo2; 
int Sensor2Pin=A4;              // Set pin for potentiometer
int Servo2Pin=3;                // Set pin for Servo
int servo2Min=55;               // Set minimum value for sensor i.e. potentiometer reading and 0 degree
int servo2Max=600;              // Set maximum value for sensor i.e. potentiometer reading and 180 degree
int posServo2[50] ;             // variable to store the servo position

// Servo 3 green
Servo myServo3; 
int Sensor3Pin=A3;              // Set pin for potentiometer
int Servo3Pin=4;                // Set pin for Servo
int servo3Min=55;               // Set minimum value for sensor i.e. potentiometer reading and 0 degree
int servo3Max=600;              // Set maximum value for sensor i.e. potentiometer reading and 180 degree
int posServo3[50] ;             // Variable to store the servo position

// Servo 4 brown
Servo myServo4; 
int Sensor4Pin=A2;              // Set pin for potentiometer
int Servo4Pin=5;                // Set pin for Servo
int servo4Min=55;               // Set minimum value for sensor i.e. potentiometer reading and 0 degree
int servo4Max=600;              // Set maximum value for sensor i.e. potentiometer reading and 180 degree
int posServo4[50] ;             // Variable to store the servo position

// Servo 5 orange
Servo myServo5; 
int Sensor5Pin=A1;              // Set pin for potentiometer
int Servo5Pin=6;                // Set pin for Servo
int servo5Min=105;              // Set minimum value for sensor i.e. potentiometer reading and 0 degree
int servo5Max=395;              // Set maximum value for sensor i.e. potentiometer reading and 180 degree
int posServo5[50];              // Variable to store the servo position

// Servo 6 purple
Servo myServo6; 
int Sensor6Pin=A0;              // Set pin for potentiometer
int Servo6Pin=7;                // Set pin for Servo
int servo6Min=105;              // Set minimum value for sensor i.e. potentiometer reading and 0 degree
int servo6Max=395;              // Set maximum value for sensor i.e. potentiometer reading and 180 degree
int posServo6[50] ;             // Variable to store the servo position



void setup() {
  // initialize the serial console
  Serial.begin(9600);
  Serial.println("Initializing...");
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  // initialize the button pins as an input:
  pinMode(buttonPin, INPUT);
  pinMode(pausePin, INPUT);
  
  Serial.println("Initialized");
  delay(100);
}

void loop() {
  // refresh button state
  buttonState = digitalRead(buttonPin);
  // run record function
  record();
  // run pause function
  pause();
  // run repeat function
  repeat();
  
}

void pause(){
  reading = digitalRead(pausePin);
  // if the pause button is pressed and it previously wasn't then the state is set to HIGH and the pause loop is initiated
   if (reading == HIGH and previous == LOW ) {
      state = HIGH;   
    }
    previous = reading; 
    while (state==HIGH){
      reading = digitalRead(pausePin);        //refresh the pause button reading
      buttonState = digitalRead(buttonPin);   //refresh the record button reading 
      Serial.println("pause");
      // This if statement will break the pause loop if the button is pressed wont enter more that every debounce milliseconds
      if (reading == HIGH && previous == LOW && millis() - time > debounce) {
        if (state == HIGH)
          state = LOW;
        else
          time = millis(); 
      }
      // The record button will also break the pause loop
      if (buttonState == HIGH){
        state = LOW;
      }
      previous = reading;
  }
  
  
}

int getMeasure(){
  size_t static index = 0;
  index++;
  return Measurement[index - 1];
}

int Measure(int Pin){
  // Function to get the median of 10 measurements. 
   
  int median;
  int Min=55;
  int Max=595;

  // The min and max can change depending on servo model
  if (Pin == Sensor1Pin){
    Min=servo1Min;              
    Max=servo1Max;
  }
  if (Pin == Sensor2Pin){
    Min=servo2Min;              
    Max=servo2Max;
  }
  if (Pin == Sensor3Pin){
    Min=servo3Min;              
    Max=servo3Max;
  }
  if (Pin == Sensor4Pin){
    Min=servo4Min;              
    Max=servo4Max;
  }
  if (Pin == Sensor5Pin){
    Min=servo5Min;              
    Max=servo5Max;
  }
  if (Pin == Sensor6Pin){
    Min=servo6Min;              
    Max=servo6Max;
  }

  // Fills the array with measurements
  for (int i = 0; i < 9;  i++) {
    Measurement[i]=constrain(analogRead(Pin),Min,Max);
  }

  size_t valuesLength = sizeof(Measurement) / sizeof(Measurement[0]);

  // determines the median of the Measurement array
  for (size_t iCount = 0; iCount < valuesLength; iCount++) {
    int rawMeasure = Measurement[iCount];
    median = medianFilter.AddValue(rawMeasure);
  }
  return median;
}

void record (){
  // Function that records the servo posistion with 1 sec interval
  count=0;
      while (buttonState == HIGH){
        Serial.println(">> record >>");
        buttonState = digitalRead(buttonPin);

        // The servo we used need to be turned off and detached to be able to turn them manually
        // The MOSFET turns off the power to the servos
        digitalWrite(13, LOW);
        delay(1);
        // The servos are detached
        myServo1.detach(); 
        myServo2.detach();
        myServo3.detach();
        myServo4.detach();
        myServo5.detach();
        myServo6.detach();
        delay(1);
        // The servo power is turned back on
        digitalWrite(13, HIGH);

        // The Measure function gets the current location of each servo that is saved in an array
        posServo1[count]=Measure(Sensor1Pin);
        posServo2[count]=Measure(Sensor2Pin);
        posServo3[count]=Measure(Sensor3Pin);
        posServo4[count]=Measure(Sensor4Pin);
        posServo5[count]=Measure(Sensor5Pin);
        posServo6[count]=Measure(Sensor6Pin);
       

        /////////////////////////////////////////////////// Delay ////////////////////////////////////////////////////

        // print out the value you read:
        Serial.print("Value: ");
        Serial.print(posServo1[count]);
        Serial.print(", ");
        Serial.print(posServo2[count]);
        Serial.print(", ");
        Serial.print(posServo3[count]);
        Serial.print(", ");
        Serial.print(posServo4[count]);
        Serial.print(", ");
        Serial.print(posServo5[count]);
        Serial.print(", ");
        Serial.println(posServo6[count]);
        Serial.print("Count: ");
        Serial.println(count);
        count=count+1;

        // A delay that dictates the resolution of the recordig i.e. how often the measurements are are done
        delay(recDlay);
        
        Serial.println("<< record <<");
        buttonState = digitalRead(buttonPin);

        // Sets the last function to be called as recording
        rec=1;
    }
    
}

int ConvertToDegree(int x, int motor){
  //Function to convert from poteniometer reading to degrees. Function was found in excel.
  //This is for the strong motors
  float Min;
  float Max;
  // Max and Min values are used to calculate the degrees
  if (motor == 1){
    Min=float(servo1Min);              
    Max=float(servo1Max);
  }
  if (motor == 2){
    Min=float(servo2Min);              
    Max=float(servo2Max);
  }
  if (motor == 3){
    Min=float(servo3Min);              
    Max=float(servo3Max);
  }
  if (motor == 4){
    Min=float(servo4Min);              
    Max=float(servo4Max);
  }
  if (motor == 5){
    Min=float(servo5Min);              
    Max=float(servo5Max);
  }
  if (motor == 6){
    Min=float(servo6Min);              
    Max=float(servo6Max);
  }

  // The equation here converts the measured value from the potentiometer to the corrosponting degree.
  // The result is then constrained to be between 0 and 180 since if wasnt within these parameters the result would be incorrect
  float val=constrain(round(((float(x)-Min)*180.0)/(Max-Min)),0,180);

  return int(val);
}

void repeat(){
  // The repeat loop will only run when the record button isnt pressed
  while (buttonState == LOW){
    // The pasue function is called
    pause();
    
    Serial.println(">> repeat >>");
    if (0 < count){
        delay(1000);
        }

    buttonState = digitalRead(buttonPin);
    
    // The MOSFET makes sure the power is on
    digitalWrite(13, HIGH);
    //Attaching Servo
    myServo1.attach(Servo1Pin);
    myServo2.attach(Servo2Pin); 
    myServo3.attach(Servo3Pin);
    myServo4.attach(Servo4Pin);
    myServo5.attach(Servo5Pin); 
    myServo6.attach(Servo6Pin);
    
    int i = 0;
      while (buttonState == LOW and i < count){
        
        //pause function is called
        pause();

        // Record button is refreshed and the loop is broken if the button is pressed
        buttonState = digitalRead(buttonPin);
        if (buttonState==HIGH){
          i=count;
        }
        
  
        for (i = 0; i < count; i += 1) { // Loop for the whole movement array
        Serial.println(">> moveto >> ");
        

        //////////// Get destination ////////////
        int moveto1=ConvertToDegree(posServo1[i],1);
        int moveto2=ConvertToDegree(posServo2[i],2);
        int moveto3=ConvertToDegree(posServo3[i],3);
        int moveto4=ConvertToDegree(posServo4[i],4);
        int moveto5=ConvertToDegree(posServo5[i],5);
        int moveto6=ConvertToDegree(posServo6[i],6);

        
        //////////// Current position ////////////
        // Sets current position to the last degree sent to the servos
        int currentPos1=myServo1.read();
        int currentPos2=myServo2.read();
        int currentPos3=myServo3.read();
        int currentPos4=myServo4.read();
        int currentPos5=myServo5.read();
        int currentPos6=myServo6.read();
        
        if (rec==1){
          // If the last function was record then the current position is measured
          currentPos1=ConvertToDegree(Measure(Sensor1Pin),1);
          currentPos2=ConvertToDegree(Measure(Sensor2Pin),2);
          currentPos3=ConvertToDegree(Measure(Sensor3Pin),3);
          currentPos4=ConvertToDegree(Measure(Sensor4Pin),4);
          currentPos5=ConvertToDegree(Measure(Sensor5Pin),5);
          currentPos6=ConvertToDegree(Measure(Sensor6Pin),6);
        }
        
        
        //////////// Difference ////////////
        int diff1=moveto1-currentPos1;
        int diff2=moveto2-currentPos2;
        int diff3=moveto3-currentPos3;
        int diff4=moveto4-currentPos4;
        int diff5=moveto5-currentPos5;
        int diff6=moveto6-currentPos6;

        /////////// Direction indicater ////////////
        int dir1=0;
        int dir2=0;
        int dir3=0;
        int dir4=0;
        int dir5=0;
        int dir6=0;
        
        
        //////////// Breaker ////////////
        //1 enables the servo, 0 turns it off
        int dir11=1;
        int dir22=1;
        int dir33=1;
        int dir44=1;
        int dir55=0;
        int dir66=0;

        ////////////////////// Sensitvity //////////////////////
        // If the movement is less than the sensitivity then the servo wont move
        if (abs(diff1) < diffSensitivity) {
            dir11=0;
          }
        if (abs(diff2) < diffSensitivity) {
            dir22=0;
          }
        if (abs(diff3) < diffSensitivity) {
            dir33=0;
          }
        if (abs(diff4) < diffSensitivity) {
            dir44=0;
          }
        if (abs(diff5) < diffSensitivity) {
            dir55=0;
          }
        if (abs(diff6) < diffSensitivity) {
            dir66=0;
          }

        
        ////////////////////// direction 1 //////////////////////
        // Defines what direction the the servo needs to move
        if (moveto1 > currentPos1) {
          dir1=1;
        }
        if (moveto1 < currentPos1) {
          dir1=-1;
        }
        
        ////////////////////// direction 2 //////////////////////
        if (moveto2 > currentPos2) {
          dir2=1;
        }
        if (moveto2 < currentPos2) {
          dir2=-1;
        }
                
        ////////////////////// direction 3 //////////////////////
        if (moveto3 > currentPos3) {
          dir3=1;
        }
        if (moveto3 < currentPos3) {
          dir3=-1;
        }
        
        ////////////////////// direction 4 //////////////////////
        if (moveto4 > currentPos4) {
          dir4=1;
        }
        if (moveto4 < currentPos4) {
          dir4=-1;
        }
        
        ////////////////////// direction 5 //////////////////////
        if (moveto5 > currentPos5) {
          dir5=1;
        }
        if (moveto5 < currentPos5) {
          dir5=-1;
        }

        ////////////////////// direction 6 //////////////////////
        if (moveto6 > currentPos6) {
          dir6=1;
        }
        if (moveto6 < currentPos6) {
          dir6=-1;
        }

        // the maximum length of movements for all enabled servos
        int MaxRepeat= max(abs(diff6*dir66),max(abs(diff5*dir55),max(abs(diff4*dir44), max(abs(diff3*dir33), max(abs(diff2*dir22), abs(diff1*dir11)))))) ;

        
        for (int pos = 1; pos < MaxRepeat; pos += 1) {
          // This loop moves all the servos from the current position the next elements of the recorded arrays
          // It is done incrementally to control the speed of the arms

          // The pause function is called so the movemnt can be paused
          pause();

          // The record button is refreshed
          buttonState = digitalRead(buttonPin);
          // If the record button is pushed the movement loop will break
          if (buttonState==HIGH){
            pos=MaxRepeat;
            i=count;
          }
          
          // If the servo is enabled then the servo will move 1 degree in the direction towards the goal
          if (dir11!=0){
            myServo1.write(currentPos1+pos*dir1);
          }
          
          if (dir22!=0){
            myServo2.write(currentPos2+pos*dir2);
          }
          
          if (dir33!=0){
            myServo3.write(currentPos3+pos*dir3);
          }
          
          if (dir44!=0){
            myServo4.write(currentPos4+pos*dir4);
          }
          
          if (dir55!=0){
            myServo5.write(currentPos5+pos*dir5);
          }

          if (dir66!=0){
            myServo6.write(currentPos6+pos*dir6);
          }


          // The servo will be disabled when the servo reaches its target position
          ////////////////////// stop 1 //////////////////////
          if (moveto1 >= currentPos1+pos*dir1 and dir1==-1) {
            dir11=0;
          }
          if (moveto1 <= currentPos1+pos*dir1 and dir1==1) {
            dir11=0;
          }
          
          ////////////////////// stop 2 //////////////////////
          if (moveto2 >= currentPos2+pos*dir2 and dir2==-1) {
            dir22=0;
          }
          if (moveto2 <= currentPos2+pos*dir2 and dir2==1) {
            dir22=0;
          }
          
          ////////////////////// stop 3 //////////////////////
          if (moveto3 >= currentPos3+pos*dir3 and dir3==-1) {
            dir33=0;
          }
          if (moveto3 <= currentPos3+pos*dir3 and dir3==1) {
            dir33=0;
          }
          
          ////////////////////// stop 4 //////////////////////
          if (moveto4 >= currentPos4+pos*dir4 and dir4==-1) {
            dir44=0;
          }
          if (moveto4 <= currentPos4+pos*dir4 and dir4==1) {
            dir44=0;
          }
          
          ////////////////////// stop 5 //////////////////////
           if (moveto5 >= currentPos5+pos*dir5 and dir5==-1) {
            dir55=0;
          }
          if (moveto5 <= currentPos5+pos*dir5 and dir5==1) {
            dir55=0;
          }
          
          ////////////////////// stop 6 //////////////////////
           if (moveto6 >= currentPos6+pos*dir6 and dir6==-1) {
            dir66=0;
          }
          if (moveto6 <= currentPos6+pos*dir6 and dir6==1) {
            dir66=0;
          }
          
          // The delay that controls the speed of the arms
          delay(dlay);

          // The last function is set as repeat
          rec=0;
        }
        

        Serial.println("<< moveto <<");
  
        }
      
    }
    Serial.println("<< repeat <<");
  }
}
