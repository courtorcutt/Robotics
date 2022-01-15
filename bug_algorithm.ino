//Encoder Pins for Hardware Interupts
#define LEncoderPin 3     
#define REncoderPin 2

//Motor shield library
#include <AFMotor.h>

//Setting Motors
AF_DCMotor left_motor(3, MOTOR34_1KHZ);  // left motor to M1 on motor control board
AF_DCMotor right_motor(1, MOTOR12_1KHZ); // right motor to M3 on motor control board

//Encoder count global variables, increase with wheel spin 
int LeftEncoderCount = 0;
int RightEncoderCount = 0;
int DelayLeftEncoderCount = 0;
int DelayRightEncoderCount = 0;

//State global variable
String state = "drive";


int DelayTime = 200;
//Defining input pins for IR and button
const int buttonInput = 14;
const int leftIR = 18; 
const int rightIR = 19; 
bool buttonPressed = false; 

//Location of robot global variable includes x and y coordinate: [x,y] 
int location [2] = {0,0};
//Location of target
int targetLocation [2] = {-1,18};
//Location the the robot travelled to farthest left
int largestEdgeLocation = 0; 
//Number of right turns, keeps track of direction
int numberOfRightTurns = 1;  

//Speed Diffrence
long speedDiff = 0; 
//Left speed
long ls = 200;
//Right speed
long rs = 200;


void setup() {
  //Attaching hardware interupts, 
  attachInterrupt(1, countLEncoder, RISING); //calls on any CHANGE on pin 3 (interrupt #1, soldered to Pin3) 
  attachInterrupt(0, countREncoder, RISING); //calls on any CHANGE on pin 2 (interrupt #0, connected to header on Pin2) 
  interrupts();
  Serial.begin (115200);    // set up Serial library at 115200 bps
  pinMode(buttonInput, INPUT);// Button
  pinMode(A3, OUTPUT); //Trigger
  pinMode(A2, INPUT); //ECHO
  pinMode(A1, INPUT); //IRVOUT
  pinMode(leftIR, INPUT);// Left IR
  pinMode(rightIR, INPUT);// Right IR
 
  // Set motor direction
 
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);

   testSensors();//Run sensor test

    while(buttonPressed == false){//Wait until button is pressed to end setup 
     if(digitalRead(buttonInput)== 0){
      buttonPressed = true; 
     delay(100);
   }
  }
}

void loop() { 
  //Sense
    //Get Bottom IR input 
    int bottomIR = analogRead(A1);  
    //Get Ultra Sonic Input 
    int ultraSonicRead = ultraSonic(); 
    //Get Encoder Input 
   if(state == "drive"){//If in drive state
      if(location[1] < targetLocation[1]){//Check to see if destination is reached
        drive(LeftEncoderCount);//If not reached drive forward
        pause(); 
      }else{//Begin return protocol if reached
        turn("left", LeftEncoderCount, RightEncoderCount);
        pause(); 
        numberOfRightTurns = 2;
        state = "return";
      }
     if(digitalRead(leftIR) == LOW){//Object Deetected while moving forward, Begin follow protocol 
        turn("left", LeftEncoderCount, RightEncoderCount);
        numberOfRightTurns++;
        pause();
        state = "follow"; 
     }
   }

   if(state == "follow"){//If state is follow
    if(location[0] == targetLocation[0] && numberOfRightTurns != 2){//if robot has return to m-line at y[0]
      pause();
      turn("left", LeftEncoderCount, RightEncoderCount);//Turn left to return to facing the target
      numberOfRightTurns = 1;
      pause();
      state = "drive";//Return to drive protocol
    }else{//Robot has not returned to m-line so continue to follow perimeter 
     drive(LeftEncoderCount); 
     pause(); 
     if(digitalRead(rightIR) == HIGH){//Check if corner has been reached
        drive(LeftEncoderCount); 
        if(location[0] > largestEdgeLocation){//Record location of farthest object edge
          largestEdgeLocation = location[0];
        }
        pause();
        turn("right", LeftEncoderCount, RightEncoderCount);//Turn around corner 
        numberOfRightTurns--;
        pause();
        drive(LeftEncoderCount); 
        pause();
     }
    }
  }

  if(state == "return"){//If state is returb
    if(location[0] < largestEdgeLocation){//Drive until clear of farthest recorded object edge
      drive(LeftEncoderCount);
      pause(); 
    }else if(numberOfRightTurns == 2){//Turn left again to face start location
      turn("left", LeftEncoderCount, RightEncoderCount);
      numberOfRightTurns = 3;
      pause(); 
    }else if((location[1] > -3)){//Drive until start location is reached (-3 for a bit of a errror margin, better to overshoot than undershoot, effect of this can be seen in the video)
      drive(LeftEncoderCount);
      pause();
    }
  }
}

void updateLocation(){//Function updates the robots location on the x-y grid when it moves based on what direction it is facing 
  if(numberOfRightTurns == 4){
    numberOfRightTurns = 0; 
  }
  if(numberOfRightTurns == 0){
     location[0]--; 
  }
  if(numberOfRightTurns == 1){
     location[1]++; 
  }
  if(numberOfRightTurns == 2){
     location[0]++; 
  }
  if(numberOfRightTurns == 3){
     location[1]--; 
  }
  Serial.print(location[0]);
}


void turn(String direction, int leftStartEncoder, int rightStartEncoder){//Function turns robot 90 degrees left or right
    if(direction == "right"){//Check for direction parameter
      left_motor.run(FORWARD);//Set motors to forward
      right_motor.run(BACKWARD);
    }else{
      left_motor.run(BACKWARD);//Set motors to forward
      right_motor.run(FORWARD);
    }
  
    while (((LeftEncoderCount - leftStartEncoder) < 17) && ((RightEncoderCount - rightStartEncoder) < 17)) {//Runs for 17 encoder ticks from whatever they were when called start
      //Serial.println(LeftEncoderCount);
      speedDiff = 12*(DelayLeftEncoderCount - DelayRightEncoderCount);//Closed loop control speed adjustment
      left_motor.setSpeed(ls - speedDiff);//Update speed accordingly 
      right_motor.setSpeed(rs + speedDiff);
      DelayLeftEncoderCount = 0; 
      DelayRightEncoderCount = 0; 
      delay(50); 
    }
    
    left_motor.setSpeed(0);//Update speed accordingly 
    right_motor.setSpeed(0); 
}


void drive(int startEncoder){//Drives robot straight forward
    left_motor.run(BACKWARD);//Set motors to forward
    right_motor.run(BACKWARD);
  
    while ((LeftEncoderCount - startEncoder) < 15) {
      //Serial.println(LeftEncoderCount);//Closed loop control speed adjustment
      speedDiff = 12*(DelayLeftEncoderCount - DelayRightEncoderCount);
      left_motor.setSpeed(ls - speedDiff);//Update speed accordingly 
      right_motor.setSpeed(rs + speedDiff);
      DelayLeftEncoderCount = 0; 
      DelayRightEncoderCount = 0; 
      delay(50); 
    }
      left_motor.setSpeed(0);//Update speed accordingly 
      right_motor.setSpeed(0); 

     updateLocation();
}

void pause(){//Pauses robot
    //Serial.println("Pause!"); 
    left_motor.setSpeed(0);//Update speed accordingly 
    right_motor.setSpeed(0);
    left_motor.run(RELEASE);//Set motors to forward
    right_motor.run(RELEASE);
    delay(1000); 
}

void countLEncoder(){ // interrupt function for left encoder
    LeftEncoderCount++;
    DelayLeftEncoderCount++;
}

void countREncoder(){ // interrupt function for right encoder
    RightEncoderCount++;
    DelayRightEncoderCount++;
}



int ultraSonic(){//Function to read ultrasonic sensor
      digitalWrite(A3, LOW);
      digitalWrite(A3, HIGH);//Send out trigger pulse
      delayMicroseconds(10); 
      digitalWrite(A3, LOW);
      unsigned long duration = pulseIn(A2, HIGH); //Get return time
      float distance = (duration*0.034)/2;//Convert to distance
      return distance; 
}



void testSensors(){//Sensor Test Function 
   left_motor.run(RELEASE);//Set motors 
   right_motor.run(RELEASE);
   Serial.println("Please spin left wheel"); 
   while(LeftEncoderCount < 20){//Check left encoder
    delay(100);
   }
   Serial.println("Success, Please spin right wheel"); 
   while(RightEncoderCount < 20){//Check right encoder
    delay(100);
   }

   Serial.println("Success, Please test right IR"); 
   while(digitalRead(rightIR)== HIGH){//Check IR
    delay(100);
   }

   Serial.println("Success, Please test left IR"); 
   while(digitalRead(leftIR)== HIGH){//Check IR
    delay(100);
   }

   Serial.println("Success, Please test UltraSonicSensor"); 
   while(ultraSonic() > 10){//Check Ultrasonic
    delay(100);
   }

   Serial.println("Success, Please test Bottom IR"); 
   while(analogRead(A1) < 13){//Check Bottom IR
    delay(100);
   }

   Serial.println("Success, All tests complete vehichle ready for use");
   
}
