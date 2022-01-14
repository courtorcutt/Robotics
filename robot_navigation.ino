// set up for wheel encoders and left and right wheels
#define LEncoderPin 3     
#define REncoderPin 2
#include <AFMotor.h>

int LeftEncoderCount1 = 0;
int RightEncoderCount1 = 0;

int LeftEncoderCount = 0;
int RightEncoderCount = 0;

int DelayTime = 200;

long prevTimeL = 0; 
long prevTimeR = 0; 

long speedDiff = 0; 
long ls = 200;
long rs = 200;

AF_DCMotor left_motor(2, MOTOR34_1KHZ);  // left motor to M2 on motor control board
AF_DCMotor right_motor(3, MOTOR12_1KHZ); // right motor to M3 on motor control board

// for checking if all sensors work, everything is set to false
boolean IRsensorLeft = false;
boolean IRsensorRight = false;
boolean downwardSensor = false;
boolean ultrasonicSensor = false;
boolean leftWheel = false;
boolean rightwheel = false;

// for set up of unltrasonic sensor
unsigned long duration;
float distance;
long i;


void setup() {
  // wheels -> interupt functions needed to detect if working when turn wheels
  attachInterrupt(1, countLEncoder, RISING); //calls on any CHANGE on pin 3 (interrupt #1, soldered to Pin3) 
  attachInterrupt(0, countREncoder, RISING); //calls on any CHANGE on pin 2 (interrupt #0, connected to header on Pin2) 
  interrupts();
  // Set motor direction at start to not be moving
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);


// set up of all pins & sensors together

    // downward looking sensor
    pinMode(A2, INPUT);

    // ultrasonic sensor
    pinMode(A1, INPUT); // Echo
    pinMode(A3, OUTPUT); // Trig

    // IR obstacle avoidance
    pinMode(A4, INPUT); // the left sensor
    pinMode(A5, INPUT); // the right sensor

  Serial.begin (115200);

  workingSensorsCheck();

  if(IRsensorLeft == true && IRsensorRight == true && downwardSensor == true && ultrasonicSensor == true && leftWheel == true && rightwheel == true){    // == OR = ??????
    Serial.println("");
    Serial.print("You can start program now");
  } else if(IRsensorLeft == false){
    Serial.println("Check your IR left sensor and restart program");
      workingSensorsCheck(); // loop back and check sensors again
  } else if(IRsensorRight == false){
    Serial.println("Check your IR right sensor");
      workingSensorsCheck(); // loop back and check sensors again
  } else if(downwardSensor == false){
    Serial.println("Check your downward sensor");
      workingSensorsCheck(); // loop back and check sensors again
  } else if(ultrasonicSensor == false){
    Serial.println("Check your ultrasonic sensor");
      workingSensorsCheck(); // loop back and check sensors again
  } else if(leftWheel == false){
    Serial.println("Check your left wheel encoder");
      workingSensorsCheck(); // loop back and check sensors again
  } else if(rightwheel == false){
    Serial.println("Check your right wheel encoder");
      workingSensorsCheck(); // loop back and check sensors again
  }

  // delay before goes to loop
  delay(3000);
}

void loop() {
  Serial.println("Driving Straight To Start");
  DriveForward(); //start/resume forward driving 
}


// functions implemented to help in numerical printing to serial monitor
float getVoltage(int pin) {
  float voltage = 5.0 * analogRead(pin) / 1024;
  return voltage;
}

// DRIVING STRAIGHT SECTION
void countLEncoder(){ // interrupt function for left encoder
    LeftEncoderCount++;
    LeftEncoderCount1++;
}

void countREncoder(){ // interrupt function for right encoder
    RightEncoderCount++;
    RightEncoderCount1++;
}


// check if all sensors are working and print to serial monitor
void workingSensorsCheck(){
  // downward facing sensor check:
  Serial.println("Downward Looking Sensor:");
  Serial.println("A2: ");
  Serial.print(getVoltage(A2));
  Serial.println("\t");

// to check I will swipe my hand under and A2 will go below 5 and I can then set the sensor to be true (working)
  if (getVoltage(A2) > 0.7){
    Serial.println("White paper detected.");
    downwardSensor = true;
  }



  // ultrasonic sensor check
  digitalWrite(A3,LOW);
  delayMicroseconds(2);
  digitalWrite(A3,HIGH);
  delayMicroseconds(10);
  digitalWrite(A3,LOW);


  duration = pulseIn(A1, HIGH); // must be HIGH from data sheet
  Serial.println("Ultrasonic Sensor Check: ");
  Serial.print("Duration: ");
  Serial.println(duration); // The length of the pulse (in microseconds) or 0 if no pulse started before the timeout.

  // ranging distance from the sensor
  Serial.print("Distance from object: ");
  distance = ((duration*0.034)/2); // this will calculate the distance in centimeters
  Serial.print(distance); //uS / 58 = centimeters or uS / 148 =inch
  Serial.println(" cm."); //uS / 58 = centimeters or uS / 148 =inch

  if(distance > 0.00){
    ultrasonicSensor = true;
  }

  IRsensorsLoop();

  // wheel check
  wheelSensorsLoop();

  
  Serial.println("\t");
  Serial.println("Status of all sensors: ");
  Serial.print(downwardSensor);
  Serial.println("\t");
  Serial.print(ultrasonicSensor);
  Serial.println("\t");
  Serial.print(IRsensorLeft);
  Serial.println("\t");
  Serial.print(IRsensorRight);
  Serial.println("\t");
  Serial.print(leftWheel);
  Serial.println("\t");
  Serial.print(rightwheel);
  Serial.println("\t");

}


void wheelSensorsLoop(){
  Serial.println("Motor test:");
  
  while(leftWheel == false){
    Serial.print("Left Encoder Check: ");
    Serial.println(LeftEncoderCount);
    if(LeftEncoderCount>0){
      leftWheel = true;
    }
  }

  while(rightwheel == false){
    Serial.print("Right Encoder Check: ");
    Serial.println(RightEncoderCount);
    if(RightEncoderCount>0){
      rightwheel = true;
    }
  }
  
}

void IRsensorsLoop(){
  // IR sensor check left and right
  while(IRsensorLeft == false){
    if(getVoltage(A4) < 0.5){
    Serial.print("Left IR sensor check: ");
    Serial.println("OBSTACLE!! on LEFT");
    IRsensorLeft = true;
    }
  }

  while(IRsensorRight == false){
    if(getVoltage(A4) < 0.5){
    Serial.print("Right IR sensor check: ");
    Serial.println("OBSTACLE!! on RIGHT");
    IRsensorRight = true;
    }
  }
  
}



// actual driving program
// Drive Straight
void DriveForward(){
    Serial.println("Forward");
    LeftEncoderCount1 = 0;//Reset encoder counters
    RightEncoderCount1 = 0; 

    left_motor.run(FORWARD); //Set motors to forward
    right_motor.run(FORWARD);

    // 3 is the scaling factor on the PWM
    speedDiff = 3*(LeftEncoderCount - RightEncoderCount); //Calculate what to change speed by based on difference between the two encoder counts in the last Delaytime seconds
      
    // left motor ran faster so reduce and vise versa for right
    left_motor.setSpeed(ls - speedDiff); 
    right_motor.setSpeed(rs + speedDiff);

    //reset encoder count 
    LeftEncoderCount = 0;
    RightEncoderCount = 0; 
    delay(DelayTime);


// while driving check ultrasonic sensor to see if approaching object -> makes sure the code isn't blocking other sensor data
      digitalWrite(A3,LOW);
      delayMicroseconds(2);
      digitalWrite(A3,HIGH);
      delayMicroseconds(10);
      digitalWrite(A3,LOW);


    duration = pulseIn(A1, HIGH); // must be HIGH from data sheet
  
    Serial.print("Duration: ");
    Serial.println(duration); // The length of the pulse (in microseconds) or 0 if no pulse started before the timeout.

    // ranging distance from the sensor
    Serial.print("Distance from object: ");
    distance = ((duration*0.034)/2); // this will calculate the distance in centimeters
    Serial.print(distance); //uS / 58 = centimeters or uS / 148 =inch
    Serial.println(" cm."); 


  if (distance <= 50 ){
    Serial.println("Approaching Object (less than 50 cm away), Slowing Down Now");
    for (i=255; i>0; i=i-50) {
    left_motor.setSpeed(i);
    right_motor.setSpeed(i);
   }
  }



// while driving forward check the downward looking sensor to see if it is driving over a white piece of paper
// -> makes sure the code isn't blocking other sensor data by blocking
      if(analogRead(A2) > 0.7){
      // stop vehicle
      Serial.println("You have travelled over the paper and vehicle will stop");
      left_motor.run(RELEASE);
      right_motor.run(RELEASE);
    }



// simutanously while moving forward is checking IR sensor data to prevent blocking/other actions
if(getVoltage(A4) < 0.5 && getVoltage(A5) < 0.5){
  Serial.println("OBSTACLE!! on BOTH SIDES");
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  delay(3000);

  DriveBackward();
  delay(1000);
  
  turn180();


  
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  delay(4000);


}else if(getVoltage(A4) < 0.5){
  Serial.println("OBSTACLE!! on LEFT");
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  delay(1000);

  right_motor.run(FORWARD);

  //degrees:-180 to 180. Positive number for turning right, negative number for turning left.
  right_motor.setSpeed(200);
  delay(150);

  DriveBackward();
  delay(500);

  
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  delay(4000);

  
}else if(getVoltage(A5) < 0.5){
  Serial.println("OBSTACLE!! on RIGHT");
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  delay(1000);

  left_motor.run(FORWARD);
    
  //degrees:-180 to 180. Positive number for turning right, negative number for turning left.
  left_motor.setSpeed(200);
  delay(150);

  DriveBackward();
  delay(500);

  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  delay(4000);

  }
  

}

void DriveBackward(){
    Serial.println("Backward");
    LeftEncoderCount1 = 0;//Reset encoder counters
    RightEncoderCount1 = 0; 

    left_motor.run(BACKWARD);//Set motors to forward
    right_motor.run(BACKWARD);

      // 3 is the scaling factor on the PWM
    speedDiff = 3*(LeftEncoderCount - RightEncoderCount); //Calculate what to change speed by based on difference between the two encoder counts in the last Delaytime seconds
      
      // left motor ran faster so reduce and vise versa for right
      left_motor.setSpeed(ls - speedDiff);//Update speed accordingly 
      right_motor.setSpeed(rs + speedDiff);
      
      LeftEncoderCount = 0; //reset encoder count 
      RightEncoderCount = 0; 
      delay(DelayTime);
}

void turn180(){
    Serial.println("180 degree turn");
    LeftEncoderCount1 = 0;//Reset encoder counters
    RightEncoderCount1 = 0; 

    left_motor.run(BACKWARD);//Set motors to forward
    right_motor.run(FORWARD);

    left_motor.setSpeed(200);
    left_motor.setSpeed(200);

    delay(400);
}
