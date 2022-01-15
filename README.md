# Robotics

One of my favourite projects this past year was assembling a robot. There are five sensors attached to the chassis. The ultrasonic sensor is detecting how far away an object is, the two infrared sensors are identifying when an object is within 2 - 30 cm away depending on the detection set, and the last reflective sensor under the chassis is identifying when objects beneath the car change (changing from wood to stone flooring). 

The Arduino file bug_algorithm.ino will allow the robot to travel around obstacles using the Bug 2 Algorithm. The Bug 2 algorithm works by defining the shortest line between programmed start and end locations. The robot follows this line until it reaches an obstacle and will turn and follow the edge of that obstacle until it returns to the line. Here is a video of the robot in action:


https://user-images.githubusercontent.com/72311187/149601188-500dfb73-8782-4f0c-b2f6-06df166bce38.mp4

