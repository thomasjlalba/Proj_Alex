# Project Alex
As part of a graded project for Engineering and Principles and Practice II

The objective of the Alex is to be a robot with search and rescue functionalities. 
Alex was required to be tele-operated and navigated from our laptops, 
while mapping out a simulated environment within a 3m^2 table.  

![20220412_094317](https://user-images.githubusercontent.com/88082961/171360616-92c6a0b1-7b6a-48bb-8a48-1f1b83875023.jpg)

![20220412_094118](https://user-images.githubusercontent.com/88082961/171361081-1b09089c-e227-45c5-ada2-3e4f97e9cb69.jpg)

Our Alex was made up of the following components:
| **Aspect**    | **Component**                              | **Function**                                                                                                                                                                                                                                                                                                                     |
|---------------|--------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Movement      | 3x Wheel                                   | Moves Alex to its desired location                                                                                                                                                                                                                                                                                               |
| Movement      | 2x Motor                                   | Transforms the electrical energy to mechanical energy to power the<br>wheel to rotate                                                                                                                                                                                                                                            |
| Movement      | Dual Motor<br>Driver Carriers<br>(DRV8833) | Contains 2 H-bridges, receiving the difference in PWM inputs to drive<br>the motors from the AA batteries with the desired parity                                                                                                                                                                                                |
| Measurement   | 2x Wheel<br>Encoder                        | Calculates the number of rotations performed by the motor, which can<br>be used to measure distance traveled                                                                                                                                                                                                                     |
| Measurement   | Ultrasonic Sensor                          | Uses pulses of ultrasonic sound waves to measure the distance between<br>obstacles and the rear of Alex                                                                                                                                                                                                                          |
| Mapping       | LIDAR<br>(RPLIDAR A1)                      | Provides coordinates of obstacles detected which then can be used to<br>map the surroundings of Alex                                                                                                                                                                                                                             |
| Power Source  | 4x AA Battery<br>with Holder               | Provides electrical energy to power the motors                                                                                                                                                                                                                                                                                   |
| Power Source  | Power Bank                                 | Provides electrical energy to power the Raspberry Pi, Arduino Uno<br>and LIDAR                                                                                                                                                                                                                                                   |
| Communication | Arduino Uno                                | Receives data packets of commands from the Raspberry Pi, to perform<br>measurements of Alex, and to move Alex to its desired location                                                                                                                                                                                            |
| Communication | Raspberry Pi                               | Provides communication to the Arduino Uno to navigate Alex around<br>its surroundings, provides communication with our personal laptops<br>to be remotely controlled, connects the LIDAR to a shared network<br>with our personal laptops, so it can send over the coordinates<br>which can then be used to map the surroundings |
| Miscellaneous | Breadboard                                 | To hold and allow for the connections of the Dual Motor Driver<br>Carriers, Motors, Wheel Encoders and Ultrasonic Sensors                                                                                                                                                                                                        |
| Miscellaneous | Wires                                      | Provides connections between the Arduino Uno, Dual Motor Driver<br>Carriers, Motors, Wheel Encoders and Ultrasonic Sensors                                                                                                                                                                                                       |

The following is the breakdown of the implementation of Alex and how each
component works and is integrated with one another:
![image](https://user-images.githubusercontent.com/88082961/171366025-4e432b92-44ae-4302-954e-6ef376dfcf4d.png)

The following is an image of the display map that can be seen from our laptop:
![image](https://user-images.githubusercontent.com/88082961/171366285-cfba10a7-58b4-4218-9fd6-4f2dbd2bf3b0.png)
