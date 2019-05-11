# Soccer Open RCJ (Maker Initiative)
by: Breno Cunha Queiroz

<p align="center">
 <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/RobotLeft.jpg" height="200">
 <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/vistaExplodida.PNG" height="200">
<img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/RobotRight.jpg" height="200">
</p>



## Introduction
  The Robots Soccer Maker Initiative project began in late 2016 at the CIC Robotics laboratory at Cândido Portinari School. Professor Fábio Ferreira was my adviser on this project and Ivisson Valverde was my co-supervisor. When this project started I had no knowledge of circuit development, simulation, and a basic understanding of programming and 3D modeling. From the project's challenge, the research initiative and parallel readings to the development of robots became more consistent. The structure, formerly made of ABS plastic in the prototype, has been replaced by polycarbonate; the controller, which was once an Arduino MEGA 2560 became an embedded system developed by the team itself composed of two microprocessors SAM3X8E. In addition, the virtual simulation environment made it possible to reduce costs and optimize the time, either by the tests and validations of the project still in the virtual robot phase, or by the importation of items and the long delivery time that prevented the construction of the physical robot in less time. Without the physical robot, the programming was developed for the virtual simulation environment.
  <p align="center">
  <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/SoccerVersions.jpg" height="200">
  </p>

### Programming
  The code was made in C++ with OOP (Object Oriented Programming). In the [Software folder](https://github.com/Brenocq/SoccerOpenRCJ/tree/master/Software) there is a PDF file with the UML diagram that describes the correlations between the objects. All the libraries ware made thinking in a library that could be used to both the physical and simulated robot. For each library, different parts of the code are executed if an Arduino IDE is detected or not. If the Arduino IDE was detected, the code for the physical robot is executed, otherwise, the code for the simulated robot is executed.

### Simulation
  The V-Rep program was used to simulate the football game environment because of its compatibility with the C++ language, which allows the same programming logic to be used in both the simulation and the physical robot. The physical robot works through the Arduino IDE. In V-REP, the physical engine ODE was used during all the tests to simulate the behavior of the real physics. However, it was necessary to make form adaptations to the sensors and controllers available in the simulated environment to represent more precisely the same ones of the physical robot. Moreover, it was necessary to decrease the resolution of the robot structure to avoid slowness due to the high amount of polygons.
  <p align="center">
  <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/jogo1.PNG" height="200">
  <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/sonar2degrees.PNG" height="200">
  <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/ObstacleAvoidance.png" height="200">
  </p>

### Circuit Design
#### Main circuit
Developed from Altium Designer version 17, the main circuit consists of 4 layers of signal with two microcontrollers Atmel SAM3X8E for data processing and control, chosen for being the most powerful used within the family of embedded circuits developed by Arduino. Individually connected to atmega16u2 "microchips" for USB communication with the computer, each is programmed individually. In addition, the integrated circuits CD74HC4067 and PCA9685 were used for the expansion of the analogue and digital (PWM) ports available for connection of the sensors. For powering most electronic devices the LM2576-5.0 was used, which converts 11.1V to 5V. However, the microcontrollers only work at 3.3V, so the NCP1117ST33T3G was used to convert 5V to 3.3V. On the reverse side of the circuit is the bluetooth HC-05 for communication between the robots.
<p align="center">
 <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/PCBMainCircuit.png" height="200">
 <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/MainCircuit.PNG" height="200">
</p>

#### Secondary circuit
The secondary circuit, in turn, performs no information processing. Instead, this board acts on the interface for the control of the motors, from 03 L298P drivers and the signal transmission of the light sensors to the microcontrollers. In addition, it is also from this board that the power coming from the 2200m 11.1V battery enters the circuit and is directed to conversion to the logical level of the electronic devices in the main circuit.
<p align="center">
 <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/PCBSecondaryCircuit.png" height="200">
 <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/DriverCircuit.jpeg" height="200">
</p>

#### Peripheral circuit
In addition, other peripheral circuits have been designed, such as a light sensor module to detect ball possession and a light sensor module to detect field lines.
<p align="center">
 <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/PCBLightSensorLineCircuit.PNG" height="200">
 <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/LineSensor.jpeg" height="200">
  <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/BallPossesion.jpeg" height="200">
</p>

_All the schematics, PCBs, libraries, and gerber files can be found [here](.)._
### 3D Structure
The chassis was modeled with the SolidWorks program and printed using polycarbonate with this [3D printer](.). The structure is divided into 3 layers.
#### Top Layer
In order to minimize the electromagnetic interference generated by the motors, the compass module 3 in 1 (magnetometer, accelerometer, gyroscope) is positioned in the upper layer (Fig. 5). In addition, the main circuit, the omni-directional camera structure and the Nextion 3.2" display are also located in this layer.
<p align="center">
 <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/camadaSup.PNG" height="200">
</p>
#### Middle Layer
In the middle layer is located the 11.1V 2200mA battery for powering the system, the secondary circuit and the dribbler device for ball possession.
<p align="center">
 <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/camadaInt.PNG" height="200">
</p>
#### Bottom Layer
In the bottom layer, the support base of the robot, are positioned the 4 motors of movement, the structure of kick, the modules LDR for reading the lines of the field and the modules LDR and LED for possession of ball.
<p align="center">
 <img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/CamadaInferior.PNG" height="200">
</p>
_All the 3D parts and 3D assemblies used in this robot can be found [here](.)._
### Human-Robot Interface
The human-robot interface is performed through a 3.2 "Nextion Display. The screens were developed through the manufacturer's program, while the images used on the screens were created by the team through the Illustrator program. Through the display it is possible to test the motors, sensors, connect the robots via bluetooth and start/pause the game program.

<p align="center">
<img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/nextion_principal.PNG" height="200">
<img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/nextion_motor.PNG" height="200">
<img src="https://github.com/Brenocq/SoccerOpenRCJ/blob/CreatingReadMe/Images/nextion_sonares.PNG" height="200">
</p>

### More information
It is possible to see more images and videos of tests carried out in the simulated physical robot during the stage of development of this project [here](.).

To see the technical documentation about the communication between robots, obstacle avoidance, path planning, ball localization with camera, robot localization with equally spaced sensors and more check [here](.)
