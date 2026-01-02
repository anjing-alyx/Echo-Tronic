Welcome to the Echo-Tronic wiki! As this is a personal documentation for my reference, I shall keep the language layman and casual.

1. [What is Echo-Tronic](#What-is-Echo-Tronic)

2. [How does it work](#How-does-it-work)
<br/> 2.1 [Hardware](##Hardware)
<br/> &nbsp; 2.1.1 [Echo-Tronic (a.k.a AnimaBarong) Design](##Echo-Tronic-(a.k.a-AnimaBarong)-Design)
<br/> &nbsp; 2.1.2 [Arduino and PCA9685](###Arduino-and-PCA9685)
<br/> 2.2 [Software](##Software)
<br/> &nbsp; 2.2.1 [Servo Angle Calculation](###Servo-Angle-Calculation)
<br/> &nbsp; 2.2.2 [MediaPipe Facial Tracking](###MediaPipe-Facial-Tracking)
<br/> &nbsp; 2.2.3 [Arduino Serial Communication](###Arduino-Serial-Communication)

3. [What is included](#What-is-included)

4. [Final Notes:](#Final-Notes)

# What is Echo-Tronic:
Echo-Tronic (a.k.a AnimaBarong) is a personal project intending to simplify animatronic facial control from 2-4 operators controlling various facial features to one animatronic rig "echoing" an actor's performance using facial tracking. In a standard rig for facial animatronics there may be 6-20+ servos depending on complexity of expression. As humans only have two hands and 10 fingers, to control 20+ servos simultaneously using RC controllers is a bit tricky- hence the 2-6+ puppeteers manning an animatronic. 

The goal of this project is to reduce manpower required to operate animatronics, allow for more organic acting/performance, and moving the servos in real-time. This, of course, also has use in the field of social robotics and human-robot interaction- however, theme park animatronics for the use case is much more fun. 

# How does it work:
## Hardware
### Echo-Tronic (a.k.a AnimaBarong) Design
The design for the animatronic initially came from a character of mine based on the [Balinese Barong](https://en.wikipedia.org/wiki/Barong_(mythology)) hence the "a.k.a", it has since changed to a sheep head which was inspired by Weta's work on the [Black Sheep Movie](https://www.wetaworkshop.com/projects/black-sheep) and the internal mechanisms based on Gustav Hoegan's work on a [Bison head in a super glue advert](https://www.biomimicstudio.org/showreel). 

All the internal structures were modelled and assembled on SolidWorks as well as 3D printed using a Bambu X1C and PLA Filament. Non-printed parts include:
1. MG90S servos (x11)
2. Various M2.5-M3 Screws
3. Paperclips and wires
4. Electronics

The internal structure was based of a lamb head I sculpted on NomadSculpt for the next step which is a test-skin. But this is an engineering documentation not model-making documentation- so I will leave the foam skin for later. The design goal was to create a modular structure that was easy to assemble and reassemble for repairs while being easy to fabricate both on the 3D printer and machine (although that part remains to be optimized). 

### Arduino and PCA9685
I use Arduino as my testbed and PCA to control my 11 servos. They are great tools for testing- plus the only things available at home.

## Software
### Servo Angle Calculation
Initially this was done via inverse kinematics however the way euclidean distance was calculated provided inaccurate results- hence why a ratio based method was used with some hard-coded distances set in the code to calculate the servo angles. The simple calculation basically allowed for easier tuning (provided you know how to code) and less computation. I also rounded each of the angles calculated to integers that were either multiples of 2 and 5 to reduce the sensitivity of the servos when moving. 

In the future, potentially for more advanced social robots with feedback loops and the need for reliable/adaptive movement- inverse kinematics with servo feedback will be used. 

### MediaPipe Facial Tracking
[MediaPipe Face Landmark Recognition](https://ai.google.dev/edge/mediapipe/solutions/vision/face_landmarker) is the machine learning portion that detects the face and provides the coordinates for facial tracking. The distance between points such as the most top and most bottom part of the eye, is now used to calculate approximately how much the servo moves. 

<p align="center"><img src="https://i.sstatic.net/T1ypF.jpg" width="50%" alt="MP Face Mesh with indices"/></p>
<p align="center">Fig. 1 MediaPipe Face Mesh with Indices Reference </p>

### Arduino Serial Communication
For now, the head still had an umbilical cord to the computer for control however since what Python sends via serial connection is a 22 bit string of servo angles, the order of which is associated to the servos pin-outs on the PCA9685. This packet of information can theoretically be wirelessly transmitted via RC which is the future plan of the work. 

# What is included
In this repository what is included is the most up to date working code and the .stl files for the animatronic hardware. No Solidworks Files will be shared.

# Final Note
Have fun looking around, this is ultimately a passion project in the somewhat declining trade of animatronics and sfx in movies. If you do use this as inspiration or the code for whatever reason, please credit and let me know because I'd love to see where it ends up. There are still a lot of bugs I am fixing as an amateur coder; sorry in advance- I'll fix them soon or whenever I get the time to!
