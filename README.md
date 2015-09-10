# joint-robot
Java program that searches for a path between initial and goal configurations of n-joint robot in 2D space

This assignment is about designing and developing the program for this simplified robotics sys-
tem.The simplified robot is a planar robot, which means it oper-
ates in a 2D workspace (rather than 3D). In particular, the 2D
workspace is a plane, represented as [0, 1] × [0, 1] ⊂ R2, 
and is populated by rectangular obstacles. 
The exact dimension and position of each obstacle in the environment is known prior to execution. 

You can read more about problem description in the assignment specification file.

## To run this program

You would require ant to build a .jar file. 

1. Clone application: 
    'git clone https://github.com/camomiles/joint-robot'

2. Go into application folder: 
    'cd joint-robot'

3. Run ant to build project:
    'ant'

4. Run application:
    'java -jar ai-7702.jar examples/inputFileName.txt outputFileName.txt'
