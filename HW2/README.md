Implementation of A* on PR2 in OpenRAVE. Key features:
1. Collision checking
2. 2 heuristics - Manthattan & Euclidean

Questions:

1. (70 points) Open the hw2 template. Implement the A* algorithm to find the shortest collision-free path for the PR2’s base from the start to the goal in the given environment:
You will be planning for translation of the base in X and Y as well as rotation about the Z axis of the robot, for 3 DOF total. You will need to determine reasonable step sizes for each DOF. If no path exists, the algorithm should print out “No Solution Found.” You may use existing code to implement the priority queue used in A*. Implement 4 variants of the A* algorithm:
        (a) “4-connected” space, with the manhattan distance heuristic
        (b) “4-connected” space, with the euclidian distance heuristic
        (c) “8-connected” space, with the manhattan distance heuristic
        (d) “8-connected” space, with the euclidian distance heuristic
For each variant save a screenshot including 1) The computed path drawn in the openrave viewer in black (of course you will not be able to draw the rotation along the path, so just draw the X and Y components of the configurations in the path). 2) The X and Y components of the collision-free configurations explored by A* in blue. 3) The X and Y components of the colliding configurations explored by A* in red. Make sure to draw the points above the environment so that you can see them. Include these screenshots in your hw2.zip.
Finally, convert your path into an openrave trajectory (this will take some work and digging through documentation). You can assign timing to the path using any reasonable method (e.g. setting a fixed time step between configurations), including using methods already implemented in openrave. Execute your path on the robot (see the hw2 template for more details).
To grade your work, we will run the command “python hw2.py” in a folder where we’ve extracted your source code. We will not run any other command or any modifications of this command.
When this command is run, your code should plan using variant (d) and execute a trajectory for the robot and we should see the robot following this trajectory in the openrave viewer.
Include answers to the following in your hw1.pdf:
1.1. (10 points) Which heuristic seems superior for 4-connected? Explain your answer.
1.2. (10 points) Which heuristic seems superior for 8-connected? Explain your answer.
1.3. (10 points) For the four variants above, which, if any, variants have an admissible heuristic and which, if any, do not? Explain your answers.


