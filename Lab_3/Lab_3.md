# COMP0182 Real-world Multi-agent Systems: Lab Sheet Three

In todays lab session we will be using [PyBullet](https://pybullet.org/wordpress/) to simulate a single agent navigating to a goal position and back. At the goal position, the agent will fetch an item which it will deliver to the start position. 

To find the path from the start position to the goal position the A* algorithm is used. To run the simulation, you will use the following repository.

**1. Clone the following directory**

```
git clone https://github.com/aydindev0/turtlebot_simulation_pybullet.git
```

**2. Set up the environment**

```
# Go to the PyBullet directory:
cd turtlebot_simulation_pybullet

# Install PyBullet:
pip3 install pybullet

#Install necessary dependencies: 
python3 -m pip install -r requirement.txt
```

**3. Run a single agent navigation scenario**

```
python3 single_robot_navigation.py

```

## Understanding the code

After running the simulation you will see that two files are generated: ```output1.yaml``` and ```output2.yaml```. These files store the waypoints computed using the A* algorithm which determine the path the robots take in the simulation. ```output1.yaml``` is the path from the start position to the fetch point, and ```output2.yaml``` is the return path from the fetch point to starting position. You can inspect the ```single_robot_navigation.py``` file to understand how these files are generated.

### Navigation function

This function drives the simulation robot using the sequence of waypoints computed in the previous step. It accepts five arguments, the first being the agent. The next four are used to drive the robot from each point in the schedule towards the goal and schedule2 towards goal2.

```
def navigation(agent, goal, schedule, goal2, schedule2):
```

While the agent is not at the goal position, the loop reads the robot position and computes the heading to the waypoint (```goal_direction```) and the robot's current heading (```Orientation```). Then the heading error ```theta = goal_direction - Orientation``` is calculated and then normalised to the range [-π,π] to avoid unnecessary rotation.

```
    while(not checkPosWithBias(basePos[0], goal, dis_th)):
        ...
        x = basePos[0][0]
        y = basePos[0][1]
        Orientation = list(p.getEulerFromQuaternion(basePos[1]))[2]
        goal_direction = math.atan2((schedule[index]["y"] - y), (schedule[index]["x"] - x))
        if(Orientation < 0):
            Orientation = Orientation + 2 * math.pi
        if(goal_direction < 0):
            goal_direction = goal_direction + 2 * math.pi

        theta = goal_direction - Orientation

        if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
            theta = theta + 2 * math.pi
        elif theta > 0 and abs(theta - 2 * math.pi) < theta:
            theta = theta - 2 * math.pi

```

Now that ```theta``` is calculated, we know how misaligned the robot is to the next waypoint as theta is 0 when pointing straight towards the next waypoint. ```cos(theta)``` is 1 when aligned, 0 when sideways and -1 when facing backwards to the next waypoint. 

By using ```cos(theta)``` combined with a linear gain ```k1``` and a angular gain  ```k2``` we can scale the rate of how fast the robot is moving forward and turning. 

```

    k1 = 20
    k2 = 5

    linear = k1 * math.cos(theta)
    angular = k2 * theta

    rightWheelVelocity = linear + angular
    leftWheelVelocity = linear - angular

```

## Exercise 

In the ```single_robot_navigation.py``` script you have cloned, experiment with the following sets of k1 and k2 gains.

| k1 | k2 | 
|-----|-----|
| 40  | 10 |
| 20  | 5  | 
| 10  | 2  | 
| 5   | 20 |

Observe the effect from changing these values in the simulation, and plot the trajectory using the ```matplotlib``` library. Compare the actual path to the desired path provided by the waypoints in the ```output.yaml``` files by plotting them in the same figure.


## References

https://github.com/JacksonChiy/turtlebot_simulation_pybullet
