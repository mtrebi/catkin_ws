# turtlebot
Turtulebot repo. 

Implementation of Bug0 navigation strategy with a Turtlebot.

Bug0 Pseudocode:

1. Head toward goal
2. Follow obstacles until you can head toward goal again
3. Continue

To do so we use:

- Subscribe to Odometry to do positioning (known current position & goal)
- Subscribe to  Laser Scan to detect objects
- Publish to Speed (Linear & Angular)

We have defined some global vars to customize the behaviour of the Bug0 algorithm:
distance (m) detection of obstacte when the robot stop and begin to turn 
self.obstacle_threshold = 1

distance (m) obstacle_threshold+obstacle_threshold_add when the robot detects that the robot i more far starts to go fowrward to avoid it
- self.obstacle_threshold_add = 0.25

distance (m) in status 2 that the robot move to try avoid the obstracle and go to the goal after turn in front of obstracle
- self.distance_try_avoid_obstacle = 0.4

this linear variable * distance to goal lets to make a variable velocity
- self.linear_constant=0.5

max speed of the robot, if is very high is possible that the robot don't have enought time for stop whith obstabce
- self.max_speed=0.25

- self.accepted_error_try_avoid_obstable = 0.1

degrees/sec that turns the robot when detects obstable.
- self.turn_speed = 25 

this linear velocity variable * distance to goal lets to make a variable linear velocity
- self.head_toward_linear_constant=1

this angular velocity variable * distance to goal lets to make a variable angular velocity
- self.head_toward_angular_constant=2

#Videos
![Turtlebo2 avoiding object](https://www.youtube.com/watch?v=7wiXgdLNfO0&list=PLeGS7otZ9mSc-kfSqJHZLcTUpjxLSWuh7)
[Turtlebo2 avoiding object](https://www.youtube.com/watch?v=7wiXgdLNfO0&list=PLeGS7otZ9mSc-kfSqJHZLcTUpjxLSWuh7 "Turtlebo2 avoiding object")

Checkout all the videos in my [youtube playlist](https://www.youtube.com/playlist?list=PLeGS7otZ9mSc-kfSqJHZLcTUpjxLSWuh7)
