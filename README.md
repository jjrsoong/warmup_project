# warmup_project

**Driving in a Square**
For this exercise I decided to employ a time-based approach. While potentially less accurate than the odometry approach, I chose this method because I am still familizaring myself with Turtlebot and object-oriented programing.

I created two Twist() commands, one for moving in a straight line and a constant speed and the other for turning. Since my turn command goes at 30 degrees a second, I initilly had my robot turn for 3 seconds. However, I suspect that between the noise and the acceleration required for the robot to reach the 30 degrees/second, a little extra time was needed -- this was done by trial and error.

Then, I simply pieced the forward() and turn() commands together via an infinite while loop. I used time.time() to keep track of the time that elapsed and when the robot should switch between forward() and turn() (and vice versa).

//Code structure:
This program uses a SquareDriver class. The run() definition keeps the program running, while the initization, the turn(), and the forward() commands are executed in the __init__ def via the time-based approach described above.
