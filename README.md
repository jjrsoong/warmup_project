# warmup_project

**Driving in a Square**
For this exercise I decided to employ a time-based approach. While potentially less accurate than the odometry approach, I chose this method because I am still familizaring myself with Turtlebot and object-oriented programing.

I created two Twist() commands, one for moving in a straight line and a constant speed and the other for turning. Since my turn command goes at 30 degrees a second, I initilly had my robot turn for 3 seconds. However, I suspect that between the noise and the acceleration required for the robot to reach the 30 degrees/second, a little extra time was needed -- this was done by trial and error.

Then, I simply pieced the forward() and turn() commands together via an infinite while loop. I used time.time() to keep track of the time that elapsed and when the robot should switch between forward() and turn() (and vice versa).

//Code structure:
This program uses a SquareDriver class. The run() definition keeps the program running, while the initization, the turn(), and the forward() commands are executed in the __init__ def via the time-based approach described above.

![Drive in a Square Gif](drive_square.gif)

**Wall Follower**
The bot divides the LIDAR scan into several distinct ranges and uses the minimum distance from each range to determine its linear and angular velocity. Broadly speaking, if the front of the bot is free of walls it will move forward (the speed varies depending on the wall's distance from other sides of the bot).

If there is a wall in front of the bot (defined as the 45 degree arc from the bot's direct front), it will execute a turn (turning speed varies depending on the walls angle from the bot's front).

Finally, as a failsafe, if the bot deviates too far from the way it will execute to turn to take it back closer to the wall.


![Wall Follwer Gif](wall_follower.gif)

**Person Follower**
Similar to Wall Follower, Person Follower divides the bot's 360 degree LIDAR scans into 9 distinct regions. There are four regions on each side of the bot with the ninth region corresponding to the 30 degree area directly in front of the bot. Each region triggers a different angular velocity, and code was written in such a way so that instructions from regions behind the bot are overwritten by instructions coming from regions closer to the front of the bot.

In the edge case where the person is directly behind the bot, the bot will prefer to turn to its right to reach the person.

There are three different "switches" to control linear speed. If the bot is far from the person, it will move quickly (currently 1m/sec). If it moves past a deceleration distance, it will slow down to 0.2 m/sec. Finally, if it overshoots its stopping distance from the person, it will use proportional control to back upto the correct stopping distance.


![Person Follower Gif](person_follower.gif)
