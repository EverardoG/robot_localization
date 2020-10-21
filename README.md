# Robot Localization With Particle Clouds
This is the base repo for the Olin Computational Robotics Robot Localization project. The goal of this project was to develop a particle filter which will allow a robot to locate itself in an environment if it has a pre existing map of the sensor readings in the environment.
## How it works in broad strokes
The robot generates a collection of possible locations in the map that it could be. The robot then compares the sensor readings at each of these locations to the map and its own sensors to determine how 'likely' the robot is to be at each of these locations. The robot weights the possible locations to represent this likelihood. Then the robot generates a new set of possible locations around the highest weighted points from the previous round. The robot then moves around its environment and computes where each possible location would be based on the movement the robot just executed. The robot then repeats the process by comparing its sensor values to those at each of the possible locations.

Over time the robot's possible positions estimates will converge on the most probable locations for the robot to be in the map with possible location estimates 'clustering around the most probable robot locations.

## Our Approach

## Interesting Extensions
Particle filters of this type are usually used to refine a robot's position and compensate for drift They often require an initial pose estimate, an estimate which represents an approximation of the robots location and orientation in the map. We wanted the robot to be able to localize itself in its environment without any understanding of its initial pose. To accomplish this we distributed particles in a uniform distribution across the map at the Beginning of localization. This then describes the state that the robot could be anywhere in the map with similar likelihood before it can collapse its particle cloud down to a smaller set of locations. 


An example of this convergence behavior can be seen here: 

![A video Depiction of a point cloud algorithm collapsing to localize the robot. Possible robot locations represented by arrows start thinly distributed across a map then gradually move together and concentrate in certain areas as the robot moves around. Gradually ares get stronger until there is a cluster of Arrows following the robot](robot_localizer/gifs/ac109_1_test.gif)

### Implications of this decision
Because we chose to have a more generalized localization approach our algorithm takes longer to converge before it discovers the true location of the robot.

Additionally, you can see in the gif that the particle filter can get tripped up with similar locations in that it got stuck between locating the robot at the ends of the hallways in the designated map. This is caused from the particle filter not having enough information to distinguish effectively between the two locations, where implementations which feature an initial pose do not tend to run into multiple similar conditions because they have the extra information of a starting point which will exclude possible 'similar solutions' found elsewhere in the map.

## Decisions on a smaller scale
We not only made important design decisions in the broader strokes of the project to create an engaging project which stretched a bit beyond the requirements, but we also made many decisions on smaller scales which provided further depth to the project. One good example of this is our implementation of the resampling step of the filter. The resampling step is where the algorithm generates a new particle cloud around the most likely locations in the old particle cloud. Many particle filters of this type generate new particles at the exact location of the highest weighted particles in the particle cloud of the previous step. Because of this, these particle filters are precise but they can never be more accurate than the closest original particle propagated through time. In our implementation we resample the points in a circular area around each of the highly weighted particles. This allows the particle cloud to test similar positions which are not dependent on the locations of the initial guesses. In this way the accuracy of the filter can be continually refined. The trade off here is that the accuracy of the particle cloud is increased the precision of our guess is decreased. This can be seen in the point cloud collapsing gif above as even when it locates the robot, the cluster of points is still quite large and the 'high likelihood' points spread for about half a meter around the robot.
