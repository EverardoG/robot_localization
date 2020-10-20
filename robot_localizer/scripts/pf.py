#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.srv import GetMap
from copy import deepcopy

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

from random import uniform

import math
import time


import numpy as np
from numpy.random import random_sample
from sklearn.neighbors import NearestNeighbors
from occupancy_field import OccupancyField
from helper_functions import TFHelper
from enum import Enum

def build_lidar_marker(t, x, y, marker_id, frame_id, namespace, color_rgb=(0.0,0.0,1.0)):
    """ A helper function for visualizing lidar data """
    marker = Marker()
    marker.header.frame_id = frame_id # "base_link";
    marker.header.stamp = t
    marker.ns = namespace # "lidar_visualization";
    marker.id = marker_id
    marker.type = Marker.CUBE;
    marker.action = Marker.ADD;
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 0.9 # Don't forget to set the alpha!
    marker.color.r = color_rgb[0]
    marker.color.g = color_rgb[1]
    marker.color.b = color_rgb[2]

    return marker

def build_deletion_marker(t, marker_id, namespace):
    """ A helper function for deleting old visualization data """
    marker = Marker()
    marker.id = marker_id
    marker.ns = namespace
    marker.action = Marker.DELETE
    marker.header.stamp = t

    return marker

class ParticleInitOptions(Enum):
    UNIFORM_DISTRIBUTION = 0
    UNIFORM_DISTRIBUTION_HARDCODED = 1
    SINGLE_PARTICLE = 2 # For testing lidar transformations

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative to the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame (meters)
            y: the y-coordinate of the hypothesis relative ot the map frame (meters)
            theta: the yaw of the hypothesis relative to the map frame (radians)
            w: the particle weight (the class does not ensure that particle weights are normalized """
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
        return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))

    def as_marker(self, marker_id):
        """ A helper function to convert to a particle to a visualization_msgs/Marker message """
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
        # return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))
        marker = Marker()
        marker.header.frame_id = "map";
        marker.header.stamp = rospy.Time.now();
        marker.ns = "hypotheses_visualization";
        marker.id = marker_id
        marker.type = Marker.ARROW;
        marker.action = Marker.ADD;
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = orientation_tuple[0];
        marker.pose.orientation.y = orientation_tuple[1];
        marker.pose.orientation.z = orientation_tuple[2];
        marker.pose.orientation.w = orientation_tuple[3];
        marker.scale.x = 0.1 # How long the arrow is
        marker.scale.y = 1/50.0#self.w/50.0 # arrow width - you can have a wide arrow that isn't tall. Arrow is not necessarily a circle
        marker.scale.z = 1/50.0#self.w/50.0
        marker.color.a = 0.9 # Don't forget to set the alpha!
        marker.color.r = 0.0
        green = self.w*750.0
        if green > 1.0: green = 1.0
        marker.color.g = green
        marker.color.b = 0.0

        return marker

    # TODO: define additional helper functions if needed

class ParticleFilter:
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            initialized: a Boolean flag to communicate to other class methods that initializaiton is complete
            base_frame: the name of the robot base coordinate frame (should be "base_link" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            laser_max_distance: the maximum distance to an obstacle we should use in a likelihood calculation
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            laser_subscriber: listens for new scan data on topic self.scan_topic
            tf_listener: listener for coordinate transforms
            tf_broadcaster: broadcaster for coordinate transforms
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            map: the map we will be localizing ourselves in.  The map should be of type nav_msgs/OccupancyGrid
    """
    def __init__(self):
        self.initialized = False        # make sure we don't perform updates before everything is setup
        rospy.init_node('pf')           # tell roscore that we are creating a new node named "pf"

        self.base_frame = "base_link"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from

        self.n_particles = 300          # the number of particles to use
        self.particle_init_options = ParticleInitOptions.UNIFORM_DISTRIBUTION

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update


        self.laser_max_distance = 2.0   # maximum penalty to assess in the likelihood field model

        # TODO: define additional constants if needed

        # Setup pubs and subs

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)
        # laser_subscriber listens for data from the lidar
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_received, queue_size=1)

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        # publish our hypotheses points
        self.hypothesis_pub = rospy.Publisher("hypotheses", MarkerArray, queue_size=10)
        # Publish the lidar scan that pf.py sees
        self.lidar_pub = rospy.Publisher("lidar_visualization", MarkerArray, queue_size=1)
        # Publish the lidar scan projected from the first hypothesis
        self.projected_lidar_pub = rospy.Publisher("projected_lidar_visualization", MarkerArray, queue_size=1)

        # enable listening for and broadcasting coordinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        self.particle_cloud = []

        # change use_projected_stable_scan to True to use point clouds instead of laser scans
        self.use_projected_stable_scan = False
        self.last_projected_stable_scan = None
        if self.use_projected_stable_scan:
            # subscriber to the odom point cloud
            rospy.Subscriber("projected_stable_scan", PointCloud, self.projected_scan_received)

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        self.initialized = True

    def update_robot_pose(self, timestamp):
        """ Update the estimate of the robot's pose given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose
                (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        # TODO: assign the latest pose into self.robot_pose as a geometry_msgs.Pose object
        # just to get started we will fix the robot's pose to always be at the origin
        self.robot_pose = Pose()

        self.transform_helper.fix_map_to_odom_transform(self.robot_pose, timestamp)

    def projected_scan_received(self, msg):
        self.last_projected_stable_scan = msg

    def update_particles_with_odom(self, msg):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.

            msg: this is not really needed to implement this, but is here just in case.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        # TODO: modify particles using delta

    def map_calc_range(self,x,y,theta):
        """ Difficulty Level 3: implement a ray tracing likelihood model... Let me know if you are interested """
        # TODO: nothing unless you want to try this alternate likelihood model
        pass

    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample.
        """
        # make sure the distribution is normalized
        self.normalize_particles()
        
        #cull particles
        #set looping variable values and initalize array to store significant points
        pointIndex = 0
        resamplingNodes = []
        nodeIndex = 0
        #iterate through all points
        while pointIndex < len(self.particle_cloud):
            #If the particle is significantly weighted
            if self.particle_cloud[pointIndex].w > 0.01:
                #add the particle to the list of significant particles (nodes)
                resamplingNodes.append(self.particle_cloud[pointIndex])
                nodeIndex += 1
            pointIndex += 1
       
        print(len(resamplingNodes))
 
        #repopulate field
        #loop through all the significant weighted particles (or nodes in the probability field)
        nodeIndex = 0
        particleIndex = 0
        while nodeIndex < len(resamplingNodes):

            #place points around nodes
            placePointIndex = 0
            #loop through the number of points that need to be placed given the weight of the particle
            while placePointIndex < self.n_particles * resamplingNodes[nodeIndex].w:
                #place point in circular area around node
                radiusRepopCircle = resamplingNodes[nodeIndex].w*10.0
                #create a point in the circular area
                self.particle_cloud[particleIndex] = Particle(uniform((resamplingNodes[nodeIndex].x - radiusRepopCircle),(resamplingNodes[nodeIndex].x + radiusRepopCircle)),uniform((resamplingNodes[nodeIndex].y - radiusRepopCircle),(resamplingNodes[nodeIndex].y + radiusRepopCircle)),uniform(0,2.0 * math.pi))
                #update iteration variables
                particleIndex += 1
                placePointIndex += 1
            nodeIndex += 1
        self.normalize_particles()
    
    def update_particles_with_laser(self, msg):
        """ Updates the particle weights in response to the scan contained in the msg """
        # Note: This only updates the weights. This does not move the particles themselves

        # TODO: try to reduce amount of data
        # maybe try 8 points or some other subsample
        # try just taking the nearest one

        # Filter out invalid ranges
        angle_range_list = []
        for angle, range_ in enumerate(msg.ranges[0:360]):
            if range_ != 0.0: # valid range
                angle_range_list.append((angle,range_))

        # Transform ranges into numpy array of xs and ys
        relative_to_robot = np.zeros((len(angle_range_list), 2))
        for index, (angle, range_) in enumerate(angle_range_list):
            relative_to_robot[index, 0] = range_ * np.cos(angle*np.pi/180.0) # xs
            relative_to_robot[index, 1] = range_ * np.sin(angle*np.pi/180.0)# ys

        # Build up an array of lidar markers for visualization
        lidar_markers = []
        for index, xy_point in enumerate(relative_to_robot):
            lidar_markers.append(build_lidar_marker(msg.header.stamp, xy_point[0], xy_point[1], index, "base_link", "lidar_visualization", (1.0,0.0,0.0)))

        # Make sure to delete any old markers
        num_deletion_markers = 360 - len(lidar_markers)
        for _ in range(num_deletion_markers):
            marker_id = len(lidar_markers)
            lidar_markers.append(build_deletion_marker(msg.header.stamp, marker_id, "lidar_visualization"))

        # Publish lidar points for visualization
        self.lidar_pub.publish(MarkerArray(markers=lidar_markers))

        # For every particle (hypothesis) we have
        for particle in self.particle_cloud:
            # Combine the xy positions of the scan with the xy w of the hypothesis
            # Rotation matrix could be helpful here (https://en.wikipedia.org/wiki/Rotation_matrix)

            # Build our rotation matrix
            R = np.array([[np.cos(particle.theta), -np.sin(particle.theta)],[np.sin(particle.theta), np.cos(particle.theta)]])

            # Rotate the points according to particle orientation
            relative_to_particle = (R.dot(relative_to_robot.T)).T
            # relative_to_particle = relative_to_robot.dot(R)

            # Translate points to be relative to map origin
            relative_to_map = deepcopy(relative_to_particle)
            relative_to_map[:,0:1] = relative_to_map[:,0:1] + particle.x * np.ones((relative_to_map.shape[0],1))
            relative_to_map[:,1:2] = relative_to_map[:,1:2] + particle.y * np.ones((relative_to_map.shape[0],1))

            # Get the distances of each projected point to nearest obstacle
            distance_list = []
            for xy_projected_point in relative_to_map:
                distance = self.occupancy_field.get_closest_obstacle_distance(xy_projected_point[0], xy_projected_point[1])
                if not np.isfinite(distance):
                    # Note: ac109 map has approximately a 10x10 bounding box
                    # Hardcode 1m as the default distance in case the projected point is off the map
                    distance = 1.0
                distance_list.append(distance)

            # Calculate a weight for for this particle
            # Note: The further away a projected point is from an obstacle point,
            #       the lower its weight should be
            weight = 1.0 / sum(distance_list)
            particle.w = weight

        # Normalize the weights
        self.normalize_particles()

        # Grab the first particle
        particle = self.particle_cloud[0]

        # Visualize the projected points around that particle
        projected_lidar_markers = []
        for index, xy_point in enumerate(relative_to_map):
            projected_lidar_markers.append(build_lidar_marker(msg.header.stamp, xy_point[0], xy_point[1], index, "map", "projected_lidar_visualization"))

        # Make sure to delete any old markers
        num_deletion_markers = 360 - len(projected_lidar_markers)
        for _ in range(num_deletion_markers):
            marker_id = len(projected_lidar_markers)
            projected_lidar_markers.append(build_deletion_marker(msg.header.stamp, marker_id, "projected_lidar_visualization"))

        # Publish the projection visualization to rviz
        self.projected_lidar_pub.publish(MarkerArray(markers=projected_lidar_markers))

        # Build up a list of all the particles as Rviz Markers
        particle_markers = [particle.as_marker(count) for count, particle in enumerate(n.particle_cloud)]

        # Publish the visualization of all the particles in Rviz
        n.hypothesis_pub.publish(MarkerArray(markers=particle_markers))

    @staticmethod
    def draw_random_sample(choices, probabilities, n):
        """ Return a random sample of n elements from the set choices with the specified probabilities
            choices: the values to sample from represented as a list
            probabilities: the probability of selecting each element in choices represented as a list
            n: the number of samples
        """
        values = np.array(range(len(choices)))
        probs = np.array(probabilities)
        bins = np.add.accumulate(probs)
        inds = values[np.digitize(random_sample(n), bins)]
        samples = []
        for i in inds:
            samples.append(deepcopy(choices[int(i)]))
        return samples

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """

        # TODO: Move the xy_theta stuff to where the robot initializes around a given set of points
        # if xy_theta is None:
        #     xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)

        # TODO create particles

        # Check how the algorithm should initialize its particles

        # Distribute particles uniformly with parameters defining the number of particles and bounding box
        if self.particle_init_options == ParticleInitOptions.UNIFORM_DISTRIBUTION:
            #create an index to track the x cordinate of the particles being created

            #calculate the number of particles to place widthwize vs hightwize along the map based on the number of particles and the dimensions of the map
            num_particles_x = math.sqrt(self.n_particles)
            num_particles_y = num_particles_x

            index_x = -20
            #iterate over the map to place points in a uniform grid
            while index_x < 15:

                index_y = -20
                while index_y < 15:
                    #create a particle at the location with a random orientation
                    new_particle = Particle(index_x,index_y,uniform(0,2 * math.pi))
                    #add the particle to the particle array
                    self.particle_cloud.append(new_particle)

                    #increment the index to place the next particle
                    index_y += 35/(num_particles_y)
                #increment index to place next column of particles
                index_x += 35/num_particles_x

        # Distribute particles uniformly, but hard-coded (mainly for quick tests)
        elif self.particle_init_options == ParticleInitOptions.UNIFORM_DISTRIBUTION_HARDCODED:
            # Make a list of hypotheses that can update based on values
            xs = np.linspace(-3,4,21)
            ys = np.linspace(-4,3,21)
            for y in ys:
                for x in xs:
                    for i in range(5):
                        new_particle = Particle(x,y,np.random.uniform(0,2 * math.pi))
                        self.particle_cloud.append(new_particle)

        elif self.particle_init_options == ParticleInitOptions.SINGLE_PARTICLE:
            new_particle = Particle(1.0,1.0, math.pi/2.0)
            self.particle_cloud.append(new_particle)

        # TODO: Optional, make a ParticleInitOptions for setting initial conditions

        # TODO: Re-enable these lines of code
        self.normalize_particles()
        # self.update_robot_pose(timestamp)

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        #set variable inital values
        index = 0
        weightSum = 0

        # calulate the total particle weight
        while index < len(self.particle_cloud):
            weightSum += self.particle_cloud[index].w
            index += 1
        index = 0

        #normalize the weight for each particle by divifdng by the total weight
        while index < len(self.particle_cloud):
            self.particle_cloud[index].w = self.particle_cloud[index].w / weightSum
            index += 1

        pass

    def publish_particles(self, msg):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.map_frame),
                                  poses=particles_conv))

    def scan_received(self, msg):
        """ This is the default logic for what to do when processing scan data.
            Feel free to modify this, however, we hope it will provide a good
            guide.  The input msg is an object of type sensor_msgs/LaserScan """
        if not(self.initialized):
            # wait for initialization to complete
            return

        if not(self.tf_listener.canTransform(self.base_frame, msg.header.frame_id, msg.header.stamp)):
            # need to know how to transform the laser to the base frame
            # this will be given by either Gazebo or neato_node
            return

        if not(self.tf_listener.canTransform(self.base_frame, self.odom_frame, msg.header.stamp)):
            # need to know how to transform between base and odometric frames
            # this will eventually be published by either Gazebo or neato_node
            return

        # calculate pose of laser relative to the robot base
        p = PoseStamped(header=Header(stamp=rospy.Time(0),
                                      frame_id=msg.header.frame_id))
        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # find out where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=msg.header.stamp,
                                      frame_id=self.base_frame),
                        pose=Pose())
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
        # store the the odometry pose in a more convenient format (x,y,theta)
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        if not(self.particle_cloud):
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif (math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh):
            # we have moved far enough to do an update!
            self.update_particles_with_odom(msg)    # update based on odometry
            if self.last_projected_stable_scan:
                last_projected_scan_timeshift = deepcopy(self.last_projected_stable_scan)
                last_projected_scan_timeshift.header.stamp = msg.header.stamp
                self.scan_in_base_link = self.tf_listener.transformPointCloud("base_link", last_projected_scan_timeshift)

            self.update_particles_with_laser(msg)   # update based on laser scan
            self.update_robot_pose(msg.header.stamp)                # update robot's pose
            self.resample_particles()               # resample particles to focus on areas of high density
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg)

if __name__ == '__main__':
    n = ParticleFilter()
    r = rospy.Rate(5)

    while not(rospy.is_shutdown()):
        try:
            rospy.loginfo("heartbeat")
            # in the main loop all we do is continuously broadcast the latest map to odom transform
            n.transform_helper.send_last_map_to_odom_transform()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
