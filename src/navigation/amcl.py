#!/usr/bin/env python

import sys
import rospy
import math
import random
import numpy as np
from geometry_msgs.msg import (Pose, Transform, PoseWithCovarianceStamped,
                               PoseArray, Quaternion, TransformStamped)
from tf.msg import tfMessage
from tf import transformations
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from copy import deepcopy
from threading import Lock
from navigation import sensor_model 
from ..utils.geom import getHeading, rotateQuaternion
from navigation.histogram import Histogram

PI_OVER_TWO = math.pi/2


class Amcl():

    def WS_LAST_FUNC(self, x): return np.mean(np.square(x))

    # Parameters
    INIT_X = 0 		    # Initial x location of robot (metres)
    INIT_Y = 0			# Initial y location of robot (metres)
    INIT_Z = 0 			# Initial z location of robot (metres)
    INIT_HEADING = 0 	# Initial orientation of robot (radians)

    # Set motion model parameters
    ODOM_ROTATION_NOISE = .1
    ODOM_TRANSLATION_NOISE = .1
    ODOM_DRIFT_NOISE = .1

    NUMBER_PREDICTED_READINGS = 70  # Number of Initial Samples

    MAX_NUM_SKIP_UPDATES = 2  # Number of updates the cloud is not updated

    KIDNAP_THRESHOLD = 100

    def __init__(self):
        # Minimum change (m/radians) before publishing new particle cloud and pose
        self._PUBLISH_DELTA = rospy.get_param("publish_delta", 0.1)

        # Initialize Fields
        self.occupancy_map = OccupancyGrid()
        self.particlecloud = PoseArray()
        self.sensor_model = sensor_model.SensorModel()
        self.estimated_pose = PoseWithCovarianceStamped()
        self.tf_message = tfMessage()
        self._update_lock = Lock()

        self.latest_scan = None
        self.prev_scan = None
        self.last_published_pose = None

        self.num_last_updates = 0
        self.ws_last_eval = [-1] * 10
        self.clustered = True

        # Set 'previous' translation to origin
        # All Transforms are given relative to 0,0,0, not in absolute coords.
        self.prev_odom_x = 0.0
        self.prev_odom_y = 0.0
        self.prev_odom_heading = 0.0
        self.last_odom_pose = None

        # Request robot's initial odometry values to be recorded in prev_odom
        self.odom_initialised = False
        self.sensor_model_initialised = False

        # Publishers
        self.amcl_pose_publisher = rospy.Publisher(
            "/amcl_pose", PoseWithCovarianceStamped)
        self.particle_cloud_publisher = rospy.Publisher(
            "/particlecloud", PoseArray)
        self.tf_publisher = rospy.Publisher("/tf", tfMessage)

        # Get Map
        rospy.loginfo("Waiting for a map...")
        try:
            self.occupancy_map = rospy.wait_for_message(
                "/map", OccupancyGrid, 20)
        except:
            rospy.logerr("Problem getting a map. Check that you have a map_server"
                         " running: rosrun map_server map_server <mapname> ")
            sys.exit(1)

        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (self.occupancy_map.info.width, self.occupancy_map.info.height,
                       self.occupancy_map.info.resolution))

        self.set_map()

        # Map parameters
        width = self.occupancy_map.info.width
        height = self.occupancy_map.info.height
        origin = self.occupancy_map.info.origin.position
        resolution = self.occupancy_map.info.resolution

        # Initialize histogram based on map parameters
        self.histogram = Histogram(width * resolution,
                                   height * resolution, origin)

        self._initialize_estimated_pose()
        self.last_published_pose = deepcopy(self.estimated_pose)

        # Subscribers
        self.laser_subscriber = rospy.Subscriber("/base_scan", LaserScan,
                                                 self.laser_callback,
                                                 queue_size=1)
        self.initial_pose_subscriber = rospy.Subscriber("/initialpose",
                                                        PoseWithCovarianceStamped,
                                                        self.initial_pose_callback)
        self.odometry_subscriber = rospy.Subscriber("/odom", Odometry,
                                                    self.odometry_callback,
                                                    queue_size=1)

    def laser_callback(self, scan):
        """
        Laser received. Store a ref to the latest scan. If robot has moved
        much, republish the latest pose to update RViz
        """
        self.latest_scan = scan
        #rate = rospy.Rate(20) # 10hz
        #if self._sufficientMovementDetected(self.estimated_pose):
        #while not rospy.is_shutdown():
        # Publish the new pose
        self.amcl_pose_publisher.publish(
            self.estimated_pose)

        # Update record of previously-published pose
        self.last_published_pose = deepcopy(
            self.estimated_pose)

        # Get updated particle cloud and publish it
        self.particle_cloud_publisher.publish(
            self.particlecloud)

        # Get updated transform and publish it
        self.tf_publisher.publish(self.tf_message)

        #rate.sleep()

    def initial_pose_callback(self, initialpose):
        """ called when RViz sends a user supplied initial pose estimate """
        rospy.loginfo("Received initialpose")
        self.set_initial_pose(initialpose)
        self.last_published_pose = deepcopy(self.estimated_pose)
        self.particle_cloud_publisher.publish(self.particlecloud)
        rospy.loginfo("Published initialpose")

    def odometry_callback(self, odometry):
        """
        Odometry received. If the filter is initialised then execute
        a filter predict step with odeometry followed by an update step using
        the latest laser.
        """
        if not self.latest_scan == None:
            self.predict_from_odometry(odometry)
            self.update_filter(self.latest_scan)

    def update_filter(self, scan):
        """
        Called whenever there is a new LaserScan message.
        This calls update methods to do actual
        particle filtering, given the map and the LaserScan, and then updates
        Transform tf appropriately.

        :Args:
            |  scan (sensor_msgs.msg.LaserScan) latest laser scan to resample
               the particle filter based on
        """
        if not self.sensor_model_initialised:
            self.sensor_model.set_laser_scan_parameters(self.NUMBER_PREDICTED_READINGS,
                                                        scan.range_max,
                                                        len(scan.ranges),
                                                        scan.angle_min,
                                                        scan.angle_max)
            self.sensor_model_initialised = True
        with self._update_lock:
            # Call particle filter update method
            self.update_particle_cloud(scan)
            self.particlecloud.header.frame_id = "map"
            self.estimated_pose.pose.pose = self.estimate_pose()
            currentTime = rospy.Time.now()

            # Given new estimated pose, now work out the new transform
            self.recalculate_transform(currentTime)
            # Insert correct timestamp in particlecloud and estimated_pose,
            # so extending subclasses don't need to worry about this, but can
            # just concentrate on updating actual particle and pose locations
            self.particlecloud.header.stamp = currentTime
            self.estimated_pose.header.stamp = currentTime

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update
        """
        # If the sensor data is the same, don't update particle cloud
        if self.prev_scan is not None:
            if self._rel_error(scan.ranges, self.prev_scan.ranges) < 1e-9:
                return

        # Store previous scan measurements
        self.prev_scan = scan

        # Increment num last updates
        self.num_last_updates += 1

        # Skip an update every `MAX_NUM_SKIP_UPDATES` times for efficiency
        if self.num_last_updates <= self.MAX_NUM_SKIP_UPDATES:
            return
        else:
            self.num_last_updates = 0

        # Generate importance weights based on scan readings
        ws = [self.sensor_model.get_weight(scan, pose)
              for pose in self.particlecloud.poses]

        # Update last weight evaluations with the most recent evaluation
        self.ws_last_eval = [self.WS_LAST_FUNC(ws)] + self.ws_last_eval[:-1]

        # Weights should sum up to 1
        ws /= np.sum(ws)

        # Resample AMCL algorithm
        self.particlecloud = self.resample_amcl(self.particlecloud, ws)

    def _generate_random_poses(self, num_poses=None):
        """Generates random poses uniformly across the map.

        This method generates `num_poses` random poses within the
        boundaries of a map. All poses face randomly.

        Args:
            num_poses (int): the amount of poses to generate. If not
                provided, `self.NUMBER_PREDICTED_READINGS` is used.
        Return:
            (geometry_msgs.msg.PoseArray): random particle poses
        """
        # Set default value
        if num_poses is None:
            num_poses = self.NUMBER_PREDICTED_READINGS
        
        # Initialize random particle array
        poses_uniform = PoseArray()

        # Define map constants
        width = self.occupancy_map.info.width
        height = self.occupancy_map.info.height
        origin = self.occupancy_map.info.origin.position
        resolution = self.occupancy_map.info.resolution

        # While we don't have `num_poses` particles
        while len(poses_uniform.poses) < num_poses:
            y = np.random.randint(height)
            x = np.random.randint(width)

            # If the random position is within map boundaries
            if self.occupancy_map.data[y * width + x] != -1:
                pose = Pose()
                theta = np.random.uniform(-np.pi, np.pi)
                pose.position.x = x * resolution + origin.x
                pose.position.y = y * resolution + origin.y
                pose.orientation = rotateQuaternion(Quaternion(w=1), theta)

                poses_uniform.poses.append(pose)

        return poses_uniform    
    
    def resample_amcl(self, poses, ws):
        """Performs full AMCL (KLD version) resampling.

        The method utilizes helper function to resample particles in an
        adaptive KLD-based way.

        Args:
            poses (geometry_msgs.msg.PoseArray): the particle poses
            ws (list): the list of importance weights for each pose
        Return:
            (geometry_msgs.msg.PoseArray): resampled particle poses
        """
        SPREAD_THRESHOLD = 150

        if not self.clustered:
            poses = self._resample_mcl_base(poses, ws)

        poses = self.resample_amcl_base(poses, ws)

        # Keep track of robot's certainty of the pose
        rospy.loginfo(f"Certainty: {np.mean(self.ws_last_eval):.4f} | " +
                      f"Threshold: {self.KIDNAP_THRESHOLD}")

        # If weights are low, dissolve particles
        #if len(poses.poses) > SPREAD_THRESHOLD and \
        if -1 not in self.ws_last_eval and self.clustered and \
           np.mean(self.ws_last_eval) <= self.KIDNAP_THRESHOLD:
            # Reinitialize the trailing of weight eval values
            self.ws_last_eval = list(map(lambda _: -1, self.ws_last_eval))
            poses = self._generate_random_poses(len(poses.poses))
            self.clustered = False

            return poses

        if len(poses.poses) <= SPREAD_THRESHOLD:
            self.clustered = True

        return poses

    def _resample_mcl_base(self, poses, ws):
        """Performs Stochastic Universal Resampling.

        This is the core resampling method for the MCL algorithm. All
        it does is simply goes through the CDF of weight distribution
        and resamples those particles with the highest weights.

        Note:
            Passed weights, i.e., `ws` must sum up to `1`.

        Args:
            poses (geometry_msgs.msg.PoseArray): the particle poses
            ws (list): the list of importance weights for each pose
        Return:
            (geometry_msgs.msg.PoseArray): resampled particle poses
        """
        # Initialize resampled particle array
        poses_resampled = PoseArray()

        # Initialize the algorithm
        i = 0
        M_inv = 1 / len(ws)
        cdf = np.cumsum(ws)
        us = [M_inv - np.random.uniform(0, M_inv)]

        # Filter by contributions of particles
        for _ in ws:
            while us[-1] > cdf[i]:
                i += 1
            us.append(us[-1] + M_inv)
            poses_resampled.poses.append(self._get_noisy_pose(poses.poses[i]))

        return poses_resampled

    def resample_amcl_base(self, poses, ws):
        """Performs KLD adaptive resampling.

        This is the core resampling method for the AMCL algorithm.
        Here, a dynamic size of particlecloud is determined by KL
        distance measure while particles are being sampled from a
        weighted distribution.

        Note:
            Passed weights, i.e., `ws` must sum up to `1`.

        Args:
            poses (geometry_msgs.msg.PoseArray): the particle poses
            ws (list): the list of importance weights for each pose
        :Return:
            (geometry_msgs.msg.PoseArray): resampled particle poses
        """
        # Initialize resampled particle array
        poses_resampled = PoseArray()

        # KLD sampling initialization
        MAX_NUM_PARTICLES = 500
        eps = 0.08
        z = 0.99
        Mx = 0

        # Assure no bins are prerecorded
        self.histogram.non_empty_bins.clear()

        # While not min or KLD calculated samples reached
        while len(poses_resampled.poses) < Mx or \
                len(poses_resampled.poses) < self.NUMBER_PREDICTED_READINGS:
            # Sample random pose, add it to resampled list
            pose = np.random.choice(poses.poses, p=ws)
            pose = self._get_noisy_pose(pose)
            poses_resampled.poses.append(pose)

            # If the pose falls into empty bin
            if self.histogram.add_if_empty(pose):
                # Number of current non-empty bins
                k = len(self.histogram.non_empty_bins)

                # Update KL distance
                if k > 1:
                    Mx = ((k - 1) / (2 * eps)) * \
                        math.pow(1 - (2 / (9 * (k - 1))) +
                                 math.sqrt(2 / (9 * (k - 1))) * z, 3)

                # Don't exceed the maximum allowed range
                Mx = MAX_NUM_PARTICLES if Mx > MAX_NUM_PARTICLES else Mx

        # Keep track of num particles generated
        rospy.loginfo(f"Generated {len(poses_resampled.poses)} particles")

        return poses_resampled

    def _rel_error(self, x, y):
        """Computes the relative error between 2 values or vectors."""
        x, y = np.array(x), np.array(y)
        return np.max(np.abs(x - y) / (np.maximum(1e-8, np.abs(x) + np.abs(y))))

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).

        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.

        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
        """
        # Initialize estimated position
        pose_hat = Pose()

        # Initialize lists of pose estimates
        pxs, pys, oxs, oys, ozs, ows = [], [], [], [], [], []

        # Generate lists for each pose estimate
        for pose in self.particlecloud.poses:
            pxs.append(pose.position.x)
            pys.append(pose.position.y)
            oxs.append(pose.orientation.x)
            oys.append(pose.orientation.y)
            ozs.append(pose.orientation.z)
            ows.append(pose.orientation.w)

        # Filter the best estimates and average the result
        pose_hat.position.x = np.mean(self._filter_naive(pxs))
        pose_hat.position.y = np.mean(self._filter_naive(pys))
        pose_hat.orientation.x = np.mean(self._filter_naive(oxs))
        pose_hat.orientation.y = np.mean(self._filter_naive(oys))
        pose_hat.orientation.z = np.mean(self._filter_naive(ozs))
        pose_hat.orientation.w = np.mean(self._filter_naive(ows))

        return pose_hat

    def _filter_naive(self, estimates, num_keep=None):
        """Naively filters the best (closest to mean) value estimates.

        Args:
            estimates (list): list of values which estimate the pose
            num_keep (int):
        Return:
            (list): list of best estimates based on `num_keep`
        """
        # Num best particles to keep
        if num_keep is None:
            num_keep = self.NUMBER_PREDICTED_READINGS // 2

        # Sorted indeces of `estimates - mean`
        ind = np.argsort(np.abs(estimates - np.mean(estimates)))

        return np.take(estimates, ind)[:num_keep]

    def recalculate_transform(self, currentTime):
        """
        Creates updated transform from /odom to /map given recent odometry and
        laser data.

        :Args:
            | currentTime (rospy.Time()): Time stamp for this update
         """
        transform = Transform()

        T_est = transformations.quaternion_matrix([self.estimated_pose.pose.pose.orientation.x,
                                                   self.estimated_pose.pose.pose.orientation.y,
                                                   self.estimated_pose.pose.pose.orientation.z,
                                                   self.estimated_pose.pose.pose.orientation.w])
        T_est[0, 3] = self.estimated_pose.pose.pose.position.x
        T_est[1, 3] = self.estimated_pose.pose.pose.position.y
        T_est[2, 3] = self.estimated_pose.pose.pose.position.z

        T_odom = transformations.quaternion_matrix([self.last_odom_pose.pose.pose.orientation.x,
                                                   self.last_odom_pose.pose.pose.orientation.y,
                                                   self.last_odom_pose.pose.pose.orientation.z,
                                                   self.last_odom_pose.pose.pose.orientation.w])
        T_odom[0, 3] = self.last_odom_pose.pose.pose.position.x
        T_odom[1, 3] = self.last_odom_pose.pose.pose.position.y
        T_odom[2, 3] = self.last_odom_pose.pose.pose.position.z
        T = np.dot(T_est, np.linalg.inv(T_odom))
        q = transformations.quaternion_from_matrix(T)  # [:3, :3])

        transform.translation.x = T[0, 3]
        transform.translation.y = T[1, 3]
        transform.translation.z = T[2, 3]
        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]

        # Insert new Transform into a TransformStamped object and add to the tf tree
        new_tfstamped = TransformStamped()
        new_tfstamped.child_frame_id = "odom"
        new_tfstamped.header.frame_id = "map"
        new_tfstamped.header.stamp = currentTime
        new_tfstamped.transform = transform

        # Add the transform to the list of all transforms
        self.tf_message = tfMessage(transforms=[new_tfstamped])

    def predict_from_odometry(self, odom):
        """
        Adds the estimated motion from odometry readings to each of the
        particles in particlecloud.

        :Args:
            | odom (nav_msgs.msg.Odometry): Recent Odometry data
        """
        with self._update_lock:
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            new_heading = getHeading(odom.pose.pose.orientation)

            # On our first run, the incoming translations may not be equal to zero, so set them appropriately
            if not self.odom_initialised:
                self.prev_odom_x = x
                self.prev_odom_y = y
                self.prev_odom_heading = new_heading
                self.odom_initialised = True

            # Find difference between current and previous translations
            dif_x = x - self.prev_odom_x
            dif_y = y - self.prev_odom_y
            dif_heading = new_heading - self.prev_odom_heading
            if dif_heading > math.pi:
                dif_heading = (math.pi * 2) - dif_heading
            if dif_heading < -math.pi:
                dif_heading = (math.pi * 2) + dif_heading

            # Update previous pure odometry location (i.e. excluding noise) with the new translation
            self.prev_odom_x = x
            self.prev_odom_y = y
            self.prev_odom_heading = new_heading
            self.last_odom_pose = odom

            # Find robot's linear forward/backward motion, given the dif_x and dif_y changes and its orientation
            distance_travelled = math.sqrt(dif_x*dif_x + dif_y*dif_y)
            direction_travelled = math.atan2(dif_y, dif_x)
            temp = abs(new_heading - direction_travelled)

            if temp < -PI_OVER_TWO or temp > PI_OVER_TWO:
                # We are going backwards
                distance_travelled = distance_travelled * -1

            # Update each particle with change in position (plus noise)
            for p in self.particlecloud.poses:

                rnd = random.normalvariate(0, 1)

                # Rotate particle according to odometry rotation, plus  noise
                p.orientation = (rotateQuaternion(p.orientation,
                                                  dif_heading + rnd * dif_heading * self.ODOM_ROTATION_NOISE))

                # Get particle's new orientation
                theta = getHeading(p.orientation)

                # Find translation in the direction of particle's orientation
                travel_x = distance_travelled * math.cos(theta)
                travel_y = distance_travelled * math.sin(theta)
                p.position.x = p.position.x + travel_x + \
                    (rnd * travel_x * self.ODOM_TRANSLATION_NOISE)
                p.position.y = p.position.y + travel_y + \
                    (rnd * travel_y * self.ODOM_DRIFT_NOISE)

    def set_map(self):
        """ Set the map for localisation """
        self.sensor_model.set_map(self.occupancy_map)
        # Map has changed, so we should reinitialise the particle cloud
        rospy.loginfo("Particle filter got map. (Re)initialising.")
        self.particlecloud = self.initialise_particle_cloud(
            self.estimated_pose)
        self.particlecloud.header.frame_id = "map"

    def _sufficientMovementDetected(self, latest_pose):
        """
        Compares the last published pose to the current pose. Returns true
        if movement is more the self._PUBLISH_DELTA
        """
        # Check that minimum required amount of movement has occurred before re-publishing
        latest_x = latest_pose.pose.pose.position.x
        latest_y = latest_pose.pose.pose.position.y
        prev_x = self.last_published_pose.pose.pose.position.x
        prev_y = self.last_published_pose.pose.pose.position.y
        location_delta = abs(latest_x - prev_x) + abs(latest_y - prev_y)

        # Also check for difference in orientation: Take a zero-quaternion,
        # rotate forward by latest_rot, and rotate back by prev_rot, to get difference)
        latest_rot = latest_pose.pose.pose.orientation
        prev_rot = self.last_published_pose.pose.pose.orientation

        q = rotateQuaternion(Quaternion(w=1.0),
                             getHeading(latest_rot))   # Rotate forward
        q = rotateQuaternion(q, -getHeading(prev_rot))  # Rotate backward
        heading_delta = abs(getHeading(q))
        #rospy.loginfo("Moved by %f"%location_delta)
        return (location_delta > self._PUBLISH_DELTA or
                heading_delta > self._PUBLISH_DELTA)

    def _initialize_estimated_pose(self):
        # Set default initial pose to initial position and orientation.
        self.estimated_pose.pose.pose.position.x = self.INIT_X
        self.estimated_pose.pose.pose.position.y = self.INIT_Y
        self.estimated_pose.pose.pose.position.z = self.INIT_Z
        self.estimated_pose.pose.pose.orientation = rotateQuaternion(Quaternion(w=1.0),
                                                                     self.INIT_HEADING)
        self.estimated_pose.header.frame_id = "map"

    def set_initial_pose(self, pose):
        """ Initialise filter with start pose """
        self.estimated_pose.pose = pose.pose
        # Estimated pose has been set, so we should now reinitialise the particle cloud around it
        rospy.loginfo("Got pose. Calling initialise_particle_cloud().")
        self.particlecloud = self.initialise_particle_cloud(
            self.estimated_pose)

    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.

        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        # Shorthand for initial pose
        pose0 = initialpose.pose.pose

        # Check the initial pose
        rospy.loginfo("Initial pose: " +
                      f"[ x {pose0.position.x:.4f} " +
                      f"| y {pose0.position.y:.4f} " +
                      f"| theta {getHeading(pose0.orientation):.4f} ]")

        # Set up initial pose array
        initial_poses = PoseArray()

        # Generate a list of noisy particles
        for _ in range(self.NUMBER_PREDICTED_READINGS):
            initial_poses.poses.append(self._get_noisy_pose(pose0))

        return initial_poses

    def _get_noisy_pose(self, pose):
        """Adds noise to one particle to generate a new pose estimate.

        This method simply draws values from Gaussian distribution
        controlled by the mean of the pose and the variance of the
        initialized noise constants.

        Args:
            pose (geometry_msgs.msg.Pose): the original pose estimate
        Return:
            (geometry_msgs.msg.Pose): noisy pose of the particle
        """
        # Initialize pose estimate
        pose_hat = Pose()

        # Samples from Gaussian with variance based on sampling noise
        x_hat = np.random.normal(pose.position.x, self.ODOM_TRANSLATION_NOISE)
        y_hat = np.random.normal(pose.position.y, self.ODOM_DRIFT_NOISE)
        theta_hat = np.random.normal(scale=self.ODOM_ROTATION_NOISE)

        # Map parameters
        width = self.occupancy_map.info.width
        height = self.occupancy_map.info.height
        origin = self.occupancy_map.info.origin.position
        resolution = self.occupancy_map.info.resolution

        # Check if pose is in the map
        map_x = int((x_hat - origin.x) / resolution)
        map_y = int((y_hat - origin.y) / resolution)
        while self.occupancy_map.data[map_y * width + map_x] == -1:
            map_y = np.random.randint(height)
            map_x = np.random.randint(width)

        x_hat = map_x * resolution + origin.x
        y_hat = map_y * resolution + origin.y

        # Assign noisy pose estimates
        pose_hat.position.x = x_hat
        pose_hat.position.y = y_hat
        pose_hat.orientation = rotateQuaternion(pose.orientation, theta_hat)

        return pose_hat


if __name__ == '__main__':
    try:
        rospy.init_node("amcl")
        Amcl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
