""" Auto-generated library, NEVER edit this! """
__author__ = 'Change name in author settings'
__copyright__ = 'Germany'
__credits__ = []
__license__ = 'TODO'
__version__ = '0.0.1'
__maintainer__ = 'Change name in author settings'
__email__ = 'semael23@gmail.com'
__status__ = 'freshly generated'

import rospy
import pymrpt
from threading import Thread, Lock
from Queue import PriorityQueue, Full, Empty
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from rospy.exceptions import ROSException
from geometry_msgs.msg import PoseStamped


class scanmatcher_odom(object):

    def __init__(self):
        self._scan_msg = LaserScan()
        self._odom_msg = Odometry()
        # parameters
        self._send_odom_base_tf = rospy.get_param('~send_odom_base_tf', default=True)
        # publishers
        self._odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        # tf broadcasters
        self._odom_base_link_tfb = tf.TransformBroadcaster()
        # tf listeners
        self._base_link_base_laser_tfl = tf.TransformListener()
        # user init
        # processing queue
        self._scan_queue = PriorityQueue(maxsize=2)
        # absolute odometry pose and current velocities
        self._current_pose = pymrpt.poses.CPose2D()
        self._v = .0
        self._w = .0
        # initialize odometry
        self._odom_msg.header.frame_id = 'odom'
        self._odom_msg.child_frame_id = 'base_link'
        # end of user init
        # subscribers
        rospy.Subscriber('/scan', LaserScan, self._scan_callback, queue_size=10)

    def get_base_link_base_laser_tf(self, stamp=None):
        pose = PoseStamped()
        if stamp is None:
            stamp = rospy.Time(0)
        try:
            trans, rot = self._base_link_base_laser_tfl.lookupTransform('/base_link', '/base_laser', stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # TODO: implement TransformListener.lookupTransform() exception handling!
            rospy.logwarn('Error while listening to /base_link -> /base_laser tf')
            return pose, False
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        return pose, True

    def send_odom_base_link_tf(self, pose):
        translation = pose.pose.position
        rotation = pose.pose.orientation
        self._odom_base_link_tfb.sendTransform(
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
            pose.header.stamp,
            '/base_link',
            '/odom')

    def run(self):
        # create worker thread (we do not .join() since it closes itself by ros shutdown)
        scanmatcher_thread = Thread(target=self.scanmatcher_worker)
        scanmatcher_thread.start()
        # keep it running
        rospy.spin()

    def _scan_callback(self, _scan_msg):
        # put scans into queue
        try:
            self._scan_queue.put_nowait([_scan_msg.header.stamp.to_nsec(), _scan_msg])
        except Full:
            rospy.logwarn('Dropped scan. Processing queue is full!')
            pass

    def scanmatcher_worker(self):
        # vars
        has_initial_scan = False
        # initialize ICP
        icp_options = pymrpt.slam.CICP.TConfigParams()
        icp = pymrpt.slam.CICP(icp_options)
        # create last and current laser maps
        last_map = pymrpt.maps.CSimplePointsMap()
        # setup last update time
        last_update_time = rospy.Time.now()
        # loop
        while not rospy.is_shutdown():
            self._scan_msg = LaserScan()
            # get scan from queue
            try:
                item = self._scan_queue.get(timeout=5)
            except Empty:
                rospy.logwarn('Got no scan from queue since 5 seconds. Scanmatcher will shutdown now!')
                continue
            self._scan_msg = item[1]
            self._scan_msg.ranges = list(self._scan_msg.ranges)
            rospy.loginfo('received scan...')
            # update current stamp for publishers
            self._current_stamp = self._scan_msg.header.stamp
            # get laser scanner pose
            laser_pose, ok = self.get_base_link_base_laser_tf()
            self._sensor_pose = pymrpt.poses.CPose3D()
            self._sensor_pose.from_ROS_Pose_msg(laser_pose.pose)
            # convert data
            observation = pymrpt.obs.CObservation2DRangeScan()
            observation.from_ROS_LaserScan_msg(self._scan_msg, self._sensor_pose)
            # set current map from scan
            current_map = pymrpt.maps.CSimplePointsMap()
            current_map.loadFromRangeScan(observation)
            # match maps
            if has_initial_scan:
                # no initial guess (pure incremental)
                initial_guess = pymrpt.poses.CPosePDFGaussian()
                # run ICP algorithm
                pose_change, running_time, info = icp.AlignPDF(last_map, current_map, initial_guess)
                rospy.loginfo('icp goodness: {}'.format(info.goodness))
                rospy.loginfo('icp run_time: {}'.format(running_time))
                rospy.loginfo('pose  change: {}'.format(pose_change.mean))
                # check goodness
        #        if info.goodness > .8:
                rospy.loginfo('...updating odometry.')
                # get time delta
                d_t = (last_update_time - self._current_stamp).to_sec()
                # update current pose and velocities
                self._current_pose += pose_change.mean
                dist = pose_change.mean.norm()
                self._v = dist / d_t
                self._w = pose_change.mean.phi / d_t
                self.publish_odom()
                # update last update time
                last_update_time = self._current_stamp
                # update last map
                last_map = current_map
        #        else:
        #            rospy.logwarn('...lost odometry...')
            else:
                rospy.loginfo('...is inital one!')
                # load initial scan to last map
                last_map = current_map
                # mark initial as received
                has_initial_scan = True
        #    rospy.loginfo('...task done!')
            # signalize work done
            self._scan_queue.task_done()

    def publish_odom(self):
        """
        Publish odometry and optionally "odom -> base_link" tf.
        """
        # setup odometry message
        self._odom_msg.header.stamp = self._current_stamp
        self._odom_msg.pose.pose = self._current_pose.to_ROS_Pose_msg()
        self._odom_msg.twist.twist.linear.x = self._v
        self._odom_msg.twist.twist.angular.z = self._w
        # publish odom
        self._odom_pub.publish(self._odom_msg)
        # publish "odom -> base_link" tf on demand
        if self._send_odom_base_tf:
           pose_msg = PoseStamped()
           pose_msg.header.stamp = self._current_stamp
           pose_msg.pose = self._odom_msg.pose.pose
           self.send_odom_base_link_tf(pose_msg)

    