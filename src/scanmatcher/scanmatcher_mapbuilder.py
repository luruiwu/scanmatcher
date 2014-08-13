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
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf
from rospy.exceptions import ROSException
from geometry_msgs.msg import PoseStamped


class scanmatcher_mapbuilder(object):

    def __init__(self):
        self._scan_msg = LaserScan()
        self._map_msg = OccupancyGrid()
        # parameters
        self._update_ang = rospy.get_param('~update_ang', default=pymrpt.utils.DEG2RAD(15.0))
        self._update_dist = rospy.get_param('~update_dist', default=0.25)
        # publishers
        self._map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        # tf listeners
        self._map_base_link_tfl = tf.TransformListener()
        # user init
        # maintained map
        self._map = pymrpt.maps.COccupancyGridMap2D(-10.0, 10.0, -10.0, 10.0, 0.05)
        # last map update pose
        self._last_update_pose = pymrpt.poses.CPose2D()
        # initial map
        self._has_initial_map = False
        # end of user init
        # subscribers
        rospy.Subscriber('/scan', LaserScan, self._scan_callback, queue_size=10)

    def get_map_base_link_tf(self, stamp=None):
        pose = PoseStamped()
        if stamp is None:
            stamp = rospy.Time(0)
        try:
            trans, rot = self._map_base_link_tfl.lookupTransform('/map', '/base_link', stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # TODO: implement TransformListener.lookupTransform() exception handling!
            rospy.logwarn('Error while listening to /map -> /base_link tf')
            return pose, False
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        return pose, True

    def run(self):
        rospy.spin()

    def _scan_callback(self, _scan_msg):
        # get robot pose in map at laserscan time
        pose_stamped = self.get_map_base_link_tf(stamp=_scan_msg.header.stamp)
        # convert pose
        robot_pose = pymrpt.poses.CPose2D()
        robot_pose.from_ROS_Pose_msg(pose_stamped.pose)
        # calculate deltas
        pose_diff = self._last_update_pose - robot_pose
        dist_inc_since_last_update = pose_diff.norm()
        ang_inc_since_last_update = pose_diff.phi
        # add laserscan to map depending on thresholds
        if (dist_inc_since_last_update > self._update_dist or ang_inc_since_last_update > self._update_ang) 
            or not self._has_initial_map:
            # get laser scanner pose
            laser_pose = self.get_base_link_base_laser_tf()
            sensor_pose = pymrpt.poses.CPose3D()
            sensor_pose.from_ROS_Pose_msg(laser_pose.pose)
            # convert scan to observation
            observation = pymrpt.obs.CObservation2DRangeScan()
            observation.from_ROS_LaserScan_msg(_scan_msg)
            # update map
            self._map.insertObservation(observation, sensor_pose)
            # publish map
            self._map_msg = self._map.to_ROS_OccupancyGrid_msg()
            self._map_msg.header.stamp = _scan_msg.header.stamp
            self._map_pub.publish(self._map_msg)
            # update last map update pose
            self._last_update_pose = robot_pose
            # signalize that we have inital map
            self._has_initial_map

    