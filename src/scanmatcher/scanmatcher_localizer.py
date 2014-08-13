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
from rospy.exceptions import ROSException
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapRequest
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
from geometry_msgs.msg import PoseStamped


class scanmatcher_localizer(object):

    def __init__(self):
        self._static_map_req = GetMapRequest()
        self._scan_msg = LaserScan()
        self._map_msg = OccupancyGrid()
        self._initialpose_msg = PoseWithCovarianceStamped()
        # parameters
        self._use_static_map = rospy.get_param('~use_static_map', default=True)
        self._init_x = rospy.get_param('~init_x', default=0.0)
        self._init_y = rospy.get_param('~init_y', default=0.0)
        self._init_phi = rospy.get_param('~init_phi', default=0.0)
        # service clients
        try:
            rospy.wait_for_service('/static_map', timeout=3)
        except ROSException as e:
            print(e.message + '. Make sure that service node is running!')
        self._static_map_sc = rospy.ServiceProxy('/static_map', GetMap)
        # tf broadcasters
        self._map_odom_tfb = tf.TransformBroadcaster()
        # tf listeners
        self._odom_base_link_tfl = tf.TransformListener()
        self._base_link_base_laser_tfl = tf.TransformListener()
        # user init
        # processing queue
        self._scan_queue = PriorityQueue(maxsize=2)
        # get static map
        if self._use_static_map:
            static_map_res = self._static_map_sc(self._static_map_req)
            static_map_res.map.data = list(static_map_res.map.data)
            # initialize map with received one
            self._map = pymrpt.maps.COccupancyGridMap2D()
            self._map.from_ROS_OccupancyGrid_msg(static_map_res.map)
            # print output
            print 'received initial STATIC map!'
            print static_map_res.map.info
        else:
            # initialize empty map and wait for published ones
            self._map = pymrpt.maps.COccupancyGridMap2D()    
        # setup transformations
        self._tf = tf.TransformerROS()
        self._map_to_odom = pymrpt.poses.CPose2D()
        self._map_to_base = pymrpt.poses.CPose2D()
        self._odom_to_base = pymrpt.poses.CPose2D()
        # setup initial map to base tf ("/odom -> /base_link" also should be (0,0,0))
        self._map_to_base.x = self._init_x
        self._map_to_base.y = self._init_y
        self._map_to_base.phi = self._init_phi
        # setup scanmatcher lock
        self._scanmatcher_lock = Lock()
        
        
        # end of user init
        # subscribers
        rospy.Subscriber('/scan', LaserScan, self._scan_callback, queue_size=10)
        rospy.Subscriber('/map', OccupancyGrid, self._map_callback, queue_size=10)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self._initialpose_callback, queue_size=10)

    def get_odom_base_link_tf(self, stamp=None):
        pose = PoseStamped()
        if stamp is None:
            stamp = rospy.Time(0)
        try:
            trans, rot = self._odom_base_link_tfl.lookupTransform('/odom', '/base_link', stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # TODO: implement TransformListener.lookupTransform() exception handling!
            rospy.logwarn('Error while listening to /odom -> /base_link tf')
            return pose, False
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        return pose, True

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

    def send_map_odom_tf(self, pose):
        translation = pose.pose.position
        rotation = pose.pose.orientation
        self._map_odom_tfb.sendTransform(
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
            pose.header.stamp,
            '/odom',
            '/map')

    def _scan_callback(self, _scan_msg):
        # put scans into queue
        try:
            self._scan_queue.put_nowait([_scan_msg.header.stamp.to_nsec(), _scan_msg])
        except Full:
            rospy.logwarn('Dropped scan. Processing queue is full!')
            pass

    def run(self):
        # create worker thread (we do not .join() since it closes itself by ros shutdown)
        scanmatcher_thread = Thread(target=self.scanmatcher_worker)
        scanmatcher_thread.start()
        # keep it running
        rospy.spin()

    def scanmatcher_worker(self):
        # vars
        has_initial_scan = False
        # initialize ICP
        icp_options = pymrpt.slam.CICP.TConfigParams()
        icp = pymrpt.slam.CICP(icp_options)
        # loop
        while not rospy.is_shutdown():
            # get scan from queue
            try:
                item = self._scan_queue.get(timeout=5)
            except Empty:
                rospy.logwarn('Got no scan from queue since 5 seconds. Scanmatcher will shutdown now!')
                continue
            self._scan_msg = LaserScan()
            self._scan_msg = item[1]
            self._scan_msg.ranges = list(self._scan_msg.ranges)
            rospy.loginfo('received scan...')
            # get laser scanner pose
            laser_pose, ok = self.get_base_link_base_laser_tf()
            self._sensor_pose = pymrpt.poses.CPose3D()
            self._sensor_pose.from_ROS_Pose_msg(laser_pose.pose)
            # self get odom pose
            odom_pose, ok = self.get_odom_base_link_tf()
            self._odom_to_base = pymrpt.poses.CPose2D()
            self._odom_to_base.from_ROS_Pose_msg(odom_pose.pose)
            # acquire lock
            self._scanmatcher_lock.acquire()
            # update current stamp for publishers
            self._current_stamp = self._scan_msg.header.stamp
            # convert data
            observation = pymrpt.obs.CObservation2DRangeScan()
            observation.from_ROS_LaserScan_msg(self._scan_msg, self._sensor_pose)
            # set current map from scan
            current_map = pymrpt.maps.CSimplePointsMap()
            current_map.loadFromRangeScan(observation)
            # match maps
            # take current maintained map to base pose as initial guess (absolut pose in map)
            initial_guess = pymrpt.poses.CPosePDFGaussian(self._map_to_base)
            # run ICP algorithm
            aligned_pose, running_time, info = icp.AlignPDF(self._map, current_map, initial_guess)
            rospy.loginfo('init.  guess: {}'.format(initial_guess.mean))
            rospy.loginfo('icp goodness: {}'.format(info.goodness))
            rospy.loginfo('icp run_time: {}'.format(running_time))
            rospy.loginfo('aligned pose: {}'.format(aligned_pose.mean))
            # check goodness
            if info.goodness > .8:
                rospy.loginfo('...updating pose in map...')
                # update MRPT pose diff (this is actually the maintained tf)
                self._map_to_base = aligned_pose.mean
                # update last update time
                last_update_time = self._current_stamp
            else:
                # warn if pose lost
                rospy.logwarn('...lost pose in map...')
            # update pose diff
            map_to_base = pymrpt.poses.CPose2D(self._map_to_base)
            map_to_base.inverse()
        #    map_to_base_pose = PoseStamped()
        #    map_to_base_pose.header.frame_id = 'base_link'
        #    map_to_base_pose.header.stamp = self._scan_msg.header.stamp
        #    map_to_base_pose.pose = map_to_base.to_ROS_Pose_msg()
        #    latest_tf_pose = PoseStamped()
        #    try:
        #        latest_tf_pose = self._tf.transformPose('odom', map_to_base_pose)
        #    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #        rospy.logwarn('error while pose transformation')
            odom_to_base = pymrpt.poses.CPose2D(self._odom_to_base)
            odom_to_base.inverse()
        
            self._map_to_odom.x = self._map_to_base.x - self._odom_to_base.x
            self._map_to_odom.y = self._map_to_base.y - self._odom_to_base.y
            self._map_to_odom.phi = self._map_to_base.phi - self._odom_to_base.phi
            self._map_to_odom.normalizePhi()
        
            print "  /map  -> /base", self._map_to_base, map_to_base
            print "- /odom -> /base", self._odom_to_base, odom_to_base
            print "= /map  -> /odom", self._map_to_odom
        
            map_to_odom_pose = PoseStamped()
            map_to_odom_pose.pose = self._map_to_odom.to_ROS_Pose_msg()
            # send "/map -> /odom" tf at scan time
            map_to_odom_time = self._scan_msg.header.stamp + rospy.Duration(2.0)
            map_to_odom_pose.header.stamp = self._scan_msg.header.stamp
            self.send_map_odom_tf(map_to_odom_pose)
            # release lock
            self._scanmatcher_lock.release()
            # signalize work done
            rospy.loginfo('...task done!')
            self._scan_queue.task_done()

    def _map_callback(self, _map_msg):
        self._map_msg = _map_msg
        if not self._use_static_map:
            # lock scanmatcher
            self._scanmatcher_lock.acquire()
            # update map
            self._map.from_ROS_OccupancyGrid_msg(self._map_msg)
            # unlock scanmatcher
            self._scanmatcher_lock.release()
            # print output
            rospy.loginfo('received new map!')

    def _initialpose_callback(self, _initialpose_msg):
        # lock scanmatcher
        self._scanmatcher_lock.acquire()
        # update map odom pose diff
        self._map_to_base.from_ROS_Pose_msg(_initialpose_msg.pose.pose)
        # unlock scanmatcher
        self._scanmatcher_lock.release()
        # print output
        rospy.loginfo('received initial pose: {}'.format(self._map_to_base))

    