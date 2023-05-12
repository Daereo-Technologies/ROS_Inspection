#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection


class Laser2PC():
	def __init__(self):
		self.laserProj=LaserProjection()
		self.pcPub=rospy.Publisher("/laserPointCloud",pc2,queue_size=1)
		self.laserSub=rospy.Subscriber("/iris/ray_scan",LaserScan,self.laserCallback)
	
	def laserCallback(self,data):
		cloud_out=self.laserProj.projectLaser(data)
		self.pcPub.publish(cloud_out)
if __name__=='__main__':
	rospy.init_node("scan_to_pcl")
	l2pc=Laser2PC()
	rospy.spin()

# # Subscribe Topic sensor_msgs/LaserScan Message


# Header header            # timestamp in the header is the acquisition time of 
#                          # the first ray in the scan.
#                          #
#                          # in frame frame_id, angles are measured around 
#                          # the positive Z axis (counterclockwise, if Z is up)
#                          # with zero angle being forward along the x axis
                         
# float32 angle_min        # start angle of the scan [rad]
# float32 angle_max        # end angle of the scan [rad]
# float32 angle_increment  # angular distance between measurements [rad]

# float32 time_increment   # time between measurements [seconds] - if your scanner
#                          # is moving, this will be used in interpolating position
#                          # of 3d points
# float32 scan_time        # time between scans [seconds]

# float32 range_min        # minimum range value [m]
# float32 range_max        # maximum range value [m]

# float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
# float32[] intensities    # intensity data [device-specific units].  If your
#                          # device does not provide intensities, please leave
#                          # the array empty.

# #Publisher Topic sensor_msgs/PointCloud2 Message

# # This message holds a collection of N-dimensional points, which may
# # contain additional information such as normals, intensity, etc. The
# # point data is stored as a binary blob, its layout described by the
# # contents of the "fields" array.

# # The point cloud data may be organized 2d (image-like) or 1d
# # (unordered). Point clouds organized as 2d images may be produced by
# # camera depth sensors such as stereo or time-of-flight.

# # Time of sensor data acquisition, and the coordinate frame ID (for 3d
# # points).
# Header header

# # 2D structure of the point cloud. If the cloud is unordered, height is
# # 1 and width is the length of the point cloud.
# uint32 height
# uint32 width

# # Describes the channels and their layout in the binary data blob.
# PointField[] fields

# bool    is_bigendian # Is this data bigendian?
# uint32  point_step   # Length of a point in bytes
# uint32  row_step     # Length of a row in bytes
# uint8[] data         # Actual point data, size is (row_step*height)

# bool is_dense        # True if there are no invalid points

