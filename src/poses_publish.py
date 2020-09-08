#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import time
from std_msgs.msg import Float32MultiArray
	

class Poses(object):
	def __init__(self):
		super(Poses, self).__init__()
		self.ini_pose = [0.0,0.0,0.0]
		self.end_pose = [15.0,14.0,0.0]
		
		
	def path_msg(self):

		msg = Path()
		msg.header.frame_id = "/map"
		msg.header.stamp = rospy.Time.now()

		poseStamped = PoseStamped()
		poseStamped.pose.position.x = self.ini_pose[0]
		poseStamped.pose.position.y = self.ini_pose[1]
		poseStamped.pose.position.z = self.ini_pose[2]

		poseStamped.pose.orientation.x = 0.0
		poseStamped.pose.orientation.y = 0.0
		poseStamped.pose.orientation.z = 0.0
		poseStamped.pose.orientation.w = 0.0
		
		msg.poses.append(poseStamped)
		print(msg)
		return msg


def main():
	try:
		print('Init node...')
		rospy.init_node('poses_node', anonymous=True)
		pubPose = rospy.Publisher('/robocol/inicio_destino',Path,queue_size=1)
		poses = Poses()
		pub = True
		time.sleep(1)
		print('Publishing poses...')
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			#if pub:
			pose_msg = poses.path_msg()
			pubPose.publish(pose_msg)
		#	pub = False
			rate.sleep()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()