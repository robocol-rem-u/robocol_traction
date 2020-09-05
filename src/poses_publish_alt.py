#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import time
from std_msgs.msg import Float32MultiArray
	

class Poses(object):
	def __init__(self):
		super(Poses, self).__init__()
		self.ini_pose = [0.1,0.0,0.0]
		self.end_pose = [0.2,0.2,0.0]
		
		
	def path_msg(self):

		path = []
		path.append(self.ini_pose)
		path.append(self.end_pose)
		
		#print(path)
		# path.poses.append(pose)
		# pose.pose = end
		# pose.pose = end
		# path.poses.append(end)
		# path_pub.publish(<path)
		# path = Path()
		# ini = Pose()
		# ini.position.x = 2
		# ini.position.y = 3
		# ini.position.z = 4
		# # path.append(ini)
		# # path.header = Header()
		# # path.poses.header = Header()
		# path.poses.append(ini)
		# # path.poses[0].pose.position.x = 2
		print(path)
		return path


		# # Init pose
		# ini_pose_msg = Pose()
		# ini_pose_msg.position = self.ini_pose
		# # End Pose
		# end_pose_msg = Pose()
		# end_pose_msg.position = self.end_pose
		
		# header = Header()
		# path = Path()
		# path.header = Header()		
		# path.poses = [ini_pose_msg,end_pose_msg]
		
		# # pose.header = data.header
		# # pose.pose = data.pose.pose
		# # path.poses.append(pose
		# # return msg
		# path = Path()
		# path.header = Header()
		# path.poses = []
		# # poseStamped = PoseStamped()
		# # poseStamped.header = Header()
		# # poseStamped.pose = pose.pose.pose
		# # path.poses.append(self.ini_pose)
		# # path.poses.append(self.end_pose)
		# # path_pub.publish(path)
		# return path

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