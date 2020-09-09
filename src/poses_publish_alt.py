#!/usr/bin/env python2
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
		self.ini_pose = [0.0,0.0,0.0]
		self.end_pose = [2.0,2.0,0.0]
		
		print('Init node...')
		rospy.init_node('poses_node', anonymous=True)
		self.pubPose = rospy.Publisher('/robocol/inicio_destino',Float32MultiArray,queue_size=1)
		
	def pub(self):
		path = []
		path.append(self.ini_pose[0])
		path.append(self.ini_pose[1])
		path.append(self.ini_pose[2])
		path.append(self.end_pose[0])
		path.append(self.end_pose[1])
		path.append(self.end_pose[2])

		msg = Float32MultiArray()
		msg.data = path
		print(msg)
		# pose_msg = poses.path_msg()
		self.pubPose.publish(msg)

def main():
	try:
		obj = Poses()
		pub = True
		time.sleep(1)
		# print('Publishing poses...')
		rate = rospy.Rate(10)
		n =0
		while not rospy.is_shutdown():
			if pub:
				obj.pub()
				if n>0:
					pub = False
				n += 1
			rate.sleep()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()
