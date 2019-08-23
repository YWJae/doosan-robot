import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

def talker():

	pub = rospy.Publisher('R_007/ur_pnp', String, queue_size=10)

	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10)
	ur_run = "1"
	pub.publish(ur_run)
	rate.sleep()

if __name__ == '__main__':
    try:
       
	talker()
    except Exception,e:
	print e
