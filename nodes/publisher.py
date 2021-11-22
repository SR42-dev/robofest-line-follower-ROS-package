import rospy
from std_msgs.msg import String

pub = rospy.Publisher('chatter', String, queue_size = 10)
rospy.init_node('talker', anonymous = True)
rate = rospy.Rate(10) # 10 Hz

while not rospy.is_shutdown() :
	
	pub_str = 'hello world. Time => ' + str(rospy.get_time())
	rospy.loginfo(pub_str)
	pub.publish(pub_str)
	rate.sleep()
