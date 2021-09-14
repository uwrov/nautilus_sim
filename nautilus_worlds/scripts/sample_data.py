import rospy
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import Int16MultiArray


dims = [MultiArrayDimension('data', 6, 16)]
layout = MultiArrayLayout(dim=dims, data_offset=0)

data = [1 for _ in range(6)]
msg = Int16MultiArray(layout=layout, data=data)
print(msg)

rospy.init_node('test_data')
p = rospy.Publisher('/nautilus/motors/pwm', Int16MultiArray, queue_size=1)
r = rospy.Rate(10)

print("publisher started on /nautilus/motors/pwm")

rospy.on_shutdown(lambda: rospy.loginfo("shutting down"))

while not rospy.is_shutdown():
  p.publish(msg)
  r.sleep()