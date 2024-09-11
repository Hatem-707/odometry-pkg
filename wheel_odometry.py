
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import math
import tf
from tf.transformations import quaternion_from_euler

wheel_diameter = 6.3
wheel_separation = 17
ticks_per_cm = 10.55

x = 0
y = 0

old_left_ticks = 0
old_right_ticks = 0
old_theta = 0

new_left_ticks = 0
new_right_ticks = 0
new_theta = 0

odom = Odometry()

def right_callback( data):
    global new_right_ticks
    new_right_ticks = data.data

def left_callback( data):
    global new_left_ticks
    new_left_ticks = data.data

def pos_and_velocity_calculation(old_time, current_time):
    global old_left_ticks, old_right_ticks, old_theta, new_left_ticks, new_right_ticks, new_theta, x, y, odom
    delta_right = new_right_ticks - old_right_ticks
    delta_left = new_left_ticks - old_left_ticks

    distance_left = delta_left / ticks_per_cm
    distance_right = delta_right / ticks_per_cm
    cycle_distance = (distance_right+distance_left)/2
    cycle_angle = math.asin((distance_right-distance_left)/wheel_separation)
    new_theta = old_theta + cycle_angle/2


    x += cycle_distance*math.cos(new_theta)
    y += cycle_distance*math.sin(new_theta)

    if current_time > old_time:
        x_velocity = cycle_distance*math.cos(new_theta)/(current_time-old_time)
        y_velocity = cycle_distance*math.sin(new_theta)/(current_time-old_time)
        z_angular = cycle_angle/(current_time-old_time)
    else:
        x_velocity = 0
        y_velocity = 0
        z_angular = 0

    quat = quaternion_from_euler(0, 0, new_theta)

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation.x = quat[0]
    odom.pose.pose.orientation.y = quat[1]
    odom.pose.pose.orientation.z = quat[2]
    odom.pose.pose.orientation.w = quat[3]
    odom.pose.covariance = 0.1
    odom.twist.twist.linear.x = x_velocity
    odom.twist.twist.linear.y = y_velocity
    odom.twist.twist.linear.z = 0.0
    odom.twist.twist.angular.x = 0.0
    odom.twist.twist.angular.y = 0.0
    odom.twist.twist.angular.z = z_angular
    odom.twist.covariance = 0.1

    old_left_ticks = new_left_ticks
    old_right_ticks = new_right_ticks
    old_theta = new_theta



if __name__ == '__main__':
    try:
        rospy.init_node('wheel_odometry', anonymous=True)
        right_sub = rospy.Subscriber('/right_ticks', Int32, right_callback)
        left_sub = rospy.Subscriber('/left_ticks', Int32, left_callback)
        odom_pub = rospy.Publisher('/odom', Odometry, queue_size=5)
        old_time = rospy.get_time()
        rate = rospy.Rate(10)
        rate.sleep()
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            pos_and_velocity_calculation(old_time, current_time)
            odom_pub.publish(odom)
            rate.sleep()
            old_time = current_time
            rospy.spin()


    except rospy.ROSInterruptException:
        pass


