###1801808###

import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

mynode_ = None
pub_ = None
regions_ = {
    'right90': 0,
    'right80': 0,
    'right70': 0,
    'right60': 0,
    'right50': 0,
    'right40': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
twstmsg_ = None


# PID control function
def PID(kp, ki, kd, target, current_input, integral_error, previous_error):
    # Define the integral error upper and lower boundaries
    lower_bound = -1.0
    upper_bound = 1.0

    # Calculate current error
    error = target - current_input

    # Integrate the error
    integral_error += error
    integral_error = min(max(integral_error, lower_bound), upper_bound)

    # Calculate the derivative of the error
    derivative_error = error - previous_error

    # Compute the PID output
    output = kp * error + ki * integral_error + kd * derivative_error

    # Update the previous error
    previous_error = error
    return output


# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if (twstmsg_ != None):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_

    regions_ = {
        # LIDAR readings are anti-clockwise
        'right130': find_nearest(msg.ranges[44:55]),
        'right120': find_nearest(msg.ranges[54:65]),
        'right110': find_nearest(msg.ranges[64:75]),
        'right100': find_nearest(msg.ranges[74:85]),
        'right90': find_nearest(msg.ranges[84:95]),
        'right80': find_nearest(msg.ranges[94:105]),
        'right70': find_nearest(msg.ranges[104:115]),
        'right60': find_nearest(msg.ranges[114:125]),
        'right50': find_nearest(msg.ranges[124:135]),
        'right40': find_nearest(msg.ranges[134:145]),
        'right30': find_nearest(msg.ranges[144:155]),
        'right20': find_nearest(msg.ranges[154:165]),
        'front': find_nearest(msg.ranges[175:185]),
        'fleft': find_nearest(msg.ranges[220:230]),
        'left': find_nearest(msg.ranges[265:275]),
        'minofright': min(min(msg.ranges[84:175]), 10),

    }
    twstmsg_ = movement()


# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)


# Initial PID parameters
pid_last_error = 0
pid_integral = 0

kp = 1.4
ki = 0.07
kd = 0.05
integral_error = 0
previous_error = 0
target = 0.4

integral_upper_limit = 1.0
integral_lower_limit = -1.0


def movement():
    global regions_, mynode_, integral_error, previous_error, kp, ki, kd
    regions = regions_

    msg = Twist()

    mean_distance = (regions['right90'] + regions['right80'] + regions['right70'] + regions['right60'] + regions[
        'right50'] + regions['right40'] + regions['right30'] + regions['right20']) / 8

    error = target - mean_distance

    integral_error += error

    integral_error = max(min(integral_upper_limit, integral_error), integral_lower_limit)
    derivative_error = error - previous_error

    previous_error = error

    control_output = PID(kp, ki, kd, target, mean_distance, integral_error, previous_error)
    msg.angular.z = control_output
    print("control_output=", control_output)

    if (regions['front']) < 0.25:
        msg.linear.x = 0.0
    else:
        msg.linear.x = -0.1

    print("msg=", msg)
    return msg


# used to stop the rosbot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    )

    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # stop the robot
    except:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
