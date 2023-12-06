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
    'bright': 0,
    'fright': 0,
}
twstmsg_ = None


# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if (twstmsg_ != None):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_

    regions_ = {
        # LIDAR readings are anti-clockwise
        'bright': find_nearest(msg.ranges[50:89]),
        'fright': find_nearest(msg.ranges[130:169]),

    }
    twstmsg_ = movement()


# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude 'zero's
    return min(min(f_list, default=10), 10)


# Calculate the membership value of a variable x based on the trapezoidal function
def trapezoidal_membership(x, a, b, c, d):
    if a <= x < b:
        return (x - a) / (b - a)
    elif b <= x < c:
        return 1
    elif c <= x <= d:
        return (d - x) / (d - c)
    return 0


# Returns the fuzzy value of a crisp value
def get_fuzzy_value(x):
    near_value = trapezoidal_membership(x, 0.0, 0.0, 0.0, 0.25)
    medium_value = trapezoidal_membership(x, 0.2, 0.33, 0.37, 0.5)
    far_value = trapezoidal_membership(x, 0.45, 0.6, 10.0, 10.0)

    return {"near": near_value, "medium": medium_value, "far": far_value}


# Iterates over every rule and checks if all the antecedents are bigger than 0
# Calculates the firing strength of a rule
# Returns a dictionary with the fired rules, the output of those rules and the firing strength
def get_fired_rules(x1, x2):
    aggregated_rules = {}

    # Fuzzify the crisp values
    x1_fuzzy = get_fuzzy_value(x1)
    x2_fuzzy = get_fuzzy_value(x2)

    # Possible fuzzy labels
    fuzzy_labels = ['near', 'medium', 'far']

    # Identify rules that are fired
    fired_rules = [(front_right, back_right)
                   for front_right in fuzzy_labels if x1_fuzzy[front_right] != 0
                   for back_right in fuzzy_labels if x2_fuzzy[back_right] != 0]

    # Creates a dictionary with the fired rules, their output and the firing strength
    for rule in fired_rules:
        firing_strength = min(x1_fuzzy[rule[0]], x2_fuzzy[rule[1]])
        activated_rule = rules.get(rule)

        aggregated_rules[rule] = {'linear': activated_rule['linear'], 'angular':
            activated_rule['angular'], 'firing_strength':
                                      firing_strength}

    return aggregated_rules


# Convert the fuzzy values into crisp values using the centroid function
def get_crisp_output(aggregated_rules):
    linear_range = {
        'slow': 0.05,
        'medium': 0.15,
        'fast': 0.25}

    angular_range = {
        'left': -0.7,
        'zero': 0.0,
        'right': 0.7}

    crisp_linear = 0.0
    crisp_angular = 0.0

    linear_numerator = 0.0

    denominator = 0.0

    angular_numerator = 0.0

    # Deffuzify values based on the rule's firing strenght
    for rule, values in aggregated_rules.items():
        firing_strength = values['firing_strength']
        linear_speed = values['linear']
        angular_speed = values['angular']

        linear_numerator = linear_numerator + linear_range[linear_speed] * firing_strength

        angular_numerator = angular_numerator + angular_range[angular_speed] * firing_strength

        denominator += firing_strength

    # Calculate crisp values
    if linear_numerator != 0:
        crisp_linear = linear_numerator / denominator
        crisp_angular = angular_numerator / denominator

    return crisp_linear, crisp_angular


# Right Edge Following Behaviour Dictionary
rules = {
    ('near', 'near'): {'linear': 'slow', 'angular': 'left'},
    ('near', 'medium'): {'linear': 'slow', 'angular': 'left'},
    ('near', 'far'): {'linear': 'slow', 'angular': 'left'},

    ('medium', 'near'): {'linear': 'medium', 'angular': 'left'},
    ('medium', 'medium'): {'linear': 'medium', 'angular': 'zero'},
    ('medium', 'far'): {'linear': 'medium', 'angular': 'right'},

    ('far', 'near'): {'linear': 'medium', 'angular': 'left'},
    ('far', 'medium'): {'linear': 'medium', 'angular': 'right'},
    ('far', 'far'): {'linear': 'fast', 'angular': 'zero'},
}


# Basic movement method
def movement():
    global regions_, mynode_
    regions = regions_

    # create an object of twist class, used to express the linear and angular velocity of the turtlebot
    msg = Twist()

    linear_speed = 0.0
    angular_speed = 0.0

    aggregated_rules = get_fired_rules(regions['fright'], regions['bright'])

    linear_speed, angular_speed = get_crisp_output(aggregated_rules)

    msg.linear.x = -linear_speed
    msg.angular.z = -angular_speed

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
