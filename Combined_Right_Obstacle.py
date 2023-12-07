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
    'right': 0,
    'bright': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
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
        'right': find_nearest(msg.ranges[90:129]),
        'fright': find_nearest(msg.ranges[130:169]),
        'front': find_nearest(msg.ranges[170:190]),
        'fleft': find_nearest(msg.ranges[191:230]),
        'left': find_nearest(msg.ranges[231:270]),

    }
    twstmsg_ = movement()


# Find the nearest point
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
    near_value = trapezoidal_membership(x, 0.0, 0.0, 0.1, 0.25)
    medium_value = trapezoidal_membership(x, 0.2, 0.33, 0.37, 0.5)
    far_value = trapezoidal_membership(x, 0.45, 0.6, 10.0, 100.0)

    return {"near": near_value, "medium": medium_value, "far": far_value}


# Iterates over every rule in the Obstacle Avoidance Dictionary and checks if all the antecedents are bigger than 0
# Calculates the firing strength of a rule
# Returns a dictionary with the fired rules, the output of those rules and the firing strength
def get_fired_rules_obstc(x1, x2, x3):
    aggregated_rules = {}

    # Fuzzify the crisp values
    x1_fuzzy = get_fuzzy_value(x1)
    x2_fuzzy = get_fuzzy_value(x2)
    x3_fuzzy = get_fuzzy_value(x3)

    # Possible fuzzy labels
    fuzzy_labels = ['near', 'medium', 'far']

    # Identify rules that are fired
    fired_rules = [(left_value, front_value, right_value)
                   for left_value in fuzzy_labels if x1_fuzzy[left_value] != 0
                   for front_value in fuzzy_labels if x2_fuzzy[front_value] != 0
                   for right_value in fuzzy_labels if x3_fuzzy[right_value] != 0]

    # Creates a dictionary with the fired rules, their output and the firing strength
    for rule in fired_rules:
        firing_strength = min(x1_fuzzy[rule[0]], x2_fuzzy[rule[1]], x3_fuzzy[rule[2]])

        activated_rule = rules_obstc.get(rule)

        aggregated_rules[rule] = {'linear': activated_rule['linear'], 'angular': activated_rule['angular'],
                                  'firing_strength': firing_strength}

    return aggregated_rules


# Iterates over every rule in the Right Edge Following Behaviour Dictionary and checks if all the antecedents are bigger than 0
# Calculates the firing strength of a rule
# Returns a dictionary with the fired rules, the output of those rules and the firing strength
def get_fired_rules_right(x1, x2):
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
        activated_rule = rules_right.get(rule)

        aggregated_rules[rule] = {'linear': activated_rule['linear'], 'angular':
            activated_rule['angular'], 'firing_strength':
                                      firing_strength}

    return aggregated_rules


# Convert the fuzzy values into crisp values using the centroid function
def get_crisp_output(aggregated_rules):
    linear_range = {
        'slow': 0.05,
        'medium': 0.085,
        'fast': 0.12}

    angular_range = {
        'left': -0.4,
        'zero': 0.0,
        'right': 0.4}

    crisp_linear = 0.0
    crisp_angular = 0.0

    linear_numerator = 0.0

    denominator = 0.0

    angular_numerator = 0.0

    # Deffuzify values based on the rule's firing strength
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


# Obstacle Avoidance Dictionary
rules_obstc = {
    ('near', 'near', 'near'): {'linear': 'slow', 'angular': 'zero'},
    ('near', 'near', 'medium'): {'linear': 'slow', 'angular': 'right'},
    ('near', 'near', 'far'): {'linear': 'medium', 'angular': 'right'},
    ('near', 'medium', 'near'): {'linear': 'slow', 'angular': 'zero'},
    ('near', 'medium', 'medium'): {'linear': 'slow', 'angular': 'right'},
    ('near', 'medium', 'far'): {'linear': 'medium', 'angular': 'right'},
    ('near', 'far', 'near'): {'linear': 'medium', 'angular': 'zero'},
    ('near', 'far', 'medium'): {'linear': 'medium', 'angular': 'right'},
    ('near', 'far', 'far'): {'linear': 'fast', 'angular': 'right'},

    ('medium', 'near', 'near'): {'linear': 'slow', 'angular': 'left'},
    ('medium', 'near', 'medium'): {'linear': 'slow', 'angular': 'left'},
    ('medium', 'near', 'far'): {'linear': 'slow', 'angular': 'right'},
    ('medium', 'medium', 'near'): {'linear': 'medium', 'angular': 'left'},
    ('medium', 'medium', 'medium'): {'linear': 'medium', 'angular': 'zero'},
    ('medium', 'medium', 'far'): {'linear': 'medium', 'angular': 'right'},
    ('medium', 'far', 'near'): {'linear': 'fast', 'angular': 'left'},
    ('medium', 'far', 'medium'): {'linear': 'fast', 'angular': 'zero'},
    ('medium', 'far', 'far'): {'linear': 'fast', 'angular': 'right'},

    ('far', 'near', 'near'): {'linear': 'slow', 'angular': 'left'},
    ('far', 'near', 'medium'): {'linear': 'slow', 'angular': 'left'},
    ('far', 'near', 'far'): {'linear': 'slow', 'angular': 'left'},
    ('far', 'medium', 'near'): {'linear': 'medium', 'angular': 'left'},
    ('far', 'medium', 'medium'): {'linear': 'medium', 'angular': 'left'},
    ('far', 'medium', 'far'): {'linear': 'medium', 'angular': 'zero'},
    ('far', 'far', 'near'): {'linear': 'medium', 'angular': 'left'},
    ('far', 'far', 'medium'): {'linear': 'medium', 'angular': 'left'},
    ('far', 'far', 'far'): {'linear': 'fast', 'angular': 'zero'},
}

# Right Edge Following Behaviour Dictionary
rules_right = {
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
    print("Min distance in front region: ", regions['front'])

    # create an object of twist class, used to express the linear and angular velocity of the turtlebot
    msg = Twist()

    aggregated_rules_obstc = get_fired_rules_obstc(regions['fleft'], regions['front'], regions['fright'])

    linear_speed_o, angular_speed_o = get_crisp_output(aggregated_rules_obstc)

    aggregated_rules_right = get_fired_rules_right(regions['fright'], regions['bright'])

    linear_speed_r, angular_speed_r = get_crisp_output(aggregated_rules_right)

    msg.linear.x = -(linear_speed_o * 0.85 + linear_speed_r * 0.15)
    msg.angular.z = -(angular_speed_o * 0.85 + angular_speed_r * 0.15)

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
