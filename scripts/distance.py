#!/usr/bin/env python3

'''Distance (node2):

 ✓A node that checks the relative distance between turtle1 and turtle2 and:
 
- publish on a topic the distance (you can use a std_msgs/Float32 for that)

- stops the moving turtle if the two turtles are “too close” (you may 
set a threshold to monitor that)

- stops the moving turtle if the position is too close to the boundaries 
(.e.g, x or y > 10.0, x or y < 1.0)


1) use the /turtle1/pose and /turtle2/pose topics to get the positions.

2) check the relative distance

3) publish the distance on a topic

4) if the distance is below a threshold, publish zero velocities 
(I can use the publishers of the user_interface node for that, so set them public)

5) if the position of the moving turtle is too close to the boundaries, publish zero velocities as well.

'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from turtlesim.msg import Pose
import math

'''super() gives you access to the parent class. In this case, 
    MinimalPublisher inherits from rclpy.node.Node. So super() allows you to call methods 
    of the Node class from your subclass!'''
    
class DistanceController(Node):

    def __init__(self):
    # rclpy is the module for using ros functionalities in python
        super().__init__('distance_node')
        self.x1_ = None
        self.y1_ = None
        self.x2_ = None
        self.y2_ = None

        # ---------subscribers for turtle poses------------
        self.sub_t1 = self.create_subscription(
             Pose,                  # type of message 
            '/turtle1/pose',        # topic 
            self.poset1_callback,   # function to call when a message is received
            10)                     # queue size

        self.sub_t2 = self.create_subscription(
             Pose,
            '/turtle2/pose',
            self.poset2_callback,
            10)
        
        #---------- publisher for distance------------
        self.dist_pub = self.create_publisher(
            Float32,
            'distance_topic', # topic name
             10) 
        
        #---------- timer to periodically check distance ------------
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1s = 10 Hz

    def poset1_callback(self, msg):
        self.x1_ = msg.x # float
        self.y1_ = msg.y
            
    
    def poset2_callback(self, msg):
        self.x2_ = msg.x
        self.y2_ = msg.y


    def timer_callback(self):
        if None in (self.x1_, self.y1_, self.x2_, self.y2_):
            return
        distance = math.sqrt((self.x2_ - self.x1_)**2 + (self.y2_ - self.y1_)**2)
        self.get_logger().info(f'Distance: {distance:.2f}') # to see distance in terminal

        # pubblish distance
        msg = Float32()
        msg.data = distance
        self.dist_pub.publish(msg)
            
        # controls: between eachother and boundaries
       

    


def main(args=None):
    rclpy.init(args=args)
    distance_node = DistanceController()
    rclpy.spin(distance_node) #keeps the node alive, listening for messages
    rclpy.shutdown()

if __name__ == '__main__':
 main()