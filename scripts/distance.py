#!/usr/bin/env python3

# PUBBLICA LA DISTANZA SOLO SE è DIVERSA DALLA PRECEDENTE
# FAI TORNARE LA TARTA INDIETROOOOO QUANDO SI AVVICINA AI BORDI
# controlla che funzioni bene!
# fai uno script per testare


'''Distance (node2):

 ✓A node that checks the relative distance between turtle1 and turtle2 and:
 
- publish on a topic the distance (you can use a std_msgs/Float32 for that)

- stops the moving turtle if the two turtles are “too close” (you may 
set a threshold to monitor that)

- stops the moving turtle if the position is too close to the boundaries 
(.e.g, x or y > 10.0, x or y < 1.0)

'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
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

        self.t1_vel = Twist()
        self.t2_vel = Twist()

        # ---------subscribers for turtle poses------------
        self.create_subscription(
             Pose,                  # type of message 
            '/turtle1/pose',        # topic 
            self.poset1_callback,   # function to call when a message is received
            10)                     # queue size

        self.create_subscription(Pose,'/turtle2/pose',self.poset2_callback,10)
        
        #---------- publisher for distance------------
        self.dist_pub = self.create_publisher(Float32,'distance_topic',10) 

        #---------- subscribers for turtle velocities ------------
        self.create_subscription(Twist,'/turtle1/cmd_vel',self.t1_vel_callback,10)
        self.create_subscription(Twist,'/turtle2/cmd_vel',self.t2_vel_callback,10)

        #---------- publishers for turtle velocities ------------
        self.vel_pub1 = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.vel_pub2 = self.create_publisher(Twist,'/turtle2/cmd_vel',10)


    def poset1_callback(self, msg):
        self.x1_ = msg.x # float
        self.y1_ = msg.y      
    
    def poset2_callback(self, msg):
        self.x2_ = msg.x
        self.y2_ = msg.y
    
    def t1_vel_callback(self, msg):
        self.t1_vel = msg
        self.controls()

    def t2_vel_callback(self, msg):
        self.t2_vel = msg
        self.controls()
    

# function to determine which turtle is moving
    def moving_turtle(self):
        if abs(self.t1_vel.linear.x) > 0.001 or abs(self.t1_vel.angular.z) > 0.001:
            return "turtle1"
        if abs(self.t2_vel.linear.x) > 0.001 or abs(self.t2_vel.angular.z) > 0.001:
            return "turtle2"
        return None
     
# controls: between eachother and boundaries

    def controls(self):
        
        moving = self.moving_turtle()
        if moving is None:
            return

        if None in (self.x1_, self.y1_, self.x2_, self.y2_):
            return

        threshold_distance = 0.5  # set your threshold here
        boundary_limit_min = 1.0
        boundary_limit_max = 10.0
        twist = Twist()  # zero velocities

        distance = math.sqrt((self.x2_ - self.x1_)**2 + (self.y2_ - self.y1_)**2)
        self.get_logger().info(f'Distance: {distance:.2f}') # to see distance in terminal

        # pubblish distance 
        msg = Float32()
        msg.data = distance
        self.dist_pub.publish(msg)

        
        # Check relative distance
        if distance < threshold_distance and moving is not None:
            self.get_logger().warning(f'{moving} is moving and too close: stopping it!')
            # stop the moving turtle
            if moving == "turtle1":
                self.vel_pub1.publish(twist)
            elif moving == "turtle2":
                self.vel_pub2.publish(twist)
        
        # Check boundaries for turtle1
        if (self.x1_ < boundary_limit_min or self.x1_ > boundary_limit_max or
            self.y1_ < boundary_limit_min or self.y1_ > boundary_limit_max):
            self.get_logger().warning('Turtle1 is too close to the boundary! Stopping turtle1.')
            self.vel_pub1.publish(twist)
    

        # Check boundaries for turtle2
        if (self.x2_ < boundary_limit_min or self.x2_ > boundary_limit_max or
            self.y2_ < boundary_limit_min or self.y2_ > boundary_limit_max):
            self.get_logger().warning('Turtle2 is too close to the boundary! Stopping turtle2.')
            self.vel_pub2.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    distance_node = DistanceController()
    rclpy.spin(distance_node) #keeps the node alive, listening for messages
    rclpy.shutdown()

if __name__ == '__main__':
 main()