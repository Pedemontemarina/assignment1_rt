#!/usr/bin/env python3

# todo: se va in basso non funziona!!!

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

        self.last_distance = None # to store the last published distance


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

        #---------- timer to call controls function periodically ------------
        self.create_timer(0.1, self.controls)

        self.get_logger().info("Distance node started.")


    def poset1_callback(self, msg):
        self.x1_ = msg.x # float
        self.y1_ = msg.y      
    
    def poset2_callback(self, msg):
        self.x2_ = msg.x
        self.y2_ = msg.y
    
    def t1_vel_callback(self, msg):
        self.t1_vel = msg

    def t2_vel_callback(self, msg):
        self.t2_vel = msg
    

# function to determine which turtle is moving
    def moving_turtle(self):
        if abs(self.t1_vel.linear.x) > 0.001 or abs(self.t1_vel.angular.z) > 0.001:
            return "turtle1"
        if abs(self.t2_vel.linear.x) > 0.001 or abs(self.t2_vel.angular.z) > 0.001:
            return "turtle2"
        return None     

    def controls(self):
        
        moving = self.moving_turtle()
        if moving is None:
            return

        if None in (self.x1_, self.y1_, self.x2_, self.y2_):
            return
        distance = math.sqrt((self.x2_ - self.x1_)**2 + (self.y2_ - self.y1_)**2)

        #print only if the distance is different from previous one
        
        if self.last_distance is None or abs(distance - self.last_distance) > 0.01:
            self.get_logger().info(f'Distance: {distance:.2f}')
            self.last_distance = distance

        # publish distance 
        msg = Float32()
        msg.data = distance
        self.dist_pub.publish(msg)

        threshold_distance = 0.5  # set your threshold here
        boundary_limit_min = 1.0
        boundary_limit_max = 10.0
        twist = Twist()


        # Check relative distance (stop moving turtle)
        if distance < threshold_distance and moving is not None:
            self.get_logger().warning(f'{moving} is moving and too close: stopping it!')
            
            if moving == "turtle1":
                twist.linear.x = - self.t1_vel.linear.x
                twist.linear.y = - self.t1_vel.linear.y
                twist.angular.z = - self.t1_vel.angular.z
                self.vel_pub1.publish(twist)
            elif moving == "turtle2":
                twist.linear.x = - self.t2_vel.linear.x
                twist.linear.y = - self.t2_vel.linear.y
                twist.angular.z = - self.t2_vel.angular.z
                self.vel_pub2.publish(twist)


        # Check boundaries for turtle1
        if (self.x1_ < boundary_limit_min or self.x1_ > boundary_limit_max or
            self.y1_ < boundary_limit_min or self.y1_ > boundary_limit_max):

            if moving == "turtle1":
                self.get_logger().warning('Turtle1 is too close to the boundary! Stopping turtle1.')
           
            twist.linear.x = - self.t1_vel.linear.x
            twist.linear.y = - self.t1_vel.linear.y
            twiast.angular.z = - self.t1_vel.angular.z
            self.vel_pub1.publish(twist)


        # Check boundaries for turtle2
        if (self.x2_ < boundary_limit_min or self.x2_ > boundary_limit_max or
            self.y2_ < boundary_limit_min or self.y2_ > boundary_limit_max):

            if moving == "turtle2":
                self.get_logger().warning('Turtle2 is too close to the boundary! Stopping turtle2.')

            twist.linear.x = - self.t2_vel.linear.x
            twist.linear.y = - self.t2_vel.linear.y
            twist.angular.z = - self.t2_vel.angular.z
            self.vel_pub2.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    distance_node = DistanceController()
    rclpy.spin(distance_node) #keeps the node alive, listening for messages
    rclpy.shutdown()

if __name__ == '__main__':
 main()