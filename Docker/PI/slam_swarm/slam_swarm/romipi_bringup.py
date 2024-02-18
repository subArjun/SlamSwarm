#!/usr/bin/python3

from slam_swarm.romipi_driver import AStar
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import math


LEFT = 0
RIGHT = 1


class romipi_bringup(Node):
    def __init__(self):
        super().__init__('romipi')
        self.romi = AStar()
        self.TwistSub = self.create_subscription(Twist,'cmd_vel', self.twist_callback,10)
        self.batteryPub = self.create_publisher(BatteryState, 'battery_state', 2)
        self.odomPub = self.create_publisher(Odometry, 'odom', 3)
        self.jointPub = self.create_publisher(JointState, 'joint_states', 3)
        self.tfBroadcaster = TransformBroadcaster(self)
        self.manager = self.create_timer(0.1, self.manager_callback)
        


    def twist_callback(self,data):
        linear_x = data.linear.x
        angular_z = data.angular.z
        self.romi.twist(linear_x,angular_z)



    def read_romi_state(self):
        # read all of the outputs
        self.battery_mv = self.romi.read_battery_millivolts()
        self.left_wheel_position, self.right_wheel_position = self.romi.read_encoders()
        self.left_wheel_velocity, self.right_wheel_velocity = self.romi.read_pose_motors()
        self.odom_pose = self.romi.read_pose_coordinate()
        self.odom_vel = self.romi.read_pose_twist() 
        self.odom_pose = self.romi.read_pose_coordinate()



    def broadcast_tf_msg(self):
        odom_tf = TransformStamped()
       
        odom_tf.header.frame_id = "odom"
        odom_tf.child_frame_id = "base_footprint"


        odom_tf.transform.translation.x = self.odom_pose[0]
        odom_tf.transform.translation.y = self.odom_pose[1]
        odom_tf.transform.translation.z = 0.0
        
        quaternion = quaternion_from_euler(0, 0, self.odom_pose[2])
        #type(pose) = geometry_msgs.msg.Pose
        odom_tf.transform.rotation.x = quaternion[0]
        odom_tf.transform.rotation.y = quaternion[1]
        odom_tf.transform.rotation.z = quaternion[2]
        odom_tf.transform.rotation.w = quaternion[3]

        odom_tf.header.stamp = self.get_clock().now().to_msg()

        self.tfBroadcaster.sendTransform(odom_tf)




    def publish_odom_state_msg(self):
        odom_state_msg = Odometry()

        # FIXME check that this shouldn't be /odom
        odom_state_msg.header.frame_id = "odom"
        # FIXME check that this shouldn't be /base_footprint
        odom_state_msg.child_frame_id = "base_footprint"

        # FIXME confirm units on read_pose_coordinate to be in meters
        odom_state_msg.pose.pose.position.x = self.odom_pose[0]
        odom_state_msg.pose.pose.position.y = self.odom_pose[1]
        odom_state_msg.pose.pose.position.z = 0.0
        quaternion = quaternion_from_euler(0, 0, self.odom_pose[2])
        #type(pose) = geometry_msgs.msg.Pose
        odom_state_msg.pose.pose.orientation.x = quaternion[0]
        odom_state_msg.pose.pose.orientation.y = quaternion[1]
        odom_state_msg.pose.pose.orientation.z = quaternion[2]
        odom_state_msg.pose.pose.orientation.w = quaternion[3]
        # FIXME confirm units on read_pose_twist to be m/s and rad/s
        odom_state_msg.twist.twist.linear.x = self.odom_vel[0]
        odom_state_msg.twist.twist.angular.z = self.odom_vel[1]

        odom_state_msg.header.stamp = self.get_clock().now().to_msg()

        self.odomPub.publish(odom_state_msg)




    def publish_joint_state_msg(self):
        joint_state_msg = JointState()
        joint_state_msg.name = ["wheel_left_joint", "wheel_right_joint"]

        # FIXME wheel position is just the encoder value, convert to radians
        self.left_rad = (self.left_wheel_position / 1437.09) * 2 * math.pi
        self.right_rad = (self.right_wheel_position / 1437.09) * 2 * math.pi
        joint_state_msg.position = [self.left_rad, self.right_rad]
        # FIXME confirm romi returns velocity in rad/s 
        joint_state_msg.velocity = [self.left_wheel_velocity, self.right_wheel_velocity]
        # we have no effort sensing, so leave blank
        joint_state_msg.effort = []

        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        self.jointPub.publish(joint_state_msg)




    def publish_battery_state_msg(self):

        battery_state_msg = BatteryState()
        battery_state_msg.header.stamp = self.get_clock().now().to_msg()
        battery_state_msg.design_capacity = 1.9
        battery_state_msg.voltage = self.battery_mv / 1000.0
        # 9.0 is 1.5 volts (for full nimh AA) times 6 batteries in series
        # 7.2 is 1.2 volts (for working nimh) times 6 batteries
        # 6.0 is the cut-off (empty)
        # so percentage = 1/3 * battery_v - 2
        # this method is not super accurate but better than nothing
        # as NimH battery discharge is very non-linear.
        # note percentage is 0-1.0 and not really a percentage per se.
        battery_state_msg.percentage = (battery_state_msg.voltage / 3.0) - 2.0
        if( self.battery_mv >= 0.00 ):
            battery_state_msg.present = True
        else:
            battery_state_msg.present = False

        self.batteryPub.publish(battery_state_msg)



    def manager_callback(self):
     self.read_romi_state()

     self.publish_battery_state_msg()
     self.publish_odom_state_msg()
     self.publish_joint_state_msg()
     self.broadcast_tf_msg()



def main():
    rclpy.init()
    bringup = romipi_bringup()
    rclpy.spin(bringup)
    bringup.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
