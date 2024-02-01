import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
import serial
import time
import math
import numpy as np
from nav_msgs.msg import Odometry
import tf2_ros

TREAD   = 0.376     #truck center2center
R_RIGHT = 0.03858  #radius
R_LEFT  = 0.03858  #radius

PR = 360.0 #Pulse count

ser = serial.Serial('/dev/cugo', 115200) 

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians. 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  return [qx, qy, qz, qw]

class CmdSubscriver(Node):
    def __init__(self):
        super().__init__('cmd_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.subscription_stop_signal = self.create_subscription(
            Bool,
            'stop_signal',
            self.stop_signal_callback,
            10)
        self.stop = False
      
    def cmd_callback(self, msg):
        if self.stop:
            # stop_signalがTrueの場合は停止
            ser.write(str.encode("c,0,0\n"))
            return
        twist=msg
        v=twist.linear.x
        omega=twist.angular.z
        vr=v-TREAD/2*omega #m/s
        vl=v+TREAD/2*omega #m/s
        omega_r=vr/R_RIGHT #rad/s
        omega_l=vl/R_LEFT  #rad/s
        n_r=omega_r/(math.pi*2) #rot/s
        n_l=omega_l/(math.pi*2) #rot/s
        rpm_r = n_r * 60
        rpm_l = n_l * 60
        
        print("L:",rpm_l," R:",rpm_r)
        ser.write(str.encode("c,"+str(int(rpm_r))+","+str(int(rpm_l))+"\n")) 

    def stop_signal_callback(self, msg):
        self.stop = msg.data
        if self.stop:
            ser.write(str.encode("c,0,0\n"))  # モータを停止

def main(args=None):
    print("Ready")
    rclpy.init(args=args)
    node = rclpy.create_node('simple_node')
    minimal_subscriber = CmdSubscriver()
    rate = node.create_rate(100)
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    odom_broadcaster = tf2_ros.TransformBroadcaster(node)

    while rclpy.ok():
      rclpy.spin_once(minimal_subscriber,timeout_sec=0.01)
      rclpy.spin_once(node) 

      line = ser.readline()          
      print(line)

      ser.reset_input_buffer()
      line=line.decode('utf-8')
      print("decoded:",line)
      l=line.split(",")
      # current_time=time.time()
      # elapsed_time=time_e-time_s
      com=l[0]
      print(l)
      if(com=="o"):
          x=float(l[1])
          y=float(l[2])
          z=float(l[3])
          print("x:",x," y:",y," theta:",z)
          quat=get_quaternion_from_euler(0,0,z)
          t = TransformStamped()
          current_time = node.get_clock().now().to_msg()
          t.header.stamp = current_time
          t.header.frame_id = "odom"
          t.child_frame_id = "base_link"
          t.transform.translation.x = x
          t.transform.translation.y = y
          t.transform.translation.z = 0.0
          t.transform.rotation.x = quat[0]
          t.transform.rotation.y = quat[1]
          t.transform.rotation.z = quat[2]
          t.transform.rotation.w = quat[3]
          odom_broadcaster.sendTransform(t)
          odom = Odometry()
          odom.header.frame_id = "odom"
          odom.header.stamp = current_time
          # set the position
          odom.pose.pose.position.x = x
          odom.pose.pose.position.y = y
          odom.pose.pose.position.z = 0.0
          odom.pose.pose.orientation.x = quat[0]
          odom.pose.pose.orientation.y = quat[1]
          odom.pose.pose.orientation.z = quat[2]
          odom.pose.pose.orientation.w = quat[3]
          # set the velocity
          odom.child_frame_id = "base_link"
          odom_pub.publish(odom)
      rate.sleep()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
