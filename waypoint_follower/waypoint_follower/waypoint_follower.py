#!/usr/bin/env python
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray  # MarkerArrayを追加
import copy
import pprint

def publish_waypoints_as_marker_array(waypoints, publisher):
    marker_array = MarkerArray()
    for i, point in enumerate(waypoints):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rclpy.clock.Clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0 
        marker_array.markers.append(marker)

    publisher.publish(marker_array)


def main():
    rclpy.init()
    navigator = BasicNavigator()
    marker_array_pub = navigator.create_publisher(MarkerArray, 'visualization_marker_array', 10)

    # Set our demo's initial pose
    inspection_route = [
        [6.117561340332031, -0.3163766860961914, 0],
        [19.073131561279297, -0.38587427139282227, 0],
        [41.06983947753906, -0.47644567489624023, 0],
        [53.01405334472656, 0.0893254280090332, 0],
        [69.01844787597656, -0.8745026588439941, 0],
        [84.63866424560547, -0.5709228515625, 0],
        [97.73971557617188, 0.3908991813659668, 0],
        [103.78224182128906, -0.24866533279418945, 0],
        [111.71541595458984, -6.486872673034668, 0],
        [136.62686157226562, -5.793644428253174, 0],
        [150.55386352539062, -6.214706897735596, 0],
        [160.7079620361328, -6.293529510498047, 0],
        [162.81710815429688, -9.516770362854004, 0],
        [162.36671447753906, -22.75074005126953, 0],
        [162.50543212890625, -39.77167892456055, 0],
        [164.4688720703125, -46.6967658996582, 0],
        [165.5019073486328, -58.34597396850586, 0],
        [162.7490234375, -90.0484619140625, 0],
        [162.53988647460938, -97.95084381103516, 0],
        [161.8004913330078, -99.05369567871094, 0],
        [147.7860565185547, -96.3263168334961, 0],
        [138.30636596679688, -95.00430297851562, 0],
        [124.10421752929688, -93.67672729492188, 0],
        [108.4398193359375, -91.44690704345703, 0],
        [97.77694702148438, -89.71407318115234, 0],
        [66.4870376586914, -84.68327331542969, 0],
        [42.671146392822266, -81.50656127929688, 0],
        [23.91305923461914, -76.12701416015625, 0],
        [8.48819637298584, -69.18920135498047, 0],
        [-2.5683822631835938, -58.575355529785156, 0],
        [-4.327842712402344, -51.83643341064453, 0],
        [-4.355998992919922, -47.282386779785156, 0],
        [-3.6993446350097656, -44.011497497558594, 0],
        [-0.5563144683837891, -33.71904754638672, 0],
        [0.31604766845703125, -31.035762786865234, 0],
        [-2.8773574829101562, -26.625194549560547, 0],
        [-12.027100563049316, -17.407445907592773, 0]
    ]
    publish_waypoints_as_marker_array(inspection_route, marker_array_pub)

    navigator.waitUntilNav2Active()
    i = 0
    while rclpy.ok():

        publish_waypoints_as_marker_array(inspection_route, marker_array_pub)

        if(i>=len(inspection_route)):
            #ナビゲーション終わり
            print('ナビゲーション終了')
            break

        initial_pose = PoseStamped()
        navigator.setInitialPose(initial_pose)

        for point in inspection_route:
            i=i+1

            print("{}番目のウェイポイントへ行きます".format(i))
            inspection_pose = PoseStamped()
            inspection_pose.header.frame_id = 'map'
            inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
            inspection_pose.pose.orientation.z = 1.0
            inspection_pose.pose.orientation.w = 0.0
            inspection_pose.pose.position.x = point[0]
            inspection_pose.pose.position.y = point[1]

            #start navigation
            nav_start = navigator.get_clock().now()
            navigator.goToPose(inspection_pose)
            
            while not navigator.isTaskComplete():
                #ナビゲーション中の処理
                feedback = navigator.getFeedback()
                
  
            #success or timeout 
            result = navigator.getResult()
            if result.name == 'SUCCEEDED':
                print('Goal succeeded!')
            elif result.name == 'CANCELED':
                print('Goal was canceled!')
            elif result.name == 'FAILED':
                print('Goal failed!')
            
        
    # Wait for navigation to fully activate
    
    

if __name__ == '__main__':
    main()
