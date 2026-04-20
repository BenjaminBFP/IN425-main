import rclpy
from rclpy.node import Node

from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose2D

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import cv2


class RRTConnect(Node):
    def __init__(self, K=0, dq=0):
        Node.__init__(self, "rrt_connect_node")

        """ Attributes """
        self.robot_pose = Pose2D()  #Current pose of the robot: self.robot.x, self.robot.y, robot.theta(last one is useless)
        self.path = []  #Path containing the waypoints computed by the RRT-Connect in the image reference frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  #used to get the position of the robot
        #TODO: add your attributes here....
        
        """ Publisher and Subscriber """
        self.create_subscription(PoseStamped, "/goal_pose", self.goalCb, 1)
        self.path_pub = self.create_publisher(Path, "/path", 1)

        """ Load the map and create the related image"""
        self.getMap()
        #TODO: create the related image


    def __del__(self):
        """ Called when the object is destroyed """
        cv2.destroyAllWindows() #destroy all the OpenCV windows you displayed


    # **********************************    
    def getMap(self):
        """ Method for getting the map """
        #DO NOT TOUCH
        map_cli = self.create_client(GetMap, "map_server/map")
        while not map_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for map service to be available...")
        map_req = GetMap.Request()
        future = map_cli.call_async(map_req)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    self.map = future.result().map  #OccupancyGrid instance!
                    self.get_logger().info("Map loaded !")
                except Exception as e:
                    self.get_logger().info(f"Service call failed {e}")
                return


    # **********************************
    def get_robot_pose(self):
        """ Get the current position of the robot """
        #DO NOT TOUCH
        try:
            trans = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time()
            )
            self.robot_pose.x = trans.transform.translation.x
            self.robot_pose.y = trans.transform.translation.y
        except TransformException as e:
            self.get_logger().info(f"Could not transform base_footprint to map: {e}")


    # **********************************
    def goalCb(self, msg):
        """ TODO - Get the goal pose """
        #get the goal pose here
        #....
        self.run()


    # **********************************
    def run(self):
        """ TODO - Implement the RRT-Connect algorithm """
        pass
        
    # **********************************
    def publishPath(self):
        """ Send the computed path so that RVIZ displays it """
        """ TODO - Transform the waypoints from pixels coordinates to meters in the map frame """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        path_rviz = []
        for pose_img in self.path:
            pose = PoseStamped()
            # pose.pose.position.x = ...
            # pose.pose.position.y = ...
            path_rviz.append(pose)
        msg.poses = path_rviz
        self.path_pub.publish(msg)


def main():
    #DO NOT TOUCH
    rclpy.init()

    node = RRTConnect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()