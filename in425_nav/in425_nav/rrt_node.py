import rclpy
from rclpy.node import Node

from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose2D

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
import cv2
import numpy as np

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
        self.buildMapImage()

        self.x_goal_image = 0
        self.y_goal_image = 0
        #TODO: create the related image

    def buildMapImage(self):
        """Construit une image OpenCV à partir de l'OccupancyGrid"""

        # 1. Récupérer les dimensions et les données brutes
        width  = self.map.info.width   # colonnes (pixels)
        height = self.map.info.height  # lignes   (pixels)
        data   = np.array(self.map.data, dtype=np.int8).reshape((height, width))

        # 2. Créer l'image en niveaux de gris (uint8)
        img = np.zeros((height, width), dtype=np.uint8)

        UNEXPLORED_VALUE = -1
        EXPLORED_VALUE   = 0
        OBSTACLE_THRESHOLD = 100   # toute valeur >= 1 est un obstacle

        img[data == UNEXPLORED_VALUE]      = 80   # gris foncé
        img[data == EXPLORED_VALUE]        = 220  # gris très clair
        img[data == OBSTACLE_THRESHOLD]    = 0    # noir

        # 3. Retourner l'image (axe Y inversé entre OccupancyGrid et image)
        self.map_img = cv2.flip(img, 0)
        cv2.imwrite("/tmp/map_debug.png", self.map_img)
        self.get_logger().info(f"Image sauvegardée — min:{self.map_img.min()} max:{self.map_img.max()} unique:{np.unique(self.map_img)}")

        cv2.imshow("Map", self.map_img)
        cv2.waitKey(100)
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
        #self.get_logger().info(f"goal position of path plannig = {msg.pose.position}")

        x = msg.pose.position.x
        y = msg.pose.position.y
    
        self.get_logger().info(f"x_map= {x}")
        self.get_logger().info(f"y_map = {y}")
   
        self.x_goal_image, self.y_goal_image = self.transition_map_image(x,y)

        self.get_logger().info(f"x_image= {self.x_goal_image}")
        self.get_logger().info(f"y_image = {self.y_goal_image}")

        self.get_robot_pose()
        
        self.get_logger().info(f"robot image x = {self.robot_pose.x}")
        self.get_logger().info(f"robot image y = {self.robot_pose.y}")

        #self.run()


    def transition_map_image(self,x,y):
        
        width = self.map.info.width
        height = self.map.info.height

        dx = -self.map.info.origin.position.x
        dy = -self.map.info.origin.position.y
        res = round(self.map.info.resolution,3)
    
        origin = np.array([[1,0,0,dx],[0,1,0,   dy],[0,0,1,0],[0,0,0,1]])@np.transpose(np.array([x,y,0,1]))
        origin_discrete = np.array([[1/res,0],[0,1/res]])@origin[:2]

        self.get_logger().info(f"height = {height},width = {width}, res = {res}")

        co_image = np.array([[1,0,0,0],[0,-1,0,height-1],[0,0,1,0],[0,0,0,1]])@np.transpose([int(origin_discrete[0]),int(origin_discrete[1]),0,1])

        return co_image[0],co_image[1]


    # **********************************
    def run(self):
        """ TODO - Implement the RRT-Connect algorithm """
        
        Tstart = np.array([[self.robot_pose.x],[self.robot_pose.y],[0],[1]])
        Tgoal = np.array([[self.x_goal_image],[self.y_goal_image],[0],[1]])
        List_Tstart = [Tstart]
        List_Tgoal = [Tgoal]
        while np.linalg.norm(Tstart-Tgoal)>0.5:
            qrand = self.rand_free_conf()
            qnew = self.extend(Tstart, qrand)
            if qnew is not None:
                List_Tstart.append(qnew)
                qnew_goal = self.connect(Tgoal, qnew)
                if qnew_goal is not None:
                    List_Tgoal.append(qnew_goal)
                    break
            self.swap(Tstart, Tgoal)
            List_Tstart, List_Tgoal = List_Tgoal, List_Tstart
            Tstart, Tgoal = Tgoal, Tstart

        
    def rand_free_conf(self):
        """ Sample a random configuration in the free space """
        pass


    def extend(self, Tstart, qrand):
        """ Move the tree by one incremental step toward q.
            Stop immediately if an obstacle is encountered.
        """

        pass

    def connect(self, Tgoal, qnew):
        """ Repeatedly call EXTEND(tree, q).
            Continue until:
            ▶ q is reached, or
            ▶ an obstacle blocks further progress.
            This is the key innovation of RRT-Connect.
        """

        pass
    
    def swap(self, Tstart, Tgoal):
        """ Swap the trees """
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