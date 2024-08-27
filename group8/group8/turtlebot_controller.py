import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

class TurtlebotController(Node):

    def __init__(self):
        super().__init__('turtlebot_controller')              # Initialise node name

        self.horizon_level = 0
        # self.camera_subscription = self.create_subscription(CompressedImage,
        #                                         '/camera/image_raw/compressed',
        #                                         self.camera_callback,
        #                                         qos_profile_sensor_data)
        self.camera_subscription = self.create_subscription(CompressedImage,
                                        '/image_raw/compressed',
                                        self.camera_callback,
                                        qos_profile_sensor_data)
        
        self.horizon_subscription = self.create_subscription(Int32,
                                                             '/horizon_level',
                                                             self.horizon_callback,
                                                             10)
        
        self.camera_feed_publisher = self.create_publisher(Image,
                                                           '/camera_feed',
                                                           qos_profile_sensor_data)
        
        self.stop_box_subscription = self.create_subscription(Int64MultiArray,
                                                        '/box_stop',
                                                        self.stop_box_callback,
                                                        qos_profile_sensor_data)
        
        self.pts_subscription = self.create_subscription(Int64MultiArray,
                                                '/pts_list',
                                                self.horiz_pts_callback,
                                                qos_profile_sensor_data)
        
        self.stop_subscription = self.create_subscription(Bool,
                                                '/stop',
                                                self.stop_callback,
                                                qos_profile_sensor_data)
        
        self.obs_subscription = self.create_subscription(Bool,
                                                '/obs',
                                                self.obs_callback,
                                                qos_profile_sensor_data)
        
        self.point_subscription = self.create_subscription(Int64MultiArray,
                                        '/points',
                                        self.points_callback,
                                        qos_profile_sensor_data)
        
        self.robo_publisher = self.create_publisher(Twist,
                                                    '/cmd_vel',
                                                    10)
        
        self.stop_subscription2 = self.create_subscription(Bool,
                                        '/stop_dynamic',
                                        self.stop_callback2,
                                        qos_profile_sensor_data)
        
        self.stop_box_subscription2 = self.create_subscription(Int64MultiArray,
                                                '/box_dynamic',
                                                self.stop_box_callback2,
                                                qos_profile_sensor_data)
        
        self.camera_subscription  # prevent unused variable warning
        self.horizon_subscription  # prevent unused variable warning
        self.stop_box=[]
        self.stop_box2=[]
        self.stop=False
        self.stop2=False
        self.bridge = CvBridge()
        self.selected_point=[]
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.robo_msg=Twist()
        self.horizon_detected = False
        self.turn_index=0
        self.pts_list=[]
        self.obs=False

    def camera_callback(self, msg):
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')                          
        if len(self.stop_box)!=0:
            if self.stop==True:
                [x1,y1,x2,y2]=self.stop_box
                cv2.putText(image, 'Stop', (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.8, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            if self.obs==True:
                [x3,y3,x4,y4]=self.stop_box2
                cv2.putText(image, 'Dynamic obstacle', (int(x3), int(y3)-10), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.8, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.rectangle(image, (int(x3), int(y3)), (int(x4), int(y4)), (0, 0, 255), 2)
                
        cv2.putText(image, 'Horizon', (0, self.horizon_level-10), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.8, (255, 0, 0), 2, cv2.LINE_AA) 
        cv2.line(image, (0, self.horizon_level), (image.shape[1], self.horizon_level), (255, 0, 0), 2, cv2.LINE_AA)
        for i in range(int(len(self.pts_list)/2)):
            x=self.pts_list[2*i]
            y=self.pts_list[(2*i)+1]
            cv2.circle(image,(x,y), 5, (0,0,255), -1)
        if len(self.selected_point)>0:
            cv2.circle(image,(int(self.selected_point[0]),int(self.selected_point[1])), 5, (0,255,0), -1)
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.camera_feed_publisher.publish(ros_image)

    def horizon_callback(self, msg):
        if not self.horizon_detected:
            self.horizon_level = msg.data
            self.horizon_detected = True
            
    def horiz_pts_callback(self,msg):
        self.pts_list=msg.data
        
    def stop_box_callback(self, msg):
        self.stop_box = msg.data
        
    def stop_box_callback2(self, msg):
        self.stop_box2 = msg.data

    def stop_callback(self, msg):
        self.stop = msg.data
        
    def stop_callback2(self, msg):
        # print(msg.data)
        self.stop2 = msg.data
        
    def obs_callback(self, msg):
        self.obs = msg.data

    def points_callback(self, msg):
        pointsX=[]
        pointsY=[]
        points = msg.data
        max_index=-1
        for i in range(int(len(points)/2)):
            pointsX.append(points[2*i])
            pointsY.append(points[(2*i)+1])
        max_y=0
        for i, y in enumerate(pointsY):
            if y>max_y:
                max_y=y
                max_index=i
        if max_index<0:
            self.selected_point=[]
        else:
            self.selected_point=[pointsX[max_index],pointsY[max_index]]
    
    def timer_callback(self):
        if self.obs==True and self.stop2 ==True:
            print("Dynamic object detected below horizon: Hence, stopping")
        if self.obs==True and self.stop2 ==False:
            print("Dynamic object detected above horizon: Hence, not stopping")
        if len(self.selected_point)>1:
            self.turn_index=0
            [x , y] = self.selected_point
            linear=0.05
            error = 320-x
            angular = 0.005*error
            if angular>0.04:
                angular = 0.04
            elif angular<-0.04:
                angular = -0.04
            if self.stop==True or self.stop2==True:
                self.robo_msg.linear.x=0.0
                self.robo_msg.angular.z=0.0
            elif self.stop==False and self.stop2==False:   
                self.robo_msg.linear.x=linear
                self.robo_msg.angular.z=angular
            self.robo_publisher.publish(self.robo_msg)
        else:
            if self.stop==True or self.stop2==True:
                self.robo_msg.linear.x=0.0
                self.robo_msg.angular.z=0.0
                self.robo_publisher.publish(self.robo_msg)
            elif self.stop==False and self.stop2==False:  
                if self.turn_index < 150:  
                    self.robo_msg.linear.x=0.0
                    self.robo_msg.angular.z=0.05
                    self.robo_publisher.publish(self.robo_msg)
                    self.turn_index=self.turn_index+1
                elif self.turn_index>=150 and self.turn_index<450:
                    self.robo_msg.linear.x=0.0
                    self.robo_msg.angular.z=-0.05
                    self.robo_publisher.publish(self.robo_msg)
                    self.turn_index=self.turn_index+1
                elif self.turn_index==450:
                    self.robo_msg.linear.x=0.0
                    self.robo_msg.angular.z=0.0
                    print("No Path found: Goal reached")
                    self.robo_publisher.publish(self.robo_msg)
                    self.turn_index=self.turn_index+1
        
        
def main(args=None):
    rclpy.init(args=args)

    node = TurtlebotController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

