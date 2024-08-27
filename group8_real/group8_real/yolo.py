import cv2 
from matplotlib import pyplot as plt 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import Bool
from std_msgs.msg import Int64MultiArray
from sensor_msgs.msg import CompressedImage

class YOLOV8(Node):

    def __init__(self):
        super().__init__('yolo_v8')
        ## Give the path to the model file. Please replace this with the path to the model file. 
        yolo_model_path = '/home/suhas99/ENPM673/aaaaaa/src/group8/group8/best.pt'
        ## Yolo model is initialized by loading the model specified in the path
        self.model = YOLO(yolo_model_path)
        
        ## A threshold for obstacle detection is declared        
        self.threshold = 0.7
        
        ## Subscriber for turtlebot camera topic to get the image        
        self.subscription = self.create_subscription(CompressedImage,'/image_raw/compressed',self.camera_callback,qos_profile_sensor_data)
        self.subscription
        ## Publisher to publish the flag value for stopping the robot when a stop sign is detected               
        self.publisher_stop = self.create_publisher(Bool,'/stop',qos_profile_sensor_data)
        ## Publisher to publish the bounding box values                
        self.publisher_box = self.create_publisher(Int64MultiArray,'/box_stop',qos_profile_sensor_data)
        ## Initializing a CV bridge to convert sensor_msgs Image to a opencv readable image format              
        self._bridge = CvBridge()

    ## Callback function for camera subscription
    def camera_callback(self, msg):
        ## Initializing the messages for publishing        
        pub_msg_stop = Bool()
        pub_msg_box = Int64MultiArray()
        ## Converting the sersor msg image to opencv readable image         
        img = self._bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        # img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        
        ## Checking if the object (Stop sign) is detected. If detected, the model outputs the position in the image where the object is detetcted and the confidence score        
        results = self.model(img)[0]
        
        ## Each detetction is considered (if there are multiple detections)
        for result in results.boxes.data.tolist():
            
            ## The bounding box values and the confidence score is extracted
            x1, y1, x2, y2, score, class_id = result
            
            ## Checking if the confidence score is greater than the defined threshold
            if score >= self.threshold:
                ## The bounding box values are published to '/box_stop' topic
                pub_msg_box.data=[int(x1),int(y1),int(x2),int(y2)]
                ## The boolean value of True will be published to the '/stop' topic
                pub_msg_stop.data=True
                self.publisher_stop.publish(pub_msg_stop)
                self.publisher_box.publish(pub_msg_box)
            ## If the confidence score is less than threshold, the boolean value of False will be published to the '/stop' topic
            else:
                pub_msg_stop.data=False
                self.publisher_stop.publish(pub_msg_stop)

## Main function of the script                           
def main(args=None):
    rclpy.init(args=args)
    
    ## Creating a Ros2 Node object
    yolo = YOLOV8()

    rclpy.spin(yolo)
    
    yolo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

