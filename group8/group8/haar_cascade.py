import cv2 
from matplotlib import pyplot as plt 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from std_msgs.msg import Int64MultiArray

class HaarCascade(Node):

    def __init__(self):
        super().__init__('haar_cascade')
        
        ## Initialize the buffer
        self.buffer = []
        ## Give the path to the model file. Please replace this with the path to the model file. 
        path = '/home/suhas99/ENPM673/aaaaaa/src/group8/group8/stop_data.xml'
        ## Harr Cascade classifier is initialized by loading the model specified in the path        
        self.stop_data = cv2.CascadeClassifier(path)         
                
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
        # img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        img = self._bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        ## Converting the bgr image to grayscale 
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        
        ## Checking if the object (Stop sign) is detected. If detected, the function outputs the position in the image where the object is detetcted.
        found = self.stop_data.detectMultiScale(img_gray, minSize=(20, 20)) 
        
        ## If detected, appending True to buffer
        if len(found) > 0:
            self.buffer.append(True)
        ## If not detected, appending False to buffer
        else:
            self.buffer.append(False)
        
        ## If the length of buffer is more than 5, the value added earliest is deleted. In other words, the buffer acts like a queue
        while len(self.buffer) > 5:
            self.buffer.pop(0)
            
        ## If more than 3 values in the fuffer is true, then it is concluded that the stop sign is detetcted. 
        if self.buffer.count(True) >= 3:
            ## The bounding box values are obtained from the output of the detectMultiScale function
            for (x, y, w, h) in found:
                ## The bounding box values are published to '/box_stop' topic
                x1,y1,x2,y2=x,y,x+w,y+h
                pub_msg_box.data=[int(x1),int(y1),int(x2),int(y2)]
                self.publisher_box.publish(pub_msg_box)
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            ## If the object is detetcted and is majority in buffer, the boolean value of True will be published to the '/stop' topic            
            pub_msg_stop.data=True
            self.publisher_stop.publish(pub_msg_stop)
            
        ## If the object is not detetcted, the boolean value of False will be published to the '/stop' topic        
        else:
            pub_msg_stop.data=False
            self.publisher_stop.publish(pub_msg_stop)

## Main function of the script                    
def main(args=None):
    rclpy.init(args=args)
    
    ## Creating a Ros2 Node object
    haar_cas = HaarCascade()

    rclpy.spin(haar_cas)
    
    haar_cas.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

