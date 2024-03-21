#================================
#參考
# https://blog.csdn.net/chouzhou9701/article/details/109992317?spm=1035.2023.3001.6557&utm_medium=distribute.pc_relevant_bbs_down.none-task-blog-2~default~OPENSEARCH~default-2.nonecase&depth_1-utm_source=distribute.pc_relevant_bbs_down.none-task-blog-2~default~OPENSEARCH~default-2.nonecase

# v4l2 圖形化界面：
# sudo apt install qv4l2
# qv4l2
#================================ 


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('H65_camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'rgb', 10)
        self.timer = self.create_timer(0.001, self.publish_image)
        self.cap = cv2.VideoCapture()
        self.cap.open(2, apiPreference=cv2.CAP_V4L2)#設定相機為V4L2 for linux to setup 

        # 設定解析度為960x720
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,3) #自動曝光
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,1) #手動曝光
        self.cap.set(cv2.CAP_PROP_EXPOSURE,45)   #設定曝光度

        self.cap.set(cv2.CAP_PROP_CONTRAST, 50)  #設定對比度
        
        #=======畫面處理=========
        # 裁切區域的長度與寬度
        self.y = 450
        self.x = 600
        or_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        or_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.image_h = int((or_h-self.y)/2)
        self.image_w = int((or_w-self.x)/2)
        #=======================

        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error('Unable to open the camera.')
            raise RuntimeError('Unable to open the camera.')

    def publish_image(self):
        ret, frame = self.cap.read()
        # frame_flipped = cv2.flip(frame, 1)
        frame = frame[self.image_h:self.image_h+self.y, self.image_w:self.image_w+self.x]
        frame = cv2.resize(frame,(640, 480))

        if ret:
            try:
                ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher_.publish(ros_image_msg)
                self.get_logger().info('Image published to camera_image topic')
            except Exception as e:
                self.get_logger().error(f'Error converting and publishing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    try:
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
    except Exception as e:
        print(f'Error during execution: {str(e)}')
    finally:
        if camera_publisher:
            camera_publisher.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
