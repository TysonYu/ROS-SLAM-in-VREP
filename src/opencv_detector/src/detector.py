#!/usr/bin/env python
import roslib
roslib.load_manifest('opencv_detector')
import rospy
import sys, select, termios, tty
import time
import Image
import os
import cv2, numpy
from std_msgs.msg import String
from std_msgs.msg import Bool
import sensor_msgs.msg
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from detection_msgs.msg import Detection
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

CV_WINDOW_TITLE = "OpenCV object detection"

# Node for object detection.
class Detector():
    def __init__(self):
        # Get the ~private namespace parameters from command line or launch file
        # Set basic paramateres
        self.subjects = ["", "Obama", "Avirl", "ZhangGuorong", "legolasGreenleaf", "Levi"]
        self.image_scale = rospy.get_param('~scale', None)
        self.throttle = rospy.get_param('~throttle', 10)
        data_path = rospy.get_param('~detector_file', '')
        pic_path = rospy.get_param('~recognition_file','')
        if not data_path:
            sys.exit(1)        

        self.throttle = None if self.throttle <= 0 else (1 / float(self.throttle))

        self.detectorCascade = cv2.CascadeClassifier(data_path)
        self.faces, self.labels = self.prepare_training_data(pic_path)
        self.face_recognizer = cv2.face.LBPHFaceRecognizer_create()
        self.face_recognizer.train(self.faces, numpy.array(self.labels))
        self.throttle_time = rospy.Time.now()

        # init camera capture stuff
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera', Image, self.image_callback, queue_size=1)

        # Subscribers and publishers
        self.detections_pub = rospy.Publisher('detections', Detection, queue_size=10)
        self.toggle_sub = rospy.Subscriber('toggle', Bool, self.toggle_callback, queue_size=10)

        # init and call detection
        self.enabled = rospy.get_param('~enabled', True)
        self.cv_window = rospy.get_param('~show_cv_window', True)
        self.message_counter = 0

    def detect_objects(self, image, draw=True):
        min_size = (10,10)
        max_size = (60,60)
        haar_scale = 1.2
        min_neighbors = 3
        haar_flags = 0

        # Convert color input image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Scale input image for faster processing
        if self.image_scale == None:
            self.image_scale = image.shape[1] / 240
        smallImage = cv2.resize(gray, (int(image.shape[1] / self.image_scale), int(image.shape[0] / self.image_scale)), interpolation=cv2.INTER_LINEAR)
        #smallImage = cv2.flip(smallImage,1)
        # Equalize the histogram
        smallImage = cv2.equalizeHist(smallImage)
        # Detect the objects
        results = self.detectorCascade.detectMultiScale(smallImage, haar_scale, min_neighbors, haar_flags, min_size, max_size)
        detections = []
        for (x, y, w, h) in results:
            pt1 = (int(x * self.image_scale), int(y * self.image_scale))
            pt2 = (int((x + w) * self.image_scale), int((y + h) * self.image_scale))
            if draw:
                cv2.rectangle(image, pt1, pt2, (255, 0, 0), 3, 8, 0)
            detection_image = image[pt1[1]:pt2[1], pt1[0]:pt2[0]]
            detections.append((pt1[0], pt1[1], pt2[0] - pt1[0], pt2[1] - pt1[1], numpy.copy(detection_image))) 
       # if draw:
        #    detections.append(image2)
        return detections

    def toggle_callback(self, data):
        self.enabled = data.data
        if self.enabled:
            rospy.loginfo("Object detection enabled")
        else:
            rospy.loginfo("Object detection disabled")
          
    def image_callback(self, data):
        if not self.enabled:
            return
        try:
            now = rospy.Time.now()
            if self.throttle and (now.to_sec() - self.throttle_time.to_sec()) < self.throttle:
                return

            self.throttle_time = now

            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #Detect_result = self.detect_objects(cv_image, self.cv_window)
            detections = self.detect_objects(cv_image, self.cv_window)
            #detections = Detect_result[0]
            #label_image = Detect_result[1]

            if len(detections) > 0:
                message = Detection()
                message.header.seq = self.message_counter
                message.header.stamp = data.header.stamp
                message.header.frame_id = data.header.frame_id
                for detection in detections:
                    message.x = detection[0]
                    message.y = detection[1]
                    message.width = detection[2]
                    message.height = detection[3]
                    message.source = 'opencv'
                    message.confidence = 1
                    message.image = self.bridge.cv2_to_imgmsg(detection[4], "bgr8")

                self.message_counter += 1
                self.detections_pub.publish(message)
            
            if self.cv_window:
                cv_image = cv2.flip(cv_image,1)          
                cv2.imshow(CV_WINDOW_TITLE, self.predict(cv_image))
                cv2.waitKey(1)

        except CvBridgeError, e:
            print e
          
    def prepare_training_data(self, data_folder_path):
    #get the directories (one directory for each subject) in data folder
        dirs = os.listdir(data_folder_path)
    #list to hold all subject faces
        faces = []
    #list to hold labels for all subjects
        labels = []
    #let's go through each directory and read images within it
        for dir_name in dirs:
            if not dir_name.startswith("s"):
                continue;
            label = int(dir_name.replace("s", ""))
            subject_dir_path = data_folder_path + "/" + dir_name
            subject_images_names = os.listdir(subject_dir_path)
            for image_name in subject_images_names:
            #ignore system files like .DS_Store
                if image_name.startswith("."):
                    continue;
                image_path = subject_dir_path + "/" + image_name
            #read image
                image = cv2.imread(image_path)
                image = cv2.resize(image, (400, 500))
            #display an image window to show the image 
            	# cv2.imshow("Training on image...", cv2.resize(image, (400, 500)))
                cv2.waitKey(100)
            #detect face
                face, rect = self.detect_face(image)
                if face is not None:
                #add face to list of faces
                    faces.append(face)
                #add label for this face
                    labels.append(label)
        return faces, labels
 

    def draw_rectangle(self, img, rect):
        (x, y, w, h) = rect
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
    
        #function to draw text on give image starting from
        #passed (x, y) coordinates. 
    def draw_text(self, img, text, x, y):
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 255), 2)


    def predict(self, test_img):
        #make a copy of the image as we don't want to chang original image
        img = test_img.copy()
        #detect face from the image
        face, rect = self.detect_face(img)
        #predict the image using our face recognizer 
        if not face is None:
            label, confidence = self.face_recognizer.predict(face)
            #get name of respective label returned by face recognizer
            label_text = self.subjects[label]
    
            #draw a rectangle around face detected
            self.draw_rectangle(img, rect)
            #draw name of predicted person
            self.draw_text(img, label_text, rect[0], rect[1]-50)
            self.draw_text(img, str(confidence), rect[0], rect[1]-20)
    
        return img

    def detect_face(self,img):
       gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
       faces = self.detectorCascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5);
       if (len(faces) == 0):
           return None, None
       (x, y, w, h) = faces[0]
       return gray[y:y+w, x:x+h], faces[0]
# Main function.    
if __name__ == '__main__':
        # Initialize the node and name it.
        rospy.init_node('opencv_detector')
        try:
            fd = Detector()
            rospy.spin()    
        except rospy.ROSInterruptException: pass
