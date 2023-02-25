import matplotlib.colors
import pandas as pd
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64
from threading import Thread
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from avai_messages.msg import FloatArray, FloatList, PointArray
from PyQt5.QtCore import *
import cv2
import sys
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from cv_bridge import CvBridge
import message_filters


class GuiNode(Node):
    hmi = None  # MainWindow

    def __init__(self):
        super().__init__("gui_node")
        cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
        self.recording = False
        self.recorded_image_counter = 0
        self.recordOne = False
        self.manual_recorded_image_counter = 0
        self.bounding_boxes = []
        self.bounding_box_arrived = 0
        self.cv_bridge_ = CvBridge()

        #Synchronized Messages
        self.image_sub = message_filters.Subscriber(self,CompressedImage,"processed_image")
        self.boundingbox_sub = message_filters.Subscriber(self,FloatArray,"bounding_box")
        ts = message_filters.TimeSynchronizer([self.boundingbox_sub , self.image_sub], 10)

        ts.registerCallback(self.drawBoundingBoxes)

        #self.subscriber_boundingBox = self.create_subscription(FloatArray,"bounding_box", self.callback_process_boundingBox,10)
        self.subscriber_Lidar = self.create_subscription(PointArray, "updated_points", self.callback_draw_map, 10)
        self.subscriber_ = self.create_subscription(CompressedImage, "processed_image", self.callback_processed_image, 10)
        self.publisher_ = self.create_publisher(Float64, "set_frequency", 10)

    def drawBoundingBoxes(self,boundingBox, imageToDraw):

        self.get_logger().info("Bounding Boxes and images arrived")
        processed_bounding_box = []
        for lst in boundingBox.lists:
            box = []
            for e in lst.elements:
                box.append(e)

            processed_bounding_box.append(box)
        self.bounding_boxes = processed_bounding_box
        self.get_logger().info(str(self.bounding_boxes))
        processed_image = self.cv_bridge_.compressed_imgmsg_to_cv2(imageToDraw)
        self.get_logger().info("Bounding Boxes and Images processed")

        for box in self.bounding_boxes:
            h, w, c = processed_image.shape
            if int(box[5]) == 0:
                text = 'Blue ' + str(round(box[4],2));
                font =  cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 0.7
                color = (255,0,0)
                thickness = 1
                cv2.putText(processed_image,text,(int(w*(box[0]-(box[2]/2))),int(h*(box[1]-(box[3]/2)))-10),font,fontScale,color,thickness,cv2.LINE_AA)
                cv2.rectangle(processed_image,(int(w*(box[0]-(box[2]/2))),int(h*(box[1]-(box[3]/2)))),(int(w*(box[0]+(box[2]/2))),int(h*(box[1]+(box[3]/2)))), (255,0,0),3)
            if int(box[5]) == 1:
                text = 'Orange ' + str(round(box[4],2));
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 0.7
                color = (0,165,255)
                thickness = 1
                cv2.putText(processed_image, text,
                            (int(w * (box[0] - (box[2] / 2))), int(h * (box[1] - (box[3] / 2))) - 10), font, fontScale,
                            color, thickness, cv2.LINE_AA)
                cv2.rectangle(processed_image,(int(w*(box[0]-(box[2]/2))),int(h*(box[1]-(box[3]/2)))),(int(w*(box[0]+(box[2]/2))),int(h*(box[1]+(box[3]/2)))), (0,165,255),3)
            if int(box[5]) == 2:
                text = 'Yellow ' + str(round(box[4],2));
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 0.7
                color = (0,255,255)
                thickness = 1
                cv2.putText(processed_image, text,
                            (int(w * (box[0] - (box[2] / 2))), int(h * (box[1] - (box[3] / 2))) - 10), font, fontScale,
                            color, thickness, cv2.LINE_AA)
                cv2.rectangle(processed_image,(int(w*(box[0]-(box[2]/2))),int(h*(box[1]-(box[3]/2)))),(int(w*(box[0]+(box[2]/2))),int(h*(box[1]+(box[3]/2)))), (0,255,255),3)


        cv2.imshow("Video", processed_image)

    def callback_draw_map(self, msg):
        """"
        self.get_logger().info("Lidar Values received")
        lidar_values = []
        for lst in msg.lists:
            lidar = []
            for e in lst.elements:
                lidar.append(e)

            lidar_values.append(lidar)
        self.get_logger().info(str(lidar_values))
        self.get_logger().info("Lidar Values processed")
        """
        lidar_values = []
        for point in msg.data:
            lidar_values.append([point.x, point.y, point.c])
        self.get_logger().info(str(lidar_values))
        self.hmi.update_plot(msg)

    def callback_processed_image(self, msg):
        processed_image = self.cv_bridge_.compressed_imgmsg_to_cv2(msg)
        #cv2.imshow("Video",processed_image)
        #self.drawBoundingBoxes(processed_image,self.bounding_boxes)
        if self.recording:
            cv2.imwrite("Image%d.png" % self.recorded_image_counter, processed_image)
            self.recorded_image_counter += 1
        if self.recordOne:
            self.recordOne = False
            cv2.imwrite("ManualImage%d.png" % self.manual_recorded_image_counter, processed_image)
            self.manual_recorded_image_counter += 1

    def callback_process_boundingBox(self,msg):
        '''
        self.get_logger().info("Bounding Boxes arrived")
        processed_bounding_box = []
        for lst in msg.lists:
            box = []
            for e in lst.elements:
                box.append(e)

            processed_bounding_box.append(box)
        self.bounding_boxes = processed_bounding_box

        self.get_logger().info(str(self.bounding_boxes))
        self.get_logger().info("Bounding Boxes processed")
        '''
        pass

class MainWindow(QWidget):
    node: GuiNode = None

    def __init__(self):
        super(MainWindow, self).__init__()

        self.VBL = QVBoxLayout()

        #self.imageLabel = QLabel()
        #self.VBL.addWidget(self.imageLabel)
        self.figure = plt.figure()
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.toolbar = NavigationToolbar2QT(self.canvas)
        self.VBL.addWidget(self.toolbar)
        self.VBL.addWidget(self.canvas)

        self.saveScreenshotBtn = QPushButton("Save single image")
        self.saveScreenshotBtn.clicked.connect(self.save_screenshot)
        self.VBL.addWidget(self.saveScreenshotBtn)

        self.toggleRecordingBtn = QPushButton("Start recording")
        self.toggleRecordingBtn.clicked.connect(self.toggle_recording)
        self.VBL.addWidget(self.toggleRecordingBtn)
        self.frequencyTxt = QLineEdit()
        self.frequencyTxt.setPlaceholderText("Frequency = 0 Images per Second")
        self.frequencyTxt.setValidator(QDoubleValidator(0.0, 177013.0, 10))
        self.frequencyTxt.returnPressed.connect(self.set_recording_frequency)
        self.VBL.addWidget(self.frequencyTxt)

        self.setLayout(self.VBL)

    def update_image(self, image):
        self.imageLabel.setPixmap(QPixmap.fromImage(image))

    def save_screenshot(self):
        self.node.recordOne = True
        self.node.get_logger().info("Recording one (1) image.")

    def toggle_recording(self):
        if self.node.recording:
            self.node.recording = False
            self.toggleRecordingBtn.setText("Start recording.")
        else:
            self.node.recording = True
            self.toggleRecordingBtn.setText("Stop recording.")

    def set_recording_frequency(self):
        frequency = float(self.frequencyTxt.text())
        msg = Float64()
        msg.data = frequency
        self.node.publisher_.publish(msg)
        self.node.get_logger().info("Sent new frequency.")

    def update_plot(self, msg):
        if len(msg.data) != 0 :
            self.figure.clear()
            ax = self.figure.add_subplot(111)
            data = []
            for point in msg.data:
                data.append([point.x, point.y, point.c])

            #Get X and Y  values
            x_values = [point[0] for point in data]
            y_values = [point[1] for point in data]

            #Get indices of points with class 4
            class_4_indices = [i for i, point in enumerate(data) if point[2] == 4]

            turtlestate = msg.turtlestate
            #colors = ['blue', 'orange', 'yellow', 'black']
            df = pd.DataFrame(data)
            dfX = df.iloc[:, 0]
            dfX = dfX.to_numpy()
            dfX = np.append(dfX, turtlestate.x)
            dfY = df.iloc[:, 1]
            dfY = dfY.to_numpy()
            dfY = np.append(dfY, turtlestate.y)
            dfClasses = df.iloc[:, 2]
            dfClasses = dfClasses.to_numpy()
            dfClasses =  np.append(dfClasses, 3)
            colors = []
            for classes in dfClasses:
                if classes == 0:
                    colors.append("blue")
                if classes == 1:
                    colors.append("orange")
                if classes == 2:
                    colors.append("yellow")
                if classes == 3:
                    colors.append("black")
                if classes == 4:
                    colors.append("red")

            length = 1.5
            angle = np.deg2rad(turtlestate.angle)
            angle2 = np.deg2rad(turtlestate.angle+31.5)
            angle3 = np.deg2rad(turtlestate.angle-31.5)
            end = [turtlestate.x + np.cos(angle) * length, turtlestate.y + np.sin(angle)*length]
            end2 = [turtlestate.x + np.cos(angle2) * length, turtlestate.y + np.sin(angle2)*length]
            end3 = [turtlestate.x + np.cos(angle3) * length, turtlestate.y + np.sin(angle3)*length]
            line = plt.Line2D([turtlestate.x, end[0]],[turtlestate.y,end[1]],linestyle='dashed')
            line2 = plt.Line2D([turtlestate.x, end2[0]], [turtlestate.y, end2[1]],linestyle='dashed',color="red")
            line3 = plt.Line2D([turtlestate.x, end3[0]], [turtlestate.y, end3[1]],linestyle='dashed',color="orange")
            ax.scatter(dfX, dfY, c=colors)
            ax.grid()
            ax.add_line(line)
            ax.add_line(line2)
            ax.add_line(line3)
            ax.set_xlim([0,10.2])
            ax.set_ylim([0,10.2])
            if class_4_indices:
                ax.plot([x_values[i] for i in class_4_indices[:-1]], [y_values[i] for i in class_4_indices[:-1]])
            self.canvas.draw()


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    hmi = MainWindow()

    node = GuiNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    hmi.node = node
    node.hmi = hmi

    thread = Thread(target=executor.spin)
    thread.start()

    try:
        hmi.show()
        sys.exit(app.exec_())
    finally:
        node.get_logger().info("Shutting down GUI node...")
        node.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()
