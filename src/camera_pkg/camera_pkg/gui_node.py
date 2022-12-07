import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64, Float64MultiArray
from threading import Thread
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from avai_messages.msg import FloatArray
from avai_messages.msg import FloatList
from PyQt5.QtCore import *
import cv2
import sys
from cv_bridge import CvBridge


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

        self.cv_bridge_ = CvBridge()

        self.subscriber_boundingBox = self.create_subscription(FloatArray,"bounding_box", self.callback_process_boundingBox,10)
        self.subscriber_ = self.create_subscription(CompressedImage, "processed_image", self.callback_processed_image, 10)
        self.publisher_ = self.create_publisher(Float64, "set_frequency", 10)

    def callback_processed_image(self, msg):
        self.get_logger().info("Received processed image.")
        processed_image = self.cv_bridge_.compressed_imgmsg_to_cv2(msg)
        cv2.imshow("Video",processed_image)
        if self.recording:
            cv2.imwrite("Image%d.png" % self.recorded_image_counter, processed_image)
            self.recorded_image_counter += 1
        if self.recordOne:
            self.recordOne = False
            cv2.imwrite("ManualImage%d.png" % self.manual_recorded_image_counter, processed_image)
            self.manual_recorded_image_counter += 1

        #q_image = cv2.cvtColor(processed_image, cv2.COLOR_BGR2RGB)
        #q_image = cv2.flip(q_image, 1)
        #q_image = QImage(q_image.data, q_image.shape[1], q_image.shape[0], QImage.Format_RGB888)
        #q_image = q_image.scaled(640, 480, Qt.KeepAspectRatio)
        #self.hmi.update_image(q_image)

    def callback_process_boundingBox(self,msg):
        self.get_logger().info("Bounding Boxes arrived")
        processed_bounding_box = []
        for lst in msg.lists:
            box = []
            for e in lst.elements:
                box.append(e)
            self.get_logger().info(str(box))
        self.bounding_boxes = box


        self.get_logger().info("Bounding Boxes processed")

class MainWindow(QWidget):
    node: GuiNode = None

    def __init__(self):
        super(MainWindow, self).__init__()

        self.VBL = QVBoxLayout()

        #self.imageLabel = QLabel()
        #self.VBL.addWidget(self.imageLabel)

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