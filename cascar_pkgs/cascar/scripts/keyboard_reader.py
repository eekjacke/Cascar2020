#!/usr/bin/env python3

import sys
import rospy
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from std_msgs.msg import String


class RosHandler(QThread):
    THROTTLE_STATE = None
    STEER_STATE = None
    f = 10
    
    def __init__(self):
        QThread.__init__(self)
        self.throttle_pub = rospy.Publisher('velocity_key', String, queue_size=10)
        self.steer_pub = rospy.Publisher('steer_key', String, queue_size=10)
        rospy.init_node('key_reader', anonymous=True)
        
    def run(self):
        rate = rospy.Rate(self.f)
        while not rospy.is_shutdown():
            if self.THROTTLE_STATE is not None:
                self.throttle_pub.publish(self.THROTTLE_STATE)
            if self.STEER_STATE is not None:
                self.steer_pub.publish(self.STEER_STATE)
            rate.sleep()

        
class CascarUI(QWidget):
    ACTIVE = "QPushButton { background-color: red }"
    INACTIVE = "QPushButton { background-color: blue }"

    def __init__(self):
        super().__init__()
        self.title = 'Cascar keyboard controller'
        self.left = 0
        self.top = 0
        self.width = 600
        self.height = 300
        self.setStyleSheet("background-color: dimgrey;")
        self.button_styles = ['SP_ArrowUp', 'SP_ArrowDown', 'SP_ArrowLeft', 'SP_ArrowRight']
        
        self.publisher = RosHandler()
        self.publisher.start()
        self.initUI()
    
    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        
        self.forward_btn = QPushButton(self)
        self.forward_btn.setIcon(self.style().standardIcon(getattr(QStyle, self.button_styles[0])))
        self.forward_btn.setGeometry(250, 100, 100, 50)
        self.forward_btn.setStyleSheet(self.INACTIVE)
        
        self.reverse_btn = QPushButton(self)
        self.reverse_btn.setIcon(self.style().standardIcon(getattr(QStyle, self.button_styles[1])))
        self.reverse_btn.setGeometry(250, 150, 100, 50)
        self.reverse_btn.setStyleSheet(self.INACTIVE)
        
        self.left_btn = QPushButton(self)
        self.left_btn.setIcon(self.style().standardIcon(getattr(QStyle, self.button_styles[2])))
        self.left_btn.setGeometry(150, 135, 100, 50)
        self.left_btn.setStyleSheet(self.INACTIVE)
        
        self.right_btn = QPushButton(self)
        self.right_btn.setIcon(self.style().standardIcon(getattr(QStyle, self.button_styles[3])))
        self.right_btn.setGeometry(350, 135, 100, 50)
        self.right_btn.setStyleSheet(self.INACTIVE)      
        
        self.show()
        
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_W:
            self.activate_throttle(self.forward_btn, 'forward')
        elif event.key() == Qt.Key_S:
            self.activate_throttle(self.reverse_btn, 'reverse')
        elif event.key() == Qt.Key_A:
            self.activate_steer(self.left_btn, 'left')
        elif event.key() == Qt.Key_D:
            self.activate_steer(self.right_btn, 'right')
            
    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_W:
            if self.publisher.THROTTLE_STATE != 'reverse':
                self.publisher.THROTTLE_STATE = None
            self.forward_btn.setStyleSheet(self.INACTIVE)
        elif event.key() == Qt.Key_S:
            self.reverse_btn.setStyleSheet(self.INACTIVE)
            if self.publisher.THROTTLE_STATE != 'forward':
                self.publisher.THROTTLE_STATE = None
        elif event.key() == Qt.Key_A:
            self.left_btn.setStyleSheet(self.INACTIVE)
            if self.publisher.STEER_STATE != 'right':
                self.publisher.STEER_STATE = None
        elif event.key() == Qt.Key_D:
            self.right_btn.setStyleSheet(self.INACTIVE)
            if self.publisher.STEER_STATE != 'left':
                self.publisher.STEER_STATE = None
            
    def change_stylesheet(self, btn, new_style):
        btn.setStyleSheet(new_style)

    @pyqtSlot()
    def activate_throttle(self, btn, new_state):
        self.publisher.THROTTLE_STATE = new_state
        self.change_stylesheet(btn, self.ACTIVE)
        
    def activate_steer(self, btn, new_state):
        self.publisher.STEER_STATE = new_state
        self.change_stylesheet(btn, self.ACTIVE)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = CascarUI()
    sys.exit(app.exec_())
