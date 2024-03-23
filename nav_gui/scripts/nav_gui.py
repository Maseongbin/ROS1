from __future__ import print_function


import math
import os,sys
import rospy
import time

print(sys.version)
import warnings
warnings.filterwarnings('ignore')


from std_msgs.msg       import String
from std_msgs.msg       import Int16
from std_msgs.msg       import Int8
from std_msgs.msg       import Bool
from std_msgs.msg       import Float32
from geometry_msgs.msg  import Pose2D
from move_waypoint.msg  import Target_waypoint_line

from PyQt5.QtCore       import *
from PyQt5.QtCore       import Qt
from PyQt5.QtWidgets    import *
from PyQt5              import uic
from pyqtgraph          import PlotWidget
from PyQt5.QtWidgets    import QWidget, QTabWidget, QAction
from PyQt5.QtGui        import *
from PyQt5.QtCore 	    import QEvent


import pyqtgraph       as pg

import matplotlib.pyplot as plt

import numpy as np
import math


form_class = uic.loadUiType('nav_gui.ui')[0]

def publish_custom_msg(x1,y1,theta1, x2,y2,theta2):
	
	waypoint_line = Target_waypoint_line()
	waypoint_line.waypoint_start_pose2d.x = x1
	waypoint_line.waypoint_start_pose2d.y = y1
	waypoint_line.waypoint_target_pose2d.x = x2
	waypoint_line.waypoint_target_pose2d.y = y2
	
	pub=rospy.Publisher('/wp/target_waypoint_line', Target_waypoint_line, queue_size=1)	
	pub.publish(waypoint_line)
	
	return


def publish_target_waypoint(x,y,theta):

	pose = Pose2D()
	pose.x = x
	pose.y = y
	pose.theta = theta
	pub=rospy.Publisher('/target_waypoint', Pose2D, queue_size=1)	
	pub.publish(pose)
	return	

def publish_waypoint_control(control_type):
		
	pub=rospy.Publisher('/flag/waypoint_control', Int8, queue_size=1)	
	pub.publish(control_type)
	return
	

	
class WindowClass(QDialog, form_class):
	
	
	def __init__(self):
		
		super(QDialog,self).__init__()
		
		rospy.init_node('target_wp_gui', anonymous=True)
		
		self.setupUi(self)
		self.setWindowTitle("WCS GUI")
		
		
		self.pushButton_run.clicked.connect(self.publish_run_topic)
		self.pushButton_stop.clicked.connect(self.publish_stop_topic)
		self.pushButton_reset.clicked.connect(self.publish_reset_topic)
		self.pushButton_custom_msg.clicked.connect(self.publish_custom_msg_topic)
	
	
	def publish_custom_msg_topic(self):
		
		x1         = float(self.lineEdit_pose_x_start.text())
		y1         = float(self.lineEdit_pose_y_start.text())
		heading1   = float(self.lineEdit_pose_heading_start.text())
		
		x2         = float(self.lineEdit_pose_x_target.text())
		y2         = float(self.lineEdit_pose_y_target.text())
		heading2   = float(self.lineEdit_pose_heading_target.text())
		
		publish_custom_msg(x1,y1,heading1,x2,y2,heading2)
		return
		
		
	def publish_run_topic(self):
		print("Run Published")
		self.label_amr_status_output.setText("Run")
		x1         = float(self.lineEdit_pose_x_start.text())
		y1         = float(self.lineEdit_pose_y_start.text())
		heading1   = float(self.lineEdit_pose_heading_start.text())
		
		x2         = float(self.lineEdit_pose_x_target.text())
		y2         = float(self.lineEdit_pose_y_target.text())
		heading2   = float(self.lineEdit_pose_heading_target.text())
		
		
		#list_text  = self.comboBox_operator.currentText()		
		#index      = self.comboBox_operator.currentIndex()
		publish_target_waypoint(x2,y2,heading2)
		
		publish_waypoint_control(1)		
			
		return
	
	def publish_stop_topic(self):				
		publish_waypoint_control(0)					
		return
		
		
	def publish_reset_topic(self):
		publish_waypoint_control(-1)					
		return
		
if __name__ == "__main__":
	app = QApplication(sys.argv)
	myWindow = WindowClass()
	myWindow.show()
	app.exec_()		
		
