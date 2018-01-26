import pypot.dynamixel
import time
import itertools
import numpy as np
import xml.etree.ElementTree as ET
from math import pi,atan,sin,cos,degrees
#import rospy
#from std_msgs.msg import String

#ang=(91.38, 87.34, 6.81, -47.16, 79.87, -80.31, -94.9, 124.18, -0.31, -2.68, 11.47, -12.7, -15.78, 14.55, -8.48, 3.91, -0.13, -4.26, 46.99)
darwin = {1: -11, 2: -4, 3: -30.5, 4: 37.05, 5: 28, 6: -15,7: 11, 8: -2.95, 9: 13, 10: 5.05,11: 17.71, 12: -16.31, 13: -11, 14: 12.7, 15: 2.51, 16: 0.22 , 17: 10.42, 18: 2.51, 19: 161 }
abmath = {11: 17.71, 12: -16.31, 13: -11, 14: 12.7, 15: 2.51, 16: 0.22}
hand = {5: 28, 6: -15}

-10.86, 4.35, -30.46, 37.05, 28.79, -15.6, 10.07, -2.95, 13.23, 5.05, 17.71, -16.31, -11.21, 12.7, 2.51, 0.22, 10.42, 2.68, 161.63

lock = 8

class Dynamixel(object) :
	def __init__(self) :
		ports=pypot.dynamixel.get_available_ports()
		if not ports :
			raise IOError("No ports found")

		print "connecting to",ports[0]

		self.dxl=pypot.dynamixel.DxlIO(ports[0])
		self.ids=self.dxl.scan(range(20))
		print self.ids
		self.dxl.enable_torque(self.ids)
		if len(self.ids)<lock :
			raise RuntimeError("Not all the motors were detected")
		self.dxl.set_moving_speed(dict(zip(self.ids,itertools.repeat(100))))


	def setSpeed(self,speed,ids) :
		self.dxl.set_moving_speed(dict(zip(ids,itertools.repeat(speed))))

	def setPos(self,pose) :
		pos={ids:angle for ids,angle in pose.items()}
		self.dxl.set_goal_position(pos)
		print pos

	def listWrite(self,list) :
		pos=dict(zip(self.ids,list))
		self.dxl.set_goal_position(pos)

	def writePos(self,dicti) :
		
		self.dxl.set_goal_position(dicti)

	def writeAng(self,ids,pose) :
		self.dxl.set_goal_position({ids:pos})
		
	def returnPos(self,ids) :

		return self.dxl.get_present_position((ids,))	


x=Dynamixel()

class XML(object) :
	def __init__(self,file) :
		try :
			tree=ET.parse(file)
			self.root=tree.getroot()
		except :
			raise RuntimeError("File not found")

	def parse(self,motion) :
		find="PageRoot/Page[@name='" +motion+ "']/steps/step"
		steps=[x for x in self.root.findall(find)]
		p_frame=str()
		p_pose=str()
		for step in steps :
			Walk(step.attrib['frame'],step.attrib['pose'],p_frame,p_pose)
			p_frame=step.attrib['frame']
			p_pose=step.attrib['pose']


class Walk(object) :
	def __init__(self,frame,pose,p_frame,p_pose) :
		self.frame=int(frame)
		self.begin={}
		self.end={}
		if not(p_pose) :
			self.frame_diff=10
			p_pose=pose
		else :
			self.frame_diff=self.frame-int(p_frame) 

		for ids,pos in enumerate(map(float,p_pose.split())) :
			self.end[ids+1]=pos	

		for ids,pos in enumerate(map(float,pose.split())) :
			self.begin[ids+1]=pos
		
			
		self.motion()
		#self.set(offsets=[darwin])

	def Offset(self,offset) :
		
		for key in offset.keys() :
			if offset[key] == 'i' :
				self.begin[key] = -self.begin[key]
				self.end[key] = -self.end[key]
			else :
				self.begin[key] += offset[key]
				self.end[key] += offset[key]
		
		

	def set(self,offsets=[]) :
		for offset in offsets :
			self.Offset(offset)
		self.motion() 

	def motion(self) :
		write=[]
		ids=[]
		f_d=abs(self.frame_diff/10)
		for key in self.end.keys() :
			#pose_diff=abs(self.end[key]-self.begin[key])
			linp=np.linspace(self.end[key],self.begin[key],f_d)
			write.append(linp)
			#write.append(self.begin[key])
			ids.append(key)	
		print "out"
		for pose in zip(*write) :
			print "in"
			x.setPos(dict(zip(ids,pose)))
			time.sleep(0.08)
			
path = "chhotu.xml"	
xml = XML(path)



def move() :
	walk_init = xml.parse("1 Init")
	walk_ready = xml.parse("2 Ready")
	
	while True:
		fw_wk = xml.parse("3 Forward walk")

if __name__ == "__main__" :
	move()


