import rclpy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Vector3, Point
from sensor_msgs.msg import LaserScan
import time, serial, math
import struct, os, json
import numpy as np
from datetime import datetime
from functools import reduce
import logging

class Rosy(Node):

	_instance = None
	def __new__(cls, *args, **kwargs):
		if cls._instance is None:
			cls._instance = super(Rosy, cls).__new__(cls)
		return cls._instance
		
		
	def __init__(self):
		if not hasattr(self, "_initialized"):
			super().__init__('rosy')
			self.vel = self.create_publisher(Point, 'vel', 100)
			self.packetos = self.create_publisher(Point, 'pose', 100)
			self.gyro = self.create_publisher(Point, 'gyro', 100)
			self.battery = self.create_publisher(Float32, 'battery/voltage', 1000)
			self.cmd_mode = self.create_subscription(Int8, 'cmd_mode', self.mode_callback, 100)
			self.cmd_vel = self.create_subscription(Point, 'cmd_vel', self.cmd_vel_callback, 100)
			self.cmd_pose = self.create_subscription(Point, 'cmd_pose', self.cmd_pose_callback, 100)
			self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 100)
			#pentru SLAM
			self.odomBroadcaster = TransformBroadcaster(self)
			self.lidarBroadcaster = StaticTransformBroadcaster(self)
			timer_period = 0.01  # seconds
			self.timer = self.create_timer(timer_period, self.check_serial)
			#cmd_pose variables
			self.target_pos = None
			self.kp = 0.8
			self.cmd = ''
			self.swcalib = False
			self.nofwcalib = False
			self.stopped = False
			self.vbat = 0
			self.v = [0, 0, 0]
			self.packet = None # robot control packet bytes to be send on serial port
			self.sp = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0)
			self.rsbuf = b''
			self.gyro_data = [0, 0, 0] #giroscop
			self.spdodo = [0, 0, 0, 0] # from robot
			self.posodo = [0, 0, math.pi*500, 0] # integrated based on spdodo
			self.alligned = False
			self.mode_value = 0
			self.lidar_ranges = 0
			self._initialized = True
		

	def cmd_vel_callback(self, msg):
		self.ctrl(msg.x, msg.y, msg.z)
		self.send()

	def cmd_pose_callback(self, msg):
		self.alligned = False
		self.target_pos = [msg.x, msg.y, msg.z]

	def mode_callback(self, msg):
		self.mode_value = msg.data
		self.get_logger().info("Am primit modul: " + str(self.mode_value))

	def lidar_callback(self, msg):
		self.lidar_ranges = msg.ranges	

	def __list_to_transform_stamped(self, frameParent, frameChild, pos, angles):
		t = TransformStamped()
		t.header.frame_id = frameParent
		t.child_frame_id = frameChild
		t.transform.translation.x = float(pos[0])
		t.transform.translation.y = float(pos[1])
		t.transform.translation.z = float(pos[2])
		if len(angles) == 3:
			q = self.__quaternion_from_euler(
				float(angles[0]), float(angles[1]), float(angles[2]))
		elif len(angles) == 4:
			q = angles
		t.transform.rotation.x = q[0]
		t.transform.rotation.y = q[1]
		t.transform.rotation.z = q[2]
		t.transform.rotation.w = q[3]
		return t

	def __quaternion_from_euler(self, ai, aj, ak):
		ai /= 2.0
		aj /= 2.0
		ak /= 2.0
		ci = np.cos(ai)
		si = np.sin(ai)
		cj = np.cos(aj)
		sj = np.sin(aj)
		ck = np.cos(ak)
		sk = np.sin(ak)
		cc = ci*ck
		cs = ci*sk
		sc = si*ck
		ss = si*sk
		q = np.empty((4, ))
		q[0] = cj*sc - sj*cs
		q[1] = cj*ss + sj*cc
		q[2] = cj*cs - sj*sc
		q[3] = cj*cc + sj*ss
		return q

	CORRECTION_FACTOR_X = 1.02
	CORRECTION_FACTOR_Y = 1.13
	CORRECTION_FACTOR_Z = 1.0236
	CORRECTION_FACTOR_X_Z_P = 0.0072 # radian/m 
	CORRECTION_FACTOR_X_Z_N = 0.00009
	CORRECTION_FACTOR_Y_Z_P = 0.0389
	CORRECTION_FACTOR_Y_Z_N = 0.0277
	
	def __ctrl(self, vx, vy, vz=0):
		#    SOF        Vxh Vxl Vyh Vyl Vzh Vzl Chk EOF
		p = b'{\x01\x02\x03\x04\x05\x06\x07\x08\x09}'
		p = bytearray(p)
		p[1] = 1 if self.swcalib or self.nofwcalib else 0
		if self.swcalib:
			vz +=-vx*(Rosy.CORRECTION_FACTOR_X_Z_P if vx > 0 else Rosy.CORRECTION_FACTOR_X_Z_N) - vy*(Rosy.CORRECTION_FACTOR_Y_Z_P if vy > 0 else Rosy.CORRECTION_FACTOR_Y_Z_N)
			vz *= Rosy.CORRECTION_FACTOR_Z
			vx *=Rosy.CORRECTION_FACTOR_X
			vy *=Rosy.CORRECTION_FACTOR_Y
		self.v = [vx, vy, vz]
		vx = int(vx*1000)
		vy = int(vy*1000)
		vz = int(vz*1000)
		p[3:5] = vx.to_bytes(2, 'big', signed=True)
		p[5:7] = vy.to_bytes(2, 'big', signed=True)
		p[7:9] = vz.to_bytes(2, 'big', signed=True)
		p[9] = 0
		for x in p[:9]:
				p[9] ^= x
		self.packet = p

	def __send(self):
		if self.packet:
			stop = self.packet[3:9] == b'\x00\x00\x00\x00\x00\x00'
			if not stop or not self.stopped:
				self.sp.write(self.packet)
				self.stopped = stop
				
	PKTLEN=25
	def __check_serial(self):
		##   SoF Mod Vxh Vxl Vyh Vyl Vzh Vzl axh axl ayh ayl azh azl gxh gxl gyh gyl gzh gzl vbh vbl Chk EoF LF
		#p = b'{\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f\x10\x11\x12\x13\x14\x15\x16}\n'
        #Cat timp exista suficiente date 
		while Rosy.PKTLEN<= (self.sp.in_waiting + len(self.rsbuf)):
			#adaugare date in buffer
			self.rsbuf += self.sp.read()
			#elimina datele incorecte
			while len(self.rsbuf)>0 and self.rsbuf[0:1] != b'{':
				self.rsbuf = self.rsbuf[1:]
			#verifica daca sunt suficiente date in buffer pentru a forma un pachet
			if len(self.rsbuf) < Rosy.PKTLEN:
				continue
			#verifica daca finalul pachetului este corect
			if self.rsbuf[Rosy.PKTLEN-2:Rosy.PKTLEN] != b'}\n':
				self.rsbuf = self.rsbuf[1:]
				continue
			#verifica checksum-ul daca este corect
			if 0 != reduce(lambda a, b: a^b, self.rsbuf[0:Rosy.PKTLEN-2]):
				self.rsbuf = self.rsbuf[1:]
				continue

			#se extrage un pachet valid si se apoi se elimina din buffer
			d = self.rsbuf[0:Rosy.PKTLEN]
			self.rsbuf = self.rsbuf[Rosy.PKTLEN:]
			tm = round(time.time()*1000,0)

			self.spdodo = [ int.from_bytes(d[2:2+2], 'big', signed='True'),
											int.from_bytes(d[4:4+2], 'big', signed='True'),
											int.from_bytes(d[6:6+2], 'big', signed='True'), tm]

			self.gyro_data = [int.from_bytes(d[14:14+2], 'big', signed='True'),
											int.from_bytes(d[16:16+2], 'big', signed='True'),
											int.from_bytes(d[18:18+2], 'big', signed='True')]

			self.vbat = struct.unpack('>h', d[20:22])[0]
			self.vbat = (self.vbat // 10) / 100.0
			msg = Float32()
			msg.data = self.vbat
			self.battery.publish(msg)
										
			if self.swcalib:
				self.spdodo[0] /= Rosy.CORRECTION_FACTOR_X
				self.spdodo[1] /= Rosy.CORRECTION_FACTOR_Y
				self.spdodo[2] /= Rosy.CORRECTION_FACTOR_Z
				self.spdodo[2] += self.spdodo[0] * (Rosy.CORRECTION_FACTOR_X_Z_P if self.spdodo[0]>0 else Rosy.CORRECTION_FACTOR_X_Z_N) + self.spdodo[1] * (Rosy.CORRECTION_FACTOR_Y_Z_P if self.spdodo[1]>0 else Rosy.CORRECTION_FACTOR_Y_Z_N) 
			
			u = self.posodo[2]/1000.0
			dt = 1e-2
			s = math.sin(u)*dt
			c = math.cos(u)*dt
			self.posodo = [ self.posodo[0] + c * self.spdodo[0] - s * self.spdodo[1], 
											self.posodo[1] + s * self.spdodo[0] + c * self.spdodo[1],
											self.posodo[2] + self.spdodo[2]*dt, tm]

			msg = Point()
			msg.x, msg.y, msg.z = self.spdodo[0]/1000,self.spdodo[1]/1000,self.spdodo[2]/1000, 
			self.vel.publish(msg)
			
			msg.x, msg.y, msg.z = self.posodo[0]/1000,self.posodo[1]/1000,self.posodo[2]/1000, 
			self.packetos.publish(msg)

			#SLAM msgs
			self.odomMsg = self.__list_to_transform_stamped("odom_frame","base_frame",
														[msg.x, msg.y, 0.0],
														[0.0, 0.0, msg.z])
			self.lidarMsg = self.__list_to_transform_stamped("base_frame","laser",
														[0.0, 0.0, 0.0],
														[0.0, 0.0, 0.0])
				
			self.odomMsg.header.stamp = self.get_clock().now().to_msg()
			self.lidarMsg.header.stamp = self.get_clock().now().to_msg()
			self.odomBroadcaster.sendTransform(self.odomMsg)
			self.lidarBroadcaster.sendTransform(self.lidarMsg)

			msg.x, msg.y, msg.z = self.gyro_data[0]/1000, self.gyro_data[1]/1000, self.gyro_data[2]/1000,
			self.gyro.publish(msg)

			self.navigate()


	def __navigate(self):
		# Modurile de navigare catre punctul tinta    
		if self.target_pos is not None:
			#diferenta dintre pozitia curenta si destinatie
			dx = self.target_pos[0] - (self.posodo[0] / 1000)
			dy = self.target_pos[1] - (self.posodo[1] / 1000)
			#unghiul dintre axa x si punctul (dx, dy)
			alpha = math.atan2(dy, dx)
			theta = self.posodo[2] / 1000
			#diferenta dintre acest unghi si orientarea theta curenta a robotului
			#unghiul de rotatie necesara robotului pentru a se orienta catre punctul tinta
			dtheta = alpha - theta
			#dtheta este in intervalul (-pi, pi)
			if dtheta > math.pi:
				dtheta -= 2 * math.pi
			elif dtheta < -math.pi:
				dtheta += 2 * math.pi
			#daca robotul a ajuns
			distance = math.sqrt(dx**2 + dy**2)
			#rotirea initiala
			vx = vy = vz = 0

			if self.mode_value == 0:
					self.target_pos = None
					vx = vy = vz = 0
			
		
			#evitarea obstacolelor utilizand forte de atractie si respingere
			if min(self.lidar_ranges) < 0.55:
				obstacle_index = np.argmin(self.lidar_ranges)
				obstacle_distance = self.lidar_ranges[obstacle_index]
				#Forta de respingere bazata pe distanta fata de obstacol
				repulsion_force = 1 / obstacle_distance
				#Unghiul de respingere
				repulsion_angle = math.radians(obstacle_index)
				#Calculeaza forta de atractie catre punctul de destinatie
				attraction_force = 1 / distance
				attraction_angle = dtheta

				#forta totala este diferenta intre forta de atractie si forta de respingere
				total_force = attraction_force - repulsion_force

				#Caluculul vitezelor pe baza fortei totale si a unghiului de respingere
				vx = total_force * math.cos(repulsion_angle)
				vy = total_force * math.sin(repulsion_angle)
				#limitare viteze
				vx = vx * 0.6
				vy = vy * 0.7

			else:
				#self.kp = 0.8
				if min(self.lidar_ranges) < 1.3:
					self.kp = 0.5
				else:
					self.kp = 0.8


				#GTP Rotation-Translation
				if self.mode_value == 1:
					if not self.alligned:
						if dtheta > 0:
							vz = math.sqrt(1.6*dtheta) * self.kp
						else:
							vz = -1 * math.sqrt(1.6*abs(dtheta)) * self.kp
					if abs(dtheta) < 0.01:
						self.alligned = True
					
					if self.alligned:
						if distance > 0.05:
							#limitare viteza
							vx = math.sqrt(1.2*distance) * self.kp
							if abs(dtheta)<0.01:
								self.alligned = False
						#daca a ajuns la destinatie se opreste
						else:
							self.target_pos = None
						
				#GTP Translation-Rotation
				if self.mode_value == 2:
					if distance > 0.03:
						#viteza proportionala cu distanta
						v = math.sqrt(1.2*distance) * self.kp
						vx = v * math.cos(alpha-theta)
						vy = v * math.sin(alpha-theta)
					else:
						if abs(dtheta) > 0.01:
							#vz = math.sqrt(abs(dtheta))
							if dtheta > 0:
								vz = math.sqrt(1.6*dtheta) * self.kp
							else:
								vz = -1 * math.sqrt(1.6*abs(dtheta)) * self.kp

						else:	
							self.target_pos = None

				#GTP All
				if self.mode_value == 3:
					if distance > 0.03:
						#viteza proportionala cu distanta
						v = math.sqrt(1.2*distance) * self.kp
						vx = v * math.cos(alpha-theta)
						vy = v * math.sin(alpha-theta)
						if dtheta > 0:
							vz = math.sqrt(3*dtheta) * self.kp
						else:
							vz = -1 * math.sqrt(3*abs(dtheta)) * self.kp
					else:						
						self.target_pos = None
			
			self.ctrl(vx, vy, vz)
			self.send()
	
	def __close(self):
			self.ctrl(0, 0)
			self.sp.close()
	 

def main(args=None):
		rclpy.init(args=args)
		rosy = Rosy()
		rclpy.spin(rosy)
		rosy.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
		main()