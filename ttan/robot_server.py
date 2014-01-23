#!/usr/bin/env python
import socket
import time, math
import SocketServer, sys 
import threading, select
import struct

MULT_jointstate = 100000.0
MSG_OUT = 1
MSG_QUIT = 2
MSG_JOINT_STATES = 3
MSG_WAYPOINT_FINISHED = 4
MSG_SPEEDL = 7
MSG_MOVEL = 8
MSG_STOPL = 9

HOST1 = '192.38.66.249'    # UR5
PORT1 = 30002 # The same port as used by the server


HOST11 = '10.59.8.118'    # local
PORT11 = 1337 # The same port as used by the server
PORTret = 50001

IP_addr = '10.59.8.118' # IP of host - this machine

connected_robot = None
connected_robot_lock = threading.Lock()
connected_robot_cond = threading.Condition(connected_robot_lock)

def setConnectedRobot(r):
    global connected_robot, connected_robot_lock
    with connected_robot_lock:
        connected_robot = r
        connected_robot_cond.notify()

def getConnectedRobot(wait=False, timeout=-1):
    started = time.time()
    with connected_robot_lock:
        if wait:
            while not connected_robot:
                if timeout >= 0 and time.time() > started + timeout:
                    break
                connected_robot_cond.wait(0.2)
        return connected_robot
                
class rotation(object):
	""" http://en.wikipedia.org/wiki/Axis-angle_representation """
	def __init__(self, axis_angle=None, rpy=None, quat=None, matrix=None):
		if axis_angle != None:
			if len(axis_angle) == 3:
				self.angle = math.sqrt(axis_angle[0]**2 + axis_angle[1]**2 + axis_angle[2]**2)
				if self.angle != 0:
					v = 1.0/self.angle
				else:
					v = 1.0
				self.axis = [axis_angle[0]*v, axis_angle[1]*v, axis_angle[2]*v]
				self.x = self.axis[0]
				self.y = self.axis[1]
				self.z = self.axis[2]
			else:
				self.x = axis_angle[0]
				self.y = axis_angle[1]
				self.z = axis_angle[2]
				self.angle = axis_angle[3]
				self.axis = [self.x, self.y, self.z]
			self.quat = self.toQuat()
			self.rpy = self.toRPY()
			self.matrix = self.toMatrix()
		elif rpy != None:
			self.rpy = rpy
			self.roll = rpy[0]
			self.pitch = rpy[1]
			self.yaw = rpy[2]
			self.axis = self.rpy2axis_angle()
			self.quat = self.toQuat()
			self.matrix = self.toMatrix()
		elif quat != None:
			self.quat = quat
			self.qx = quat[0]
			self.qy = quat[1]
			self.qz = quat[2]
			self.qw = quat[3]
			self.axis = self.quat2axis_angle()
			self.rpy = self.toRPY()
			self.matrix = self.toMatrix()
		elif matrix != None:
			self.matrix = matrix
			self.quat = self.matrix2quat()
			self.axis = self.quat2axis_angle()
			self.rpy = self.toRPY()
		else:
			print "Invalid initialization"
		
	def __repr__(self):
		return self.__str__
      
	def __str__(self):
		return "[x: " + str(self.x) + \
				" y: " + str(self.y) + \
				" z: " + str(self.z) + \
				"] angle: " + str(self.angle) + \
				"\nQuaternion: " + str(self.quat) + \
				"\nRPY: " + str(self.rpy)
    
	def rpy2axis_angle(self):
		c1 = math.cos(self.pitch/2)
		s1 = math.sin(self.pitch/2)
		c2 = math.cos(self.yaw/2)
		s2 = math.sin(self.yaw/2)
		c3 = math.cos(self.roll/2)
		s3 = math.sin(self.roll/2)
		c1c2 = c1*c2
		s1s2 = s1*s2
		w =c1c2*c3 - s1s2*s3
		x =c1c2*s3 + s1s2*c3
		y =s1*c2*c3 + c1*s2*s3
		z =c1*s2*c3 - s1*c2*s3
		self.angle = 2 * math.acos(w)
		norm = x*x+y*y+z*z
		if norm < 0.0001: # when all euler angles are zero angle =0 so we can set axis to anything to avoid divide by zero
			self.x=1
			self.y=0
			self.z=0
		else:
			norm =math.sqrt(norm)
			self.x = x/norm
			self.y = y/norm
			self.z = z/norm
		return [self.x, self.y, self.z]


	def quat2axis_angle(self):
		self.angle = 2 * math.acos(self.qw)
		s = math.sqrt(1-self.qw**2)
		if s < 0.0001: #test to avoide divide-y-zero, if small s, direction is not important
			self.x = self.qx
			self.y = self.qy
			self.z = self.qz
		else:
			self.x = self.qx/s
			self.y = self.qy/s
			self.z = self.qz/s
		return [self.x, self.y, self.z]
		
	def matrix2quat(self):
		tr = self.matrix[0][0] + self.matrix[1][1] + self.matrix[2][2]
		if tr > 0:
			S = math.sqrt(tr + 1) * 2
			self.qw = 0.25 * S
			self.qx = (self.matrix[2][1] - self.matrix[1][2]) / S
			self.qy = (self.matrix[0][2] - self.matrix[2][0]) / S
			self.qz = (self.matrix[1][0] - self.matrix[0][1]) / S
		elif self.matrix[0][0] > self.matrix[1][1] and self.matrix[0][0] > self.matrix[2][2]:
			S = math.sqrt(1 + self.matrix[0][0] - self.matrix[1][1] - self.matrix[2][2]) * 2
			self.qw = (self.matrix[2][1] - self.matrix[1][2]) / S
			self.qx = 0.25 * S
			self.qy = (self.matrix[1][0] + self.matrix[0][1]) / S
			self.qz = (self.matrix[0][2] + self.matrix[2][0]) / S
		elif self.matrix[1][1] > self.matrix[2][2]:
			S = math.sqrt(1 + self.matrix[1][1] - self.matrix[0][0] - self.matrix[2][2]) * 2
			self.qw = (self.matrix[0][2] - self.matrix[2][0]) / S
			self.qx = (self.matrix[1][0] + self.matrix[0][1]) / S
			self.qy = 0.25 * S
			self.qz = (self.matrix[2][1] + self.matrix[1][2]) / S
		else:
			S = math.sqrt(1 + self.matrix[2][2] - self.matrix[0][0] - self.matrix[1][1]) * 2
			self.qw = (self.matrix[1][0] - self.matrix[0][1]) / S
			self.qx = (self.matrix[0][2] + self.matrix[2][0]) / S
			self.qy = (self.matrix[2][1] + self.matrix[1][2]) / S
			self.qz = 0.25 * S
		return [self.qx, self.qy, self.qz, self.qw]
		
		
	def matrix2axis_angle(self):
		m = self.matrix
		epsilon = 0.0001 # margin to allow for rounding errors
		epsilon2 = 0.001 # margin to distinguish between 0 and 180 degrees
		if math.fabs(m[0][1]-m[1][0])< epsilon and math.fabs(m[0][2]-m[2][0])< epsilon and math.fabs(m[1][2]-m[2][1])< epsilon:
			#singularity found
			if math.fabs(m[0][1]+m[1][0]) < epsilon2 and math.fabs(m[0][2]+m[2][0]) < epsilon2 and math.fabs(m[1][2]+m[2][1]) < epsilon2 and math.fabs(m[0][0]+m[1][1]+m[2][2]-3) < epsilon2:
			# this singularity is identity matrix so angle = 0
				self.angle = 0.0
				self.x = 0.0
				self.y = 0.0
				self.z = 0.0

			else:
				#otherwise this singularity is angle = 180
				self.angle = math.pi
				xx = (m[0][0]+1)/2
				yy = (m[1][1]+1)/2
				zz = (m[2][2]+1)/2
				xy = (m[0][1]+m[1][0])/4
				xz = (m[0][2]+m[2][0])/4
				yz = (m[1][2]+m[2][1])/4
				if xx > yy and xx > zz: # m[0][0] is the largest diagonal term
					if xx< epsilon:
						self.x = 0
						self.y = math.sqrt(0.5)
						self.z = math.sqrt(0.5)
					else:
						self.x = math.sqrt(xx)
						self.y = xy/self.x
						self.z = xz/self.x
				elif yy > zz: # m[1][1] is the largest diagonal term
					if yy< epsilon:
						self.x = math.sqrt(0.5)
						self.y = 0
						self.z = math.sqrt(0.5)
					else:
						self.y = math.sqrt(yy)
						self.x = xy/self.y
						self.z = yz/self.y
				else: # m[2][2] is the largest diagonal term so base result on this
					if zz< epsilon:
						self.x = math.sqrt(0.5)
						self.y = math.sqrt(0.5)
						self.z = 0
					else:
						self.z = math.sqrt(zz)
						self.x = xz/self.z
						self.y = yz/self.z
		else:
			# as we have reached here there are no singularities so we can handle normally
			s = math.sqrt((m[2][1] - m[1][2])*(m[2][1] - m[1][2]) +(m[0][2] - m[2][0])*(m[0][2] - m[2][0]) +(m[1][0] - m[0][1])*(m[1][0] - m[0][1])) # used to normalise
			if math.fabs(s) < 0.001:
				s=1
				# prevent divide by zero, should not happen if matrix is orthogonal and should be
				# caught by singularity test above, but I've left it in just in case
			self.angle = math.acos(( m[0][0] + m[1][1] + m[2][2] - 1)/2)
			self.x = (m[2][1] - m[1][2])/s
			self.y = (m[0][2] - m[2][0])/s
			self.z = (m[1][0] - m[0][1])/s
		return [self.x, self.y, self.z]	

        
	def toMatrix(self):
		"""http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/index.htm"""
		c = math.cos(self.angle)
		s = math.sin(self.angle)
		t = 1.0 - c

		m00 = c + self.x*self.x*t
		m11 = c + self.y*self.y*t
		m22 = c + self.z*self.z*t

		tmp1 = self.x*self.y*t
		tmp2 = self.z*s
		m10 = tmp1+tmp2
		m01 = tmp1-tmp2
		tmp1 = self.x*self.z*t
		tmp2 = self.y*s
		m20 = tmp1 - tmp2
		m02 = tmp1 + tmp2
		tmp1 = self.y*self.z*t
		tmp2 = self.x*s
		m21 = tmp1 + tmp2
		m12 = tmp1 - tmp2

		return [[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]]

	def toQuat(self):
		self.qx = self.x * math.sin(self.angle/2)
		self.qy = self.y * math.sin(self.angle/2)
		self.qz = self.z * math.sin(self.angle/2)
		self.qw = math.cos(self.angle/2)
		return [self.qx, self.qy, self.qz, self.qw]

	def toRPY(self):
		s = math.sin(self.angle)
		c = math.cos(self.angle)
		t = 1-c

		if self.x*self.y*t + self.z*s > 0.998: #north pole singularity detected
			self.pitch = 2*math.atan2(self.x * math.sin(self.angle/2), math.cos(self.angle/2))
			self.yaw = math.pi/2
			self.roll = 0
		  
		elif self.x*self.y*t + self.z*s < -0.998: #south pole singularity detected
			self.pitch = -2*math.atan2(self.x * math.sin(self.angle/2), math.cos(self.angle/2))
			self.yaw = -math.pi/2
			self.roll = 0
		else:
			self.pitch = math.atan2(self.y * s - self.x*self.z*t, 1 - (self.y**2+self.z**2) * t)
			self.yaw = math.asin(self.x*self.y*t + self.z*s)
			self.roll = math.atan2(self.x * s - self.y * self.z * t , 1 - (self.x**2 + self.z**2) * t)
		return [self.roll, self.pitch, self.yaw]

def matrix_mul(A, B):
	rows_A = len(A)
	cols_A = len(A[0])
	cols_B = len(B[0])
	# Create the result matrix
	# Dimensions would be rows_A x cols_B
	C = [[0 for row in range(cols_B)] for col in range(rows_A)]
	#print C
	for i in range(rows_A):
		for j in range(cols_B):
			for k in range(cols_A):
				C[i][j] += A[i][k]*B[k][j]
	return C

GRIPPER_LENGTH = 0.2165 #from end of robot to tpc
#gripper_rot = rotation(quat=[0.9067021790861796, -0.4217714528513915, -7.57036110815195e-10, 1.627436581311051e-09])
gripper_rot = rotation(rpy=[math.pi, 0, -0.870796332179587])
gripper_trans = gripper_rot.matrix
gripper_trans[0].append(0.006442176844920656)
gripper_trans[1].append(-0.0076484218959709666)
gripper_trans[2].append(GRIPPER_LENGTH)
gripper_trans.append([0.0, 0.0, 0.0, 1.0])

gripper_invrot = rotation(quat=[0.9067021790861796, -0.4217714528513915, -7.57036110815195e-10, -1.627436581311051e-09])
gripper_invtrans = gripper_invrot.matrix
gripper_invtrans[0].append(-0.01)
gripper_invtrans[1].append(0.0)
gripper_invtrans[2].append(GRIPPER_LENGTH)
gripper_invtrans.append([0.0, 0.0, 0.0, 1.0])

def arm2gripper_transform(pose, rot):
	global gripper_trans
	#do fast deepcopy
	robot_trans = []
	for i in range(len(rot.matrix)):
		robot_trans.append(0)
		robot_trans[i] = [j for j in rot.matrix[i]]
	robot_trans[0].append(pose[0])
	robot_trans[1].append(pose[1])
	robot_trans[2].append(pose[2])
	robot_trans.append([0.0, 0.0, 0.0, 1.0])
	end_t = matrix_mul(robot_trans, gripper_trans)
	end_t.pop() #removes [0.0, 0.0, 0.0, 1.0]
	x = end_t[0].pop()
	y = end_t[1].pop()
	z = end_t[2].pop()
	rot_mat = rotation(matrix=end_t)
	return [x, y, z], rot_mat
	
def gripper2arm_transform(pose, rot):
	global gripper_invtrans
	#do fast deepcopy
	robot_trans = []
	for i in range(len(rot.matrix)):
		robot_trans.append(0)
		robot_trans[i] = [j for j in rot.matrix[i]]
	robot_trans[0].append(pose[0])
	robot_trans[1].append(pose[1])
	robot_trans[2].append(pose[2])
	robot_trans.append([0.0, 0.0, 0.0, 1.0])
	end_t = matrix_mul(robot_trans,gripper_invtrans)
	end_t.pop() #removes [0.0, 0.0, 0.0, 1.0]
	x = end_t[0].pop()
	y = end_t[1].pop()
	z = end_t[2].pop()
	rot_mat = rotation(matrix=end_t)
	return [x, y, z], rot_mat
	
def illegalPose(pose_arm, rot_arm, joint_pos):
	pose_gripper, rot_gripper = arm2gripper_transform(pose_arm, rot_arm)
	if pose_arm[1] > 0.18:
		return True
	elif pose_arm[2] < 0.39:
		return True
	elif joint_pos[1] > -42.0*math.pi/180.0 or joint_pos[1] < (-180.0+42.0)*math.pi/180.0:
		return True
	return False
        

class UR5TCPHandler(SocketServer.BaseRequestHandler):

	def recv_more(self):
		while True:
			r, _, _ = select.select([self.request], [], [], 0.2)
			if r:
				more = self.request.recv(4096)
				if not more:
				    raise socket.error("Stopped hearing from robot.  Disconnected")
				return more
			else:
				if self.position and \
				        self.time < int(round((time.time()+1) * 1000000)):
				    print "Stopped hearing from robot.  Disconnected"
				    raise socket.error("Stopped hearing from robot.  Disconnected")


	def handle(self):
		self.time = 0.0
		self.position = None
		self.velocity = None
		self.effort = None
		self.pose_arm = None
		self.rot_arm = None
		self.pose_tpc = None
		self.rot_tpc = None
		self.moving = False
		self.stopped = False
		self.socket_lock = threading.Lock()
		setConnectedRobot(self)
		print "Got connection from UR5"
		try:
			buf = self.recv_more()
			# self.request is the client connection
			while True:
			
				mtype = struct.unpack_from("!i", buf, 0)[0]
				buf = buf[4:]
				#print "Message type:", mtype

				if mtype == MSG_OUT:
					# Unpacks string message, terminated by tilde
					i = buf.find("~")
					while i < 0:
						buf = buf + self.recv_more()
						i = buf.find("~")
						if len(buf) > 2000:
							raise Exception("Probably forgot to terminate a string: %s..." % buf[:150])
					s, buf = buf[:i], buf[i+1:]
					print "Out: ", s

				elif mtype == MSG_QUIT:
					print "Quitting"
				#	raise EOF("Received quit")

				elif mtype == MSG_JOINT_STATES:
					while len(buf) < 4*(6*4): #four bytes each, six values pr type, four types (pos, vel, tau, fwd_kin)
						buf = buf + self.recv_more()
					state_mult = struct.unpack_from("!%ii" % (4*6), buf, 0)
					buf = buf[4*6*4:]
					state = [s / MULT_jointstate for s in state_mult]
					self.time = int(round(time.time() * 1000000))
					self.position = state[:6]
					self.velocity = state[6:12]
					self.effort = state[12:18]
					self.pose_arm = state[18:21]
					self.rot_arm = rotation(axis_angle = state[21:24])
					self.pose_tpc, self.rot_tpc = arm2gripper_transform(self.pose_arm, self.rot_arm)
					self.moving = False
					for v in self.velocity:
						if math.fabs(v) > 0.01:
							self.moving = True
							break
					if self.moving: 
						if illegalPose(self.pose_arm, self.rot_arm, self.position): #not good, as robot is stuck here. Should move to safe position
							if not self.stopped:
								self.send_msg(struct.pack("!i", MSG_STOPL))
								self.stopped = True
								print "Robot moved into illegal pose!"
						else:
							self.stopped = False
					
					
			
				elif mtype == MSG_WAYPOINT_FINISHED:
					while len(buf) < 4:
						buf = buf + self.recv_more()
					waypoint_id = struct.unpack_from("!i", buf, 0)[0]
					buf = buf[4:]
					print "Waypoint finished (not handled)"
				
				else:
					print "Got rubbish from UR5: ", mtype
				if not buf:
					buf = buf + self.recv_more()
		except socket.error:
			print "Connection closed"
			setConnectedRobot(None)

	def send_msg(self, msg):
		with self.socket_lock:
			self.request.send(msg)
		

class InterfaceTCPHandler(SocketServer.BaseRequestHandler):
	"One instance per connection.  Override handle(self) to customize action."
	def handle(self):
		print "Got connection on command interface"
		self.rot_mode = "rpy"
		self.pose_mode = "tool"
		while True:
			robot = getConnectedRobot(wait=True, timeout=20.0)
			while robot:
				buf = self.request.recv(1024).strip().lower()  # clip input at 1Kb
				if buf == "quit" or buf == "q" or buf == "exit":
					command = 'q'
					break
				robot = getConnectedRobot(wait=False)
				command = buf.split('(')[0]

				if buf == str(MSG_JOINT_STATES) or buf == "status":
					self.request.send("Time (ns)  : " + str(robot.time))
					self.request.send("\nPosition   : " + str(robot.position))
					self.request.send("\nVelocity   : " + str(robot.velocity))
					self.request.send("\nEffort     : " + str(robot.effort))
					if self.pose_mode == "arm":
						self.request.send("\nArm pose   : " + str(robot.pose_arm))
						if self.rot_mode == "rpy":
							self.request.send("\nRPY        : " + str(robot.rot_arm.rpy))
						elif self.rot_mode == "quat":
							self.request.send("\nQuaternion : " + str(robot.rot_arm.quat))
						elif self.rot_mode == "axis":
							self.request.send("\nAxis angle : " + str(robot.rot_arm.axis) + str(robot.rot_arm.angle))
						else:
							self.request.send("\RO3 matrix  : " + str(robot.rot_arm.matrix[0]))
							self.request.send("\n           : " + str(robot.rot_arm.matrix[1]))
							self.request.send("\n           : " + str(robot.rot_arm.matrix[2]))

					else:
						self.request.send("\nTool pose   : " + str(robot.pose_tpc))
						if self.rot_mode == "rpy":
							self.request.send("\nRPY        : " + str(robot.rot_tpc.rpy))
						elif self.rot_mode == "quat":
							self.request.send("\nQuaternion : " + str(robot.rot_tpc.quat))
						elif self.rot_mode == "axis":
							self.request.send("\nAxis angle : " + str(robot.rot_tpc.axis) + str(robot.rot_tpc.angle))
						else:
							self.request.send("\RO3 matrix  : " + str(robot.rot_tpc.matrix[0]))
							self.request.send("\n           : " + str(robot.rot_tpc.matrix[1]))
							self.request.send("\n           : " + str(robot.rot_tpc.matrix[2]))
					self.request.send("\n")
				elif buf[:7] == "setmode":
					pose = buf.find("pose")
					rot = buf.find("rot")
					if pose > 0:
						pose = buf[pose + buf[pose:].find("=") +1:].strip()
						if pose.find(" ") > 0:
							pose = pose[:pose.find(" ")]
						if pose == "arm" or pose=="tool":
							self.pose_mode = pose
							self.request.send("Setting pose reference to " + pose + "\n")
						else:
							self.request.send("Error: Valid pose references are 'arm' and 'tool'. I got " + pose + "\n")
					if rot > 0:
						rot = buf[rot + buf[rot:].find("=") +1:].strip()
						if rot.find(" ") > 0:
							rot = rot[:rot.find(" ")]
						if rot == "rpy" or rot=="quat" or rot=="matrix" or rot=="axis":
							self.rot_mode = rot
							self.request.send("Setting rotation mode to " + rot + "\n")
						else:
							self.request.send("Error: Valid pose references are 'rpy', 'quat', 'matrix' and 'axis'. I got " + pose + "\n")
						
					
				elif command == "movel":
					pose = list()
					a = 0.5
					v = 0.3
					t = 0.0
					r = 0.0
					wp = 999
					try:
						if buf.find('(') == -1 or buf.find('[') == -1 or buf.find(',') == -1 or buf.find(']') == -1 or buf.find(')') == -1:
							self.request.send("Poorly formatted MOVEL command (Could not find expected parenthesis, commas or brackets)\n")
							continue
						for ch in buf.split('[')[1].split(']')[0].split(','):
							pose.append(float(ch))
						if self.rot_mode == "rpy":
							if len(pose) != 6:
								self.request.send("Poorly formatted MOVEL command (Supply 6 values for the pose [x, y, z, r, p, y])\n")		
								continue		
							rot = rotation(rpy=pose[3:])
						elif self.rot_mode == "quat":
							if len(pose) != 7:
								self.request.send("Poorly formatted MOVEL command (Supply 7 values for the pose [x, y, z, i, j, k, w])\n")		
								continue		
							rot = rotation(quat=pose[3:])
						elif self.rot_mode == "axis":
							if len(pose) != 6 or len(pose) != 7:
								self.request.send("Poorly formatted MOVEL command (Supply 6 or 7 values for the pose [x, y, z, ax, ay, az (th)])\n")		
								continue		
							rot = rotation(axis_angle=pose[3:])
						else:
							if len(pose) != 15:
								self.request.send("Poorly formatted MOVEL command (Supply 15 values for the pose [x, y, z, m00, m01, m02, ...])\n")		
								continue									
							rot = rotation(matrix=pose[3:])
						if self.pose_mode=="tool":
							pose_arm, rot_arm = gripper2arm_transform(pose, rot)
							pose = pose_arm
							rot = rot_arm
						pose.append(rot.x * rot.angle)
						pose.append(rot.y * rot.angle)
						pose.append(rot.z * rot.angle)
						
						
						if illegalPose(pose, rot, [0, -1.57, 0, 0, 0, 0]):
							self.request.send("Movement to pose " + str(pose) + " is suspended for security reasons\n")		
							continue								
						params = buf.split(']')[1][:-1].split(',')
						if len(params) >= 2: #first is the (empty) string between ] and ,
							a = float(params[1])
						if len(params) >= 3:
							v = float(params[2])
						if len(params) >= 4:
							t = float(params[3])
						if len(params) >= 5:
							r = float(params[4])	
						if len(params) >= 6:
							wp = float(params[5])	
						print "got a movel command to waypoint " +str(wp)+ " at pose " + str(pose) + " with params: " + str(a) + ', ' + str(v) + ', ' + str(t) + ', ' + str(r)
						params = [MSG_MOVEL, wp] + \
							[MULT_jointstate * q for q in pose] + \
							[MULT_jointstate * a, MULT_jointstate * v, MULT_jointstate * t, MULT_jointstate * r]
						buf = struct.pack("!%ii" % len(params), *params)
						if buf.find("-v") < 0: #makes it possible for testing everything without sending the actual command
							robot.send_msg(buf)
					except socket.error:
						self.request.send("Poorly formatted MOVEL command (Unknown error)\n")		
						continue	
				
				elif command == "speedl":
					pose = list()
					a = 1.2
					t_min = 0.3
					wp = 999
					try:
						if buf.find('(') == -1 or buf.find(']') == -1 or buf.find(']') == -1 or buf.find(')') == -1:
							self.request.send("Poorly formatted SPEEDL command (Could not find expected parenthesis or brackets)\n")
							continue
						for ch in buf.split('[')[1].split(']')[0].split(','):
							pose.append(float(ch))
						if len(pose) != 6:
							self.request.send("Poorly formatted SPEEDL command (Supply 6 values for the pose speed)\n")		
							continue			
						params = buf.split(']')[1][:-1].split(',')
						if len(params) >= 2: #first is the (empty) string between ] and ,
							a = float(params[1])
						if len(params) >= 3:
							t_min = float(params[2])
						if len(params) >= 4:
							wp = float(params[3])
						print "got a SPEEDL command to waypoint " +str(wp)+ " at pose " + str(pose) + " with params: " + str(a) + ', ' + str(t_min)
						params = [MSG_SPEEDL, wp] + \
								[MULT_jointstate * q for q in pose] + \
								[MULT_jointstate * a, MULT_jointstate * t_min]
						buf = struct.pack("!%ii" % len(params), *params)
						robot.send_msg(buf)
					except:
						self.request.send("Poorly formatted SPEEDL command (Unknown error)\n")		
						continue
					
				elif command == "stopl":
					print "Got STOPL()"
					robot.send_msg(struct.pack("!i", MSG_STOPL))
								
				
				else:
					pass #self.request.send("Unknown command: " + buf + "\n")
			if command == "q":
				break

		print "Connection to command interface closed"

class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
	allow_reuse_address = True
	daemon_threads = True
	pass
	
def setup_server():
	"""
	Sends URScript to robot
	"""
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((HOST1, PORT1)) ## OBS: MODIFIED !! HOST 11(eleven)
	s.send('def driverProg():\n')
	s.send('\tMULT_jointstate = ' + str(MULT_jointstate) + '\n') # All data sent from the robot is multiplied with this constant as the data is sent as integers
	s.send('\tMSG_OUT = ' + str(MSG_OUT) + '\n')  
	s.send('\tMSG_QUIT = ' + str(MSG_QUIT) + '\n')  
	s.send('\tMSG_JOINT_STATES = ' + str(MSG_JOINT_STATES) + '\n')  
	s.send('\tMSG_WAYPOINT_FINISHED = ' + str(MSG_WAYPOINT_FINISHED) + '\n')  
	s.send('\tMSG_SPEEDL = ' + str(MSG_SPEEDL) + '\n')  
	s.send('\tMSG_STOPL = ' + str(MSG_STOPL) + '\n')  
	s.send('\tMSG_MOVEL = ' + str(MSG_MOVEL) + '\n')  

	s.send('\tdef send_out(msg):\n')
	s.send('\t\tenter_critical\n')
	s.send('\t\tsocket_send_int(MSG_OUT)\n')
	s.send('\t\tsocket_send_string(msg)\n')
	s.send('\t\tsocket_send_string("~")\n')
	s.send('\t\texit_critical\n')
	s.send('\tend\n') #rem \t

	s.send('\tdef send_waypoint_finished(waypoint_id):\n')
	s.send('\t\tenter_critical\n')
	s.send('\t\tsocket_send_int(MSG_WAYPOINT_FINISHED)\n')
	s.send('\t\tsocket_send_int(waypoint_id)\n')
	s.send('\t\texit_critical\n')
	s.send('\tend\n')

	s.send('\tthread statePublisherThread():\n') #This thread runs continuosly on the robot controller
	s.send('\t\tdef send_joint_state():\n')
	s.send('\t\t\tq = get_joint_positions()\n')
	s.send('\t\t\tqdot = get_joint_speeds()\n')
	s.send('\t\t\ttau = get_joint_torques()\n')
	s.send('\t\t\tfwd_kin = get_forward_kin()\n')
	s.send('\t\t\tenter_critical\n')
	s.send('\t\t\tsocket_send_int(MSG_JOINT_STATES)\n') # use 'struct.unpack_from("!i", buf, 0)[0]' and check that it is equal to this number
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * q[0]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * q[1]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * q[2]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * q[3]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * q[4]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * q[5]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * qdot[0]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * qdot[1]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * qdot[2]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * qdot[3]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * qdot[4]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * qdot[5]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * tau[0]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * tau[1]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * tau[2]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * tau[3]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * tau[4]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * tau[5]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * fwd_kin[0]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * fwd_kin[1]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * fwd_kin[2]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * fwd_kin[3]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * fwd_kin[4]))\n')
	s.send('\t\t\tsocket_send_int(floor(MULT_jointstate * fwd_kin[5]))\n')
	s.send('\t\t\texit_critical\n')
	s.send('\t\tend\n')
	s.send('\t\tsend_joint_state()\n')
	s.send('\t\twhile True:\n')
	s.send('\t\t\tsend_joint_state()\n')
	s.send('\t\t\tsync()\n') #wait for next time slice
	s.send('\t\tend\n')
	s.send('\t\tsync()\n')
	s.send('\tend\n')

	s.send('\tsocket_open("' + str(IP_addr) + '", '+str(PORTret)+')') #open a socket back to this computer
	s.send('\tsend_out("Hello")\n')
	s.send('\tthread_state = run statePublisherThread()\n') #spins the thread
	
	s.send('\tQUEUE_NONE = 0\n')
	s.send('\tcmd_queue_state = QUEUE_NONE\n')
	s.send('\tcmd_queue_id = 0\n')
	s.send('\tQUEUE_RUNNING = 1\n')
	s.send('\tcmd_queue_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n')
	s.send('\tcmd_queue_params = [0.0, 0.0, 0.0, 0.0]\n')
	
	s.send('\tdef set_queue(id, q, params):\n')
	s.send('\t\tenter_critical\n')
	s.send('\t\tcmd_queue_state = QUEUE_RUNNING\n')
	s.send('\t\tcmd_queue_id = id\n')
	s.send('\t\tcmd_queue_q = q\n')
	s.send('\t\tcmd_queue_params = params\n')
	s.send('\t\texit_critical\n')
	s.send('\tend\n')
	s.send('\tthread QueueThread():\n')
	s.send('\t\tstate = QUEUE_NONE\n')
	s.send('\t\twhile True:\n')
	s.send('\t\t\t# Latches the new command\n')
	s.send('\t\t\tenter_critical\n')
	s.send('\t\t\tq = cmd_queue_q\n')
	s.send('\t\t\tid = cmd_queue_id\n')
	s.send('\t\t\tstate = cmd_queue_state\n')
	s.send('\t\t\tparams = cmd_queue_params\n')
	s.send('\t\t\tcmd_queue_state = QUEUE_NONE\n')
	s.send('\t\t\texit_critical\n')
	s.send('\t\t\tif state == QUEUE_RUNNING:\n')
	s.send('\t\t\t\tif id == MSG_SPEEDL:\n')
	s.send('\t\t\t\t\tsend_out("speedl started")\n')# Sends the command
	s.send('\t\t\t\t\tspeedl(q, params[0], params[1])\n')
	s.send('\t\t\t\t\tsend_waypoint_finished(waypoint_id)\n')
	s.send('\t\t\t\t\tsend_out("speedl finished")\n')
	s.send('\t\t\t\tend\n')
	s.send('\t\t\telse:\n')
	s.send('\t\t\t\t#send_out("Idle")\n')
	s.send('\t\t\t\tsync()\n')
	s.send('\t\t\tend\n')
	s.send('\t\tend\n')
	s.send('\tend\n')

	s.send('\tthread_brake = run QueueThread()\n') #spins the thread
	s.send('\twhile True:\n')
	s.send('\t\tll = socket_read_binary_integer(1)\n')
	s.send('\t\tif ll[0] == 0:\n') #socket_read_binary_integer(1)[0] is the number of bytes read
#	s.send('\t\t\tsend_out("Received nothing")\n')
	s.send('\t\t\tsync()\n')
	s.send('\t\telif ll[0] > 1:\n')
	s.send('\t\t\tsend_out("Received too many things")\n')
	s.send('\t\telse:\n')
	
	s.send('\t\t\tmtype = ll[1]\n')
	s.send('\t\t\tif mtype == MSG_QUIT:\n') # got quit
	s.send('\t\t\t\tsend_out("Received QUIT")\n')
	s.send('\t\t\t\tsocket_send_int(MSG_QUIT)\n')
	s.send('\t\t\t\tbreak\n')
	
	s.send('\t\t\telif mtype == MSG_MOVEL:\n') # got movel
	s.send('\t\t\t\tsend_out("Received movel")\n')
	s.send('\t\t\t\tparams_mult = socket_read_binary_integer(1+6+4)\n')
	s.send('\t\t\t\tif params_mult[0] == 0:\n')
	s.send('\t\t\t\t\tsend_out("Received no parameters for movel message")\n')
	s.send('\t\t\t\tend\n')

	s.send('\t\t\t\twaypoint_id = params_mult[1]\n')  # Unpacks the parameters
	s.send('\t\t\t\tq = p[params_mult[2] / MULT_jointstate,\n')
	s.send('\t\t\t\t\tparams_mult[3] / MULT_jointstate,\n')
	s.send('\t\t\t\t\tparams_mult[4] / MULT_jointstate,\n')
	s.send('\t\t\t\t\tparams_mult[5] / MULT_jointstate,\n')
	s.send('\t\t\t\t\tparams_mult[6] / MULT_jointstate,\n')
	s.send('\t\t\t\t\tparams_mult[7] / MULT_jointstate]\n')
	s.send('\t\t\t\ta = params_mult[8] / MULT_jointstate\n')
	s.send('\t\t\t\tv = params_mult[9] / MULT_jointstate\n')
	s.send('\t\t\t\tt = params_mult[10] / MULT_jointstate\n')
	s.send('\t\t\t\tr = params_mult[11] / MULT_jointstate\n')

	s.send('\t\t\t\tsend_out("movel started")\n')# Sends the command
	s.send('\t\t\t\tmovel(q, a, v, t, r)\n')
	s.send('\t\t\t\tsend_waypoint_finished(waypoint_id)\n')
	s.send('\t\t\t\tsend_out("movel finished")\n')


	s.send('\t\t\telif mtype == MSG_SPEEDL:\n') # got speedl
	s.send('\t\t\t\tsend_out("Received speedl")\n')
	s.send('\t\t\t\tparams_mult = socket_read_binary_integer(1+6+2)\n')
	s.send('\t\t\t\tif params_mult[0] == 0:\n')
	s.send('\t\t\t\t\tsend_out("Received no parameters for speed message")\n')
	s.send('\t\t\t\tend\n')

	s.send('\t\t\t\twaypoint_id = params_mult[1]\n')  # Unpacks the parameters
	s.send('\t\t\t\tq = [params_mult[2] / MULT_jointstate,\n')
	s.send('\t\t\t\t\tparams_mult[3] / MULT_jointstate,\n')
	s.send('\t\t\t\t\tparams_mult[4] / MULT_jointstate,\n')
	s.send('\t\t\t\t\tparams_mult[5] / MULT_jointstate,\n')
	s.send('\t\t\t\t\tparams_mult[6] / MULT_jointstate,\n')
	s.send('\t\t\t\t\tparams_mult[7] / MULT_jointstate]\n')
	s.send('\t\t\t\ta = params_mult[8] / MULT_jointstate\n')
	s.send('\t\t\t\tt_min = params_mult[9] / MULT_jointstate\n')
	s.send('\t\t\t\tset_queue(MSG_SPEEDL, q, [a, t_min, 0, 0])\n')

	s.send('\t\t\telif mtype == MSG_STOPL:\n') # got stopl
	s.send('\t\t\t\tsend_out("Received stopl")\n')
	s.send('\t\t\t\tstopl(0.2)\n')
	s.send('\t\t\t\tsend_out("stopl finished")\n')

	s.send('\t\t\telse:\n')
	s.send('\t\t\t\tsend_out("Unsupported command")\n')
	s.send('\t\t\tend\n')
	s.send('\t\tend\n')
	s.send('\t\tsync()')
	s.send('\tend\n') 	
	s.send('end\n')
	time.sleep(1)
	#s.send('driverProg()\n')
	#s.close()
	return s

if __name__ == "__main__":
	#URScript_server = setup_server()
	UR5server = ThreadedTCPServer((str(IP_addr), PORTret), UR5TCPHandler)
	UR5server_thread = threading.Thread(target=UR5server.serve_forever)
	UR5server_thread.daemon = True
	UR5server_thread.start()
	
	Interfaceserver = ThreadedTCPServer((str(IP_addr), 31001), InterfaceTCPHandler)
	Interfaceserver_thread = threading.Thread(target=Interfaceserver.serve_forever)
	Interfaceserver_thread.daemon = True
	Interfaceserver_thread.start()
	# terminate with Ctrl+c
	try:
		r = getConnectedRobot(wait=True, timeout=2.0)
		while True:
			if getConnectedRobot(wait=False):
				time.sleep(0.5)
			else:
				while True:
					print "Robot not connected"
					#URScript_server.close()
					#URScript_server = setup_server()
					r = getConnectedRobot(wait=True, timeout=3.0)
					if r:
						break
					
				print "Robot connected"
	except KeyboardInterrupt:
		print "now quitting"
		Interfaceserver.shutdown()
		#URScript_server.close()
		UR5server.shutdown()

