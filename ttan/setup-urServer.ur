def driverProg():
	MULT_jointstate = 100000.0
	MSG_OUT = 1
	MSG_QUIT = 2
	MSG_JOINT_STATES = 3
	MSG_WAYPOINT_FINISHED = 4
	MSG_SPEEDL = 7
	MSG_STOPL = 9
	MSG_MOVEL = 8
	def send_out(msg):
		enter_critical
		socket_send_int(MSG_OUT)
		socket_send_string(msg)
		socket_send_string("~")
		exit_critical
	end
	def send_waypoint_finished(waypoint_id):
		enter_critical
		socket_send_int(MSG_WAYPOINT_FINISHED)
		socket_send_int(waypoint_id)
		exit_critical
	end
	thread statePublisherThread():
		def send_joint_state():
			q = get_joint_positions()
			qdot = get_joint_speeds()
			tau = get_joint_torques()
			fwd_kin = get_forward_kin()
			enter_critical
			socket_send_int(MSG_JOINT_STATES)
			socket_send_int(floor(MULT_jointstate * q[0]))
			socket_send_int(floor(MULT_jointstate * q[1]))
			socket_send_int(floor(MULT_jointstate * q[2]))
			socket_send_int(floor(MULT_jointstate * q[3]))
			socket_send_int(floor(MULT_jointstate * q[4]))
			socket_send_int(floor(MULT_jointstate * q[5]))
			socket_send_int(floor(MULT_jointstate * qdot[0]))
			socket_send_int(floor(MULT_jointstate * qdot[1]))
			socket_send_int(floor(MULT_jointstate * qdot[2]))
			socket_send_int(floor(MULT_jointstate * qdot[3]))
			socket_send_int(floor(MULT_jointstate * qdot[4]))
			socket_send_int(floor(MULT_jointstate * qdot[5]))
			socket_send_int(floor(MULT_jointstate * tau[0]))
			socket_send_int(floor(MULT_jointstate * tau[1]))
			socket_send_int(floor(MULT_jointstate * tau[2]))
			socket_send_int(floor(MULT_jointstate * tau[3]))
			socket_send_int(floor(MULT_jointstate * tau[4]))
			socket_send_int(floor(MULT_jointstate * tau[5]))
			socket_send_int(floor(MULT_jointstate * fwd_kin[0]))
			socket_send_int(floor(MULT_jointstate * fwd_kin[1]))
			socket_send_int(floor(MULT_jointstate * fwd_kin[2]))
			socket_send_int(floor(MULT_jointstate * fwd_kin[3]))
			socket_send_int(floor(MULT_jointstate * fwd_kin[4]))
			socket_send_int(floor(MULT_jointstate * fwd_kin[5]))
			exit_critical
		end
		send_joint_state()
		while True:
			send_joint_state()
			sync()
		end
		sync()
	end
	socket_open("10.59.8.118", 50001)	send_out("Hello")
	thread_state = run statePublisherThread()
	QUEUE_NONE = 0
	cmd_queue_state = QUEUE_NONE
	cmd_queue_id = 0
	QUEUE_RUNNING = 1
	cmd_queue_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	cmd_queue_params = [0.0, 0.0, 0.0, 0.0]
	def set_queue(id, q, params):
		enter_critical
		cmd_queue_state = QUEUE_RUNNING
		cmd_queue_id = id
		cmd_queue_q = q
		cmd_queue_params = params
		exit_critical
	end
	thread QueueThread():
		state = QUEUE_NONE
		while True:
			# Latches the new command
			enter_critical
			q = cmd_queue_q
			id = cmd_queue_id
			state = cmd_queue_state
			params = cmd_queue_params
			cmd_queue_state = QUEUE_NONE
			exit_critical
			if state == QUEUE_RUNNING:
				if id == MSG_SPEEDL:
					send_out("speedl started")
					speedl(q, params[0], params[1])
					send_waypoint_finished(waypoint_id)
					send_out("speedl finished")
				end
			else:
				#send_out("Idle")
				sync()
			end
		end
	end
	thread_brake = run QueueThread()
	while True:
		ll = socket_read_binary_integer(1)
		if ll[0] == 0:
			sync()
		elif ll[0] > 1:
			send_out("Received too many things")
		else:
			mtype = ll[1]
			if mtype == MSG_QUIT:
				send_out("Received QUIT")
				socket_send_int(MSG_QUIT)
				break
			elif mtype == MSG_MOVEL:
				send_out("Received movel")
				params_mult = socket_read_binary_integer(1+6+4)
				if params_mult[0] == 0:
					send_out("Received no parameters for movel message")
				end
				waypoint_id = params_mult[1]
				q = p[params_mult[2] / MULT_jointstate,
					params_mult[3] / MULT_jointstate,
					params_mult[4] / MULT_jointstate,
					params_mult[5] / MULT_jointstate,
					params_mult[6] / MULT_jointstate,
					params_mult[7] / MULT_jointstate]
				a = params_mult[8] / MULT_jointstate
				v = params_mult[9] / MULT_jointstate
				t = params_mult[10] / MULT_jointstate
				r = params_mult[11] / MULT_jointstate
				send_out("movel started")
				movel(q, a, v, t, r)
				send_waypoint_finished(waypoint_id)
				send_out("movel finished")
			elif mtype == MSG_SPEEDL:
				send_out("Received speedl")
				params_mult = socket_read_binary_integer(1+6+2)
				if params_mult[0] == 0:
					send_out("Received no parameters for speed message")
				end
				waypoint_id = params_mult[1]
				q = [params_mult[2] / MULT_jointstate,
					params_mult[3] / MULT_jointstate,
					params_mult[4] / MULT_jointstate,
					params_mult[5] / MULT_jointstate,
					params_mult[6] / MULT_jointstate,
					params_mult[7] / MULT_jointstate]
				a = params_mult[8] / MULT_jointstate
				t_min = params_mult[9] / MULT_jointstate
				set_queue(MSG_SPEEDL, q, [a, t_min, 0, 0])
			elif mtype == MSG_STOPL:
				send_out("Received stopl")
				stopl(0.2)
				send_out("stopl finished")
			else:
				send_out("Unsupported command")
			end
		end
		sync()	end
end
#driverProg()
