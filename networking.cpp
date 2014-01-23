#include <iostream>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string>
#include <string.h>
#include <sstream>


// http://stackoverflow.com/questions/16486361/creating-a-basic-c-c-tcp-socket-writer

using namespace std;

//Target host details: (ROBOT )
#define PORT 30002
#define HOST "192.38.66.249"

//Target host details:
//#define PORT 1337
//#define HOST "127.0.0.1"

  
#define CALLBACKHOST "10.59.8.118"
#define CALLBACKPORT 50001
  
  
  
class networking{
  


  int sd,interface;
  
public:
  void messageSend(string message)
{
    
    

   
    send(sd, (char *)message.c_str(), strlen((char *)message.c_str()), 0);
    
}

void startNet(){
  
  struct sockaddr_in server;
  struct in_addr ipv4addr;
  struct hostent *hp;
  
    
     sd = socket(AF_INET,SOCK_STREAM,0);
    server.sin_family = AF_INET;

    inet_pton(AF_INET, HOST, &ipv4addr);
    hp = gethostbyaddr(&ipv4addr, sizeof ipv4addr, AF_INET);
   

    bcopy(hp->h_addr, &(server.sin_addr.s_addr), hp->h_length);
    server.sin_port = htons(PORT);

    connect(sd, (const sockaddr *)&server, sizeof(server));
  
  
}

void stopNet(){
  
  close(sd);
  
}


  void initRobotServer(){
    
const double MULT_jointstate = 100000.0;
const int MSG_OUT = 1;
const int MSG_QUIT = 2;
const int MSG_JOINT_STATES = 3;
const int MSG_WAYPOINT_FINISHED = 4;
const int MSG_SPEEDL = 7;
const int MSG_MOVEL = 8;
const int MSG_STOPL = 9;

    std::stringstream ipss,cmdss;
    ipss << "\tsocket_open(\"" << CALLBACKHOST << "\"," << CALLBACKPORT << ")";
    std::string ip = ipss.str();
    
    string cmd;
    
    messageSend("def driverProg():\n");
    
    cmdss.str(""); cmdss << "\tMULT_jointstate = " << MULT_jointstate << "\n"; cmd = cmdss.str();
    messageSend(cmd); // All data sent from the robot is multiplied with this constant as the data is sent as integers

    cmdss.str(""); cmdss << "\tMSG_OUT = " << MSG_OUT << "\n"; cmd = cmdss.str();
    messageSend(cmd);
 
    cmdss.str(""); cmdss << "\tMSG_QUIT = " << MSG_QUIT << "\n"; cmd = cmdss.str();
    messageSend(cmd);
   
    cmdss.str(""); cmdss << "\tMSG_JOINT_STATES = " << MSG_JOINT_STATES << "\n"; cmd = cmdss.str();
    messageSend(cmd);
    
    cmdss.str(""); cmdss << "\tMSG_WAYPOINT_FINISHED = " << MSG_WAYPOINT_FINISHED << "\n"; cmd = cmdss.str();
    messageSend(cmd);

    cmdss.str(""); cmdss << "\tMSG_SPEEDL = " << MSG_SPEEDL << "\n"; cmd = cmdss.str();
    messageSend(cmd);
  
    cmdss.str(""); cmdss << "\tMSG_STOPL = " << MSG_STOPL << "\n"; cmd = cmdss.str();
    messageSend(cmd);
 
   cmdss.str(""); cmdss << "\tMSG_MOVEL = " << MSG_MOVEL << "\n"; cmd = cmdss.str();
    messageSend(cmd);  

	messageSend("\tdef send_out(msg):\n");
	messageSend("\t\tenter_critical\n");
	messageSend("\t\tsocket_send_int(MSG_OUT)\n");
	messageSend("\t\tsocket_send_string(msg)\n");
	messageSend("\t\tsocket_send_string(\"~\")\n");
	messageSend("\t\texit_critical\n");
	messageSend("\tend\n"); //rem \t

	messageSend("\tdef send_waypoint_finished(waypoint_id):\n");
	messageSend("\t\tenter_critical\n");
	messageSend("\t\tsocket_send_int(MSG_WAYPOINT_FINISHED)\n");
	messageSend("\t\tsocket_send_int(waypoint_id)\n");
	messageSend("\t\texit_critical\n");
	messageSend("\tend\n");

	messageSend("\tthread statePublisherThread():\n"); //This thread runs continuosly on the robot controller
	messageSend("\t\tdef send_joint_state():\n");
	messageSend("\t\t\tq = get_joint_positions()\n");
	messageSend("\t\t\tqdot = get_joint_speeds()\n");
	messageSend("\t\t\ttau = get_joint_torques()\n");
	messageSend("\t\t\tfwd_kin = get_forward_kin()\n");
	messageSend("\t\t\tenter_critical\n");
	messageSend("\t\t\tsocket_send_int(MSG_JOINT_STATES)\n"); // use 'struct.unpack_from("!i", buf, 0)[0]' and check that it is equal to this number
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * q[0]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * q[1]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * q[2]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * q[3]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * q[4]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * q[5]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * qdot[0]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * qdot[1]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * qdot[2]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * qdot[3]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * qdot[4]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * qdot[5]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * tau[0]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * tau[1]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * tau[2]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * tau[3]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * tau[4]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * tau[5]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * fwd_kin[0]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * fwd_kin[1]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * fwd_kin[2]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * fwd_kin[3]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * fwd_kin[4]))\n");
	messageSend("\t\t\tsocket_send_int(floor(MULT_jointstate * fwd_kin[5]))\n");
	messageSend("\t\t\texit_critical\n");
	messageSend("\t\tend\n");
	messageSend("\t\tsend_joint_state()\n");
	messageSend("\t\twhile True:\n");
	messageSend("\t\t\tsend_joint_state()\n");
	messageSend("\t\t\tsync()\n"); //wait for next time slice
	messageSend("\t\tend\n");
	messageSend("\t\tsync()\n");
	messageSend("\tend\n");

	messageSend(ip); //open a socket back to this computer
	messageSend("\tsend_out(\"Hello\")\n");
	messageSend("\tthread_state = run statePublisherThread()\n"); //spins the thread
	
	messageSend("\tQUEUE_NONE = 0\n");
	messageSend("\tcmd_queue_state = QUEUE_NONE\n");
	messageSend("\tcmd_queue_id = 0\n");
	messageSend("\tQUEUE_RUNNING = 1\n");
	messageSend("\tcmd_queue_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n");
	messageSend("\tcmd_queue_params = [0.0, 0.0, 0.0, 0.0]\n");
	
	messageSend("\tdef set_queue(id, q, params):\n");
	messageSend("\t\tenter_critical\n");
	messageSend("\t\tcmd_queue_state = QUEUE_RUNNING\n");
	messageSend("\t\tcmd_queue_id = id\n");
	messageSend("\t\tcmd_queue_q = q\n");
	messageSend("\t\tcmd_queue_params = params\n");
	messageSend("\t\texit_critical\n");
	messageSend("\tend\n");
	messageSend("\tthread QueueThread():\n");
	messageSend("\t\tstate = QUEUE_NONE\n");
	messageSend("\t\twhile True:\n");
	messageSend("\t\t\t# Latches the new command\n");
	messageSend("\t\t\tenter_critical\n");
	messageSend("\t\t\tq = cmd_queue_q\n");
	messageSend("\t\t\tid = cmd_queue_id\n");
	messageSend("\t\t\tstate = cmd_queue_state\n");
	messageSend("\t\t\tparams = cmd_queue_params\n");
	messageSend("\t\t\tcmd_queue_state = QUEUE_NONE\n");
	messageSend("\t\t\texit_critical\n");
	messageSend("\t\t\tif state == QUEUE_RUNNING:\n");
	messageSend("\t\t\t\tif id == MSG_SPEEDL:\n");
	messageSend("\t\t\t\t\tsend_out(\"speedl started\")\n");//# Sends the command
	messageSend("\t\t\t\t\tspeedl(q, params[0], params[1])\n");
	messageSend("\t\t\t\t\tsend_waypoint_finished(waypoint_id)\n");
	messageSend("\t\t\t\t\tsend_out(\"speedl finished\")\n");
	messageSend("\t\t\t\tend\n");
	messageSend("\t\t\telse:\n");
	messageSend("\t\t\t\t#send_out(\"Idle\")\n");
	messageSend("\t\t\t\tsync()\n");
	messageSend("\t\t\tend\n");
	messageSend("\t\tend\n");
	messageSend("\tend\n");

	messageSend("\tthread_brake = run QueueThread()\n"); //spins the thread
	messageSend("\twhile True:\n");
	messageSend("\t\tll = socket_read_binary_integer(1)\n");
	messageSend("\t\tif ll[0] == 0:\n"); //socket_read_binary_integer(1)[0] is the number of bytes read
//	messageSend("\t\t\tsend_out("Received nothing")\n");
	messageSend("\t\t\tsync()\n");
	messageSend("\t\telif ll[0] > 1:\n");
	messageSend("\t\t\tsend_out(\"Received too many things\")\n");
	messageSend("\t\telse:\n");
	
	messageSend("\t\t\tmtype = ll[1]\n");
	messageSend("\t\t\tif mtype == MSG_QUIT:\n"); // got quit
	messageSend("\t\t\t\tsend_out(\"Received QUIT\")\n");
	messageSend("\t\t\t\tsocket_send_int(MSG_QUIT)\n");
	messageSend("\t\t\t\tbreak\n");
	
	messageSend("\t\t\telif mtype == MSG_MOVEL:\n"); // got movel
	messageSend("\t\t\t\tsend_out(\"Received movel\")\n");
	messageSend("\t\t\t\tparams_mult = socket_read_binary_integer(1+6+4)\n");
	messageSend("\t\t\t\tif params_mult[0] == 0:\n");
	messageSend("\t\t\t\t\tsend_out(\"Received no parameters for movel message\")\n");
	messageSend("\t\t\t\tend\n");

	messageSend("\t\t\t\twaypoint_id = params_mult[1]\n");  // Unpacks the parameters
	messageSend("\t\t\t\tq = p[params_mult[2] / MULT_jointstate,\n");
	messageSend("\t\t\t\t\tparams_mult[3] / MULT_jointstate,\n");
	messageSend("\t\t\t\t\tparams_mult[4] / MULT_jointstate,\n");
	messageSend("\t\t\t\t\tparams_mult[5] / MULT_jointstate,\n");
	messageSend("\t\t\t\t\tparams_mult[6] / MULT_jointstate,\n");
	messageSend("\t\t\t\t\tparams_mult[7] / MULT_jointstate]\n");
	messageSend("\t\t\t\ta = params_mult[8] / MULT_jointstate\n");
	messageSend("\t\t\t\tv = params_mult[9] / MULT_jointstate\n");
	messageSend("\t\t\t\tt = params_mult[10] / MULT_jointstate\n");
	messageSend("\t\t\t\tr = params_mult[11] / MULT_jointstate\n");

	messageSend("\t\t\t\tsend_out(\"movel started\")\n");// Sends the command
	messageSend("\t\t\t\tmovel(q, a, v, t, r)\n");
	messageSend("\t\t\t\tsend_waypoint_finished(waypoint_id)\n");
	messageSend("\t\t\t\tsend_out(\"movel finished\")\n");


	messageSend("\t\t\telif mtype == MSG_SPEEDL:\n"); // got speedl
	messageSend("\t\t\t\tsend_out(\"Received speedl\")\n");
	messageSend("\t\t\t\tparams_mult = socket_read_binary_integer(1+6+2)\n");
	messageSend("\t\t\t\tif params_mult[0] == 0:\n");
	messageSend("\t\t\t\t\tsend_out(\"Received no parameters for speed message\")\n");
	messageSend("\t\t\t\tend\n");

	messageSend("\t\t\t\twaypoint_id = params_mult[1]\n"); // Unpacks the parameters
	messageSend("\t\t\t\tq = [params_mult[2] / MULT_jointstate,\n");
	messageSend("\t\t\t\t\tparams_mult[3] / MULT_jointstate,\n");
	messageSend("\t\t\t\t\tparams_mult[4] / MULT_jointstate,\n");
	messageSend("\t\t\t\t\tparams_mult[5] / MULT_jointstate,\n");
	messageSend("\t\t\t\t\tparams_mult[6] / MULT_jointstate,\n");
	messageSend("\t\t\t\t\tparams_mult[7] / MULT_jointstate]\n");
	messageSend("\t\t\t\ta = params_mult[8] / MULT_jointstate\n");
	messageSend("\t\t\t\tt_min = params_mult[9] / MULT_jointstate\n");
	messageSend("\t\t\t\tset_queue(MSG_SPEEDL, q, [a, t_min, 0, 0])\n");

	messageSend("\t\t\telif mtype == MSG_STOPL:\n"); // got stopl
	messageSend("\t\t\t\tsend_out(\"Received stopl\")\n");
	messageSend("\t\t\t\tstopl(0.2)\n");
	messageSend("\t\t\t\tsend_out(\"stopl finished\")\n");

	messageSend("\t\t\telse:\n");
	messageSend("\t\t\t\tsend_out(\"Unsupported command\")\n");
	messageSend("\t\t\tend\n");
	messageSend("\t\tend\n");
	messageSend("\t\tsync()");
	messageSend("\tend\n"); 	
	messageSend("end\n");
    
    
    
  }


  void startInterface (){
  
  struct sockaddr_in server;
  struct in_addr ipv4addr;
  struct hostent *hp;
  
    
     interface = socket(AF_INET,SOCK_STREAM,0);
    server.sin_family = AF_INET;

    inet_pton(AF_INET, CALLBACKHOST, &ipv4addr);
    hp = gethostbyaddr(&ipv4addr, sizeof ipv4addr, AF_INET);
   

    bcopy(hp->h_addr, &(server.sin_addr.s_addr), hp->h_length);
    server.sin_port = htons(31001);

    connect(interface, (const sockaddr *)&server, sizeof(server));
  
  
}

void stopInterface(){
  
  close(interface);
  
}
  
  void testInterface(string in){
    
    
    send(interface, (char *)in.c_str(), strlen((char *)in.c_str()), 0);
    
  }
  
  
};

