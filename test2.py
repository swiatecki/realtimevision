import socket
import time

print "Start : %s" % time.ctime()

HOST1 = '192.38.66.249'    # UR5
PORT1 = 30003              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST1, PORT1))
s.send('def kuk():\n')
#s.send('  i=0\n')
#s.send('  while i < 10:\n')
s.send('\tspeedj([0.1,0,0,0,0,0],0.1,2)\n')
s.send('\tspeedj([0,0,0,0,0,0],0.1,2)\n')
#s.send('    movel(p[0.0,-0.5-i*0.03,0.28,2.77,1.49,0],5,0.5)\n')
#s.send('    movel(p[0.1,-0.5-i*0.03,0.28,2.77,1.49,0],5,0.5)\n')
#s.send('    movel(p[0.1,-0.5-i*0.03,0.18,2.77,1.49,0],5,0.5)\n')
#s.send('    i=i+1\n')
#s.send('  end\n')
s.send('end\n')

#print 'Received', repr(data)
#time.sleep(5)
s.close()
print "End : %s" % time.ctime()

