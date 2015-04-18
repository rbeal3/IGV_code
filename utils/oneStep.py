#!/usr/bin/env python

import Tkinter as tk
import getopt
import socket
import time
import sys

SLAVE = '192.168.1.2'
PORT = 23

# python oneStep.py 1 #will go forward 2 feet
# python oneStep.py -r 2 # will turn right 180 degrees
# 
#
#

class App(object):
    def __init__(self):
			self.reset()
			self.font = pygame.font.Font(None, 20)

def main(argv):

	try:
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		print 'Connecting....'
		sock.connect((SLAVE, PORT))
		print 'Starting TP:'
		sock.sendall('TP;\r')
		data = sock.recv(1024)
		print data
	except:
		print "A connection could not be made, giving up."
		sock.close()
		sys.exit(1) 
	print "Successfully connected, initializing acceleration variables"
	cmd = 'AC 0,0,0,0,50000,50000;\r'
	print cmd
	sock.sendall(cmd)
	cmd = 'DC 0,0,0,0,50000,50000;\r'
	print cmd
	sock.sendall(cmd)
	cmd = 'OE 0,0,0,0,1,1,0,0;\r'#error fail ON
	print cmd
	sock.sendall(cmd)
	cmd = 'ER ,,,,32767,32767,,;\r'#error fail setting
	print cmd
	sock.sendall(cmd)
#handle args
	try:
		print "parsing command line!\n"
		opts, args = getopt.getopt(argv, 'lr')
	except getopt.GetoptError:
		print 'failed to parse cmd line'
	if len(sys.argv)==1:
		print "you need to include a turn distance -i for inverse direction"
		return 0;

	turnAmount=args[0]*25000
	distance=args[0]*26500
	cmd = 'PR 0,0,0,0,'+distance+','+distance+';\r'

	print "distance =" + distance
	print "the command will be:"
	print cmd
	time.sleep(5)

	for o, a in opts:
		if o in ("-r"): #right turn
			print "Gir Will now turn right!\n"
			cmd = 'PR 0,0,0,0,-'+turnAmount+','+turnAmount+';\r'
		elif o in ("-l"):
			print "Gir Will now turn left!\n"
			cmd = 'PR 0,0,0,0,'+turnAmount+',-'+turnAmount+';\r'
		
		#do something with 
	sock.sendall(cmd)
	data = sock.recv(1024)
	print "turn command sent:"
	print cmd
	print data
	cmd = 'BG;\r'
	sock.sendall(cmd)
	data = sock.recv(1024)
	time.sleep(0.1)

	print "command sent: (hopefully BG)"
	print cmd

#code to bot above this
	try:
			#root.mainloop()
			#root.destroy()
			sock.close()
	except:
			#root.destroy()
			sock.close()
			raise

if __name__ == '__main__':
    main(sys.argv[1:])
