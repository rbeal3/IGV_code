#!/usr/bin/env python

import Tkinter as tk
import getopt
import socket
import time
import sys

SLAVE = '192.168.1.2'
PORT = 23


class App(object):
    def __init__(self):
			self.reset()
			self.font = pygame.font.Font(None, 20)
    def task(self, socket):
			cmd = "PR 0,0,0,0,10000,10000;BG;"
			print "comand from task" + cmd
			socket.sendall(cmd + '\r')
			data = socket.recv(1024)
			print data

			print "if this ever runs, note it"
			root.after(20, self.task, socket)

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
	cmd = 'AC 0,0,0,0,50000,50000; DC 0,0,0,0,50000,50000;\r'
	sock.sendall(cmd)
	cmd = 'OE 0,0,0,0,1,1,0,0;\r'#error fail ON
	sock.sendall(cmd)
	cmd = 'ER ,,,,32767,32767,,;\r'#error fail setting
	sock.sendall(cmd)
	cmd = 'KI 0,0,0,0,0.1,0.1,0,0;\r'#motor gain adjustment
	sock.sendall(cmd)
#handle args
	try:
		print "parsing command line!\n"
		opts, args = getopt.getopt(argv, 'lr')
	except getopt.GetoptError:
		print 'failed to parse cmd line'
	if args[0] == '': #go straight
		print "you need to include a turn distance -i for inverse direction"
	elif args[0] == '0': #go straight
		print "Gir Will now take one step!\n"
		cmd = 'PR 0,0,0,0,10000,10000; BG;\r'
	for o, a in opts:
		if o in ("-l"): #left turn
			print "Gir Will now turn left!\n"
			cmd = 'PR 0,0,0,0,-'+args[0]+','+args[0]+'; BG;\r'
		elif o in ("-r"):
			print "Gir Will now turn right!\n"
			cmd = 'PR 0,0,0,0,'+args[0]+',-'+args[0]+'; BG;\r'
		
		#do something with 
	sock.sendall(cmd)
	data = sock.recv(1024)
	print cmd
	print data
	cmd = 'BG;\r'
	sock.sendall(cmd)
	data = sock.recv(1024)
	time.sleep(0.1)

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
