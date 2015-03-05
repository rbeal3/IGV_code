import pygame
import socket
import sys
import os
from subprocess import call

# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)

SLAVE = '192.168.1.2'
PORT = 23

imageName = 'lab_test_'
imageNumber = 0
suffix = '.pcd'
# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputing the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def printt(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    SLAVE = '192.168.1.2'
    PORT = 23
    print "Connecting to galil at", SLAVE
    sock.connect((SLAVE, PORT))
    print 'Waiting . . . .'
    sock.sendall('TP;\r')
    data = sock.recv(1024)
    print "recved"
    print data
except:
    print 'A connection could not be made to the designated server, giving up.'
    sock.close()
    sys.exit(1)

print "Successfully connected, initializing movement variables"
cmd = 'AC 0,50000,0,0,50000; DC 0,50000,0,0,50000;\r'
sock.sendall(cmd)
cmd = 'OE 1,1,1,1,1,1,1,1;\r'
sock.sendall(cmd)
cmd = 'ER 32767,32767,32767,32767,32767,32767,32767,32767;\r'
sock.sendall(cmd)
forwardMovement = 0
turnMovement = 0
pygame.init()

# Set the width and height of the screen [width,height]
size = [500, 700]
screen = pygame.display.set_mode(size)

pygame.display.set_caption("Gir Driver")

#Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Initialize the joysticks
pygame.joystick.init()

# Get ready to print
textPrint = TextPrint()

# -------- Main Program Loop -----------
while done==False:
    # EVENT PROCESSING STEP
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop

        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")


    # DRAWING STEP
    # First, clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
    screen.fill(WHITE)
    textPrint.reset()

    # Get count of joysticks
    joystick_count = pygame.joystick.get_count()


    # For each joystick:
    for i in range(joystick_count):

        cmd = 'TE;'
        sock.sendall(cmd + '\r')
        data = sock.recv(1024)
        textPrint.printt(screen, "Error: " + data)

        lMotor = 0
        rMotor = 0
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = joystick.get_numaxes()

        for i in range( axes ):
            axis = joystick.get_axis( i )
            if i == 1 and abs(axis) > 0.2:
                forwardMovement = -10000*axis
            elif i == 1 and abs(axis) < 0.2:
                forwardMovement = 0
            if i == 3 and abs(axis) > 0.2:
                turnMovement = -8000*axis
            elif i == 3 and abs(axis) < 0.2:
                turnMovement = 0
            textPrint.printt(screen, "Axis {} value: {:>6.3f}".format(i, axis) )
        textPrint.unindent()

        lMotor += forwardMovement - turnMovement
        rMotor += forwardMovement + turnMovement
        if forwardMovement  == 0 and turnMovement == 0:
            cmd = 'ST;'
        else:
            cmd = 'JG 0,'+ str(int(lMotor)) + ',0,0,' + str(int(rMotor)) + ';BG;\r'
        sock.sendall(cmd)
        data = sock.recv(1024)

        textPrint.printt(screen, "lMotor value: {:>6.3f}".format(lMotor))
        textPrint.printt(screen, "rMotor value: {:>6.3f}".format(rMotor))
        textPrint.printt(screen, "Sending cmd: " + cmd)

    	buttons = joystick.get_numbuttons()

    	for i in range( buttons ):
        	button = joystick.get_button( i )  
        	if i == 0 and button == 1:
        	    cmd = "RS;"
        	    sock.sendall( cmd + '\r')
        	elif i == 1 and button == 1:
        	    call('../bb2cloud/bb2cloud', imageName + str(imageNumber) + suffix)
        	    call('beep', '440 10')
        	    call('../pcdViewer/pcdViewer', imageName + str(imageNumber) + suffix)
		elif i == 2 and button == 1:
		    pygame.quit()



    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT

    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

    # Limit to 10 frames per second
    clock.tick(10)


# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit ()
