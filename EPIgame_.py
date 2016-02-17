from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys

# "open sound control" library
import OSC

import time as time_ #make sure we don't override time


if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

# colors for drawing different bodies 
SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
                  pygame.color.THECOLORS["blue"], 
                  pygame.color.THECOLORS["green"], 
                  pygame.color.THECOLORS["orange"], 
                  pygame.color.THECOLORS["purple"], 
                  pygame.color.THECOLORS["yellow"], 
                  pygame.color.THECOLORS["violet"]]


class BodyGameRuntime(object):
    def __init__(self):
        pygame.init()

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()
        
        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1), 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)

        pygame.display.set_caption("EPI interactive game")

        # Loop until the user clicks the close button.
        self._done = False
        self._client = OSC.OSCClient()
	#self._client.connect(('192.168.43.217', 9000)) ## my pc on android
	self._client.connect(('192.168.1.104', 5005))
        # Used to manage how fast the screen updates OBS! THIS LINE IS REPEATED
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)
        
        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)

        # here we will store skeleton data 
        self._bodies = None

# DEFINE two points to draw a line between them
    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good 
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)

        try:
            # hack COLOR AND SIZE oflines between joints
            # counter = ( time_.time() * 1000 ) % 100 # 0 to 100 in a 1/10 of a sec
            # print ( counter )            
            # size = 8 + 0.07*counter
            
            # DRAW A LINE BETWEEN THE TWO JOUINTS WITH COLOR AND SIZE
            # pygame.draw.line(self._frame_surface, color, start, end, size)
            pygame.draw.line(self._frame_surface, color, start, end, 8)
            
        except: # need to catch it due to possible invalid positions (with inf)
            pass
    
    # define function to send OSC messages
    def sendOSC_twoPoints(self, joints, jointPoints, joint0, joint1):
        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;
	
	# when a joint is not tracked send signal - WHAT'S WRONG HERE
	if (joint0State == PyKinectV2.TrackingState_NotTracked):
	    print('jO not tracked')
	    oscmsg_leftNO = OSC.OSCMessage()
	    oscmsg_leftNO.setAddress("/leftNO")
	    oscmsg_leftNO.append(0)
	    self._client.send(oscmsg_leftNO)

	if (joint1State == PyKinectV2.TrackingState_NotTracked):
	    print('j1 not tracked')
	    oscmsg_rightNO = OSC.OSCMessage()
	    oscmsg_rightNO.setAddress("/rightNO")
	    oscmsg_rightNO.append(0)
	    self._client.send(oscmsg_rightNO)
        
	# when a joint is "inferred" send signal 
	if (joint0State == PyKinectV2.TrackingState_Inferred):
	    oscmsg_leftNO = OSC.OSCMessage()
	    oscmsg_leftNO.setAddress("/leftNO")
	    oscmsg_leftNO.append(0)
	    self._client.send(oscmsg_leftNO)
	else:
	    oscmsg_leftNO = OSC.OSCMessage()
	    oscmsg_leftNO.setAddress("/leftNO")
	    oscmsg_leftNO.append(1)
	    self._client.send(oscmsg_leftNO)

	if (joint1State == PyKinectV2.TrackingState_Inferred):
   	    oscmsg_rightNO = OSC.OSCMessage()
	    oscmsg_rightNO.setAddress("/rightNO")
	    oscmsg_rightNO.append(0)
	    self._client.send(oscmsg_rightNO)
	else:
	    oscmsg_rightNO = OSC.OSCMessage()
	    oscmsg_rightNO.setAddress("/rightNO")
	    oscmsg_rightNO.append(1)
	    self._client.send(oscmsg_rightNO)

	# both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return
	
	
        # ok, at least one is good - joint0 is left hand, joint1 is right hand
        leftX = jointPoints[joint0].x 
	leftY = jointPoints[joint0].y
	rightX = jointPoints[joint1].x
	rightY = jointPoints[joint1].y
	# print 'point1 X is %s point1 Y is %s ' \
	#'point2 X is %s point2 Y is %s' %  (jointPoints[joint0].x, jointPoints[joint0].y,jointPoints[joint1].x, jointPoints[joint1].y)

        try:
           # create message object
            oscmsg1 = OSC.OSCMessage()
            oscmsg2 = OSC.OSCMessage()
	    oscmsg3 = OSC.OSCMessage()
	    oscmsg4 = OSC.OSCMessage()

            # set the tag of the message
            oscmsg1.setAddress("/leftX")
            oscmsg2.setAddress("/leftY")
	    oscmsg3.setAddress("/rightX")
	    oscmsg4.setAddress("/rightY")

            # its content
            oscmsg1.append(leftX)
            oscmsg2.append(leftY)   
            oscmsg3.append(rightX)
            oscmsg4.append(rightY)

            self._client.send(oscmsg1)
            self._client.send(oscmsg2)
	    self._client.send(oscmsg3)
	    self._client.send(oscmsg4)
            
        except: # need to catch it due to possible invalid positions (with inf)
            pass
    
    def draw_body(self, joints, jointPoints, color):
        # Torso
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft);
    
        # Right Arm    
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_ThumbRight);

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft);

        # Right Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight);

        # Left Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft);

    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def run(self):
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE: # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'], 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
                    
            # --- Game logic should go here

            # --- Getting frames and drawing  
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data 
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            if self._bodies is not None: 
                # GET NUMBER OF BODIES AND DRAW EACH ONE
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked: 
                        continue 
                    
                    joints = body.joints
                    # convert joint coordinates to color space 
                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    self.draw_body(joints, joint_points, SKELETON_COLORS[i])
                    
                    # send body points through OSC
                    self.sendOSC_twoPoints( joints, joint_points, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandRight)
            	    #print(joint_points[PyKinectV2.JointType_HandRight].x,joint_points[PyKinectV2.JointType_HandRight].y)
                    #print('that was right, now left')
		    #print(joint_points[PyKinectV2.JointType_HandLeft].x,joint_points[PyKinectV2.JointType_HandLeft].y)
    
            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size) 
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));
            self._screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None
            pygame.display.update()

            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

            # --- Limit to 60 frames per second
            self._clock.tick(60)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        print 'Fechei'
        pygame.quit()


#__main__ = "Kinect v2 Body Game"
__main__ = "Kinect v2 EPI Game"
game = BodyGameRuntime();
game.run();

