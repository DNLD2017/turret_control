#!/usr/bin/env python
import rospy
import maestro
from geometry_msgs.msg import Point

class Pantilt:

    # Open the serial port used by the Maestro and set the ports
    # the pan & tilt servos are pluged in.
    def __init__(self,port=0,pan=0,tilt=1):
        self.maestro = maestro.Controller(port)
        self.pan = pan
        self.tilt = tilt

    # Set the min & max values of pan & tilt servos. Values in microseconds.
    # Center is 1500, max is 2250 and min is 750
    def setMinMax(self, minpan, mintilt, maxpan, maxtilt):
        self.maestro.setRange(self.pan, 4*minpan, 4*maxpan)
        self.maestro.setRange(self.tilt, 4*mintilt, 4*maxtilt)

    # Set the speed and accel of servos. Default values inleash the values.
    def setSpeedAcc(self, speedpan = 0, speedtilt = 0, accpan = 0, acctilt = 0):
        self.maestro.setSpeed(self.pan, speedpan)
        self.maestro.setSpeed(self.tilt, speedtilt)
        self.maestro.setAccel(self.pan, accpan)
        self.maestro.setAccel(self.tilt, acctilt)

    # Rotate the servos at the desired speed. Value in microseconds.
    def movePan(self, speed):

        self.maestro.setTarget(self.pan, 4*speed)

    def moveTilt(self, speed):

        self.maestro.setTarget(self.tilt, 4*speed)

    def getPan(self):
        return self.maestro.getPosition(self.pan)/4

    def getTilt(self):
        return self.maestro.getPosition(self.tilt)/4
p=Pantilt(0,0,1)
def controlPos(erreurPan, erreurTilt,p):
    p.movePan(erreurPan)
    p.moveTilt(erreurTilt)

def callback(data,p):
    rospy.loginfo(rospy.get_caller_id() + 'I heard x %s', data.x)
    rospy.loginfo(rospy.get_caller_id() + 'I heard y %s', data.y)
    u1=((data.x)/1.6)+1500
    u2=((data.y)/1.2)+1496
    v1=u1
    v2=u2
    print('erreurPan= ',int(u1))
    print('erreurTilt= ',int(u2))
    print('')
    controlPos(int(u1), int(u2),p)

def myhook():
    controlPos(1500, 1496,p)

def listener(p):

    rospy.init_node('listenerErreurPixel', anonymous=True)
    rospy.Subscriber('/turret1/erreurPixel', Point, callback,p)
    rospy.on_shutdown(myhook)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    p.setMinMax(750, 750, 2250, 2250)
    listener(p)
