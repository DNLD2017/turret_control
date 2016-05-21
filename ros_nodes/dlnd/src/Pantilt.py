import Maestro

class Pantilt:

	# Open the serial port used by the Maestro and set the ports
	# the pan & tilt servos are pluged in.
	def __init__(self,port=0,pan=0,tilt=1):
		self.maestro = Maestro.Controller(port)
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
