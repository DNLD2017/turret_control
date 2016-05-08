import numpy as np
from vibes import *
from pyIbex import *
import time

os.system('./VIBes-viewer &')
time.sleep(0.5) #Attente de l'ouverture de l'afficheur
vibes.beginDrawing()
vibes.newFigure('One ring to rule them all...')
vibes.setFigureProperties({'x':0, 'y':0, 'width': 800, 'height':500})

pi=np.pi

X0 = IntervalVector([[-50,50],[-50,50]])
xtr1=IntervalVector([[0,0],[0,0]])
xtr2=IntervalVector([[30,30],[0,0]])
xtr3=IntervalVector([[-30,-30],[30,30]])
theta1=Interval(pi/2-pi/6*0.1,pi/2+pi/6*0.1)
theta2=Interval(3*pi/4-pi/6*0.1,3*pi/4+pi/6*0.1)
theta3=Interval(0-pi/6*0.1,0+pi/6*0.1)
f1=Function("x","y","(x-%s)*sin(%s)-cos(%s)*(y-%s)" % (xtr1[0], theta1, theta1, xtr1[1]))
f2=Function("x","y","(x-%s)*sin(%s)-cos(%s)*(y-%s)" % (xtr2[0], theta2, theta2, xtr2[1]))
f3=Function("x","y","(x-%s)*sin(%s)-cos(%s)*(y-%s)" % (xtr3[0], theta3, theta3, xtr3[1]))
C1=CtcFwdBwd(f1,Interval(0,0))
C2=CtcFwdBwd(f2,Interval(0,0))
C3=CtcFwdBwd(f3,Interval(0,0))
C=C1&C2&C3
pySIVIA(X0,C,0.1)
vibes.drawCircle(0,0,1,"yellow")
vibes.drawCircle(30,0,1,"yellow")
vibes.drawCircle(-30,30,1,"yellow")
vibes.axisEqual()
vibes.endDrawing()

