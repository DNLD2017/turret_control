import numpy as np
from vibes import *
from pyIbex import *
import time


if __name__=="__main__":
    os.system('./VIBes-viewer &')
    time.sleep(0.5) #Attente de l'ouverture de l'afficheur
    vibes.beginDrawing()
    vibes.newFigure('Title')
    vibes.setFigureProperties({'x':0, 'y':0, 'width': 800, 'height':500})
    pi=np.pi
    X0 = IntervalVector([[-50,50],[-50,50],[-50,50]])
    xtr1=IntervalVector([[0,0],[0,0]])
    xtr2=IntervalVector([[30,30],[0,0]])
    xtr3=IntervalVector([[-30,-30],[30,30]])
    theta1=Interval(pi/2-pi/6*0.1,pi/2+pi/6*0.1)
    theta2=Interval(3*pi/4-pi/6*0.1,3*pi/4+pi/6*0.1)
    theta3=Interval(0-pi/6*0.1,0+pi/6*0.1)
    f1=Function("f1","f2","f3","cos(%s)*cos(%s)-f1,cos(%s)*sin(%s)-f2,-sin(%s)-f3",%(etheta,epsi,etheta,epsi,etheta))
    f2=Function("f1","f2","f3","\
    cos(%s)*cos(%s)*%s+(-cos(%s)*sin(%s)+sin(%s)*cos(%s)*sin(%s))*%s+(sin(%s)*sin(%s)+sin(%s)*cos(%s)*cos(%s))*%s-f1,\
    cos(%s)*sin(%s)*%s+(cos(%s)*cos(%s)+sin(%s)*sin(%s)*sin(%s))*%s+(-cos(%s)*sin(%s)+sin(%s)*cos(%s)*sin(%s))*%s-f2,\
    -sin(%s)*%s+cos(%s)*sin(%s)*%s+cos(%s)*cos(%s)*%s-f3\
    ",%(theta,psi,f1[0],phi,psi,theta,psi,phi,f1[1],psi,phi,theta,psi,phi,f1[2],theta,psi,f1[0],phi,psi,theta,psi,phi,f1[1],psi,phi,theta,phi,psi,f1[2],theta,f1[0],theta,phi,f1[1],theta,phi,f1[2]))
    f3=Function("x","y","z","%s*(x-%s)-%s*(y-%s),%s*(x-%s)-%s*(z-%s),%s*(y-%s)-%s*(z-%s)",%(f2[2],p[1],f2[1],p[2],f2[3],p[1],f2[1],p[3],f2[3],p[2],f2[2],p[3]))
    C=CtcFwdBwd(f3,Interval(0,0))
    pySIVIA(X0,C,0.1)
    vibes.drawCircle(0,0,1,"[yellow]")
    vibes.drawCircle(30,0,1,"[yellow]")
    vibes.drawCircle(-30,30,1,"[yellow]")
    vibes.axisEqual()
    vibes.endDrawing()

