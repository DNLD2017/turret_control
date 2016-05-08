import numpy as np
import psutil
from vibes import *
from pyIbex import *
import time

pi=np.pi
R=50
# Renvoie la position du drone en fonction du temps
def drone(t):
    return R*np.cos(0.5*t),R*np.sin(0.5*t)

# Renvoie l'angle avec lequel la tourelle tr voit le drone à l'instant t
def angle(tr,t,eps):
    tx,ty=tr[0],tr[1]
    drx,dry=drone(t)
    theta=np.arctan2((ty-dry),(tx-drx))
    return Interval(theta-eps*pi/180,theta+eps*pi/180)

def detect(tr,eps):
    tr1,tr2=tr[0],tr[1]
    theta=angle(tr,t,eps)
    xtr1=Interval(tr1-0.5,tr1+0.5)
    xtr2=Interval(tr2-0.5,tr2+0.5)
    return Function("x","y","(x-%s)*sin(%s)-cos(%s)*(y-%s)" % (xtr1, theta, theta, xtr2))

def contractor(trLst,eps):
    C=[]
    for tr in trLst:
        if len(C)==0:
            C.append(CtcFwdBwd(detect(tr,eps)))
        else:
            C[0]=C[0] & CtcFwdBwd(detect(tr,eps))
    return C[0]

def printTr(trLst):
    for tr in trLst:
        vibes.drawCircle(tr[0],tr[1],1,"yellow[black]")

def launch():
    os.system('pkill -f ./VIBes-viewer')
    os.system('./VIBes-viewer &')
    time.sleep(0.5)

if __name__=="__main__":
    # Ouverture de vibes et de la figure
    launch()
    vibes.beginDrawing()
    vibes.newFigure('One ring to rule them all...')
    vibes.setFigureProperties({'x':0, 'y':0, 'width': 800, 'height':800})

    eps=3
    
    # Intervalle contenant le drone à l'origine
    X0 = IntervalVector([[-110,110],[-110,110]])
    # Postion exacte des tourelles
    tr1=[-20,0]
    tr2=[20,0]
    tr3=[0,20]
    tr4=[-15,-15]
    tr5=[15,15]
    tr6=[0,-20]
    tr7=[-15,15]
    tr8=[15,-15]
    tr9=[-70,0]
    tr10=[70,0]
    tr11=[0,70]
    tr12=[-65,-65]
    tr13=[65,65]
    tr14=[0,-70]
    tr15=[-65,65]
    tr16=[65,-65]
    trLst=[tr1,tr2,tr3,tr4,tr5,tr6,tr7,tr8,tr9,tr10,tr11,tr12,tr13,tr14,tr15,tr16]
    t=0
    while t<8:
        vibes.clearFigure()
        C=contractor(trLst,eps)
        # Calcul des boites contenant le drone et affichage
        pySIVIA(X0,C,1)
        # Affichage des tourelles
        printTr(trLst)
        xdr,ydr=drone(t)
        vibes.drawCircle(xdr,ydr,1,"red[red]")
        vibes.drawCircle(0,0,30,"red")
        vibes.axisEqual()
        t=t+0.01
        time.sleep(0.05)
    vibes.endDrawing()

