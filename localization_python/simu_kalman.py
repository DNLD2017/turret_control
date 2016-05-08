# -*- coding: utf-8 -*-
import numpy as np
from vibes import *
from pyIbex import *
import time

pi=np.pi
R=50
vmax=20

# Renvoie la position du drone en fonction du temps
def drone(t):
    return R*np.cos(3/5*vmax/R*t),R*np.sin(4/5*vmax/R*t)

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

def kalman(x0,G0,u,y,G_alpha,G_beta,A,C):
    S=np.dot(C,np.dot(G0,np.transpose(C)))+G_beta
    K=np.dot(G0,np.dot(np.transpose(C),np.linalg.inv(S)))
    y_tilde=y-np.dot(C,x0)
    xup=x0+np.dot(K,y_tilde)
    Gup=G0-np.dot(K,np.dot(C,G0))
    x1=np.dot(A,xup)+u
    G1=np.dot(A,np.dot(Gup,np.transpose(A)))+G_alpha
    return xup,Gup,x1,G1

def center(X):
    return (X[0].lb()+X[0].ub())/2,(X[1].lb()+X[1].ub())/2

def diam(X):
    return np.sqrt((-X[0].lb()+X[0].ub())**2+(-X[1].lb()+X[1].ub())**2)


if __name__=="__main__":
    # Ouverture de vibes et de la figure
    launch()
    vibes.beginDrawing()
    vibes.newFigure('One ring to rule them all...')
    vibes.setFigureProperties({'x':0, 'y':0, 'width': 800, 'height':800})
    
    # Constantes nécessaires
    eps=5
    dt=0.02
    lmax=vmax*dt
    size=10.597 # Proba de 0.995
    
    # Intervalle contenant le drone à l'origine
    X0 = IntervalVector([[-110,110],[-110,110]])
    # Variables du kalman
    x0=np.matrix('0;0;0;0')
    u=np.matrix('0;0;0;0')
    G1=np.matrix('150000 0 0 0; 0 150000 0 0; 0 0 150000 0;0 0 0 150000')
    G_alpha=np.matrix([[lmax/np.sqrt(size), 0, 0, 0],[0, lmax/np.sqrt(size), 0, 0],[0, 0, vmax/np.sqrt(size), 0],[0, 0, 0, vmax/np.sqrt(size)]])
    Amat=np.matrix('0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0')
    Cmat=np.matrix('1 0 0 0;0 1 0 0')
    
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
    # Liste des tourelles
    trLst=[tr1,tr2,tr3,tr4,tr5,tr6,tr7,tr8,tr9,tr10,tr11,tr12,tr13,tr14,tr15,tr16]
    t=0
    while t<8:
        vibes.clearFigure()
        vibes.drawBox(X0[0].lb(),X0[0].ub(),X0[1].lb(),X0[1].ub(),'black[blue]')
        # Affichage des tourelles
        printTr(trLst)
        
        # Calcul des boites contenant le drone et affichage
        C=contractor(trLst,eps)
        [res_out,res_y]=pySIVIA(X0,C,1,draw_boxes=False,save_result=True)
        XDR=res_y[0]
        for inter in res_y:
            XDR=XDR|inter
        # Affichage de la boite englobante
        vibes.drawBox(XDR[0].lb(),XDR[0].ub(),XDR[1].lb(),XDR[1].ub(),'black[green]')
        # Transformation en ellipse
        y=center(XDR)
        y=np.matrix([[y[0]],[y[1]]])
        l=diam(XDR)
        G_beta=np.matrix([[l/2/np.sqrt(size), 0],[0,l/2/np.sqrt(size)]])
        # Application du filtre de kalman
        x0,G0,x1,G1=kalman(x0,G1,u,y,G_alpha,G_beta,Amat,Cmat)
        # Affichage du drone
        xdr,ydr=drone(t)
        vibes.drawCircle(xdr,ydr,1,"red[red]")
        # Affichage de l'estimation du drone par kalman
        vibes.drawCircle(x0.item(0),x0.item(1),0.5,"yellow[yellow]")
        # Affichage de l'erreur d'estimation à 0.995 
        vibes.drawEllipse(x0.item(0),x0.item(1),2*G0.item((0,0))*np.sqrt(size),2*G0.item((1,1))*np.sqrt(size),0.0)
        # Affichage de l'ellipse englobante
        vibes.drawEllipse(y.item(0),y.item(1),G_beta.item((0,0))*np.sqrt(size),G_beta.item((1,1))*np.sqrt(size),0.0,color='green')
        vibes.axisEqual()
        t=t+dt
        time.sleep(0.05)
    vibes.endDrawing()

