import cv2
import copy;
import numpy as np

def color(event, x, y, flags,param):
    #param:  0 => booleen clic
    #        1 => pos x du pixel selectionne par clic
    #        2 => pos y du pixel selectionne par clic
    #        3 => h pixel selectionne par clic
    #        4 => s pixel selectionne par clic
    #        5 => v pixel selectionne par clic
    
    cop=copy.copy(frame)  #Fait une copie du tableau de donnees de la video (Format BGR)
    hsv=cv2.cvtColor(cop,cv2.COLOR_BGR2HSV) #Transforme donnees BGR en HSV
    

    
    if (event==cv2.EVENT_LBUTTONUP): #Si clic gauche enregistre pos (x,y) de la souris
        
        param[0]=True
        pixelHSV=hsv[(y,x)] #couleur du pixel en pos (x,y) en HSV
        pixelRGB=frame[(y,x)] #couleur du pixel en (x,y) en BGR
        
        #Met a jour param
        param[1]=x
        param[2]=y
        param[3]=(int)(pixelHSV[0])
        param[4]=(int)(pixelHSV[1])
        param[5]=(int)(pixelHSV[2])
        
        
        print "x,y =",x,y
        print "hsv= ",pixelHSV        # Gives you HSV at clicked point
        print "im= ",pixelRGB    # qGives you RGB at clicked point
        
       
         
def track(imgOriginal,h,s,v,compteur,n,tabcX,tabcY):
    objectDetected=False;  #booleen = True si un objet est detecte
    
    if compteur>n-1:
        compteur=0
   
    
    #Cree une plage de couleurs autour de la couleur de la couleur HSV entree en parametre de la fonction
    if h-10>0:
        hLow=h-10
    else :
        hLow=0
    
    if h+10<180:
        hHigh=h+10
    else :
        hHigh=180
    
    if s+20<255:
        sHigh=s+20
    else:
        sHigh=255
        
    if s-20>0:
        sLow=s-20
    else:
        sLow=0
        
    if v+20<255:
        vHigh=v+20
    else:
        vHigh=255
        
    if v-20>0:
        vLow=v-20
    else:
        vLow=0
    
    colorMin=(hLow,sLow,vLow)
    colorMax=(hHigh,sHigh,vHigh)
    
    #Couleur complementaire de la couleur selectionnee
    
     
    #Convertit en HSV image entree en parametre de la fonction
    imgHSV = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2HSV)
    
    if hLow>0 and sLow>0 and vLow>0:
        
        #Applique un filtre sur la plage de couleur donnee 
        imgThresh = cv2.inRange(imgHSV, colorMin, colorMax)
        

     
        #Floute pour avoir moins de bruit
        imgThresh = cv2.GaussianBlur(imgThresh, (61, 61), 2)                 
        

        #Affiche image filtree sur la couleur
        cv2.imshow('imgThresh',imgThresh)
        
        #Detecte contour observes sur image filtree
        imCont, contours, hierarchy=cv2.findContours(imgThresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);
        
        #Met a jour booleen (si contours contient des donnees alors objet detecte)
        if (len(contours)>0):
            objectDetected=True;
    
        else :
            objectDetected=False;
    
    if objectDetected :
        #On prend plus gros contour en supposant que c'est probablement l'objet cherche (derniere position dans la liste contour)
        
        cnt = contours[len(contours)-1];
        
        
        #Dessine un rectangle autour de la zone 
        x,y,w,h = cv2.boundingRect(cnt);
        cv2.rectangle(imgOriginal,(x,y),(x+w,y+h),(0,255,0),2);
    
        
        #trouve le centre de la zone encadree
        M = cv2.moments(cnt);
        
        if M['m00']!=0:
          
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            
         
            tabcX[compteur]=cx
            tabcY[compteur]=cy
                
            
            compteur=compteur+1
            
            moycX=(int)(np.mean(tabcX))
            moycY=(int)(np.mean(tabcY))
            cv2.circle(imgOriginal,(moycX,moycY),1, (0, 0, 255), 4)
            
    return compteur, tabcX,tabcY
if __name__ == '__main__':
    
    n=5
    cap = cv2.VideoCapture(0)#Ouvre flux video webCam
    click=False #initialise booleen click gauche 
    x=-1
    y=-1
    h,s,v=-1,-1,-1
    par=[click,x,y,h,s,v]
    compteur=0
    tabcX=[0]*n
    tabcY=[0]*n
    
    
    while (True):
        # display the image and wait for a keypress
        
        #enregistre image video au temps t
        ret, frame = cap.read()
        if par[0]==False:            
            cv2.imshow('frame',frame)
        
        #appelle la fonction color en faisant attention aux commades de la souris. Renvoie les donnees contenues dans par    
        cv2.setMouseCallback("frame", color, par)
        
        if par[0]==True: #Si clic gauche detecte
            
            #Dessine un cercle autour du pixel selectionne
            cv2.circle(frame, (par[1],par[2]),1, (0, 255, 0), 2)
            
            #Appelle fonction track pour tracker couleur pixel selectionne
            compteur, tabcX, tabcY=track(frame,par[3],par[4],par[5],compteur,n,tabcX,tabcY)
            
           
            
            #Affiche image
            cv2.imshow('frame',frame)
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
           
          
   
    cap.release()
    cv2.destroyWindow("image")
