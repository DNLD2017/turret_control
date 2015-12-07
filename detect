import cv2

import numpy as np


#S'il y a des mouvements detectes, dessine un rectangle autour de la plus grande surface en mouvement
#Return centre de cette surface
def searchForMovement(videoTresh,video):
    objectDetected=False;
   
    imCont, contours, hierarchy=cv2.findContours(videoTresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);
    
    
    if (len(contours)>0):
        objectDetected=True;
    
    else :
        objectDetected=False;
    

    
    if objectDetected :
        #On prend plus gros contour en supposant que c'est probablement l'objet cherche (derniere position dans la liste contour)
        
        cnt = contours[len(contours)-1];
        
        x,y,w,h = cv2.boundingRect(cnt);
        cv2.rectangle(video,(x,y),(x+w,y+h),(0,255,0),2);
        
        #cv2.imshow("Frame1",video);
        
        #trouve le centre de la zone encadree
        M = cv2.moments(cnt);
        if M['m00']!=0:
          
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
    
    
            

if __name__=='__main__': 
    sensity=30;
    blur=20;
    capture=cv2.VideoCapture(0);


    while(True):
        #capture image webcam au temps t
        ret,frame1=capture.read();
    
        #transforme capture en niveaux de gris
        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    
    
        #idem avec image t+dt
        ret,frame2=capture.read();
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    
        #Soustrait image t+dt - image t    
        diffIm=cv2.absdiff(gray1,gray2)
    
        #supprime niveau de gris, retourne image binaire noire/blanche
        retTresh, threshIm=cv2.threshold(diffIm,sensity,255,cv2.THRESH_BINARY)
    
        #Affiche capture image non traitee
        #cv2.imshow('frame',frame1)
    
        #Affiche soustraction des 2 images (diffIm)
        #cv2.imshow('diffIm',diffIm)
    
        #Affiche image binaire
        #cv2.imshow('threshim',threshIm)
    
        #Floute l'image pour supprimer bruit
        blurIm=cv2.blur(threshIm,(blur,blur));
    
        #supprime niveau de gris, retourne image binaire noire/blanche
        retTreshBlur, threshImBlur=cv2.threshold(blurIm,sensity,255,cv2.THRESH_BINARY)
    
        #Affiche image floutee binaire
        #cv2.imshow('threshimBlur',threshImBlur)
        #Pour quitter appuyer sur 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
        #Detection mouvement
        searchForMovement(threshImBlur,frame1)
        cv2.imshow("Frame1",frame1);
    
    capture.release()
    cv2.destroyAllWindows()
