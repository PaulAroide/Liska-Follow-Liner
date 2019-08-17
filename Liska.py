#################
#     LIBRARY   #
#################

import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import RPi.GPIO as IO


#####################
#Initialize picamera#
#####################

camera = PiCamera()
camera.resolution =(640,480)
camera.framerate = 30
rawCapture = PiRGBArray(camera,size=(640,480))
time.sleep(1)

####################
#VARIABLES GLOBALES#
####################

seuil = 150								#Valeur initiale du seuil pour gray2bin
increment = 0								#Increment pour courbe
mat_courbe = np.zeros([180,500,3])					#Matrice pour la courbe

arret = False
Fin = False
Mem_Fin=False
Corresp_Aire_Ligne = True
Past = False
Virage = False

rouge = (0,0,255)							#Couleurs
vert = (0,255,0)
bleu = (255,0,0)
jaune = (0,255,255)
rose = (255,0,255)
blanc = (255,255,255)

##########
#FONCTION#
##########

#Filtre image

def filtre():
	img_base = frame.array								#Image de base
	img_gray = cv2.cvtColor(img_base,cv2.COLOR_BGR2GRAY)				#Conversion nuances de gris
	img_gray = cv2.GaussianBlur(img_gray.copy(),(5,5),0)				#Floutage
	ret,img_bin = cv2.threshold(img_gray,seuil,255,cv2.THRESH_BINARY)		#Conversion noir et blanc
	img_bin = cv2.erode(img_bin.copy(), None, iterations=3)				#Erosion de l'image
	img_bin = cv2.dilate(img_bin.copy(), None, iterations=2)

	return img_base,img_gray,img_bin

#Control clavier

def _input(code):
	if code == ord("q"):
		return True
	if code == ord("s"):
		global mat_courbe
		cv2.imwrite('Angle.png',mat_courbe)
	return False

#Moyenne d'une image bin

def moy_img(img):
	moy = 0
	nb = 0
	for i in range(0,480,5):
		for j in range(0,640,5):
			nb += 1
			moy += img[i,j]
	moy = round(moy/nb,0)

	return moy

#Affichage courbe

def aff_courbe(angle):
	global rouge
	global vert
	global increment
	global mat_courbe
	cv2.putText(mat_courbe,"180",(10,30),font,0.8,blanc,1,cv2.LINE_AA)
	cv2.putText(mat_courbe,"90",(15,90),font,0.8,blanc,1,cv2.LINE_AA)
	cv2.putText(mat_courbe,"0",(15,170),font,0.8,blanc,1,cv2.LINE_AA)
	for i in range(0,500,10):
		mat_courbe[90,i] = vert
	if (increment >= 500):
		increment = 0
		mat_courbe = np.zeros([180,500,3])
	if (angle > 0):
		angle = 180 - angle
	else:
		angle = -angle
	
	mat_courbe[angle,increment] = rouge
	increment += 1
	cv2.imshow('courbe angle',mat_courbe)

#Gestion Moteurs

def MotD(angle):
    global var_Y
    alpha=(angle/360.)*2*np.pi
    if angle<50 :
        res = 0
    elif angle<90 and angle>50:
        res = ((np.cos((alpha-np.pi/2)*4.5)+1)*0.05)/var_Y
    elif angle>=90 and angle<130:
        res = ((np.cos((alpha-(13*np.pi/18))*4.5)+1)*0.1+0.1)/var_Y
    else :
        res = 0.3/var_Y
    if res<0:
        res=-res
    return float(res*100)
        
def MotG(angle):
    Nangle=180-angle
    return MotD(Nangle)
    
    
############################
#	INIT MOTEUR		#
############################

IO.setwarnings(False)
IO.setmode(IO.BOARD)
IO.cleanup()
IO.setup(38, IO.OUT)
IO.setup(37, IO.OUT)

motorg = IO.PWM(38, 1000)
motord = IO.PWM(37, 1000)

i=0

motorg.start(i)
motord.start(i)

############################
#	PROG PRINCIPAL     #
############################

font = cv2.FONT_HERSHEY_SIMPLEX


print("Veuillez mettre le seuil (0-255) -> Programme de tests de seuil : 'Seuil.py'")
seuil = float(input())

for frame in camera.capture_continuous(rawCapture, format ="bgr", use_video_port=True):

	img_base, img_gray, img_bin = filtre()						#Acquisition image

	img,contours,hie = cv2.findContours(img_bin.copy(),1,cv2.CHAIN_APPROX_NONE)	#Trouve et affiche les contours
    
	if len(contours)>0 :

		c2= max(contours,key=cv2.contourArea)					#Approximation des contours
		epsilon = (0.0001)*cv2.arcLength(c2,True)
		approx = cv2.approxPolyDP(c2,epsilon,True)
		rect_cnt=cv2.minAreaRect(c2)              #début des calculs d'aire
		area_rect = cv2.boxPoints(rect_cnt)
		area_rect = np.int0(area_rect)
		area_rect=cv2.contourArea(area_rect)    #Valeur de l'aire du rectangle
		area_cnt=cv2.contourArea(c2)        #Valeur de l'aire de la ligne
        
        ### Comparaison d'aire : Permet de déterminer si il y a un croisement ###
		if (area_rect<area_cnt*2):
                    Corresp_Aire_Ligne = True
		
		if (area_rect>area_cnt*2):
                    Corresp_Aire_Ligne = False
		
		
		### Détermination de l'angle ###
		img,pastilles,hie = cv2.findContours(img_bin.copy(),1,cv2.CHAIN_APPROX_NONE)		

		[x,y,vx,vy] = cv2.fitLine(c2,cv2.DIST_L2,0,0.01,0.01)			#Recherche et trace de la droite moyenne
		g = int((-vx*y/x) + vy)
		d = int(((640-vx)*y/x)+vy)
		cv2.line(img_base,(639,d),(0,g),rouge,5)	

		angle = int((360*(float(np.arctan((g-d)/639)))/np.pi)/2)%180				#Déduction et affichage de l'angle
		cv2.rectangle(img_base,(20,8), (220,40),0,-1,4,0)
		
		aff_courbe(angle)							#Afficher la courbe
			
                ### Détection des Pastilles ###
		if len(pastilles) > 0:

			ct = max(pastilles,key=cv2.contourArea)				#Trouve & affiche les pastilles
			rect = cv2.minAreaRect(ct)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			_,_,wop,hop=cv2.boundingRect(box)
			Aspect_ratio=float(wop)/float(hop)
			if Aspect_ratio>1:                          		
                            M = cv2.moments(box)						#Trouve le centre des pastilles
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])
                            cv2.circle(img_base,(cx,cy),5,rouge,-1,0)
                            
                            m = (g-d)/-639							#Détection arrivé ou virage
                            dx = (cy-g)/m
                            if (cx > dx):
                                    Past = False
                                    Fin = True
                                    Mem_Fin = True
                            else:
                                    Past = True
                                    Fin = False
			else:
                            Past = False
                            Fin = False
            
        ### Détection d'état de virage ###
	if Past==False and Virage==True:
           Virage = False
	elif Past==True and Virage==False:
           Virage = True
            
	### Sécurité au cas où l'angle est négatif ###
	if (angle<0):
                angle = 180 + angle
                
        ### Détection des croisements ###
	if (Virage==False and Corresp_Aire_Ligne==False):
                angle = 90
                
        ### Commande Moteurs ### 
	motorg.ChangeDutyCycle(MotG(angle))
	motord.ChangeDutyCycle(MotD(angle))
	
    ### Fin de piste ###
    if Fin == False and Mem_Fin == True:
        arret=True
    
	### Mise en arret du programme ###
	key = cv2.waitKey(1)								#Control par clavier
	arret = _input(key)
	if(arret == True):
		break

	rawCapture.truncate(0)












	
	
