#################
#     LIBRARY   #
#################

import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math


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

seuil = 255								#Valeur initiale du seuil pour gray2bin
t1 = 0									#Compteur de temps
faire_seuil = True							#Autorisation du seuil
increment = 0								#Increment pour courbe
mat_courbe = np.zeros([180,500,3])					#Matrice pour la courbe

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

def input(code):
	if code == ord("q"):
		return True
	if code == ord("s"):
		global mat_courbe
		cv2.imwrite('Angle.png',mat_courbe)
	return False

#Mise a jour trackbar

def Maj_track1(s):
	global seuil
	seuil = s

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

#Trouve le seuil de conversion gray2bin

def auto_seuil(limite):
	global seuil
	img,img1,img_bin = filtre()
	moy = moy_img(img_bin)
	while (moy<limite):
		seuil -= 5
		if(seuil <= 0):
			seuil = 2
			print("Erreur : pas assez de lumiere\n")
			break
		img,img1,img_bin = filtre()
		moy = moy_img(img_bin)

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

def Arrive():
	cv2.putText(img_base,"ARRIVE !!",(30,200),font,3,vert,1,cv2.LINE_AA)
def Virage():
	cv2.putText(img_base,"VIRAGE",(30,200),font,3,vert,1,cv2.LINE_AA)

############################
#	PROG PRINCIPAL     #
############################

cv2.namedWindow('img_base') 								#Création fenétre et trackbar
cv2.namedWindow('img_bin')
cv2.moveWindow('img_bin',700,50)
cv2.moveWindow('img_base',0,0)

font = cv2.FONT_HERSHEY_SIMPLEX

cv2.createTrackbar('seuil', 'img_base',seuil, 255, Maj_track1)

print ("Start")

for frame in camera.capture_continuous(rawCapture, format ="bgr", use_video_port=True):

	
	periode = int((time.time() - t1) * 1000)					#Mesure de temps en 1 boucle
	t1 = time.time()
	#print("Periode = {}".format(periode))

	if (faire_seuil):								#Fait le seuil automatiquement une fois
		auto_seuil(40)
		#print("Seuil = {} \n".format(seuil))
		faire_seuil = False
		cv2.setTrackbarPos('seuil','img_base', seuil) 

	img_base, img_gray, img_bin = filtre()						#Acquisition image

	img,contours,hie = cv2.findContours(img_bin.copy(),1,cv2.CHAIN_APPROX_NONE)	#Trouve et affiche les contours
	cv2.drawContours(img_base,contours,-1,jaune,2)
	
	if len(contours)>0 :
                
		c2= max(contours,key=cv2.contourArea)					#Approximation des contours
		epsilon = (0.0001)*cv2.arcLength(c2,True)
		approx = cv2.approxPolyDP(c2,epsilon,True)
		
		rect_cnt=cv2.minAreaRect(c2)              #Croisement ou pas croisement (comparaison d'aire)
		area_rect = cv2.boxPoints(rect_cnt)
		area_rect = np.int0(area_rect)
		area_rect=cv2.contourArea(area_rect)
		area_cnt=cv2.contourArea(c2)
		if (area_rect<area_cnt*2):
                    print("LIGNE !!!")
		
		
		cv2.drawContours(img_base,[approx],-1,rose,3)
		cv2.drawContours(img_bin,[approx],-1,0,-1)

		if (area_rect>area_cnt*2):
                    print("Pas LIGNE !!!")

		img,pastilles,hie = cv2.findContours(img_bin.copy(),1,cv2.CHAIN_APPROX_NONE)		

		[x,y,vx,vy] = cv2.fitLine(c2,cv2.DIST_L2,0,0.01,0.01)			#Recherche et trace de la droite moyenne
		g = int((-vx*y/x) + vy)
		d = int(((640-vx)*y/x)+vy)
		cv2.line(img_base,(639,d),(0,g),rouge,5)	

		angle = int(math.degrees(math.atan((g-d)/639)))				#Déduction et affichage de l'angle
		if (angle<0):
			angle = 180 + angle
		texte = "Angle = " + str(angle)
		cv2.rectangle(img_base,(20,8), (220,40),0,-1,4,0)
		cv2.putText(img_base,texte,(30,30),font,0.9,blanc,1,cv2.LINE_AA)
		
		aff_courbe(angle)							#Afficher la courbe
			

		if len(pastilles) > 0:

			ct = max(pastilles,key=cv2.contourArea)				#Trouve & affiche les pastilles
			rect = cv2.minAreaRect(ct)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			_,_,wop,hop=cv2.boundingRect(box)
			Aspect_ratio=float(wop)/float(hop)
			if Aspect_ratio>1:
                            cv2.drawContours(img_base,[box],-1,bleu,2)				
                            M = cv2.moments(box)						#Trouve le centre des pastilles
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])
                            cv2.circle(img_base,(cx,cy),5,rouge,-1,0)
                            
                            m = (g-d)/-639							#Détection arrivé ou virage
                            dx = (cy-g)/m
                            if (cx > dx):
                                    Arrive()
                            else:
                                    Virage()


	cv2.imshow('img_base', img_base)						#Affichage imgs
	#cv2.imshow('img_gray', img_gray)
	cv2.imshow('img_bin', img_bin)

	key = cv2.waitKey(1)								#Control par clavier
	arret = input(key)
	if(arret == True):
		break

	rawCapture.truncate(0)












	
	