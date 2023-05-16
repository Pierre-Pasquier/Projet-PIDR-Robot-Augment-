import sys, time, math
import RPi.GPIO as GPIO
from collections import deque
import thymiodirect
from thymiodirect import Connection
from thymiodirect import Thymio
from time import sleep

import numpy as np

import cv2 as cv



class FPS(object):  # à voir si on garde
    def __init__(self, avarageof=50):
        self.frametimestamps = deque(maxlen=avarageof)

    def __call__(self):
        self.frametimestamps.append(time.time())
        if (len(self.frametimestamps) > 1):
            return len(self.frametimestamps) / \
                   (self.frametimestamps[-1] - self.frametimestamps[0])
        else:
            return 0.0


class TextWriter(object) :
    def __init__(self):
        self._font                   = cv.FONT_HERSHEY_SIMPLEX
        self._fontScale              = 1
        self._fontColor              = (0,0,255)
        self._lineType               = 2
    def __call__(self, img, text, pos=(40,40)) :
        cv.putText(img, text, pos,
                    self._font,
                    self._fontScale,
                    self._fontColor,
                    self._lineType)

class CircleDrawer(object) :
    def __init__(self):

        self._center_thickness = 1
        self._center_color = (0, 0, 255)

    def __call__(self, img, circles, color=(0,255,0), thickness=4, bbox=False) :

        circle = np.round(circles[0]).astype("int")

        x, y, r = circle
        cv.circle(output, (x, y), r, color, thickness)

        cv.drawMarker(  output, (x,y), color, thickness=thickness)


        if bbox :
            cv.rectangle(output, (x-r, y-r), (x+r, y+r),
                         self._center_color,
                         self._center_thickness)



def nothing(*arg):  # used as a call back for GUIq
    pass

def avancer(node_id):
    global vitesse_avance
    th[node_id]["motor.left.target"] = vitesse_avance
    th[node_id]["motor.right.target"] = vitesse_avance

def tourner_droite(node_id):
    global vitesse_tourne
    th[node_id]["motor.left.target"] = vitesse_tourne
    th[node_id]["motor.right.target"] = 0 #valeur potentiellement à modifier


def tourner_gauche(node_id):
    global vitesse_tourne
    th[node_id]["motor.left.target"] = 0 #valeur potentiellement à modifier
    th[node_id]["motor.right.target"] = vitesse_tourne

def on():
    GPIO.output(16, GPIO.HIGH)

def off():
    GPIO.output(16, GPIO.LOW)



def attraper_balle():
    global balle
    on()
    GPIO.output(26,GPIO.HIGH)
    print("Balle attrapée")
    time.sleep(1)   #à voir si on garde
    GPIO.output(26,GPIO.LOW)
    balle = 1
    off()
    #GPIO.cleanup()  à voir si on le laisse ici ou après chaque appel de fonction

def relacher_balle():
    global balle
    on()
    GPIO.output(20,GPIO.HIGH)
    print("Balle relachée")
    time.sleep(1)   #à voir si on garde
    GPIO.output(20,GPIO.LOW)
    balle = 0
    off()
    #GPIO.cleanup()  à voir si on le laisse ici ou après chaque appel de fonction




def retour(node_id):
    global SuiviLigne,murinf,mursup,noirsup,balle,grisinf,grissup,ligne_grise,DepartLigneNoire,Vitesse,start,active_cam,tourne

    print(th[node_id]["prox.ground.reflected"][0],th[node_id]["prox.ground.reflected"][1],active_cam)

    if tourne == 1 :
        print("aaaaaaaaaaaa",th[node_id]["prox.ground.reflected"][0],th[node_id]["prox.ground.reflected"][1],active_cam)
        if th[node_id]["prox.ground.reflected"][0] < th[node_id]["prox.ground.reflected"][1] :
            th[node_id]["motor.left.target"] = -200
            th[node_id]["motor.right.target"] = 200
        else :
            th[node_id]["motor.left.target"] = 200
            th[node_id]["motor.right.target"] = -200

        sleep(1.3)
        th[node_id]["motor.left.target"] = 200
        th[node_id]["motor.right.target"] = 200
        sleep(1)
        #on dépose la boule
        print("Appel recherche")
        tourne = 0

    if balle == 1 :

        th[node_id]["motor.left.target"]=Vitesse
        th[node_id]["motor.right.target"]=Vitesse


        #on gere les murs avec le onevent et la couleur aus sol entre 400 et 600
        if th[node_id]["prox.ground.reflected"][0]> murinf and  th[node_id]["prox.ground.reflected"][0]< mursup :
                th[node_id]["motor.left.target"]= Vitesse*3


        if th[node_id]["prox.ground.reflected"][1]> murinf and  th[node_id]["prox.ground.reflected"][1]< mursup :
                th[node_id]["motor.right.target"]=Vitesse*3

        

        #on a trouvé une ligne noire
        if th[node_id]["prox.ground.reflected"][0]< noirsup or th[node_id]["prox.ground.reflected"][1]< noirsup :
            DepartLigneNoire += 1
            if th[node_id]["prox.ground.reflected"][0]< noirsup and th[node_id]["prox.ground.reflected"][1]< noirsup:
                SuiviLigne=1
                if DepartLigneNoire == 5 and SuiviLigne == 0 :
                    th[node_id]["motor.left.target"] = 30
                    th[node_id]["motor.right.target"] = -30
                    print("aaaaaaaaaaaaaa")
                    sleep(3)

            #print(th[node_id]["prox.ground.reflected"][0])
            #moyen de savoir qu'on viens d'une ligne

            #on dévie à gauche
            if th[node_id]["prox.ground.reflected"][0]> noirsup :
                th[node_id]["motor.right.target"] = 0

            #on dévie à droite
            elif th[node_id]["prox.ground.reflected"][1]> noirsup :
                th[node_id]["motor.left.target"] = 0
                
                
        if th[node_id]["prox.ground.reflected"][0] > murinf and th[node_id]["prox.ground.reflected"][1] > murinf and SuiviLigne == 1 :
            th[node_id]["motor.right.target"]= -150
            sleep(1.3)
            th[node_id]["motor.right.target"]=Vitesse
            SuiviLigne = 0
            
            
        #on est arrivé au bout de la ligne
        if th[node_id]["prox.ground.reflected"][0] < grissup and th[node_id]["prox.ground.reflected"][1]<grissup and th[node_id]["prox.ground.reflected"][0]>grisinf and th[node_id]["prox.ground.reflected"][1]>grisinf and SuiviLigne==1:
            SuiviLigne=0
            DepartLigneNoire = 0
            th[node_id]["motor.left.target"] = 0
            th[node_id]["motor.right.target"] = 0
            print("On pose la balle")
            relacher_balle()
            sleep(2)
            balle = 0
            th[node_id]["motor.left.target"] = 60
            th[node_id]["motor.right.target"] = 60
            sleep(1)
            th[node_id]["motor.left.target"] = 0
            th[node_id]["motor.right.target"] = 0
            sleep(1)
            tourne = 1

            #on repassera en mode random pour ne plus prendre en compte les lignes noires au sol et chercher les lignes grises

    if balle == 0 :
        if start == 0 :
            start = time.time()
        print("Dans recherche")
        print(ligne_grise,start-time.time())

        th[node_id]["motor.left.target"]=Vitesse
        th[node_id]["motor.right.target"]=Vitesse

        #on gere les murs avec le onevent et la couleur aus sol entre 400 et 600
        if th[node_id]["prox.ground.reflected"][0]> murinf and  th[node_id]["prox.ground.reflected"][0]< mursup :
                th[node_id]["motor.left.target"] = Vitesse*3

        if th[node_id]["prox.ground.reflected"][1]> murinf and  th[node_id]["prox.ground.reflected"][1]< mursup :
                th[node_id]["motor.right.target"] = Vitesse*3




        #on a trouvé une ligne grise
        if (th[node_id]["prox.ground.reflected"][0]> grisinf  and th[node_id]["prox.ground.reflected"][0]< grissup)  or (th[node_id]["prox.ground.reflected"][1]> grisinf  and th[node_id]["prox.ground.reflected"][1]< grissup) :
            ligne_grise = 1
            #si on a du blanc on test juste d'accélerer un peu du cote opposé pour voir si il chope rapidement du blanc : si oui alors on est au bout de la ligne gris, sinon c'est juste une simple deviation légère
            if (th[node_id]["prox.ground.reflected"][0]> grisinf  and th[node_id]["prox.ground.reflected"][0]<grissup) and (th[node_id]["prox.ground.reflected"][1]< grisinf  or  th[node_id]["prox.ground.reflected"][1]> grissup):
                th[node_id]["motor.left.target"] = 0


            elif (th[node_id]["prox.ground.reflected"][1]> grisinf  and  th[node_id]["prox.ground.reflected"][1]< grissup) and (th[node_id]["prox.ground.reflected"][0]< grisinf  or th[node_id]["prox.ground.reflected"][0]> grissup) :
                th[node_id]["motor.right.target"] = 0



            #si on a du blanc on test juste d'accélerer un peu du cote opposé pour voir si il chope rapidement du blanc : si oui alors on est au bout de la ligne gris, sinon c'est juste une simple deviation légère
        elif th[node_id]["prox.ground.reflected"][0]<grisinf and ligne_grise == 1 :
            th[node_id]["motor.left.target"] = Vitesse
            sleep(1)
            #si les deux sont dans le noir alors ça veut dire qu'on est arrive au bout de la ligne grise
            if th[node_id]["prox.ground.reflected"][0]<grisinf and th[node_id]["prox.ground.reflected"][1]<grisinf :
                print("ddddddddddddddddddddddddddddddddddddddddddddddddd")
                if time.time() - start > 5 :#on detecte pas de balle :
                    print("On tourne")
                    active_cam = 1
            else :
                th[node_id]["motor.right.target"] = Vitesse*2

            #on se dirige vers la boule et une fois qu’on l’a on passe en mode recherche de ligne noires


        #si on a du blanc on test juste d'accélerer un peu du cote opposé pour voir si il chope rapidement du blanc : si oui alors on est au bout de la ligne gris, sinon c'est juste une simple deviation légère
        elif th[node_id]["prox.ground.reflected"][1]<grisinf and ligne_grise == 1:
            th[node_id]["motor.right.target"] = Vitesse
            sleep(1)
            #si les deux sont dans le noir alors ça veut dire qu'on est arrive au bout de la ligne grise
            if th[node_id]["prox.ground.reflected"][0]<grisinf and th[node_id]["prox.ground.reflected"][1]<grisinf :
                    if time.time() - start > 5000 :#on detecte pas de balle :
                        print("On tourne")
                        active_cam = 1

            else :
                th[node_id]["motor.left.target"] =Vitesse*2

            #on se dirige vers la boule et une fois qu’on l’a on passe en mode recherche de ligne noires



def detect_circles(frame):
    global balle_detectee,balle,active_cam
    # Conversion en niveaux de gris
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Réduction du bruit avec un flou gaussien
    blur = cv.GaussianBlur(gray, (5, 5), 0)

    # Détection des cercles avec la transformée de Hough
    circles = cv.HoughCircles(blur, cv.HOUGH_GRADIENT, dp=1, minDist=1000000, param1=50, param2=30, minRadius=0, maxRadius=100)

    facteur_couleur = 0.1
            # compare to the previoussly detected circle
    if circles is not None:
        # Conversion des coordonnées et rayons en entiers
        circles = np.round(circles[0, :]).astype("int")

        # Dessiner les cercles détectés sur l'image
        for (x, y, r) in circles:
            cv.circle(frame, (x, y), r, (0, 255, 0), 4)


        balle_detectee = 1
        print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        circles_det = circles
        print(circles_det[0][0], circles_det[0][1], circles_det[0][2])  # x,y,r
        pix_rouge = frame[int(circles_det[0][1])-1][int(circles_det[0][0])-1][2]
        pix_vert = frame[int(circles_det[0][1])-1][int(circles_det[0][0])-1][1]
        pix_bleu = frame[int(circles_det[0][1])-1][int(circles_det[0][0])-1][0]
        if  pix_rouge > 1.5 * pix_vert and pix_rouge > 1.5 * pix_bleu:
            x = circles_det[0][0]
            y = circles_det[0][1]
            r = circles_det[0][2]
#Si balle droit devant
            if len(frame[0])*0.40 < x < len(frame[0])*0.60 :  #valeurs à tester
                th.set_variable_observer(id, avancer)
                print("On avance, on avance, on avance, on avance,on avance,on avance,on avance,on avance,on avance,on avance ")
#si balle à gauche
            elif x < len(frame[0])*0.40 :
                th.set_variable_observer(id, tourner_gauche)
                print("à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche")

    #si balle à droite
            elif x > len(frame[0])*0.60 :
                th.set_variable_observer(id, tourner_droite)
                print("à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite")

            elif balle_detectee == 0 :
                old_circle = np.zeros((1, 3))
                print(None, None, None)
                th.set_variable_observer(id, tourner_droite)
                print("Rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien")
#Si balle dans zone collecteur
        partition = 15
        for k in range(1,partition):
            pix_rouge_bas = frame[len(frame)-30][k*len(frame[0])//partition-1][2]
            pix_vert_bas = frame[len(frame)-30][k*len(frame[0])//partition-1][1]
            pix_bleu_bas = frame[len(frame)-30][k*len(frame[0])//partition-1][0]
            if pix_rouge_bas > 1.7 * pix_vert_bas and pix_rouge_bas > 1.7 * pix_bleu_bas : #and balle == 0 :  # valeurs à tester
                attraper_balle()
                break
    elif balle_detectee == 0 :
        th.set_variable_observer(id, tourner_droite)
        active_cam = 1

    return frame


# initialisation pour electro-aimant

mode=GPIO.getmode()
GPIO.cleanup()








Forward=26
Backward=20
Switch=16
GPIO.setmode(GPIO.BCM)
GPIO.setup(Forward, GPIO.OUT)
GPIO.setup(Backward, GPIO.OUT)
GPIO.setup(Switch, GPIO.OUT)

SuiviLigne=0
Vitesse=100
murinf = 700
mursup = 800
noirsup = 200
grisinf = 300
grissup =400
ligne_grise = 0
DepartLigneNoire = 0

# initialisation pour thymio
port = "/dev/ttyACM0"

th = Thymio(serial_port=port, on_connect=lambda node_id:print(f"{node_id} is connected"))

th.connect()

id = th.first_node()

vitesse_avance = 150
vitesse_tourne = 75

balle = 1




# Capture vidéo en direct
video_capture = cv.VideoCapture(0)

while True:
    active_cam = 0
    balle_detectee = 0
    start = 0
    tourne = 0


    while balle == 0:
        # Capture de chaque frame de la vidéo
        ret, frame = video_capture.read()

        # Vérification si la capture est réussie
        if not ret:
            break

        # Détection des cercles dans la frame
        output_frame = detect_circles(frame)


        cv.imshow('frame', frame)

        # Press Q on keyboard to stop
        if cv.waitKey(1) & 0xFF == ord('q'):
            break


    if balle == 1 :

        while not active_cam :
            th.set_variable_observer(id,retour)
        print("bbbbbbb")







