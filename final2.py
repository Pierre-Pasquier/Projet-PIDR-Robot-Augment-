import sys, time, math
import RPi.GPIO as GPIO
from collections import deque
import thymiodirect
from thymiodirect import Connection
from thymiodirect import Thymio

import numpy as np

import cv as cv

global balle




def nothing(*arg):  # used as a call back for GUIq
    pass

def avancer(node_id):
    global vitesse
    th[node_id]["motor.left.target"] = vitesse
    th[node_id]["motor.right.target"] = vitesse

def tourner_droite(node_id):
    global vitesse
    th[node_id]["motor.left.target"] = vitesse
    th[node_id]["motor.right.target"] = -vitesse #valeur potentiellement à modifier


def tourner_gauche(node_id):
    global vitesse
    th[node_id]["motor.left.target"] = -vitesse #valeur potentiellement à modifier
    th[node_id]["motor.right.target"] = vitesse

def attraper_balle():
    global balle
    GPIO.output(Forward,GPIO.HIGH)
    print("Balle attrapée")
    time.sleep(1)   #à voir si on garde
    GPIO.output(Forward,GPIO.LOW)
    GPIO.cleanup()  #à voir si on le laisse ici ou après chaque appel de fonction
    balle = 1

def relacher_balle():
    global balle
    GPIO.output(Forward,GPIO.LOW)
    print("Balle attrapée")
    time.sleep(1)   #à voir si on garde
    GPIO.output(Forward,GPIO.HIGH)
    GPIO.cleanup()  #à voir si on le laisse ici ou après chaque appel de fonction
    balle = 0


# initialisation pour electro-aimant

mode = GPIO.getmode()

GPIO.cleanup()

Forward = 26
Backward = 20

GPIO.setmode(GPIO.BCM)
GPIO.setup(Forward, GPIO.OUT)
GPIO.setup(Backward, GPIO.OUT)


# initialisation pour thymio
port = "/dev/ttyACM0"

th = Thymio(serial_port=port, on_connect=lambda node_id:print(f"{node_id} is connected"))

th.connect()

id = th.first_node()


def detect_red_ball(image):
    # Convertir l'image en espace de couleur HSV
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # Définir les plages de couleur pour le rouge en HSV
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    upper_red2 = np.array([170, 100, 100])

    # Masquer les pixels rouges de l'image
    mask1 = cv.inRange(hsv_image, lower_red, upper_red)
    mask2 = cv.inRange(hsv_image, upper_red2, upper_red2)
    mask = cv.bitwise_or(mask1, mask2)

    # Appliquer une opération de flou pour réduire le bruit
    blurred = cv.GaussianBlur(mask, (9, 9), 2)

    # Détecter les cercles dans l'image floutée en utilisant la transformée de Hough
    circles = cv.HoughCircles(blurred, cv.HOUGH_GRADIENT, dp=1, minDist=100, param1=50, param2=30, minRadius=10, maxRadius=100)

    if circles is not None:
        # Convertir les coordonnées et le rayon des cercles en entiers
        circles = np.round(circles[0, :]).astype("int")

        for (x, y, r) in circles:
            # Dessiner le cercle détecté sur l'image originale
            cv.circle(image, (x, y), r, (0, 255, 0), 4)
            partition = 15
            for k in range(1,partition):
                pix_rouge_bas = image[len(image)-5][k*len(image[0])//partition-1][2]
                pix_vert_bas = image[len(image)-5][k*len(image[0])//partition-1][1]
                pix_bleu_bas = image[len(image)-5][k*len(image[0])//partition-1][0]
                if pix_rouge_bas > 1.7 * pix_vert_bas and pix_rouge_bas > 1.7 * pix_bleu_bas : #and balle == 0 :  # valeurs à tester
                    attraper_balle()
                    #balle = 1
                    break

            #Si balle droit devant
            if len(image[0])*0.40 < x < len(image[0])*0.60 : #and balle == 0 :  #valeurs à tester
                th.set_variable_observer(id, avancer)
                print("On avance, on avance, on avance, on avance,on avance,on avance,on avance,on avance,on avance,on avance ")

            #si balle à gauche
            elif x < len(image[0])*0.40 : #and balle == 0 :
                th.set_variable_observer(id, tourner_gauche)
                print("à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche")

            #si balle à droite
            elif x > len(image[0])*0.60 : #and balle == 0 :
                th.set_variable_observer(id, tourner_droite)
                print("à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite")


        # Afficher l'image avec les cercles détectés
        cv.imshow("Red Ball Detection", image)
    else:
        print("Aucun cercle rouge n'a été détecté.")
        th.set_variable_observer(id, tourner_droite)
        print("Rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien")

# Capture de la vidéo en direct à partir de la caméra
cap = cv.VideoCapture(0)

while True:
    # Lecture de l'image de la caméra
    ret, image = cap.read()
    print(ret,cap)

    # Vérifier si la capture de la vidéo est réussie
    if not ret:
        break

    # Détecter la balle rouge dans l'image
    detect_red_ball(image)

    # Quitter la boucle si la touche 'q' est enfoncée
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer les ressources
cap.release()
cv.destroyAllWindows()




