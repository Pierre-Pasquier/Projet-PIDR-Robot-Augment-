import sys, time, math
import RPi.GPIO as GPIO
from collections import deque
import thymiodirect
from thymiodirect import Connection
from thymiodirect import Thymio

import numpy as np

import cv2 as cv

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

vitesse = 20

if __name__ == "__main__":
    print("openCV version {}".format(cv.__version__))
    # print(cv.getBuildInformation())
    # Create a VideoCapture object
    cap = cv.VideoCapture(0)

    # Check if camera opened successfully
    if cap.isOpened() == False:
        print("Unable to read camera feed")
        sys.exit(0)


    while True:
        taille_j = 20
        taille_k = 20
        # read a frame
        ret, frame = cap.read()
        H = len(frame[0])
        V = len(frame)
        for j in range(1,taille_j):
            for k in range(1,taille_k):
                pix_rouge_bas = frame[(j//taille_j)*V-1][k*len(frame[0])//taille_k-1][2]
                pix_vert_bas = frame[(j//taille_j)*V-1][k*len(frame[0])//taille_k-1][1]
                pix_bleu_bas = frame[(j//taille_j)*V-1][k*len(frame[0])//taille_k-1][0]
                if pix_rouge_bas > 1.7 * pix_vert_bas and pix_rouge_bas > 1.7 * pix_bleu_bas : #and balle == 0 :  # valeurs à tester
                    x = (j//taille_j)*V-1
                    y = k*len(frame[0])//taille_k-1
                    break
                else :
                    x = None
                    y = None


        if x != None :
            if len(frame[0])*0.40 < x < len(frame[0])*0.60 : #and balle == 0 :  #valeurs à tester
                th.set_variable_observer(id, avancer)
                print("On avance, on avance, on avance, on avance,on avance,on avance,on avance,on avance,on avance,on avance ")

            #si balle à gauche
            elif x < len(frame[0])*0.40 : #and balle == 0 :
                th.set_variable_observer(id, tourner_gauche)
                print("à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche, à gauche")

            #si balle à droite
            elif x > len(frame[0])*0.60 : #and balle == 0 :
                th.set_variable_observer(id, tourner_droite)
                print("à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite, à droite")

        else :
            old_circle = np.zeros((1, 3))
            print(None, None, None)

            th.set_variable_observer(id, tourner_droite)
            print("Rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien")




