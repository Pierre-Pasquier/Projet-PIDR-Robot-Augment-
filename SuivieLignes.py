import sys, time, math
import RPi.GPIO as GPIO
from collections import deque
import thymiodirect
from thymiodirect import Connection
from thymiodirect import Thymio

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
    GPIO.output(Switch, GPIO.HIGH)
def off():
    GPIO.output(Switch, GPIO.LOW)



def attraper_balle():
    global balle
    on()
    GPIO.output(Forward,GPIO.HIGH)
    print("Balle attrapée")
    time.sleep(1)   #à voir si on garde
    GPIO.output(Forward,GPIO.LOW)
    balle = 1
    #GPIO.cleanup()  #à voir si on le laisse ici ou après chaque appel de fonction
    off()

def relacher_balle():
    global balle
    on()
    GPIO.output(Forward,GPIO.LOW)
    print("Balle attrapée")
    time.sleep(1)   #à voir si on garde
    GPIO.output(Forward,GPIO.HIGH)
    #GPIO.cleanup()  #à voir si on le laisse ici ou après chaque appel de fonction
    balle = 0
    off()




def retour(node_id):
    global SuiviLigne,murinf,mursup,noirsup,balle,grisinf,grissup,ligne_grise,DepartLigneNoire
    print(th[node_id]["prox.ground.reflected"][0])

    th[node_id]["motor.left.target"]=Vitesse
    th[node_id]["motor.right.target"]=Vitesse


    #on gere les murs avec le onevent et la couleur aus sol entre 400 et 600
    if th[node_id]["prox.ground.reflected"][0]> murinf and  th[node_id]["prox.ground.reflected"][0]< mursup :
            th[node_id]["motor.left.target"]= Vitesse*3


    if th[node_id]["prox.ground.reflected"][1]> murinf and  th[node_id]["prox.ground.reflected"][1]< mursup :
            th[node_id]["motor.right.target"]=Vitesse*3




    #on a trouvé une ligne noire
    if th[node_id]["prox.ground.reflected"][0]< noirsup or th[node_id]["prox.ground.reflected"][1]< noirsup :
        if th[node_id]["prox.ground.reflected"][0]< noirsup and th[node_id]["prox.ground.reflected"][1]< noirsup :
            DepartLigneNoire += 1
            if DepartLigneNoire == 1:
                th[node_id]["motor.left.target"] = 30
                th[node_id]["motor.right.target"] = -30
                print("aaaaaaaaaaaaaa")
                sleep(3)

        print(th[node_id]["prox.ground.reflected"][0])
        #moyen de savoir qu'on viens d'une ligne
        SuiviLigne=1

        #on dévie à gauche
        if th[node_id]["prox.ground.reflected"][0]> noirsup :
            th[node_id]["motor.right.target"] = 0

        #on dévie à droite
        elif th[node_id]["prox.ground.reflected"][1]> noirsup :
            th[node_id]["motor.left.target"] = 0

    #on est arrivé au bout de la ligne
    if th[node_id]["prox.ground.reflected"][0]> murinf and th[node_id]["prox.ground.reflected"][1]>murinf and SuiviLigne==1:
        SuiviLigne=0
        DepartLigneNoire = 0
        print("On pose la balle")
        balle = 0
        th[node_id]["motor.left.target"] = 200
        th[node_id]["motor.right.target"] = -200

        sleep(1)
        #on dépose la boule
        th.set_variable_observer(node_id,recherche)
        #on repassera en mode random pour ne plus prendre en compte les lignes noires au sol et chercher les lignes grises




def recherche(node_id):
	#quand ou a pas la boule :
    global SuiviLigne,murinf,mursup,noirsup,balle,grisinf,grissup,ligne_grise,DepartLigneNoire,active_cam


    th[node_id]["motor.left.target"]=Vitesse
    th[node_id]["motor.right.target"]=Vitesse

    #on gere les murs avec le onevent et la couleur aus sol entre 400 et 600
    if th[node_id]["prox.ground.reflected"][0]> murinf and  th[node_id]["prox.ground.reflected"][0]< mursup :
            th[node_id]["motor.left.target"] = Vitesse*3

    if th[node_id]["prox.ground.reflected"][1]> murinf and  th[node_id]["prox.ground.reflected"][1]< mursup :
            th[node_id]["motor.right.target"] = Vitesse*3




    #on a trouvé une ligne grise
    if th[node_id]["prox.ground.reflected"][0]> grisinf  and th[node_id]["prox.ground.reflected"][0]< grissup  or th[node_id]["prox.ground.reflected"][1]> grisinf  and th[node_id]["prox.ground.reflected"][1]< grissup:
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
            if True :#on detecte pas de balle :
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
                active_cam = 1

        else :
            th[node_id]["motor.left.target"] =Vitesse*2

        #on se dirige vers la boule et une fois qu’on l’a on passe en mode recherche de ligne noires



# initialisation pour electro-aimant

mode=GPIO.getmode()
#GPIO.cleanup()
Forward=26
Backward=20
Switch=16

GPIO.setmode(GPIO.BCM)
GPIO.setup(Forward, GPIO.OUT)
GPIO.setup(Backward, GPIO.OUT)
GPIO.setup(Switch, GPIO.OUT)






# initialisation pour thymio
port = "/dev/ttyACM0"

th = Thymio(serial_port=port, on_connect=lambda node_id:print(f"{node_id} is connected"))

th.connect()

id = th.first_node()

vitesse_avance = 20
vitesse_tourne = 5

balle = 0

while True:
    active_cam = 0
    print("openCV version {}".format(cv.__version__))
    # print(cv.getBuildInformation())
    # Create a VideoCapture object
    cap = cv.VideoCapture(0)

    # Check if camera opened successfully
    if cap.isOpened() == False:
        print("Unable to read camera feed")
        sys.exit(0)

    width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)

    #
    # hough parameters
    #
    #
    diag = math.sqrt(width ** 2 + height ** 2)  # image diagonal
    dist = 10000  # distance between 2 hugh circles
    min_rad = 0  # min and max of considered circles
    max_rad = 70
    dmax = 0  # min distance between 2 circles
    dchange = 0  # max distance between old and new circle
    hough_param_1 = 1  # see https://docs.opencv.org/4.5.1/dd/d1a/group__imgproc__feature.html#ga47849c3be0d0406ad3ca45db65a25d2d
    hough_param_2 = 1

    # Main loop
    #
    #
    old_circle = np.zeros((1, 3))
    min_circle = np.zeros((1, 3))
    max_circle = np.zeros((1, 3))
    min_circle[0][0] = int(width / 2)
    min_circle[0][1] = int(height / 2)
    min_circle[0][2] = int(min_rad)
    max_circle[0][0] = int(width / 2)
    max_circle[0][1] = int(height / 2)
    max_circle[0][2] = int(max_rad)


    counter = 0
    work_freq = 1  # process every work_freq  frame

    fps = FPS()
    writer = TextWriter()
    drawer = CircleDrawer()

    kernel = np.ones((3, 3), np.uint8)  # used below in erosion/dilatation

    while balle == 0:
        counter += 1

        # read a frame
        ret, frame = cap.read()
        print(frame[0][0])
        if ret != True:
            continue  # if could not,  skip

        output = frame.copy()

        #
        # Preprocessing
        #
        #
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        gray = cv.GaussianBlur(gray, (5, 5), 0)
        gray = cv.medianBlur(gray, 5)
        gray = cv.adaptiveThreshold(gray, 255, \
                                    cv.ADAPTIVE_THRESH_GAUSSIAN_C, \
                                    cv.THRESH_BINARY, 11, 3.5)

        gray = cv.erode(gray, kernel, iterations=1)
        gray = cv.dilate(gray, kernel, iterations=1)

        #
        # hough
        #
        if counter % work_freq == 0:
            counter = 0

            # read GUI values
            hough_param_1 = max(hough_param_1, 1)
            hough_param_2 = max(hough_param_2, 1)

            # detect
            circles_det = cv.HoughCircles(gray, cv.HOUGH_GRADIENT,
                                          2,
                                          dist,
                                          param1=hough_param_1,
                                          param2=hough_param_2,
                                          minRadius=min_rad,
                                          maxRadius=max_rad)

            facteur_couleur = 0.1
            # compare to the previoussly detected circle
            if circles_det is not None:
                circles_det = circles_det[0]
                diff = (circles_det - old_circle) ** 2
                n_diff = np.sum(diff)
                print(circles_det[0][0], circles_det[0][1], circles_det[0][2])  # x,y,r
                pix_rouge = frame[int(circles_det[0][1])][int(circles_det[0][0])][2]
                pix_vert = frame[int(circles_det[0][1])][int(circles_det[0][0])][1]
                pix_bleu = frame[int(circles_det[0][1])][int(circles_det[0][0])][0]
                if n_diff > dmax ** 2  and pix_rouge > 1.5 * pix_vert and pix_rouge > 1.5 * pix_bleu:
                    old_circle = circles_det
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

                    else:
                        old_circle = np.zeros((1, 3))
                        print(None, None, None)
                        th.set_variable_observer(id, tourner_droite)
                        print("Rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien")
                else :
                    th.set_variable_observer(id, tourner_droite)

#Si balle dans zone collecteur
                partition = 15
                for k in range(1,partition):
                    pix_rouge_bas = frame[len(frame)-30][k*len(frame[0])//partition-1][2]
                    pix_vert_bas = frame[len(frame)-30][k*len(frame[0])//partition-1][1]
                    pix_bleu_bas = frame[len(frame)-30][k*len(frame[0])//partition-1][0]
                    if pix_rouge_bas > 1.7 * pix_vert_bas and pix_rouge_bas > 1.7 * pix_bleu_bas : #and balle == 0 :  # valeurs à tester
                        attraper_balle()
                        break
            else :
                th.set_variable_observer(id, tourner_droite)
                active_cam = 1
        drawer(output, old_circle)

        writer(output, "Fps={:06.2f}".format(fps()))
        cv.imshow('frame', output)

        # Press Q on keyboard to stop
        if cv.waitKey(1) & 0xFF == ord('q'):
            break


    if balle == 1 :

        while not active_cam :
            th.set_variable_observer(node_id,retour)







