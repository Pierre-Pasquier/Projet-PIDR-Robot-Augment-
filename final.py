import sys, time, math
import RPi.GPIO as GPIO
from collections import deque
import thymiodirect
from thymiodirect import Connection
from thymiodirect import Thymio

import numpy as np

import cv2 as cv

global balle

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


    kernel = np.ones((3, 3), np.uint8)  # used below in erosion/dilatation

    while True:
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
                if n_diff > dmax ** 2 and pix_rouge > 1.7 * pix_vert and pix_rouge > 1.7 * pix_bleu:
                    old_circle = circles_det

                x = circles_det[0][0]
                y = circles_det[0][1]
                r = circles_det[0][2]



                #Si balle dans zone collecteur
                partition = 15
                for k in range(1,partition):
                    pix_rouge_bas = frame[len(frame)-5][k*len(frame[0])//partition-1][2]
                    pix_vert_bas = frame[len(frame)-5][k*len(frame[0])//partition-1][1]
                    pix_bleu_bas = frame[len(frame)-5][k*len(frame[0])//partition-1][0]
                    if pix_rouge_bas > 1.7 * pix_vert_bas and pix_rouge_bas > 1.7 * pix_bleu_bas : #and balle == 0 :  # valeurs à tester
                        #attraper_balle()
                        #balle = 1
                        break

                #Si balle droit devant
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

            else:
                old_circle = np.zeros((1, 3))
                print(None, None, None)

                th.set_variable_observer(id, tourner_droite)
                print("Rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien, rien")
        drawer(output, old_circle)

        writer(output, "Fps={:06.2f}".format(fps()))
        cv.imshow('frame', output)

        # Press Q on keyboard to stop
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()

    # Closes all the frames
    cv.destroyAllWindows()




