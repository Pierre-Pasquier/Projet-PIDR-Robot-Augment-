import time
import numpy as np
import cv2 as cv
import RPi.GPIO as GPIO
from thymiodirect import Thymio

# Constantes pour les broches GPIO
FORWARD_PIN = 26
BACKWARD_PIN = 20

# Constantes pour la détection de cercles
DISTANCE_THRESHOLD = 10000
MIN_RADIUS = 0
MAX_RADIUS = 70
HOUGH_PARAM_1 = 1
HOUGH_PARAM_2 = 1

# Initialisation pour electro-aimant
GPIO.setmode(GPIO.BCM)
GPIO.setup(FORWARD_PIN, GPIO.OUT)
GPIO.setup(BACKWARD_PIN, GPIO.OUT)

# Initialisation pour Thymio
th = Thymio(serial_port="/dev/ttyACM0", on_connect=lambda node_id: print(f"{node_id} is connected"))
th.connect()
node_id = th.first_node()
vitesse = 20
balle = 0

def move_forward(node_id):
    global vitesse
    th[node_id]["motor.left.target"] = vitesse
    th[node_id]["motor.right.target"] = vitesse

def turn_right(node_id):
    global vitesse
    th[node_id]["motor.left.target"] = vitesse
    th[node_id]["motor.right.target"] = -vitesse

def turn_left(node_id):
    global vitesse
    th[node_id]["motor.left.target"] = -vitesse
    th[node_id]["motor.right.target"] = vitesse

def grab_ball():
    global balle
    GPIO.output(FORWARD_PIN, GPIO.HIGH)
    print("Balle attrapée")
    time.sleep(1)
    GPIO.output(FORWARD_PIN, GPIO.LOW)
    balle = 1

def release_ball():
    global balle
    GPIO.output(FORWARD_PIN, GPIO.LOW)
    print("Balle relâchée")
    time.sleep(1)
    GPIO.output(FORWARD_PIN, GPIO.HIGH)
    balle = 0

class CircleDrawer(object):
    def init(self):
        self._center_thickness = 1
        self._center_color = (0, 0, 255)


def __call__(self, img, circles, color=(0, 255, 0), thickness=4, bbox=False):
    circle = np.round(circles[0]).astype("int")
    x, y, r = circle
    cv.circle(output, (x, y), r, color, thickness)
    cv.drawMarker(output, (x, y), color, thickness=thickness)
    if bbox:
        cv.rectangle(output, (x - r, y - r), (x + r, y + r), self._center_color, self._center_thickness)
class TextWriter(object):
    def init(self):
        self._font = cv.FONT_HERSHEY_SIMPLEX
        self._fontScale = 1
        self._fontColor = (0, 0, 255)
        self._lineType = 2


def __call__(self, img, text, pos=(40, 40)):
    cv.putText(img, text, pos, self._font, self._fontScale, self._fontColor, self._lineType)

    fps = FPS()
    writer = TextWriter()
    drawer = CircleDrawer()
    kernel = np.ones((3, 3), np.uint8)


    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Unable to read camera feed")
        sys.exit(0)


    width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)


    old_circle = np.zeros((1, 3))
    min_circle = (int(width / 2), int(height / 2), int(MIN_RADIUS))
    max_circle = (int(width / 2), int(height / 2), int(MAX_RADIUS))
    counter = 0
    work_freq = 1

    while True:
        counter += 1
        ret, frame = cap.read()
        if ret != True:
            continue

    output = frame.copy()

    # Prétraitement de l'image
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    gray = cv.GaussianBlur(gray, (5, 5), 0)
    gray = cv.medianBlur(gray, 5)
    gray = cv.erode(gray, kernel, iterations=1)
    gray = cv.dilate(gray, kernel, iterations=1)

    # Détection de cercles
    circles = cv.HoughCircles(
        gray,
        cv.HOUGH_GRADIENT,
        dp=1,
        minDist=MIN_DISTANCE,
        param1=PARAM1,
        param2=PARAM2,
        minRadius=min_circle[2],
        maxRadius=max_circle[2],
    )

    # Si des cercles sont détectés
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")

        # Filtrage des cercles similaires aux précédents
        if old_circle.shape[0] > 0:
            diff = np.abs(circles - old_circle)
            mask = np.sum(diff, axis=1) > MAX_DIFFERENCE
            circles = circles[mask, :]

        # Si des cercles sont toujours présents après filtrage
        if circles.shape[0] > 0:
            # Mise à jour des cercles précédents
            old_circle = circles

            # Sélection du cercle le plus grand
            idx = np.argmax(circles[:, 2])
            circle = circles[idx]

            # Récupération des coordonnées du cercle
            x, y, r = circle

            # Dessin du cercle et de la boîte englobante
            drawer(output, circle, color=(0, 255, 0), thickness=4, bbox=True)

            # Vérification si le cercle correspond à une balle
            if r > BALL_RADIUS_THRESHOLD:
                # Si la balle n'a pas encore été attrapée
                if balle == 0:
                    grab_ball()
            else:
                # Si la balle a été attrapée et le cercle est plus petit
                if balle == 1:
                    release_ball()

    # Affichage de l'image et du texte
    writer(output, f"Counter: {counter}")
    cv.imshow("Output", output)
    fps.update()

    # Sortie si la touche 'q' est pressée
    if cv.waitKey(1) & 0xFF == ord("q"):
        break

    fps.stop()
    cap.release()
    cv.destroyAllWindows()















