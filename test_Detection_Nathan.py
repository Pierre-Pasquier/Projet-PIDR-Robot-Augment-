import cv2
import numpy as np

# Fonction de détection de cercles dans une image
def detect_circles(frame):
    # Conversion en niveaux de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Réduction du bruit avec un flou gaussien
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Détection des cercles avec la transformée de Hough
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, dp=1, minDist=1000000, param1=50, param2=30, minRadius=0, maxRadius=100)

    if circles is not None:
        # Conversion des coordonnées et rayons en entiers
        circles = np.round(circles[0, :]).astype("int")

        # Dessiner les cercles détectés sur l'image
        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)

    return frame

# Capture vidéo en direct
video_capture = cv2.VideoCapture(0)

while True:
    # Capture de chaque frame de la vidéo
    ret, frame = video_capture.read()

    # Vérification si la capture est réussie
    if not ret:
        break

    # Détection des cercles dans la frame
    output_frame = detect_circles(frame)

    # Affichage de la frame résultante
    cv2.imshow("Circles Detection", output_frame)

    # Quitter la boucle si la touche 'q' est pressée
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer les ressources
video_capture.release()
cv2.destroyAllWindows()