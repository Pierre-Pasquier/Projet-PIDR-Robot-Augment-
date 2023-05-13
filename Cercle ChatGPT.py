import cv2
import numpy as np

def detect_red_ball(image):
    # Convertir l'image en espace de couleur HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Définir les plages de couleur pour le rouge en HSV
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    upper_red2 = np.array([170, 100, 100])

    # Masquer les pixels rouges de l'image
    mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
    mask2 = cv2.inRange(hsv_image, upper_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Appliquer une opération de flou pour réduire le bruit
    blurred = cv2.GaussianBlur(mask, (9, 9), 2)

    # Détecter les cercles dans l'image floutée en utilisant la transformée de Hough
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=100, param1=50, param2=30, minRadius=10, maxRadius=100)

    if circles is not None:
        # Convertir les coordonnées et le rayon des cercles en entiers
        circles = np.round(circles[0, :]).astype("int")

        for (x, y, r) in circles:
            # Dessiner le cercle détecté sur l'image originale
            cv2.circle(image, (x, y), r, (0, 255, 0), 4)

        # Afficher l'image avec les cercles détectés
        cv2.imshow("Red Ball Detection", image)
    else:
        print("Aucun cercle rouge n'a été détecté.")

# Capture de la vidéo en direct à partir de la caméra
cap = cv2.VideoCapture(0)

while True:
    # Lecture de l'image de la caméra
    ret, frame = cap.read()

    # Vérifier si la capture de la vidéo est réussie
    if not ret:
        break

    # Détecter la balle rouge dans l'image
    detect_red_ball(frame)

    # Quitter la boucle si la touche 'q' est enfoncée
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer les ressources
cap.release()
cv2.destroyAllWindows()
