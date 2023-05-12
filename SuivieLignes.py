#dans l’ensemble du code je met la valeur 200 pour prox.ground.reflected[1] et [2] quand je cherche une ligne noire mais ça va dépendre de la valeur du gris genre on aura noir 0-200 -> gris 300-500 et blanc >600 et ça c’est par le teste qu’on le saura
#il faut voir si 1000 pour la distance aux murs est cohérent vis à vis de la proximité

#on a pour valeur de gris: 0-200 = noir / 200-400 = gris / 400-600 = gris clair / >600 = blanc

#quand on a la boule : 
from codecs import BOM_UTF16_LE
from tempfile import tempdir
from time import sleep
import thymiodirect
from thymiodirect import *

port="/dev/ttyACN0"

th = Thymio(serial_port=port, on_connect=lambda node_id:print(f"{node_id} is connected"))

th.connect()

node_id=th.first_node()


SuiviLigne=0
Vitesse=100


def test(node_id):


	if #la position du récepteur à boule est basse alors:
		th[node_id]["motor.left.target"]=Vitesse
		th[node_id]["motor.right.target"]=Vitesse


		#on gere les murs avec le onevent et la couleur aus sol entre 400 et 600
		if th[node_id]["prox.ground.reflected"][0]> 400 and  th[node_id]["prox.ground.reflected"][0]< 600 :
				th[node_id]["motor.left.target"]= Vitesse
				sleep(2000)

		if th[node_id]["prox.ground.reflected"][1]> 400 and  th[node_id]["prox.ground.reflected"][1]< 600 :
				th[node_id]["motor.right.target"]=Vitesse
				sleep(2000)
		


		#on a trouvé une ligne noire
		while th[node_id]["prox.ground.reflected"][0]< 200 or th[node_id]["prox.ground.reflected"][1]< 200 :

			#moyen de savoir qu'on viens d'une ligne 
			SuiviLigne=1

			#on dévie à gauche
			if th[node_id]["prox.ground.reflected"][0]> 200 :
				th[node_id]["motor.left.target"] = Vitesse*2
		
			#on dévie à droite
			elif th[node_id]["prox.ground.reflected"][1]> 200 :
				th[node_id]["motor.right.target"] = Vitesse*2

		#on est arrivé au bout de la ligne 
		if th[node_id]["prox.ground.reflected"][0]> 200 and th[node_id]["prox.ground.reflected"][1]>200 and SuiviLigne==1:
			SuiviLigne=0
			#on dépose la boule
			#on repassera en mode random pour ne plus prendre en compte les lignes noires au sol et chercher les lignes grises
			




	#quand ou a pas la boule :

	if #la position du récepteur à boule est haute alors:
		th[node_id]["motor.left.target"]=300
		th[node_id]["motor.right.target"]=300

		#on gere les murs avec le onevent et la couleur aus sol entre 400 et 600
		if th[node_id]["prox.ground.reflected"][0]> 400 and  th[node_id]["prox.ground.reflected"][0]< 600 :
				th[node_id]["motor.left.target"] = Vitesse*2
				sleep(2000)

		if th[node_id]["prox.ground.reflected"][1]> 400 and  th[node_id]["prox.ground.reflected"][1]< 600 :
				th[node_id]["motor.right.target"] = Vitesse*2
				sleep(2000)
			



		#on a trouvé une ligne grise
		while th[node_id]["prox.ground.reflected"][0]> 200  and th[node_id]["prox.ground.reflected"][0]< 400 or th[node_id]["prox.ground.reflected"][1]> 200  and th[node_id]["prox.ground.reflected"][1]< 400:

			#si on a du blanc on test juste d'accélerer un peu du cote opposé pour voir si il chope rapidement du blanc : si oui alors on est au bout de la ligne gris, sinon c'est juste une simple deviation légère
			if th[node_id]["prox.ground.reflected"][0]> 200  and th[node_id]["prox.ground.reflected"][0]<400 and th[node_id]["prox.ground.reflected"][1]< 200  or  th[node_id]["prox.ground.reflected"][1]> 400:       
				th[node_id]["motor.left.target"] = Vitesse*2


			elif th[node_id]["prox.ground.reflected"][1]> 200  and  th[node_id]["prox.ground.reflected"][1]< 400 and th[node_id]["prox.ground.reflected"][0]< 200  or th[node_id]["prox.ground.reflected"][0]> 400 :
				th[node_id]["motor.right.target"] = Vitesse*2



			#si on a du blanc on test juste d'accélerer un peu du cote opposé pour voir si il chope rapidement du blanc : si oui alors on est au bout de la ligne gris, sinon c'est juste une simple deviation légère
			elif th[node_id]["prox.ground.reflected"][0]<200 :
				th[node_id]["motor.left.target"] = Vitesse*2
				sleep(1000)
				#si les deux sont dans le noir alors ça veut dire qu'on est arrive au bout de la ligne grise
				if th[node_id]["prox.ground.reflected"][0]<200 and th[node_id]["prox.ground.reflected"][1]<200 :
					while #on detecte pas de balle :
						th[node_id]["motor.left.target"] = Vitesse
						th[node_id]["motor.right.target"] = Vitesse
				else :
					th[node_id]["motor.right.target"] = Vitesse*2
					sleep(2000)
				#on se dirige vers la boule et une fois qu’on l’a on passe en mode recherche de ligne noires


			#si on a du blanc on test juste d'accélerer un peu du cote opposé pour voir si il chope rapidement du blanc : si oui alors on est au bout de la ligne gris, sinon c'est juste une simple deviation légère
			elif th[node_id]["prox.ground.reflected"][1]<200 :
				th[node_id]["motor.right.target"] = Vitesse*2
				sleep(1000)
				#si les deux sont dans le noir alors ça veut dire qu'on est arrive au bout de la ligne grise
				if th[node_id]["prox.ground.reflected"][0]<200 and th[node_id]["prox.ground.reflected"][1]<200 :
						th[node_id]["motor.left.target"] =Vitesse
						th[node_id]["motor.right.target"] = Vitesse
				else :
					th[node_id]["motor.left.target"] =Vitesse*2
					sleep(2000)
				#on se dirige vers la boule et une fois qu’on l’a on passe en mode recherche de ligne noires

th.set_variable_observer(node_id,test)