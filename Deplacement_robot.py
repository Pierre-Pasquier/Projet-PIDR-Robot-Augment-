import thymiodirect
from thymiodirect import Connection
from thymiodirect import Thymio


port = "COM6"

th = Thymio(serial_port=port,
            on_connect=lambda node_id:print(f"{node_id} is connected"))

th.connect()

id = th.first_node()


prox_prev = 0
def obs(node_id):
    global prox_prev
    prox = (th[node_id]["prox.horizontal"][5]
            - th[node_id]["prox.horizontal"][2]) // 10
    if prox != prox_prev:
        th[node_id]["motor.left.target"] = prox
        th[node_id]["motor.right.target"] = prox
        print(prox)
        if prox > 5:
            th[id]["leds.top"] = [0, 32, 0]
        elif prox < -5:
            th[id]["leds.top"] = [32, 32, 0]
        elif abs(prox) < 3:
            th[id]["leds.top"] = [0, 0, 32]
        prox_prev = prox

th.set_variable_observer(id, obs)
th.disconnect()







