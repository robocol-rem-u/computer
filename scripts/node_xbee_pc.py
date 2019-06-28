#!/usr/bin/env python
import rospy, socket, _thread, serial, threading, thread, time
from master_msgs.msg import connection, arm_Orders, traction_Orders
import numpy as np


#### CONSTANTS ####

HOST = '192.168.0.101'
PORT = 7071
DIRECCION_XBEE = "/dev/tty.usbserial-A100RG3E"
BAUD_RATE = 115200
rf_wifi_selector= 0 #1 significa WIFI, 2 significa RF
rf_wifi_selector_anterior =0

SEPARADOR_POSITIVO = "#"
SEPARADOR_NEGATIVO = "!"
lock =threading.Lock()



### Principal node ###
def node_xbee_pc():

    rospy.init_node('node_xbee_pc', anonymous=True)
    # Se suscribe a al topico de la informacion de la conexion sobre quien manda informacion
    rospy.Subscriber ('topic_connection', connection, connection_Callback)
    # Se suscribe a al topico que indica ordenes sobre los brazos
    rospy.Subscriber ('topic_arm_orders', arm_Orders, arm_Orders_Callback)
    # Se suscribe a al topico que indica ordenes sobre ruedas
    rospy.Subscriber ('topic_traction_orders', traction_Orders, traction_Orders_Callback)
    rate = rospy.Rate (10)

    enviarMensajeInicializacion()
    while not rospy.is_shutdown ():
        rate.sleep ()



### CALLBACKS of ROS topics ###

def connection_Callback(param):
    global order_connect, rf_wifi_selector, rf_wifi_selector_anterior
    rf_wifi_selector=param
    if rf_wifi_selector != rf_wifi_selector_anterior:
        cambiarEstado(rf_wifi_selector)
    rf_wifi_selector_anterior=rf_wifi_selector
    pass


def arm_Orders_Callback(param):
    pass



def traction_Orders_Callback(param):
    pass



### START XBEE CODE ###
if rf_wifi_selector==1:
    try:
        xBeeSerial = serial.Serial(DIRECCION_XBEE, baudrate=BAUD_RATE)
        print("\t \t -- Conexion con XBee S8 inicializada --")
    except:
        print("\033[1;31mNo se pudo inicializar la conexion con la XBee S8 \033[0;0m")
        thread.interrupt_main()
if rf_wifi_selector==2:
    s = socket.socket()
    s.settimeout(1)
    try:
        s.connect((HOST, PORT))
        s.send(str.encode("C+RF"))
    except:
        print("\033[1;31mError en conexion Wi-Fi \033[0;0m")
        _thread.interrupt_main()
    s.close()
if order_connect== 0:
    print("\033[1;31mNo estan activas todas las conexiones, encontrara funcionalidad limitada \033[0;0m")
print("\t \t -- Modo de control RF --")

### Mensaje de inicializacion ###
def enviarMensajeInicializacion():
    global EnviarMensaje
    while EnviarMensaje:
        nMsg = 1#3
        print("Enviando Inicializacion")
        transmitirMensaje("A"+str(nMsg)+"#I0#I1#I2#I3#I4#I5#")
        time.sleep(1)


#### METODO PARA TRANSMITIR MENSAJES WIFI - RF ####

def transmitirMensaje(mensaje):
    retornar = True
    lock.acquire()
    global rf_wifi_selector
    if rf_wifi_selector == 1:
        s = socket.socket()
        s.settimeout(1)
        try:
            s.connect((HOST, PORT))
            s.send((mensaje).encode())
        except:
            print("\033[1;31mError en conexion Wi-Fi \033[0;0m")
            retornar = False
        s.close()
    elif rf_wifi_selector == 2:
        try:
            xBeeSerial.write((mensaje).encode())
        except:
            print("\033[1;31mError en conexion con Xbee S8 \033[0;0m")
            retornar = False
    else:
        print("\033[1;31mError \033[0;0m")
    print("Comando transmitido:",mensaje,"Estado:",retornar)
    lock.release()
    return retornar



#### CAMBIAR TIPO DE COMUNICACION ####
def cambiarEstado(nueva_com):
    global modo
    if nueva_com == 1:
        modo = "C+WIFI"
    elif nueva_com==2:
        modo="C+RF"
    print("Modo de control:",modo)
    lock.acquire()
    s = socket.socket()
    s.settimeout(1)
    try:
        s.connect((HOST, PORT))
        s.send(str.encode(modo))
    except:
        print("\033[1;31mError en conexion Wi-Fi \033[0;0m")
    s.close()
    lock.release()


#### METODO PARA RECIBIR LAS ACCIONES DE UN JOYSTICK ####
# Requiere convertir x, y entre -1 y 1
# Requiere filtrar las acciones para no hacerlas muy seguido
def procesarJoystick(x, y, sensibilidad, sobreEscribirPWM=False, PWM_I=0, PWM_D=0):
    global EnviarMensaje, ultimo_izquierdo, ultimo_derecho

    if not sobreEscribirPWM:
        (calc_PWM_izq, calc_PWM_der) = steering(x, y, sensibilidad)
    else:
        (calc_PWM_izq, calc_PWM_der) = (int(PWM_I), int(PWM_D))

    StringIzquierda = ("L" + str(calc_PWM_izq) + SEPARADOR_NEGATIVO) if (calc_PWM_izq >= 0) else (
    "L" + str(-calc_PWM_izq) + SEPARADOR_POSITIVO)
    StringDerecha = ("R" + str(calc_PWM_der) + SEPARADOR_NEGATIVO) if (calc_PWM_der >= 0) else (
    "R" + str(-calc_PWM_der) + SEPARADOR_POSITIVO)

    MensajeSeguridadMotores = ""

    if np.sign(ultimo_izquierdo) != np.sign(calc_PWM_izq) and calc_PWM_izq != 0 and np.sign(ultimo_izquierdo) != 0:
        MensajeSeguridadMotores += "L0#"

    if np.sign(ultimo_derecho) != np.sign(calc_PWM_der) and calc_PWM_der != 0 and np.sign(ultimo_derecho) != 0:
        MensajeSeguridadMotores += "R0#"

    if np.abs(ultimo_izquierdo - calc_PWM_izq) > 3 or np.abs(ultimo_derecho - calc_PWM_der) > 3 or (
            calc_PWM_der == 0 and ultimo_derecho != 0) or (calc_PWM_izq == 0 and ultimo_izquierdo != 0):
        EnviarMensaje = not transmitirMensaje(MensajeSeguridadMotores + StringIzquierda + StringDerecha)

        ultimo_izquierdo = calc_PWM_izq
        ultimo_derecho = calc_PWM_der

#### METODO PARA DETERMINAR LOS PWM A PARTIR DE LAS COORDENADAS DE UN JOYSTICK (Metodo diamante encontrado en internet) ####
def steering(x, y, sensibilidad_rcv):
    # convert to polar
    r = math.hypot(-x, -y)
    t = math.atan2(-y, -x)
    # rotate by 45 degrees
    t += math.pi / 4
    # back to cartesian
    left = r * math.cos(t)
    right = r * math.sin(t)
    # rescale the new coords
    left = left * math.sqrt(2)
    right = right * math.sqrt(2)
    # clamp to -1/+1
    left = max(-1, min(left, 1))
    right = max(-1, min(right, 1))
    return int(sensibilidad_rcv*left), int(sensibilidad_rcv*right)



if __name__ == '__main__':
    try:
        node_xbee_pc()
    except rospy.ROSInterruptException:
        pass
