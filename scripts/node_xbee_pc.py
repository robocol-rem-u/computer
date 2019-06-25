#!/usr/bin/env python
import rospy, socket, _thread, serial, threading, thread, time
from master_msgs.msg import connection, arm_Orders, traction_Orders


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
def enviarMensajeInicializacion():
    global EnviarMensaje
    while EnviarMensaje:
        nMsg = 1#3
        print("Enviando Inicializacion")
        transmitirMensaje("A"+str(nMsg)+"#I0#I1#I2#I3#I4#I5#")
        time.sleep(1)




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






if __name__ == '__main__':
    try:

        node_xbee_pc()
    except rospy.ROSInterruptException:
        pass
