#!/usr/bin/env python3

import tkinter
from tokenize import String
from numpy import expand_dims

import rospy
from pynput import keyboard
from std_msgs.msg import String
from geometry_msgs.msg import Twist


ventana=tkinter.Tk()
ventana.title("Angulo Brazo Robotico")
ventana.resizable(0,0)
frame=tkinter.Frame(ventana)
frame.pack(expand=True)
label1=tkinter.Label(frame, text="Coloque el angulo en que quiere que se mueva el brazo",padx=10)
label1.grid(row=0,column=0)
angulo=tkinter.StringVar()
cuadrotexto1=tkinter.Entry(frame, textvariable=angulo,width=45)
cuadrotexto1.grid(row=1,column=0, padx=10, pady=10)

boton=tkinter.Button(ventana,text="Cerrar",command=ventana.destroy, padx=26)
boton.pack()

ventana.mainloop()

global letra
global angulo_servo

angulo_servo=angulo.get()

def on_press(key):
    
    global letra

    if format(key.char)=="u":
        letra="u"

    if format(key.char)=="j":
        letra="j"
    
    if format(key.char)=="i":
        letra="i"
    
    if format(key.char)=="k":
        letra="k"

    if format(key.char)=="o":
        letra="o"

    if format(key.char)=="l":
        letra="l"
    
    if format(key.char)=="y":
        letra="y"
    
    if format(key.char)=="h":
        letra="h"
    
    try:
        print('Tecla alfanumérica {0} presionada'.format(key.char))
    except AttributeError:
        print('Tecla especial {0} presionada'.format(key.char))

def on_release(key):
    global letra
    letra=""
    print('Tecla {0} liberada'.format(key))
    if key == keyboard.Key.esc:
        return False

listener = keyboard.Listener(on_press=on_press,on_release=on_release)
listener.start()  

if __name__ == '__main__':
    try:
        rospy.init_node('turtle_manipulator_teleop',anonymous=False)
        pub=rospy.Publisher('/robot_manipulator_teleop',String, queue_size=10)
        pub2=rospy.Publisher('/angulo',String, queue_size=10)
        letra=String()
        rate=rospy.Rate(20)       
        while not rospy.is_shutdown():
            pub.publish(letra)
            pub2.publish(angulo_servo)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass    
