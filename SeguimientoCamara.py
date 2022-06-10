#!/usr/bin/env python3

import cv2
import numpy as np
from logging import root
from tokenize import String
import tkinter

from numpy import expand_dims

import os
import rospy
from pynput import keyboard
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

ventana=tkinter.Tk()
ventana.title("Angulo Brazo Robotico")
ventana.resizable(0,0)
frame=tkinter.Frame(ventana)
frame.pack(expand=True)
label1=tkinter.Label(frame, text="Coloque el número del color deseado (1=Rojo, 2=Azul, 3=Amarillo)",padx=10)
label1.grid(row=0,column=0)
color=tkinter.StringVar()
cuadrotexto1=tkinter.Entry(frame, textvariable=color,width=45)
cuadrotexto1.grid(row=1,column=0, padx=10, pady=10)

boton=tkinter.Button(ventana,text="Cerrar",command=ventana.destroy, padx=26)
boton.pack()

ventana.mainloop()

color_pelota=color.get()

if color_pelota=="1":
    tonobajo=np.array([0, 100, 20], np.uint8)   
    tonooscuro=np.array([8, 255, 255], np.uint8)

elif color_pelota=="2":
    tonobajo=np.array([90, 100, 20], np.uint8)   
    tonooscuro=np.array([120, 255, 255], np.uint8)

elif color_pelota=="3":
    tonobajo=np.array([27, 100, 20], np.uint8)   
    tonooscuro=np.array([50, 255, 255], np.uint8)


def movimiento(cap):
    frame=cap

    pub=rospy.Publisher('/robot_manipulator_camara',String, queue_size=10)
    caso=String()
    rate=rospy.Rate(100)
    
    frame=cv2.flip(frame, 1)
    frameHSV=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mascara=cv2.inRange(frameHSV, tonobajo, tonooscuro)
    contornos, _=cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contornos, -1, (255, 0, 0), 4)
    
    for c in contornos:
        area=cv2.contourArea(c)
        if area > 6000:
            M=cv2.moments(c)
            if M["m00"] == 0:
                M["m00"]=1
            x=int(M["m10"]/M["m00"])
            y=int(M["m01"]/M["m00"])
            cv2.circle(frame, (x, y), 7, (0, 0, 255), -1)
            fuente=cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, '{},{}'.format(x, y), (x+10, y), fuente, 1.2, (0, 0, 255), 2, cv2.LINE_AA)
            nuevocontorno=cv2.convexHull(c)
            cv2.drawContours(frame, [nuevocontorno], 0, (255, 0, 0), 3)
            
            if x < 90 and y < 96:
                caso="izq1arr1"
                pub.publish(caso)
                rate.sleep()
            elif x < 90 and 96 < y <= 192:
                caso="izq1arr2"
                pub.publish(caso)
                rate.sleep()
            elif x < 90 and 192 < y <= 288:
                caso="izq1ctr"
                pub.publish(caso)
                rate.sleep()
            elif x < 90 and 288 <= y < 384:
                caso="izq1aba2"
                pub.publish(caso)
                rate.sleep()
            elif x < 90 and y >= 384:
                caso="izq1aba1"
                pub.publish(caso)
                rate.sleep()
            elif 90 <= x < 180 and y < 96:
                caso="izq2arr1"
                pub.publish(caso)
                rate.sleep()
            elif 90 <= x < 180 and 96 < y <= 192:
                caso="izq2arr2"
                pub.publish(caso)
                rate.sleep()
            elif 90 <= x < 180 and 192 < y <= 288:
                caso="izq2ctr"
                pub.publish(caso)
                rate.sleep()
            elif 90 <= x < 180 and 288 <= y < 384:
                caso="izq2aba2"
                pub.publish(caso)
                rate.sleep()
            elif 90 <= x < 180 and y >= 384:
                caso="izq2aba1"
                pub.publish(caso)
                rate.sleep()
            elif 180 <= x < 260 and y < 96:
                caso="izq3arr1"
                pub.publish(caso)
                rate.sleep()
            elif 180 <= x < 260 and 96 < y <= 192:
                caso="izq3arr2"
                pub.publish(caso)
                rate.sleep()
            elif 180 <= x < 260 and 192 < y <= 288:
                caso="izq3ctr"
                pub.publish(caso)
                rate.sleep()
            elif 180 <= x < 260 and 288 <= y < 384:
                caso="izq3aba2"
                pub.publish(caso)
                rate.sleep()
            elif 180 <= x < 260 and y >= 384:
                caso="izq3aba1"
                pub.publish(caso)
                rate.sleep()
            elif 260 <= x < 370 and y < 96:
                caso="ctrarr1"
                pub.publish(caso)
                rate.sleep()
            elif 260 <= x < 370 and 96 < y <= 192:
                caso="ctrarr2"
                pub.publish(caso)
                rate.sleep()
            elif 260 <= x < 370 and 192 < y <= 288:
                caso="ctrctr"
                pub.publish(caso)
                rate.sleep()
            elif 260 <= x < 370 and 288 <= y < 384:
                caso="ctraba2"
                pub.publish(caso)
                rate.sleep()
            elif 260 <= x < 370 and y >= 384:
                caso="ctraba1"
                pub.publish(caso)
                rate.sleep()
            elif 370 <= x < 460 and y < 96:
                caso="der3arr1"
                pub.publish(caso)
                rate.sleep()
            elif 370 <= x < 460 and 96 < y <= 192:
                caso="der3arr2"
                pub.publish(caso)
                rate.sleep()
            elif 370 <= x < 460 and 192 < y <= 288:
                caso="der3ctr"
                pub.publish(caso)
                rate.sleep()
            elif 370 <= x < 460 and 288 <= y < 384:
                caso="der3aba2"
                pub.publish(caso)
                rate.sleep()
            elif 370 <= x < 460 and y >= 384:
                caso="der3aba1"
                pub.publish(caso)
                rate.sleep()
            elif 460 <= x < 550 and y < 96:
                caso="der2arr1"
                pub.publish(caso)
                rate.sleep()
            elif 460 <= x < 550 and 96 < y <= 192:
                caso="der2arr2"
                pub.publish(caso)
                rate.sleep()
            elif 460 <= x < 550 and 192 < y <= 288:
                caso="der2ctr"
                pub.publish(caso)
                rate.sleep()
            elif 460 <= x < 550 and 288 <= y < 384:
                caso="der2aba2"
                pub.publish(caso)
                rate.sleep()
            elif 460 <= x < 550 and y >= 384:
                caso="der2aba1"
                pub.publish(caso)
                rate.sleep()
            elif x <= 550 and y < 96:
                caso="der1arr1"
                pub.publish(caso)
                rate.sleep()
            elif x <= 550 and 96 < y <= 192:
                caso="der1arr2"
                pub.publish(caso)
                rate.sleep()
            elif x <= 550 and 192 < y <= 288:
                caso="der1ctr"
                pub.publish(caso)
                rate.sleep()
            elif x <= 550 and 288 <= y < 384:
                caso="der1aba2"
                pub.publish(caso)
                rate.sleep()
            elif x <= 550 and y >= 384:
                caso="der1aba1"
                pub.publish(caso)
                rate.sleep()

        #frame=cv2.resize(frame, (1280,720))
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == 27:
            break
        
def lecturacamara(imagen):
    global captura
    bridge=CvBridge()
    captura=bridge.imgmsg_to_cv2(imagen,"bgr8")
    movimiento(captura)
        
def camara():
    rospy.init_node('turtle_manipulator_camara',anonymous=False)
    rospy.Subscriber('/usb_cam/image_raw', Image, lecturacamara)

if __name__ == '__main__':
    camara()
    rospy.spin()
