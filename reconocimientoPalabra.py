#!/usr/bin/env python3


from unittest import result
import cv2                              # Importamos librerias
import pytesseract
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os
import easyocr
from matplotlib import pyplot as plt



#Vamos a capturar el objeto que queremos identificar
def reconocimiento(cap):
    read=easyocr.Reader(['en'], gpu=False)
    gray=cv2.cvtColor(cap, cv2.COLOR_RGB2GRAY)
    result=read.readtext(gray)
    text=""
    for res in result:
        text += res[1]+""
    im_top_left=tuple(result[0][0][0])
    im_bottom_right=tuple(result[0][0][2])
    im_text=result[0][1]
    font=cv2.FONT_HERSHEY_SIMPLEX
    print(text)
    cv2.rectangle(cap, im_top_left, im_bottom_right,(50,50,355),5)
    cv2.putText(cap,text,(50,70),font,1,(50,50,25),2)
    plt.imshow(cap,cmap='gray', interpolation='bicubic')
    plt.xticks([]), plt.yticks([])
    plt.show()


def lecturacamara(imagen):
    global captura
    bridge=CvBridge()
    captura=bridge.imgmsg_to_cv2(imagen,"bgr8")
    reconocimiento(captura)
        
def camara():
    rospy.init_node('ReconocerTexto',anonymous=False)
    rospy.Subscriber('/usb_cam/image_raw', Image, lecturacamara)

if __name__ == '__main__':
    camara()
    rospy.spin()