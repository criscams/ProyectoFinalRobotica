
 
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from taller1_grupo6.srv import turtle_bot_player, turtle_bot_playerResponse


vel_msg = ''
vel_pub = ''
    
def handle_turtle_player(request):
    global vel_msg
    global vel_pub
    rospy.is_shutdown()
    file = open('/home/josecm/catkin_ws/src/taller1_grupo6/result'+request.input_file,'r')
    lineas = file.readlines() #Si el archivo .txt tiene varias lineas
    lineasstr = str(lineas)
    l = list(filter(lambda x: x[0].lower() in 'awsdo',lineasstr))
    print('Hola')
    for key in l:
        if key=="a":
            vel_msg.linear.x=0.5
            vel_msg.angular.z=-(1/0.1533)
            print('Es una a')
    
        if key=="w":
            vel_msg.linear.x=1
            vel_msg.angular.z=0.0
            print('Es una w')
    
        if key=="s":
            vel_msg.linear.x=-1
            vel_msg.angular.z=0.0
            print('Es una s')
    
        if key=="d":
            vel_msg.linear.x=0.5
            vel_msg.angular.z=(1/0.1533)
            print('Es una d')

        if key=="o":
            vel_msg.linear.x=0.0
            vel_msg.angular.z=0.0
            print('Es una o')

        rate = rospy.Rate(9)
        vel_pub.publish(vel_msg)
        rate.sleep()

    file.close()
    vel_msg.linear.x=0.0
    vel_msg.angular.z=0.0
    vel_pub.publish(vel_msg)
    return turtle_bot_playerResponse('Ya realice el recorrido')
    

def turtle_player_server():
    global vel_pub
    global vel_msg 
    rospy.init_node('turtle_bot_player_server',anonymous=False)
    vel_pub=rospy.Publisher('/turtlebot_cmdVel',Twist, queue_size=10)
    vel_msg=Twist()
    s = rospy.Service('turtle_bot_player', turtle_bot_player, handle_turtle_player)
    rospy.spin()

if __name__ == "__main__":
    turtle_player_server()