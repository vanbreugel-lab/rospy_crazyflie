import rospy
import time
import rospy_crazyflie.client
import sys

crazyflies = rospy_crazyflie.client.get_crazyflies(server='/crazyflie_server')
client = rospy_crazyflie.client.Client(crazyflies[0])
client.land()
