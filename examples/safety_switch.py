import rospy
import time
import sys
import rospy_crazyflie.client
from rospy_crazyflie.srv import *
from rospy_crazyflie.msg import *

if __name__ == "__main__":
    rospy.init_node('log_example')

    # Connect to a Crazyflie on the server
    crazyflies = rospy_crazyflie.client.get_crazyflies(server='/crazyflie_server')
    client = rospy_crazyflie.client.Client(crazyflies[0])

    # Set up global variables
    range_l = 0; range_r = 0; range_f = 0; range_b = 0; range_u = 0; range_d = 0
    up_flag = False

    # Add a log for MultiRanger sensor data
    name = 'safety_ranger'
    variables = [LogVariable('range.left', 'float'),
                LogVariable('range.right', 'float'),
                LogVariable('range.front', 'float'),
                LogVariable('range.back', 'float'),
                LogVariable('range.up', 'float'),
                LogVariable('range.zrange', 'float')]
    period_ms = 10
    
    # This callback is called by the CrazyflieClient object when new data is available
    def ranger_callback(data, timestamp):
        global up_flag, range_l, range_r, range_f, range_b, range_u, range_d
        print("calling the ranger function")
        range_l = data['range.left']
        range_r = data['range.right']
        range_f = data['range.front']
        range_b = data['range.back']
        range_u = data['range.up']
        range_d = data['range.zrange']
        if data['range.up'] < 200:
            up_flag = True      # Signal crazyflie to land
            print("Landing triggered by obstacle above")
              
    # Configures the crazyflie to log the data
    client.add_log_config(name,variables,period_ms,callback=ranger_callback)

    # Fly!
    #client.take_off(1)
    client.wait()

    # Main loop
    while True:
    	if up_flag:
    		client.stop()
    		client.wait()
    		client.land()
    		client.wait()
    	else:
    		print("operation safe")
    		
