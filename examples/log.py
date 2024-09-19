"""
Copyright (c) 2018, Joseph Sullivan
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the <project name> project.
"""

"""
This example demonstrates how to use the CrazyflieClient object to retrieve
Ad Hoc logging configurations of vehicle telemetry. In particular, this example
will demonstrate how to access a log variable only found a custom firmwareself.

In this example, the vehicle is logging a variable called 'eag.eag', which is a float
that represents the output voltage of an analog sensor.

This example is provided for reference, and will fail if the Crazyflie does not
contain an 'eag.eag' log variable.
"""

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

    """
    @config_name    name given to the log configuration

    @variables      List of LogVariables. LogVariable objects specify what data
                    is retrieved from the Crazyflie's table of contents.

                    The first argument of LogVariable is the name of the variable
                    as it appears in the table of contents. The second argument
                    is the type. Valid types are specified in cflib.crazyflie.log

    @period_in_ms   Period between transmissions, specified in milliseconds
    @callback       Optional callback which is called whenever variable(s) are updated
    """
    # Add a log for MultiRanger sensor data
    name = 'ranger'
    variables = [LogVariable('range.left', 'float'),
                LogVariable('range.right', 'float'),
                LogVariable('range.front', 'float'),
                LogVariable('range.back', 'float'),
                LogVariable('range.up', 'float'),
                LogVariable('range.zrange', 'float')]
    period_ms = 10
    
    # Set up callback to print data to terminal
    def ranger_callback(data, timestamp):
        # This is called by the CrazyflieClient object when new data is available
        print("Ranger Data: L:{}, R:{}, F:{}, B:{}, U:{}, D:{}".format(data['range.left'],
              data['range.right'], data['range.front'], data['range.back'], 
              data['range.up'], data['range.zrange']))
        print("Enter anything to exit")
              
    # Configures the crazyflie to log the data
    client.add_log_config(
        name,
        variables,
        period_ms,
        callback=ranger_callback
    )


    input("enter anything to exit")
    del client
    sys.exit(0)
