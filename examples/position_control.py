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
This example demonstrates how to use the CrazyflieClient to perform feed-forward
position control. In order for this to work, a FlowDeck is required, otherwise,
the vehicle will surely drift and crash.

The control is called feed-forward because there is no position feedback in the system.
Hence the vehicle navigates by "dead reckoning" based on the assumption that the velocity
controller is ideal. Errors will build over time.
"""

import rospy
import time
import rospy_crazyflie.client
import sys

if __name__ == "__main__":
    rospy.init_node("position_control_example")

    # Connect to the crazyflie
    crazyflies = rospy_crazyflie.client.get_crazyflies(server='/crazyflie_server')
    client = rospy_crazyflie.client.Client(crazyflies[0])

    # Give command and wait until each is finished before executing next
    client.take_off(.5) # Takeoff to .5 meters
    client.wait()
    client.forward(.2)  # go .5 meters forward
    client.wait()
    client.back(.2)     # go .5 meters backward
    client.wait()
    client.left(.2)     # go .5 meters left
    client.wait()
    client.right(.2)    # go .5 meters right
    client.wait()

    # Many more position and velocity commands are implemented in CrazyflieClient!

    client.land()
    client.wait()

    del client
    sys.exit(0)
