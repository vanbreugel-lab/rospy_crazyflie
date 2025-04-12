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

from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig as cfLogConfig

import rospy
import actionlib
from rospy_crazyflie.msg import *
import json

class Log:
    """
    This object manages the creation of vehicle telemetry logging configurations,
    and publishing log data to ros topics.
    """

    def __init__(self, name, crazyflie):
        self._name = name
        self._cf = crazyflie

        # Set up the AddLogConfig Action server for custom / ad hoc log configurations
        self._addLogConfig_as = actionlib.SimpleActionServer(
            name + '/add_log_config',
            AddLogConfigAction,
            self._addLogConfig_cb,
            False
        )
        self._addLogConfig_as.start()
        self._logs = {}

        # Set up log configs for standard log groups
        self._controller_rpy_rate_pub = rospy.Publisher(
            self._name + '/ControllerRPYRate',
            ControllerRPYRate,
            queue_size=100
        )
        self._controller_rpy_rate_config = cfLogConfig(name='ControllerRPYRate', period_in_ms=50)
        self._controller_rpy_rate_config.data_received_cb.add_callback(self._controller_rpy_rate_cb)
        self._controller_rpy_rate_config.add_variable('controller.rollRate', 'float')
        self._controller_rpy_rate_config.add_variable('controller.pitchRate', 'float')
        self._controller_rpy_rate_config.add_variable('controller.yawRate', 'float')
        self._cf.log.add_config(self._controller_rpy_rate_config)

        self._controller_rpyt_pub = rospy.Publisher(
            self._name + '/ControllerRPYT',
            ControllerRPYT,
            queue_size=100
        )
        self._controller_rpyt_config = cfLogConfig(name='ControllerRPYT', period_in_ms=50)
        self._controller_rpyt_config.data_received_cb.add_callback(self._controller_rpyt_cb)
        self._controller_rpyt_config.add_variable('controller.actuatorThrust', 'float')
        self._controller_rpyt_config.add_variable('controller.roll', 'float')
        self._controller_rpyt_config.add_variable('controller.pitch', 'float')
        self._controller_rpyt_config.add_variable('controller.yaw', 'float')
        self._cf.log.add_config(self._controller_rpyt_config)

        self._kalman_position_pub = rospy.Publisher(
            self._name + '/KalmanPositionEst',
            KalmanPositionEst,
            queue_size=100
        )
        self._kalman_position_config = cfLogConfig(name='KalmanPositionEst', period_in_ms=50)
        self._kalman_position_config.data_received_cb.add_callback(self._kalman_position_cb)
        self._kalman_position_config.add_variable('kalman.stateX', 'float')
        self._kalman_position_config.add_variable('kalman.stateY', 'float')
        self._kalman_position_config.add_variable('kalman.stateZ', 'float')
        self._kalman_position_config.add_variable('kalman.statePX', 'float')
        self._kalman_position_config.add_variable('kalman.statePY', 'float')
        self._kalman_position_config.add_variable('kalman.statePZ', 'float')
        #self._kalman_position_config.add_variable('kalman.q0', 'float')
        #self._kalman_position_config.add_variable('kalman.q1', 'float')
        #self._kalman_position_config.add_variable('kalman.q2', 'float')
        #self._kalman_position_config.add_variable('kalman.q3', 'float')
        #self._kalman_position_config.add_variable('kalman.stateD0', 'float')
        #self._kalman_position_config.add_variable('kalman.stateD1', 'float')
        #self._kalman_position_config.add_variable('kalman.stateD2', 'float')
        #self._kalman_position_config.add_variable('kalman.rtFinal', 'float')
        #self._kalman_position_config.add_variable('kalman.rtPred', 'float')
        #self._kalman_position_config.add_variable('kalman.rtUpdate', 'float')
        self._cf.log.add_config(self._kalman_position_config)

        self._motor_power_pub = rospy.Publisher(
            self._name + '/MotorPower',
            MotorPower,
            queue_size=100
        )
        self._motor_power_config = cfLogConfig(name='MotorPower', period_in_ms=50)
        self._motor_power_config.data_received_cb.add_callback(self._motor_power_cb)
        self._motor_power_config.add_variable('motor.m4', 'int32_t')
        self._motor_power_config.add_variable('motor.m1', 'int32_t')
        self._motor_power_config.add_variable('motor.m2', 'int32_t')
        self._motor_power_config.add_variable('motor.m3', 'int32_t')
        self._cf.log.add_config(self._motor_power_config)

        self._posCtl_pub = rospy.Publisher(
            self._name + '/posCtl',
            posCtl,
            queue_size=100
        )
        self._posCtl_config = cfLogConfig(name='posCtl', period_in_ms=50)
        self._posCtl_config.data_received_cb.add_callback(self._pos_ctl_cb)
        self._posCtl_config.add_variable('posCtl.targetVX', 'float')
        self._posCtl_config.add_variable('posCtl.targetVY', 'float')
        self._posCtl_config.add_variable('posCtl.targetVZ', 'float')
        self._posCtl_config.add_variable('posCtl.targetX', 'float')
        self._posCtl_config.add_variable('posCtl.targetY', 'float')
        self._posCtl_config.add_variable('posCtl.targetZ', 'float')
        self._cf.log.add_config(self._posCtl_config)

        self._stabilizer_pub = rospy.Publisher(
            self._name + '/Stabilizer',
            Stabilizer,
            queue_size=100
        )
        self._stabilizer_config = cfLogConfig(name='Stabilizer', period_in_ms=50)
        self._stabilizer_config.data_received_cb.add_callback(self._stabilizer_cb)
        self._stabilizer_config.add_variable('stabilizer.roll', 'float')
        self._stabilizer_config.add_variable('stabilizer.pitch', 'float')
        self._stabilizer_config.add_variable('stabilizer.yaw', 'float')
        self._stabilizer_config.add_variable('stabilizer.thrust', 'uint16_t')
        self._cf.log.add_config(self._stabilizer_config)
        
        self._acc_pub = rospy.Publisher(
            self._name + '/Acceleration',
            Acceleration,
            queue_size=100
        )
        self._acc_config = cfLogConfig(name='Acceleration', period_in_ms=50)
        self._acc_config.data_received_cb.add_callback(self._acc_cb)
        self._acc_config.add_variable('acc.x', 'float')
        self._acc_config.add_variable('acc.y', 'float')
        self._acc_config.add_variable('acc.z', 'float')
        self._cf.log.add_config(self._acc_config)
        
        self._gyro_pub = rospy.Publisher(
            self._name + '/Gyro',
            Gyro,
            queue_size=100
        )
        self._gyro_config = cfLogConfig(name='Gyro', period_in_ms=50)
        self._gyro_config.data_received_cb.add_callback(self._gyro_cb)
        self._gyro_config.add_variable('gyro.x', 'float')
        self._gyro_config.add_variable('gyro.y', 'float')
        self._gyro_config.add_variable('gyro.z', 'float')
        #self._gyro_config.add_variable('gyro.xRaw', 'int16_t')
        #self._gyro_config.add_variable('gyro.yRaw', 'int16_t')
        #self._gyro_config.add_variable('gyro.zRaw', 'int16_t')
        self._cf.log.add_config(self._gyro_config)
        
        self._state_est_pub = rospy.Publisher(
            self._name + '/StateEst',
            StateEst,
            queue_size=100
        )
        self._state_est_config = cfLogConfig(name='StateEst', period_in_ms=50)
        self._state_est_config.data_received_cb.add_callback(self._state_est_cb)
        self._state_est_config.add_variable('stateEstimate.ax', 'float')
        self._state_est_config.add_variable('stateEstimate.ay', 'float')
        self._state_est_config.add_variable('stateEstimate.az', 'float')
        self._state_est_config.add_variable('stateEstimate.vx', 'float')
        self._state_est_config.add_variable('stateEstimate.vy', 'float')
        self._state_est_config.add_variable('stateEstimate.vz', 'float')
        #self._state_est_config.add_variable('stateEstimate.pitch', 'float')
        #self._state_est_config.add_variable('stateEstimate.roll', 'float')
        #self._state_est_config.add_variable('stateEstimate.yaw', 'float')
        self._cf.log.add_config(self._state_est_config)
        
        self._range_pub = rospy.Publisher(
            self._name + '/Range',
            Range,
            queue_size=100
        )
        self._range_config = cfLogConfig(name='Range', period_in_ms=50)
        self._range_config.data_received_cb.add_callback(self._range_cb)
        self._range_config.add_variable('range.back', 'uint16_t')
        self._range_config.add_variable('range.front', 'uint16_t')
        self._range_config.add_variable('range.left', 'uint16_t')
        self._range_config.add_variable('range.right', 'uint16_t')
        self._range_config.add_variable('range.up', 'uint16_t')
        self._range_config.add_variable('range.zrange', 'uint16_t')
        self._cf.log.add_config(self._range_config)

        self._mag_pub = rospy.Publisher(
            self._name + '/Mag',
            Mag,
            queue_size=100
        )
        self._mag_config = cfLogConfig(name='Mag', period_in_ms=50)
        self._mag_config.data_received_cb.add_callback(self._mag_cb)
        self._mag_config.add_variable('mag.x', 'float')
        self._mag_config.add_variable('mag.y', 'float')
        self._mag_config.add_variable('mag.z', 'float')
        self._cf.log.add_config(self._mag_config)

        self._pos_est_alt_pub = rospy.Publisher(
            self._name + '/PosEstAlt',
            PosEstAlt,
            queue_size=100
        )
        self._pos_est_alt_config = cfLogConfig(name='PosEstAlt', period_in_ms=50)
        self._pos_est_alt_config.data_received_cb.add_callback(self._pos_est_alt_cb)
        self._pos_est_alt_config.add_variable('posEstAlt.estimatedZ', 'float')
        self._pos_est_alt_config.add_variable('posEstAlt.estVZ', 'float')
        self._pos_est_alt_config.add_variable('posEstAlt.velocityZ', 'float')
        self._cf.log.add_config(self._pos_est_alt_config)

        self._state_est_z_pub = rospy.Publisher(
            self._name + '/StateEstZ',
            StateEstZ,
            queue_size=100
        )
        self._state_est_z_config = cfLogConfig(name='StateEstZ', period_in_ms=50)
        self._state_est_z_config.data_received_cb.add_callback(self._state_est_z_cb)
        self._state_est_z_config.add_variable('stateEstimateZ.x', 'int16_t')
        self._state_est_z_config.add_variable('stateEstimateZ.y', 'int16_t')
        self._state_est_z_config.add_variable('stateEstimateZ.z', 'int16_t')
        self._state_est_z_config.add_variable('stateEstimateZ.vx', 'int16_t')
        self._state_est_z_config.add_variable('stateEstimateZ.vy', 'int16_t')
        self._state_est_z_config.add_variable('stateEstimateZ.vz', 'int16_t')
        self._state_est_z_config.add_variable('stateEstimateZ.ax', 'int16_t')
        self._state_est_z_config.add_variable('stateEstimateZ.ay', 'int16_t')
        self._state_est_z_config.add_variable('stateEstimateZ.az', 'int16_t')
        #self._state_est_z_config.add_variable('stateEstimateZ.quat', 'uint32_t') #problem here
        self._state_est_z_config.add_variable('stateEstimateZ.rateRoll', 'int16_t')
        self._state_est_z_config.add_variable('stateEstimateZ.ratePitch', 'int16_t')
        self._state_est_z_config.add_variable('stateEstimateZ.rateYaw', 'int16_t')
        self._cf.log.add_config(self._state_est_z_config)

        #self._controller_pub = rospy.Publisher(
            #self._name + '/Controller',
            #Controller,
            #queue_size=100
        #)
        #self._controller_config = cfLogConfig(name='Controller', period_in_ms=100)
        #self._controller_config.data_received_cb.add_callback(self._controller_cb)
        #self._controller_config.add_variable('controller.cmd_thrust', 'float')
        #self._controller_config.add_variable('controller.cmd_roll', 'float')
        #self._controller_config.add_variable('controller.cmd_pitch', 'float')
        #self._controller_config.add_variable('controller.cmd_yaw', 'float')
        #self._controller_config.add_variable('controller.r_roll', 'float')
        #self._controller_config.add_variable('controller.r_pitch', 'float')
        #self._controller_config.add_variable('controller.r_yaw', 'float')
        #self._controller_config.add_variable('controller.accelz', 'float')
        #self._controller_config.add_variable('controller.actuatorThrust', 'float')
        #self._controller_config.add_variable('controller.roll', 'float')
        #self._controller_config.add_variable('controller.pitch', 'float')
        #self._controller_config.add_variable('controller.yaw', 'float')
        #self._controller_config.add_variable('controller.rollRate', 'float')
        #self._controller_config.add_variable('controller.pitchRate', 'float')
        #self._controller_config.add_variable('controller.yawRate', 'float')
        #self._controller_config.add_variable('controller.ctr_yaw', 'int16_t') #INT NAMES MAY BE WRONG
        #self._cf.log.add_config(self._controller_config)

        self._motion_pub = rospy.Publisher(
            self._name + '/Motion',
            Motion,
            queue_size=100
        )
        self._motion_config = cfLogConfig(name='Motion', period_in_ms=50)
        self._motion_config.data_received_cb.add_callback(self._motion_cb)
        self._motion_config.add_variable('motion.deltaX', 'int16_t')
        self._motion_config.add_variable('motion.deltaY', 'int16_t')
        self._cf.log.add_config(self._motion_config)


        self._ext_pos_pub = rospy.Publisher(
            self._name + '/ExtPos',
            ExtPos,
            queue_size=100
        )
        self._ext_pos_config = cfLogConfig(name='ExtPos', period_in_ms=50)
        self._ext_pos_config.data_received_cb.add_callback(self._ext_pos_cb)
        self._ext_pos_config.add_variable('ext_pos.X', 'float')
        self._ext_pos_config.add_variable('ext_pos.Y', 'float')
        self._ext_pos_config.add_variable('ext_pos.Z', 'float')
        self._cf.log.add_config(self._ext_pos_config)
        
        self._batterystate_pub = rospy.Publisher(
            self._name + '/BatteryState',
            BatteryState,
            queue_size=100
        )
        # print("BatteryState log config")
        self._batterystate_config = cfLogConfig(name='BatteryState', period_in_ms=100)
        self._batterystate_config.data_received_cb.add_callback(self._batterystate_cb)
        self._batterystate_config.add_variable('pm.vbat', 'float')
        self._cf.log.add_config(self._batterystate_config)

    def _addLogConfig_cb(self, goal):
        """ Implements the AddLogConfig Action"""
        # Construct the cflib LogConfig object
        config = goal.config
        new_config = cfLogConfig(name=config.name, period_in_ms=config.period)
        new_config.data_received_cb.add_callback(self._log_data_cb)
        new_config.error_cb.add_callback(self._log_error_cb)
        for variable in config.variables:
            new_config.add_variable(variable.name, variable.type)
        self._logs[config.name] = new_config

        # Add this config to the Crazyflie
        try:
            self._cf.log.add_config(new_config)
            new_config.start()
            self._logs[config.name] = {}
            self._logs[config.name] = {'config' : new_config}
            self._logs[config.name]['publisher'] = rospy.Publisher(
                self._name + '/' + config.name,
                LogData,
                queue_size = 100
            )
            self._addLogConfig_as.set_succeeded()
        except BaseException as e:
            print("failed to add log %s" % (str(e)))
            self._addLogConfig_as.set_aborted()


    def _log_error_cb(self, logconf, msg):
        """
        Error handler from CrazyflieLibPython for all custom / ad hoc log configurations
        """
        # TODO:
        pass

    def _log_data_cb(self, timestamp, data, logconf):
        """
        Callback from CrazyflieLibPython for all custom / ad hoc log configurations
        """
        new_data = LogData(json.dumps(data), timestamp)
        pub = self._logs[logconf.name]['publisher']
        pub.publish(new_data)

    def stop_logs(self):
        """
        Stops log configurations,
        """
        for log_name in self._logs.keys():
            print("stopping log %s" %log_name)
            log = self._logs.pop(log_name)
            log['config'].stop()
            log['config'].delete()
            log['publisher'].unregister()
            del log

        self.stop_log_controller_rpy_rate()
        self.stop_log_controller_rpyt()
        self.stop_log_kalman_position_est()
        self.stop_log_motor_power()
        self.stop_log_pos_ctl()
        self.stop_log_stabilizer()
        self.stop_log_acc()
        self.stop_log_gyro()
        self.stop_log_state_est()
        self.stop_log_range()
        self.stop_log_mag()
        self.stop_log_pos_est_alt()
        self.stop_log_state_est_z()
        self.stop_log_motion()
        self.stop_log_ext_pos()
        #self.stop_log_controller()
        self.stop_log_battery()


    def log_controller_rpy_rate(self, period_in_ms=100):
        """ Starts ControllerRPYRate log config"""
        self._controller_rpy_rate_config.period_in_ms = period_in_ms
        self._controller_rpy_rate_config.start()

    def log_controller_rpyt(self, period_in_ms=100):
        """ Starts ControllerRPYT log config """
        self._controller_rpyt_config.period_in_ms = period_in_ms
        self._controller_rpyt_config.start()

    def log_kalman_position_est(self, period_in_ms=100):
        """ Starts KalmanPositionEst log config """
        self._kalman_position_config.period_in_ms = period_in_ms
        self._kalman_position_config.start()

    def log_motor_power(self, period_in_ms=100):
        """ Starts MotorPower log config """
        self._motor_power_config.period_in_ms = period_in_ms
        self._motor_power_config.start()

    def log_pos_ctl(self, period_in_ms=100):
        """ Starts posCtl log config """
        self._posCtl_config.period_in_ms = period_in_ms
        self._posCtl_config.start()

    def log_stabilizer(self, period_in_ms=100):
        """ Starts Stabilizer log config """
        self._stabilizer_config.period_in_ms = period_in_ms
        self._stabilizer_config.start()
        
    def log_acc(self, period_in_ms=100):
        """ Starts Acceleration log config """
        self._acc_config.period_in_ms = period_in_ms
        self._acc_config.start()
    
    def log_gyro(self, period_in_ms=100):
        """ Starts Gyro log config """
        self._gyro_config.period_in_ms = period_in_ms
        self._gyro_config.start()

    def log_state_est(self, period_in_ms=100):
        """ Starts StateEst log config """
        self._state_est_config.period_in_ms = period_in_ms
        self._state_est_config.start()
        
    def log_range(self, period_in_ms=100):
        """ Starts Range log config """
        self._range_config.period_in_ms = period_in_ms
        self._range_config.start()

    def log_mag(self, period_in_ms=100):
        """ Starts Mag log config """
        self._mag_config.period_in_ms = period_in_ms
        self._mag_config.start()

    def log_pos_est_alt(self, period_in_ms=100):
        """ Starts PosEstAlt log config """
        self._pos_est_alt_config.period_in_ms = period_in_ms
        self._pos_est_alt_config.start()

    def log_state_est_z(self, period_in_ms=100):
        """ Starts StateEstZ log config """
        self._state_est_z_config.period_in_ms = period_in_ms
        self._state_est_z_config.start()

    #def log_controller(self, period_in_ms=100):
        #""" Starts Controller log config """
        #self._controller_config.period_in_ms = period_in_ms
        #self._controller_config.start()
        
    def log_motion(self, period_in_ms=100):
        """ Starts Motion log config """
        self._motion_config.period_in_ms = period_in_ms
        self._motion_config.start()

    def log_ext_pos(self, period_in_ms=100):
        """ Starts ExtPos log config """
        self._ext_pos_config.period_in_ms = period_in_ms
        self._ext_pos_config.start()
        
    def log_battery(self, period_in_ms=100):
        """ Starts BatteryState log config """
        self._batterystate_config.period_in_ms = period_in_ms
        self._batterystate_config.start()
    
    def stop_log_controller_rpy_rate(self):
        """ Stops ControllerRPYRate log config"""
        self._controller_rpy_rate_config.stop()

    def stop_log_controller_rpyt(self):
        """ Stops ControllerRPYT log config """
        self._controller_rpyt_config.stop()

    def stop_log_kalman_position_est(self):
        """ Stops KalmanPositionEst log config """
        self._kalman_position_config.stop()

    def stop_log_motor_power(self):
        """ Stops MotorPower log config """
        self._motor_power_config.stop()

    def stop_log_pos_ctl(self):
        """ Stops posCtl log config """
        self._posCtl_config.stop()

    def stop_log_stabilizer(self):
        """ Stops Stabilizer log config """
        self._stabilizer_config.stop()
        
    def stop_log_acc(self):
        """ Stops Acceleration log config """
        self._acc_config.stop()
        
    def stop_log_gyro(self):
        """ Stops Gyro log config """
        self._gyro_config.stop()

    def stop_log_state_est(self):
        """ Stops StateEst log config """
        self._state_est_config.stop()

    def stop_log_range(self):
        """ Stops Range log config """
        self._range_config.stop()

    def stop_log_mag(self):
        """ Stops Mag log config """
        self._mag_config.stop()

    def stop_log_pos_est_alt(self):
        """ Stops PosEstAlt log config """
        self._pos_est_alt_config.stop()

    def stop_log_state_est_z(self):
        """ Stops StateEstZ log config """
        self._state_est_z_config.stop()

    #def stop_log_controller(self):
        #""" Stops Controller log config """
        #self._controller_config.stop()

    def stop_log_motion(self):
        """ Stops Motion log config """
        self._motion_config.stop()      

    def stop_log_ext_pos(self):
        """ Stops ExtPos log config """
        self._ext_pos_config.stop()
        
    def stop_log_battery(self):
        """ Stops BatteryState log config """
        self._batterystate_config.stop()
         
    def _controller_rpy_rate_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes ControllerRPYRate messages """
        msg = ControllerRPYRate()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.rollRate = data['controller.rollRate']
        msg.pitchRate = data['controller.pitchRate']
        msg.yawRate = data['controller.yawRate']
        self._controller_rpy_rate_pub.publish(msg)

    def _controller_rpyt_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes ControllerRPYT messages """
        msg = ControllerRPYT()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.roll = data['controller.roll']
        msg.pitch = data['controller.pitch']
        msg.yaw = data['controller.yaw']
        msg.actuatorThrust = data['controller.actuatorThrust']
        self._controller_rpyt_pub.publish(msg)

    def _kalman_position_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes KalmanPositionEst messages """
        msg = KalmanPositionEst()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.stateX = data['kalman.stateX']
        msg.stateY = data['kalman.stateY']
        msg.stateZ = data['kalman.stateZ']
        msg.statePX = data['kalman.statePX']
        msg.statePY = data['kalman.statePY']
        msg.statePZ = data['kalman.statePZ']
        #msg.q0 = data['kalman.q0']
        #msg.q1 = data['kalman.q1']
        #msg.q2 = data['kalman.q2']
        #msg.q3 = data['kalman.q4']
        #msg.stateD0 = data['kalman.stateD0']
        #msg.stateD1 = data['kalman.stateD1']
        #msg.stateD2 = data['kalman.stateD2']
        #msg.rtFinal = data['kalman.rtFinal']
        #msg.rtPred = data['kalman.rtPred']
        #msg.rtUpdate = data['kalman.rtUpdate']
        self._kalman_position_pub.publish(msg)

    def _motor_power_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes MotorPower messages """
        msg = MotorPower()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.m4 = data['motor.m4']
        msg.m1 = data['motor.m1']
        msg.m2 = data['motor.m2']
        msg.m3 = data['motor.m3']
        self._motor_power_pub.publish(msg)

    def _pos_ctl_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes posCtl messages """
        msg = posCtl()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.targetVX = data['posCtl.targetVX']
        msg.targetVY = data['posCtl.targetVY']
        msg.targetVZ = data['posCtl.targetVZ']
        msg.targetX = data['posCtl.targetX']
        msg.targetY = data['posCtl.targetY']
        msg.targetZ = data['posCtl.targetZ']
        self._posCtl_pub.publish(msg)

    def _stabilizer_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes Stabilizer messages """
        msg = Stabilizer()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.roll = data['stabilizer.roll']
        msg.pitch = data['stabilizer.pitch']
        msg.yaw = data['stabilizer.yaw']
        msg.thrust = data['stabilizer.thrust']
        self._stabilizer_pub.publish(msg)
        
    def _acc_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes Acceleration messages """
        msg = Acceleration()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.x = data['acc.x']
        msg.y = data['acc.y']
        msg.z = data['acc.z']
        self._acc_pub.publish(msg)
 
    def _gyro_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes Gyro messages """
        msg = Gyro()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.x = data['gyro.x']
        msg.y = data['gyro.y']
        msg.z = data['gyro.z']
        #msg.xRaw = data['gyro.xRaw']
        #msg.yRaw = data['gyro.yRaw']
        #msg.zRaw = data['gyro.zRaw']
        self._gyro_pub.publish(msg)
        
    def _state_est_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes StateEst messages """
        msg = StateEst()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.ax = data['stateEstimate.ax']
        msg.ay = data['stateEstimate.ay']
        msg.az = data['stateEstimate.az']
        msg.vx = data['stateEstimate.vx']
        msg.vy = data['stateEstimate.vy']
        msg.vz = data['stateEstimate.vz']
        #msg.pitch = data['stateEstimate.pitch']
        #msg.roll = data['stateEstimate.roll']
        #msg.yaw = data['stateEstimate.yaw']
        self._state_est_pub.publish(msg)

    def _range_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes Range messages """
        msg = Range()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.back = data['range.back']
        msg.front = data['range.front']
        msg.left = data['range.left']
        msg.right = data['range.right']
        msg.up = data['range.up']
        msg.zrange = data['range.zrange']
        self._range_pub.publish(msg)

    def _mag_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes Mag messages """
        msg = Mag()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.x = data['mag.x']
        msg.y = data['mag.y']
        msg.z = data['mag.z']
        self._mag_pub.publish(msg)

    def _pos_est_alt_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes PosEstAlt messages """
        msg = PosEstAlt()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.estimatedZ = data['posEstAlt.estimatedZ']
        msg.estVZ = data['posEstAlt.estVZ']
        msg.velocityZ = data['posEstAlt.velocityZ']
        self._pos_est_alt_pub.publish(msg)

    def _state_est_z_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes StateEstZ messages """
        msg = StateEstZ()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.x = data['stateEstimateZ.x']
        msg.y = data['stateEstimateZ.y']
        msg.z = data['stateEstimateZ.z']
        msg.vx = data['stateEstimateZ.vx']
        msg.vy = data['stateEstimateZ.vy']
        msg.vz = data['stateEstimateZ.vz']
        msg.ax = data['stateEstimateZ.ax']
        msg.ay = data['stateEstimateZ.ay']
        msg.az = data['stateEstimateZ.az']
        #msg.quat = data['stateEstimateZ.quat']
        msg.rateRoll = data['stateEstimateZ.rateRoll']
        msg.ratePitch = data['stateEstimateZ.ratePitch']
        msg.rateYaw = data['stateEstimateZ.rateYaw']
        self._state_est_z_pub.publish(msg)

    #def _controller_cb(self, timestamp, data, logconfig):
        #""" Callback from CrazyflieLibPython, publishes Controller messages """
        #msg = Controller()
        #msg.stamp = rospy.get_rostime()
        #msg.cfstamp = timestamp
        #msg.cmd_thrust = data['controller.cmd_thrust']
        #msg.cmd_roll = data['controller.cmd_roll']
        #msg.cmd_pitch = data['controller.cmd_pitch']
        #msg.cmd_yaw = data['controller.cmd_yaw']
        #msg.r_roll = data['controller.r_roll']
        #msg.r_pitch = data['controller.r_pitch']
        #msg.r_yaw = data['controller.r_yaw']
        #msg.accelz = data['controller.accelz']
        #msg.actuatorThrust = data['controller.actuatorThrust']
        #msg.roll = data['controller.roll']
        #msg.pitch = data['controller.pitch']
        #msg.yaw = data['controller.yaw']
        #msg.rollRate = data['controller.rollRate']
        #msg.pitchRate = data['controller.pitchRate']
        #msg.yawRate = data['controller.yawRate']
        #msg.ctr_yaw = data['controller.ctr_yaw']
        #self._controller_pub.publish(msg)
        
    def _motion_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes Motion messages """
        msg = Motion()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.deltaX = data['motion.deltaX']
        msg.deltaY = data['motion.deltaY']
        self._motion_pub.publish(msg)
        
    def _ext_pos_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes ExtPos messages """
        msg = ExtPos()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.X = data['ext_pos.X']
        msg.Y = data['ext_pos.Y']
        msg.Z = data['ext_pos.Z']
        self._ext_pos_pub.publish(msg)
        
        
    def _batterystate_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes BatteryState messages """
        msg = BatteryState()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.vbat = data['pm.vbat']
        self._batterystate_pub.publish(msg)
