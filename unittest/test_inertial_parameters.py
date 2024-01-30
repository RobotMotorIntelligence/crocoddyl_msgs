#!/usr/bin/env python

###############################################################################
# BSD 3-Clause License
#
# Copyright (C) 2023-2024, Heriot-Watt University
# Copyright note valid unless otherwise stated in individual files.
# All rights reserved.
###############################################################################

import os
import random
import time
import unittest

import numpy as np
import pinocchio

import rospy
import rosunit

from crocoddyl_ros import (
    MultibodyInertialParametersRosPublisher,
    MultibodyInertialParametersRosSubscriber
)


class TestInertialParameters(unittest.TestCase):
    def setUp(self):
        rospy.init_node("crocoddyl_ros", anonymous=True)

    def test_communication(self):
        model = pinocchio.buildSampleModelHumanoid()
        pub = MultibodyInertialParametersRosPublisher(model.nbodies, "inertial_parameters")
        sub = MultibodyInertialParametersRosSubscriber(model.nbodies, "inertial_parameters")
        time.sleep(1)
        # create the name index
        parameters = {}
        names = model.names.tolist()
        # publish the inertial parameters
        for i in range(model.nbodies):
            parameters[names[i]] = model.inertias[i].toDynamicParameters()
            
        while True:
            pub.publish(parameters)
            if sub.has_new_msg():
                break
        
        _parameters = sub.get_inertial_parameters()
        for i in range(model.nbodies):
            self.assertTrue(np.allclose(_parameters[names[i]], parameters[names[i]], atol=1e-9), "Wrong parameters in " + names[i])

if __name__ == "__main__":
    rosunit.unitrun("crocoddyl_msgs", "inertial_parameters", TestInertialParameters)
