#!/usr/bin/env python

###############################################################################
# BSD 3-Clause License
#
# Copyright (C) 2023-2023, Heriot-Watt University
# Copyright note valid unless otherwise stated in individual files.
# All rights reserved.
###############################################################################

import os
import random
import time
import unittest

import numpy as np
import pinocchio

ROS_VERSION = int(os.environ["ROS_VERSION"])
if ROS_VERSION == 2:
    import rclpy
else:
    import rospy
    import rosunit

from crocoddyl_ros import (
    ContactStatus,
    ContactType,
    WholeBodyStateRosPublisher,
    WholeBodyStateRosSubscriber,
    toReduced,
)


class TestWholeBodyState(unittest.TestCase):
    def setUp(self):
        if ROS_VERSION == 2:
            rclpy.init()
        else:
            rospy.init_node("crocoddyl_ros", anonymous=True)
        # Create random state
        self.t = random.uniform(0, 1)
        self.p = {
            "lleg_effector_body": pinocchio.SE3.Random(),
            "rleg_effector_body": pinocchio.SE3.Random(),
        }
        self.pd = {
            "lleg_effector_body": pinocchio.Motion.Random(),
            "rleg_effector_body": pinocchio.Motion.Random(),
        }
        type = random.randint(0, 1)
        if type == 0:
            contact_type = ContactType.LOCOMOTION
        else:
            contact_type = ContactType.MANIPULATION
        status = random.randint(0, 3)
        if status == 0:
            contact_status = ContactStatus.UNKNOWN
        elif status == 1:
            contact_status = ContactStatus.SEPARATION
        elif status == 2:
            contact_status = ContactStatus.STICKING
        else:
            contact_status = ContactStatus.SLIPPING
        self.f = {
            "lleg_effector_body": [
                pinocchio.Force.Random(),
                contact_type,
                contact_status,
            ],
            "rleg_effector_body": [
                pinocchio.Force.Random(),
                contact_type,
                contact_status,
            ],
        }
        self.s = {
            "lleg_effector_body": [np.random.rand(3), random.uniform(0, 1)],
            "rleg_effector_body": [np.random.rand(3), random.uniform(0, 1)],
        }

    def test_communication(self):
        model = pinocchio.buildSampleModelHumanoid()
        sub = WholeBodyStateRosSubscriber(model, "whole_body_state")
        pub = WholeBodyStateRosPublisher(model, "whole_body_state")
        time.sleep(1)
        # publish whole-body state messages
        q = pinocchio.randomConfiguration(model)
        q[:3] = np.random.rand(3)
        v = np.random.rand(model.nv)
        tau = np.random.rand(model.nv - 6)
        while True:
            pub.publish(self.t, q, v, tau, self.p, self.pd, self.f, self.s)
            if sub.has_new_msg():
                break
        # get whole-body state
        _t, _q, _v, _tau, _p, _pd, _f, _s = sub.get_state()
        self.assertEqual(self.t, _t, "Wrong time interval")
        self.assertTrue(np.allclose(q, _q, atol=1e-9), "Wrong q")
        self.assertTrue(np.allclose(v, _v, atol=1e-9), "Wrong v")
        self.assertTrue(np.allclose(tau, _tau, atol=1e-9), "Wrong tau")
        for name in self.p:
            M, [_t, _R] = self.p[name], _p[name]
            F, _F = self.f[name], _f[name]
            S, _S = self.s[name], _s[name]
            self.assertTrue(
                np.allclose(M.translation, _t, atol=1e-9),
                "Wrong contact translation at " + name,
            )
            self.assertTrue(
                np.allclose(M.rotation, _R, atol=1e-9),
                "Wrong contact rotation at " + name,
            )
            self.assertTrue(
                np.allclose(self.pd[name].vector, _pd[name], atol=1e-9),
                "Wrong contact velocity translation at " + name,
            )
            self.assertTrue(
                np.allclose(F[0], _F[0], atol=1e-9),
                "Wrong contact wrench translation at " + name,
            )
            self.assertTrue(F[1] == _F[1], "Wrong contact type at " + name)
            self.assertTrue(F[2] == _F[2], "Wrong contact status at " + name)
            self.assertTrue(
                np.allclose(S[0], _S[0], atol=1e-9),
                "Wrong contact surface translation at " + name,
            )
            self.assertEqual(
                S[1], _S[1], "Wrong contact friction coefficient at " + name
            )

    def test_communication_with_reduced_model(self):
        model = pinocchio.buildSampleModelHumanoid()
        locked_joints = ["larm_elbow_joint", "rarm_elbow_joint"]
        qref = pinocchio.randomConfiguration(model)
        reduced_model = pinocchio.buildReducedModel(
            model, [model.getJointId(name) for name in locked_joints], qref
        )
        sub = WholeBodyStateRosSubscriber(
            model, locked_joints, qref, "whole_body_state"
        )
        pub = WholeBodyStateRosPublisher(model, locked_joints, qref, "whole_body_state")
        time.sleep(1)
        # publish whole-body state messages
        q = pinocchio.randomConfiguration(model)
        q[:3] = np.random.rand(3)
        v = np.random.rand(model.nv)
        tau = np.random.rand(model.nv - 6)
        q, v, tau = toReduced(model, reduced_model, q, v, tau)
        while True:
            pub.publish(self.t, q, v, tau, self.p, self.pd, self.f, self.s)
            if sub.has_new_msg():
                break
        # get whole-body state
        _t, _q, _v, _tau, _p, _pd, _f, _s = sub.get_state()
        self.assertEqual(self.t, _t, "Wrong time interval")
        self.assertTrue(np.allclose(q, _q, atol=1e-9), "Wrong q")
        self.assertTrue(np.allclose(v, _v, atol=1e-9), "Wrong v")
        self.assertTrue(np.allclose(tau, _tau, atol=1e-9), "Wrong tau")
        for name in self.p:
            M, [_t, _R] = self.p[name], _p[name]
            F, _F = self.f[name], _f[name]
            S, _S = self.s[name], _s[name]
            self.assertTrue(
                np.allclose(M.translation, _t, atol=1e-9),
                "Wrong contact translation at " + name,
            )
            self.assertTrue(
                np.allclose(M.rotation, _R, atol=1e-9),
                "Wrong contact rotation at " + name,
            )
            self.assertTrue(
                np.allclose(self.pd[name].vector, _pd[name], atol=1e-9),
                "Wrong contact velocity translation at " + name,
            )
            self.assertTrue(
                np.allclose(F[0], _F[0], atol=1e-9),
                "Wrong contact wrench translation at " + name,
            )
            self.assertTrue(F[1] == _F[1], "Wrong contact type at " + name)
            self.assertTrue(F[2] == _F[2], "Wrong contact status at " + name)
            self.assertTrue(
                np.allclose(S[0], _S[0], atol=1e-9),
                "Wrong contact surface translation at " + name,
            )
            self.assertEqual(
                S[1], _S[1], "Wrong contact friction coefficient at " + name
            )

    def test_communication_with_non_locked_joints(self):
        model = pinocchio.buildSampleModelHumanoid()
        locked_joints = []
        qref = pinocchio.randomConfiguration(model)
        reduced_model = pinocchio.buildReducedModel(
            model, [model.getJointId(name) for name in locked_joints], qref
        )
        sub = WholeBodyStateRosSubscriber(
            model, locked_joints, qref, "whole_body_state"
        )
        pub = WholeBodyStateRosPublisher(model, locked_joints, qref, "whole_body_state")
        time.sleep(1)
        # publish whole-body state messages
        q = pinocchio.randomConfiguration(model)
        q[:3] = np.random.rand(3)
        v = np.random.rand(model.nv)
        tau = np.random.rand(model.nv - 6)
        q, v, tau = toReduced(model, reduced_model, q, v, tau)
        while True:
            pub.publish(self.t, q, v, tau, self.p, self.pd, self.f, self.s)
            if sub.has_new_msg():
                break
        # get whole-body state
        _t, _q, _v, _tau, _p, _pd, _f, _s = sub.get_state()
        self.assertEqual(self.t, _t, "Wrong time interval")
        self.assertTrue(np.allclose(q, _q, atol=1e-9), "Wrong q")
        self.assertTrue(np.allclose(v, _v, atol=1e-9), "Wrong v")
        self.assertTrue(np.allclose(tau, _tau, atol=1e-9), "Wrong tau")
        for name in self.p:
            M, [_t, _R] = self.p[name], _p[name]
            F, _F = self.f[name], _f[name]
            S, _S = self.s[name], _s[name]
            self.assertTrue(
                np.allclose(M.translation, _t, atol=1e-9),
                "Wrong contact translation at " + name,
            )
            self.assertTrue(
                np.allclose(M.rotation, _R, atol=1e-9),
                "Wrong contact rotation at " + name,
            )
            self.assertTrue(
                np.allclose(self.pd[name].vector, _pd[name], atol=1e-9),
                "Wrong contact velocity translation at " + name,
            )
            self.assertTrue(
                np.allclose(F[0], _F[0], atol=1e-9),
                "Wrong contact wrench translation at " + name,
            )
            self.assertTrue(F[1] == _F[1], "Wrong contact type at " + name)
            self.assertTrue(F[2] == _F[2], "Wrong contact status at " + name)
            self.assertTrue(
                np.allclose(S[0], _S[0], atol=1e-9),
                "Wrong contact surface translation at " + name,
            )
            self.assertEqual(
                S[1], _S[1], "Wrong contact friction coefficient at " + name
            )

if __name__ == "__main__":
    if ROS_VERSION == 2:
        unittest.main()
    else:
        rosunit.unitrun("crocoddyl_msgs", "whole_body_state", TestWholeBodyState)
