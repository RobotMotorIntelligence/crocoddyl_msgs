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
    WholeBodyTrajectoryRosPublisher,
    WholeBodyTrajectoryRosSubscriber,
    toReduced,
)


class TestWholeBodyTrajectory(unittest.TestCase):
    def setUp(self) -> None:
        if ROS_VERSION == 2:
            rclpy.init()
        else:
            rospy.init_node("crocoddyl_ros", anonymous=True)
        # Create random trajectories
        h = random.uniform(0.1, 0.2)
        self.ts = np.arange(0.0, 1.0, h).tolist()
        N = len(self.ts)
        self.ps, self.pds, self.fs, self.ss = [], [], [], []
        for _ in range(N):
            self.ps.append(
                {
                    "lleg_effector_body": pinocchio.SE3.Random(),
                    "rleg_effector_body": pinocchio.SE3.Random(),
                }
            )
            self.pds.append(
                {
                    "lleg_effector_body": pinocchio.Motion.Random(),
                    "rleg_effector_body": pinocchio.Motion.Random(),
                }
            )
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
            self.fs.append(
                {
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
            )
            self.ss.append(
                {
                    "lleg_effector_body": [np.random.rand(3), random.uniform(0, 1)],
                    "rleg_effector_body": [np.random.rand(3), random.uniform(0, 1)],
                }
            )

    def test_communication(self):
        model = pinocchio.buildSampleModelHumanoid()
        sub = WholeBodyTrajectoryRosSubscriber(model, "whole_body_trajectory")
        pub = WholeBodyTrajectoryRosPublisher(model, "whole_body_trajectory")
        time.sleep(1)
        # publish whole-body trajectory messages
        N = len(self.ts)
        xs = []
        for _ in range(N):
            q = pinocchio.randomConfiguration(model)
            q[:3] = np.random.rand(3)
            v = np.random.rand(model.nv)
            xs.append(np.hstack([q, v]))
        us = [np.random.rand(model.nv - 6) for _ in range(N)]
        while True:
            pub.publish(self.ts, xs, us, self.ps, self.pds, self.fs, self.ss)
            if sub.has_new_msg():
                break
        # get whole-body trajectory
        _ts, _xs, _us, _ps, _pds, _fs, _ss = sub.get_trajectory()
        for i in range(N):
            self.assertEqual(self.ts[i], _ts[i], "Wrong time interval at " + str(i))
            self.assertTrue(
                np.allclose(xs[i], _xs[i], atol=1e-9), "Wrong x at " + str(i)
            )
            self.assertTrue(np.allclose(us, _us, atol=1e-9), "Wrong u at " + str(i))
            for name in self.ps[i]:
                M, [_t, _R] = self.ps[i][name], _ps[i][name]
                F, _F = self.fs[i][name], _fs[i][name]
                S, _S = self.ss[i][name], _ss[i][name]
                self.assertTrue(
                    np.allclose(M.translation, _t, atol=1e-9),
                    "Wrong contact translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(M.rotation, _R, atol=1e-9),
                    "Wrong contact rotation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(self.pds[i][name].vector, _pds[i][name], atol=1e-9),
                    "Wrong contact velocity translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(F[0], _F[0], atol=1e-9),
                    "Wrong contact wrench translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    F[1] == _F[1], "Wrong contact type at " + name + ", " + str(i)
                )
                self.assertTrue(
                    F[2] == _F[2], "Wrong contact status at " + name + ", " + str(i)
                )
                self.assertTrue(
                    np.allclose(S[0], _S[0], atol=1e-9),
                    "Wrong contact surface translation at " + name + ", " + str(i),
                )
                self.assertEqual(
                    S[1],
                    _S[1],
                    "Wrong contact friction coefficient at " + name + ", " + str(i),
                )

    def test_communication_with_reduced_model(self):
        model = pinocchio.buildSampleModelHumanoid()
        locked_joints = ["larm_elbow_joint", "rarm_elbow_joint"]
        qref = pinocchio.randomConfiguration(model)
        reduced_model = pinocchio.buildReducedModel(
            model, [model.getJointId(name) for name in locked_joints], qref
        )
        sub = WholeBodyTrajectoryRosSubscriber(
            model, locked_joints, qref, "whole_body_trajectory"
        )
        pub = WholeBodyTrajectoryRosPublisher(
            model, locked_joints, qref, "whole_body_trajectory"
        )
        time.sleep(1)
        # publish whole-body trajectory messages
        N = len(self.ts)
        xs, us = [], []
        for _ in range(N):
            q = pinocchio.randomConfiguration(model)
            q[:3] = np.random.rand(3)
            v = np.random.rand(model.nv)
            tau = np.random.rand(model.nv - 6)
            q, v, tau = toReduced(model, reduced_model, q, v, tau)
            xs.append(np.hstack([q, v]))
            us.append(tau)
        while True:
            pub.publish(self.ts, xs, us, self.ps, self.pds, self.fs, self.ss)
            if sub.has_new_msg():
                break
        # get whole-body trajectory
        _ts, _xs, _us, _ps, _pds, _fs, _ss = sub.get_trajectory()
        for i in range(N):
            self.assertEqual(self.ts[i], _ts[i], "Wrong time interval at " + str(i))
            self.assertTrue(
                np.allclose(xs[i], _xs[i], atol=1e-9), "Wrong x at " + str(i)
            )
            self.assertTrue(np.allclose(us, _us, atol=1e-9), "Wrong u at " + str(i))
            for name in self.ps[i]:
                M, [_t, _R] = self.ps[i][name], _ps[i][name]
                F, _F = self.fs[i][name], _fs[i][name]
                S, _S = self.ss[i][name], _ss[i][name]
                self.assertTrue(
                    np.allclose(M.translation, _t, atol=1e-9),
                    "Wrong contact translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(M.rotation, _R, atol=1e-9),
                    "Wrong contact rotation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(self.pds[i][name].vector, _pds[i][name], atol=1e-9),
                    "Wrong contact velocity translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(F[0], _F[0], atol=1e-9),
                    "Wrong contact wrench translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    F[1] == _F[1], "Wrong contact type at " + name + ", " + str(i)
                )
                self.assertTrue(
                    F[2] == _F[2], "Wrong contact status at " + name + ", " + str(i)
                )
                self.assertTrue(
                    np.allclose(S[0], _S[0], atol=1e-9),
                    "Wrong contact surface translation at " + name + ", " + str(i),
                )
                self.assertEqual(
                    S[1],
                    _S[1],
                    "Wrong contact friction coefficient at " + name + ", " + str(i),
                )

    def test_communication_with_non_locked_joints(self):
        model = pinocchio.buildSampleModelHumanoid()
        locked_joints = ["larm_elbow_joint", "rarm_elbow_joint"]
        qref = pinocchio.randomConfiguration(model)
        reduced_model = pinocchio.buildReducedModel(
            model, [model.getJointId(name) for name in locked_joints], qref
        )
        sub = WholeBodyTrajectoryRosSubscriber(
            model, locked_joints, qref, "whole_body_trajectory"
        )
        pub = WholeBodyTrajectoryRosPublisher(
            model, locked_joints, qref, "whole_body_trajectory"
        )
        time.sleep(1)
        # publish whole-body trajectory messages
        N = len(self.ts)
        xs, us = [], []
        for _ in range(N):
            q = pinocchio.randomConfiguration(model)
            q[:3] = np.random.rand(3)
            v = np.random.rand(model.nv)
            tau = np.random.rand(model.nv - 6)
            q, v, tau = toReduced(model, reduced_model, q, v, tau)
            xs.append(np.hstack([q, v]))
            us.append(tau)
        while True:
            pub.publish(self.ts, xs, us, self.ps, self.pds, self.fs, self.ss)
            if sub.has_new_msg():
                break
        # get whole-body trajectory
        _ts, _xs, _us, _ps, _pds, _fs, _ss = sub.get_trajectory()
        for i in range(N):
            self.assertEqual(self.ts[i], _ts[i], "Wrong time interval at " + str(i))
            self.assertTrue(
                np.allclose(xs[i], _xs[i], atol=1e-9), "Wrong x at " + str(i)
            )
            self.assertTrue(np.allclose(us, _us, atol=1e-9), "Wrong u at " + str(i))
            for name in self.ps[i]:
                M, [_t, _R] = self.ps[i][name], _ps[i][name]
                F, _F = self.fs[i][name], _fs[i][name]
                S, _S = self.ss[i][name], _ss[i][name]
                self.assertTrue(
                    np.allclose(M.translation, _t, atol=1e-9),
                    "Wrong contact translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(M.rotation, _R, atol=1e-9),
                    "Wrong contact rotation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(self.pds[i][name].vector, _pds[i][name], atol=1e-9),
                    "Wrong contact velocity translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(F[0], _F[0], atol=1e-9),
                    "Wrong contact wrench translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    F[1] == _F[1], "Wrong contact type at " + name + ", " + str(i)
                )
                self.assertTrue(
                    F[2] == _F[2], "Wrong contact status at " + name + ", " + str(i)
                )
                self.assertTrue(
                    np.allclose(S[0], _S[0], atol=1e-9),
                    "Wrong contact surface translation at " + name + ", " + str(i),
                )
                self.assertEqual(
                    S[1],
                    _S[1],
                    "Wrong contact friction coefficient at " + name + ", " + str(i),
                )


if __name__ == "__main__":
    if ROS_VERSION == 2:
        unittest.main()
    else:
        rosunit.unitrun(
            "crocoddyl_msgs", "whole_body_trajectory", TestWholeBodyTrajectory
        )
