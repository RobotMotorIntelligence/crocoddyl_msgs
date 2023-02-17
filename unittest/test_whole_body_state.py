import time
import random
import numpy as np
import rospy
import pinocchio
from crocoddyl_ros import ContactType, ContactStatus
from crocoddyl_ros import WholeBodyStateRosPublisher, WholeBodyStateRosSubscriber
import unittest
import rostest


class TestWholeBodyState(unittest.TestCase):

    def test_bindings(self):
        rospy.init_node('crocoddyl_ros', anonymous=True)
        model = pinocchio.buildSampleModelHumanoid()
        sub = WholeBodyStateRosSubscriber(model)
        pub = WholeBodyStateRosPublisher(model)
        time.sleep(1)
        # publish whole-body state messages
        t = random.uniform(0, 1)
        q = pinocchio.randomConfiguration(model)
        q[:3] = np.random.rand(3)
        v = np.random.rand(model.nv)
        tau = np.random.rand(model.nv - 6)
        p = {
            'lleg_effector_body': pinocchio.SE3.Random(),
            'rleg_effector_body': pinocchio.SE3.Random()
        }
        pd = {
            'lleg_effector_body': pinocchio.Motion.Random(),
            'rleg_effector_body': pinocchio.Motion.Random()
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
        f = {
            'lleg_effector_body':
            [pinocchio.Force.Random(), contact_type, contact_status],
            'rleg_effector_body':
            [pinocchio.Force.Random(), contact_type, contact_status]
        }
        s = {
            'lleg_effector_body': [np.random.rand(3),
                                   random.uniform(0, 1)],
            'rleg_effector_body': [np.random.rand(3),
                                   random.uniform(0, 1)]
        }
        while True:
            pub.publish(t, q, v, tau, p, pd, f, s)
            if sub.has_new_msg():
                break
        # get whole-body state
        _t, _q, _v, _tau, _p, _pd, _f, _s = sub.get_state()
        self.assertEqual(t, _t, "Wrong time interval")
        self.assertTrue(np.allclose(q, _q, atol=1e-9), "Wrong q")
        self.assertTrue(np.allclose(v, _v, atol=1e-9), "Wrong v")
        self.assertTrue(np.allclose(tau, _tau, atol=1e-9), "Wrong tau")
        for name in p:
            M, [_t, _R] = p[name], _p[name]
            F, _F = f[name], _f[name]
            S, _S = s[name], _s[name]
            self.assertTrue(np.allclose(M.translation, _t, atol=1e-9),
                            "Wrong contact translation at " + name)
            self.assertTrue(np.allclose(M.rotation, _R, atol=1e-9),
                            "Wrong contact rotation at " + name)
            self.assertTrue(np.allclose(pd[name].vector, _pd[name], atol=1e-9),
                            "Wrong contact velocity translation at " + name)
            self.assertTrue(np.allclose(F[0], _F[0], atol=1e-9),
                            "Wrong contact wrench translation at " + name)
            self.assertTrue(F[1] == _F[1], "Wrong contact type at " + name)
            self.assertTrue(F[2] == _F[2], "Wrong contact status at " + name)
            self.assertTrue(np.allclose(S[0], _S[0], atol=1e-9),
                            "Wrong contact surface translation at " + name)
            self.assertEqual(S[1], _S[1],
                             "Wrong contact friction coefficient at " + name)


if __name__ == '__main__':
    rostest.rosrun("crocoddyl_msgs", "whole_body_state", TestWholeBodyState)