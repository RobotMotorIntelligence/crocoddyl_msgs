import time
import random
import numpy as np
import rospy
import pinocchio
from crocoddyl_ros import ContactType, ContactStatus
from crocoddyl_ros import WholeBodyTrajectoryRosPublisher, WholeBodyTrajectoryRosSubscriber
import unittest
import rosunit


class TestWholeBodyTrajectory(unittest.TestCase):

    def test_bindings(self):
        rospy.init_node('crocoddyl_ros', anonymous=True)
        model = pinocchio.buildSampleModelHumanoid()
        sub = WholeBodyTrajectoryRosSubscriber(model)
        pub = WholeBodyTrajectoryRosPublisher(model)
        time.sleep(1)
        # publish whole-body trajectory messages
        h = random.uniform(0.1, 0.2)
        ts = np.arange(0., 1., h).tolist()
        N = len(ts)
        xs, ps, pds, fs, ss = [], [], [], [], []
        for _ in range(N):
            q = pinocchio.randomConfiguration(model)
            q[:3] = np.random.rand(3)
            v = np.random.rand(model.nv)
            xs.append(np.hstack([q, v]))
            ps.append({
                'lleg_effector_body': pinocchio.SE3.Random(),
                'rleg_effector_body': pinocchio.SE3.Random()
            })
            pds.append({
                'lleg_effector_body': pinocchio.Motion.Random(),
                'rleg_effector_body': pinocchio.Motion.Random()
            })
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
            fs.append({
                'lleg_effector_body':
                [pinocchio.Force.Random(), contact_type, contact_status],
                'rleg_effector_body':
                [pinocchio.Force.Random(), contact_type, contact_status]
            })
            ss.append({
                'lleg_effector_body':
                [np.random.rand(3), random.uniform(0, 1)],
                'rleg_effector_body':
                [np.random.rand(3), random.uniform(0, 1)]
            })
        us = [np.random.rand(model.nv - 6) for _ in range(N)]
        while True:
            pub.publish(ts, xs, us, ps, pds, fs, ss)
            if sub.has_new_msg():
                break
        # get whole-body trajectory
        _ts, _xs, _us, _ps, _pds, _fs, _ss = sub.get_trajectory()
        for i in range(N):
            self.assertEqual(ts[i], _ts[i], "Wrong time interval at " + str(i))
            self.assertTrue(np.allclose(xs[i], _xs[i], atol=1e-9),
                            "Wrong x at " + str(i))
            self.assertTrue(np.allclose(us, _us, atol=1e-9),
                            "Wrong u at " + str(i))
            for name in ps[i]:
                M, [_t, _R] = ps[i][name], _ps[i][name]
                F, _F = fs[i][name], _fs[i][name]
                S, _S = ss[i][name], _ss[i][name]
                self.assertTrue(
                    np.allclose(M.translation, _t, atol=1e-9),
                    "Wrong contact translation at " + name + ", " + str(i))
                self.assertTrue(
                    np.allclose(M.rotation, _R, atol=1e-9),
                    "Wrong contact rotation at " + name + ", " + str(i))
                self.assertTrue(
                    np.allclose(pds[i][name].vector, _pds[i][name], atol=1e-9),
                    "Wrong contact velocity translation at " + name + ", " +
                    str(i))
                self.assertTrue(
                    np.allclose(F[0], _F[0], atol=1e-9),
                    "Wrong contact wrench translation at " + name + ", " +
                    str(i))
                self.assertTrue(
                    F[1] == _F[1],
                    "Wrong contact type at " + name + ", " + str(i))
                self.assertTrue(
                    F[2] == _F[2],
                    "Wrong contact status at " + name + ", " + str(i))
                self.assertTrue(
                    np.allclose(S[0], _S[0], atol=1e-9),
                    "Wrong contact surface translation at " + name + ", " +
                    str(i))
                self.assertEqual(
                    S[1], _S[1], "Wrong contact friction coefficient at " +
                    name + ", " + str(i))


if __name__ == '__main__':
    rosunit.unitrun("crocoddyl_msgs", "whole_body_trajectory",
                    TestWholeBodyTrajectory)
