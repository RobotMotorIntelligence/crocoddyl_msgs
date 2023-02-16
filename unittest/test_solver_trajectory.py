import time
import random
import subprocess, os, signal
import numpy as np
import rospy
from crocoddyl_ros import ControlType, ControlParametrization
from crocoddyl_ros import SolverTrajectoryRosPublisher, SolverTrajectoryRosSubscriber
import unittest


class TestSolverTrajectory(unittest.TestCase):

    def test_bindings(self):
        rospy.init_node('crocoddyl_ros', anonymous=True)
        sub = SolverTrajectoryRosSubscriber()
        pub = SolverTrajectoryRosPublisher()
        time.sleep(1)
        # publish solver trajectory messages
        h = random.uniform(0.1, 0.2)
        ts = np.arange(0., 1., h).tolist()
        N = len(ts)
        dts = [h] * N
        nx = random.randint(4, 20)
        nu = min(nx, random.randint(4, 20))
        xs = [np.random.rand(nx) for _ in range(N)]
        dxs = [np.random.rand(nx) for _ in range(N)]
        us = [np.random.rand(nu) for _ in range(N)]
        Ks = [np.random.rand(nu, nx) for _ in range(N)]
        t = random.randint(0, 1)
        if t == 0:
            type = ControlType.EFFORT
        else:
            type = ControlType.ACCELERATION_CONTACTFORCE
        p = random.randint(0, 2)
        if p == 0:
            param = ControlParametrization.POLYZERO
        elif p == 1:
            param = ControlParametrization.POLYONE
        else:
            param = ControlParametrization.POLYTWO
        types = [type for _ in range(N)]
        params = [param for _ in range(N)]
        for _ in range(2):
            pub.publish(ts, dts, xs, dxs, us, Ks, types, params)
            time.sleep(1)
        # get solver trajectory
        _ts, _dts, _xs, _dxs, _us, _Ks, _types, _params = sub.get_solver_trajectory(
        )
        for i in range(N):
            self.assertEqual(ts[i], _ts[i], "Wrong time interval at " + str(i))
            self.assertEqual(dts[i], _dts[i],
                             "Wrong duration interval at " + str(i))
            self.assertTrue(np.allclose(xs[i], _xs[i], atol=1e-9),
                            "Wrong x at " + str(i))
            self.assertTrue(np.allclose(dxs[i], _dxs[i], atol=1e-9),
                            "Wrong dx at " + str(i))
            self.assertTrue(np.allclose(us[i], _us[i], atol=1e-9),
                            "Wrong u at " + str(i))
            self.assertTrue(np.allclose(Ks[i], _Ks[i], atol=1e-9),
                            "Wrong K at " + str(i))
            self.assertTrue(types[i] == _types[i], "Wrong type at " + str(i))
            self.assertTrue(params[i] == _params[i], "Wrong type at " + str(i))


if __name__ == '__main__':
    roscore = subprocess.Popen("roscore",
                               stdout=subprocess.PIPE,
                               shell=True,
                               preexec_fn=os.setsid)
    unittest.main()
    os.killpg(os.getpgid(roscore.pid), signal.SIGTERM)