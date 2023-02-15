import time
import random
import subprocess, os, signal
import rospy
from crocoddyl_ros import SolverStatisticsRosPublisher, SolverStatisticsRosSubscriber
import unittest


class TestSolverStatistics(unittest.TestCase):

    def test_bindings(self):
        
        rospy.init_node('crocoddyl_ros', anonymous=True)
        sub = SolverStatisticsRosSubscriber()
        pub = SolverStatisticsRosPublisher()
        time.sleep(1)

        iterations = int(random.uniform(0., 10.))
        total_time = random.uniform(0., 10.)
        solve_time = random.uniform(0., 10.)
        cost = random.uniform(0., 10.)
        regularization = random.uniform(0., 10.)
        step_length = random.uniform(0., 1.)
        dyn_feas = 1e-5 * random.uniform(0., 10.)
        eq_feas = 1e-5 * random.uniform(0., 10.)
        ineq_feas = 1e-5 * random.uniform(0., 10.)
        for _ in range(2):
            pub.publish(iterations, total_time, solve_time, cost,
                        regularization, step_length, dyn_feas, eq_feas,
                        ineq_feas)
            time.sleep(1)

        _iterations, _total_time, _solve_time, _cost, _regularization, _step_length, _dyn_feas, _eq_feas, _ineq_feas = sub.get_solver_statistics(
        )
        print(sub.get_solver_statistics())
        self.assertEqual(iterations, _iterations)
        self.assertAlmostEqual(total_time, _total_time, places=5)
        self.assertAlmostEqual(solve_time, _solve_time, places=5)
        self.assertAlmostEqual(cost, _cost, places=5)
        self.assertAlmostEqual(regularization, _regularization, places=5)
        self.assertAlmostEqual(step_length, _step_length, places=5)
        self.assertAlmostEqual(dyn_feas, _dyn_feas, places=5)
        self.assertAlmostEqual(eq_feas, _eq_feas, places=5)
        self.assertAlmostEqual(ineq_feas, _ineq_feas, places=5)


if __name__ == '__main__':
    roscore = subprocess.Popen("roscore", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
    unittest.main()
    os.killpg(os.getpgid(roscore.pid), signal.SIGTERM)