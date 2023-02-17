import time
import random
import rospy
from crocoddyl_ros import SolverStatisticsRosPublisher, SolverStatisticsRosSubscriber
import unittest
import rostest


class TestSolverStatistics(unittest.TestCase):

    def test_bindings(self):
        rospy.init_node('crocoddyl_ros', anonymous=True)
        sub = SolverStatisticsRosSubscriber()
        pub = SolverStatisticsRosPublisher()
        time.sleep(1)
        # publish solver statistics messages
        iterations = random.randint(0, 10)
        total_time = random.uniform(0., 10.)
        solve_time = random.uniform(0., 10.)
        cost = random.uniform(0., 10.)
        regularization = random.uniform(0., 10.)
        step_length = random.uniform(0., 1.)
        dyn_feas = 1e-5 * random.uniform(0., 10.)
        eq_feas = 1e-5 * random.uniform(0., 10.)
        ineq_feas = 1e-5 * random.uniform(0., 10.)
        while True:
            pub.publish(iterations, total_time, solve_time, cost,
                        regularization, step_length, dyn_feas, eq_feas,
                        ineq_feas)
            if sub.has_new_msg():
                break
        # get solver statistics
        _iterations, _total_time, _solve_time, _cost, _regularization, _step_length, _dyn_feas, _eq_feas, _ineq_feas = sub.get_solver_statistics(
        )
        self.assertEqual(iterations, _iterations, "Wrong number of iterations")
        self.assertAlmostEqual(total_time, _total_time, places=5)
        self.assertAlmostEqual(solve_time, _solve_time, places=5)
        self.assertAlmostEqual(cost, _cost, places=5)
        self.assertAlmostEqual(regularization, _regularization, places=5)
        self.assertAlmostEqual(step_length, _step_length, places=5)
        self.assertAlmostEqual(dyn_feas, _dyn_feas, places=5)
        self.assertAlmostEqual(eq_feas, _eq_feas, places=5)
        self.assertAlmostEqual(ineq_feas, _ineq_feas, places=5)


if __name__ == '__main__':
    rostest.rosrun("crocoddyl_msgs", "solver_statistics", TestSolverStatistics)