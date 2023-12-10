import unittest
from MILP_OOP import Vehicle, MILP_Model
import plotting


class TestSystem(unittest.TestCase):
    def setUp(self) -> None:
        pass

    def test_singular_car_desired_time(self):
        d0 = 100
        v0 = 25
        t0 = d0/v0
        list_vehicles = [Vehicle(1, t0=t0, v0=v0, d0=d0, k='North')]

        test = MILP_Model("single car", list_vehicles)
        test.initialize_variables()
        test.initialize_constraints()
        test.initialize_objective_function(w_1=0, w_2=1)
        test.optimize()
        solution = test.getvariables()
        self.assertEqual(solution['t[0]'], t0)

    def test_singular_car_fastest_time(self):
        d0 = 100
        v0 = 25
        t0 = d0/v0
        list_vehicles = [Vehicle(1, t0=t0, v0=v0, d0=d0, k='North')]

        test = MILP_Model("single car", list_vehicles)
        test.initialize_variables()
        test.initialize_constraints()
        test.initialize_objective_function(w_1=0, w_2=1)
        test.optimize()
        solution = test.getvariables()
        self.assertEqual(solution['t[0]'], t0)


if __name__ == '__main__':
    unittest.main()