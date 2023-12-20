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

        test = MILP_Model("single car desired", list_vehicles)
        test.initialize_variables()
        test.initialize_constraints()
        test.initialize_objective_function(w_1=0, w_2=1)
        test.optimize()
        solution = test.getvariables()
        self.assertEqual(solution['t[0]'], t0)
        plotting.plot_vehicle_position(list_vehicles)

    def test_singular_car_fastest_time(self):
        d0 = 87.5
        v0 = 0
        t0 = 100
        t_fastest = 8 + 1/3

        list_vehicles = [Vehicle(1, t0=t0, v0=v0, d0=d0, k='North')]
        test = MILP_Model("single car fast", list_vehicles, v_max=15)
        test.initialize_variables()
        test.initialize_constraints()
        test.initialize_objective_function(w_1=1, w_2=0)
        test.optimize()
        solution = test.getvariables()
        self.assertAlmostEqual(solution['t[0]'], t_fastest)

    def test_two_car_same_direction(self):
        list_vehicles = [Vehicle(0, t0=4, v0=10, d0=50, k='North'), Vehicle(1, t0=4.1, v0=10, d0=50, k='North')]
        test = MILP_Model("two cars north", list_vehicles)
        test.initialize_variables()
        test.initialize_constraints()
        test.initialize_objective_function(w_1=0, w_2=1)
        test.optimize()
        solution = test.getvariables()
        self.assertAlmostEqual(solution['t[1]'] - solution['t[0]'], test.t_gap1)

    def test_two_car_opposite_direction(self):
        list_vehicles = [Vehicle(0, t0=4, v0=10, d0=50, k='North'), Vehicle(1, t0=4.1, v0=10, d0=50, k='South')]
        test = MILP_Model("two cars north and south", list_vehicles)
        test.initialize_variables()
        test.initialize_constraints()
        test.initialize_objective_function(w_1=0, w_2=1)
        test.optimize()
        solution = test.getvariables()
        self.assertAlmostEqual(solution['t[1]'], 4.1)
        self.assertAlmostEqual(solution['t[0]'], 4.0)

    def test_two_car_perpendicular_direction(self):
        list_vehicles = [Vehicle(0, t0=4, v0=10, d0=50, k='North'), Vehicle(1, t0=4.1, v0=10, d0=50, k='East')]
        test = MILP_Model("two cars north and east", list_vehicles)
        test.initialize_variables()
        test.initialize_constraints()
        test.initialize_objective_function(w_1=0, w_2=1)
        test.optimize()
        solution = test.getvariables()
        self.assertAlmostEqual(solution['t[1]'] - solution['t[0]'], test.t_gap2)



if __name__ == '__main__':
    unittest.main()