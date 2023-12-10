import unittest
from MILP_OOP import Vehicle, MILP_Model
import plotting


class TestSystem(unittest.TestCase):
    def setUp(self) -> None:
        pass

    def test_singular_car(self):
        list_vehicles = [Vehicle(1, t0=4, v0=25, d0=100, k='North')]

        test = MILP_Model("single car", list_vehicles)
        test.initialize_variables()
        test.initialize_constraints()
        test.initialize_objective_function()
        test.optimize()
        print(test.getvariables())
        plotting.plot_vehicle_position(test.vehicles)


if __name__ == '__main__':
    unittest.main()