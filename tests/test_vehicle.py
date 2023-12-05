import unittest
from MILP_OOP import Vehicle

class TestVehicle(unittest.TestCase):
    def setUp(self):
        pass

    def test_vehicle_initialization(self):
        vehicle = Vehicle(idx=1, k="North", d0=100, v0=25, t0=5, t_access=10)

        self.assertEqual(vehicle.idx, 1)
        self.assertEqual(vehicle.k, "North")
        self.assertEqual(vehicle.d0, 100)
        self.assertEqual(vehicle.v0, 25)
        self.assertEqual(vehicle.t0, 5)
        self.assertEqual(vehicle.t_access, 10)

    def test_default_vehicle_initialization(self):
        default_vehicle = Vehicle(idx=1)

        self.assertEqual(default_vehicle.idx, 1)

    # Add more tests as needed

if __name__ == '__main__':
    unittest.main()