import unittest
from gurobipy import Model, GRB, LinExpr, quicksum
from MILP_OOP import MILP_Model
from MILP_OOP import Vehicle

class TestMILPModel(unittest.TestCase):
    def setUp(self):
        pass

    def test_initialization(self):
        list_vehicles = [Vehicle(1), Vehicle(2)]
        milp_model = MILP_Model("test_model", list_vehicles)

        self.assertEqual(milp_model.MILP.getAttr("ModelName"), "test_model")
        self.assertEqual(milp_model.no_vehicles, 2)
        self.assertEqual(len(milp_model.vehicles), 2)

    def test_initialize_variables(self):
        pass

    def test_initialize_constraints(self):
        list_vehicles = [Vehicle(1), Vehicle(2), Vehicle(3), Vehicle(4)]
        milp_model = MILP_Model("test_model", list_vehicles)
        milp_model.initialize_variables()
        milp_model.initialize_constraints()

        self.assertTrue((2, 3) in milp_model.C2.keys())
        self.assertTrue((0, 1) in milp_model.C3.keys())

    def test_initialize_objective_function(self):
        pass

    def test_optimize(self):
        list_vehicles = [Vehicle(1), Vehicle(2)]
        milp_model = MILP_Model("test_model", list_vehicles)
        milp_model.initialize_variables()
        milp_model.initialize_constraints()
        milp_model.initialize_objective_function()

        status = milp_model.optimize()
        self.assertEqual(milp_model.MILP.status, GRB.OPTIMAL)

    def test_getvariables(self):
        pass


if __name__ == '__main__':
    unittest.main()