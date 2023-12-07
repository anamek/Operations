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
        no_vehicles = 4
        list_vehicles = []
        for i in range(no_vehicles):
            list_vehicles.append(Vehicle(i))
        milp_model = MILP_Model("test_model", list_vehicles)
        milp_model.initialize_variables()

        all_vars = milp_model.MILP.getVars()
        self.assertEqual(len(all_vars), no_vehicles+no_vehicles*(no_vehicles-1))

        names = milp_model.MILP.getAttr("VarName", all_vars)
        t_vars = [x for x in names if x.startswith('t')]
        B_vars = [x for x in names if x.startswith('B')]
        self.assertEqual(len(t_vars), no_vehicles)
        self.assertEqual(len(B_vars), no_vehicles*(no_vehicles-1))

    def test_initialize_constraints(self):
        list_vehicles = [Vehicle(1), Vehicle(2), Vehicle(3), Vehicle(4)]
        milp_model = MILP_Model("test_model", list_vehicles)
        milp_model.initialize_variables()
        milp_model.initialize_constraints()

        self.assertTrue((2, 3) in milp_model.C2.keys())
        self.assertTrue((0, 1) in milp_model.C3.keys())

    def test_initialize_objective_function(self):
        list_vehicles = [Vehicle(1), Vehicle(2), Vehicle(3), Vehicle(4)]
        milp_model = MILP_Model("test_model", list_vehicles)
        milp_model.initialize_variables()
        milp_model.initialize_constraints()
        milp_model.initialize_objective_function()
        all_vars = milp_model.MILP.getVars()
        var_names = milp_model.MILP.getAttr("VarName", all_vars)
        J1_vars = [x for x in var_names if x == 'slack_delta_t_access']
        J2_vars = [x for x in var_names if x.startswith('slack_delta_t_access_abs')]
        self.assertEqual(len(J1_vars), 1)
        self.assertEqual(len(J2_vars), milp_model.no_vehicles)
        all_cons = milp_model.MILP.getConstrs()
        cons_names = milp_model.MILP.getAttr("ConstrName", all_cons)
        all_J_cons = [x for x in cons_names if x.startswith('cons_t_access')]
        self.assertEqual(len(all_J_cons), milp_model.no_vehicles*3)
        J2_cons = [x for x in cons_names if x.startswith('cons_t_access_pos_difference') or
                   x.startswith('cons_t_access_neg_difference')]
        self.assertEqual(len(J2_cons), milp_model.no_vehicles*2)

    def test_optimize(self):
        list_vehicles = [Vehicle(1), Vehicle(2)]
        milp_model = MILP_Model("test_model", list_vehicles)
        milp_model.initialize_variables()
        milp_model.initialize_constraints()
        milp_model.initialize_objective_function()

        status = milp_model.optimize()
        self.assertEqual(milp_model.MILP.status, GRB.OPTIMAL)

    def test_getvariables(self):
        no_vehicles = 4
        list_vehicles = []
        for i in range(no_vehicles):
            list_vehicles.append(Vehicle(i))
        milp_model = MILP_Model("test_model", list_vehicles)
        milp_model.initialize_variables()
        milp_model.initialize_constraints()
        milp_model.initialize_objective_function()
        status = milp_model.optimize()
        solution = milp_model.getvariables()

        t_var = [name for [name, value] in solution if name.startswith('t')]
        B_var = [name for [name, value] in solution if name.startswith('B')]
        J1_var = [name for [name, value] in solution if name == 'slack_delta_t_access']
        self.assertEqual(len(t_var), no_vehicles)
        self.assertEqual(len(B_var), no_vehicles*(no_vehicles-1))
        self.assertEqual(len(J1_var), 1)





if __name__ == '__main__':
    unittest.main()