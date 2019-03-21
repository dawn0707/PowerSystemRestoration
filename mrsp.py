# encoding: utf-8
"""
@author: Chen Zhang
@file: power_flow_calculation_gurobi.py
@time: 2019/3/12 20:32
"""
from power_network import PN
import math
from gurobipy import *


class MRSP():

    def __init__(self, pn, damaged_node):
        self.pn = pn
        # self.damaged_line = damaged_line
        self.damaged_node = damaged_node

        self.y = []
        self.z = []
        self.pl = []
        self.pv_gen = []
        self.pv_load = []
        self.theta = []

        self.model = Model("MRSP")
        self.node_num = self.pn.bus_num + self.pn.gen_num + self.pn.load_num
        self.y_num = self.node_num + self.pn.branch_num
        self.z_num = self.y_num

        self.y_value = []
        self.repair = []
        self.not_repair = []

        self.initialization()

    def initialization(self):
        self.set_damaged_node()
        self.set_variables()
        self.set_obj()
        self.set_constraints()
        self.optimize()

    def set_variables(self):
        self.y = self.model.addVars(1, self.y_num, vtype=GRB.BINARY, name="y")
        self.z = self.model.addVars(1, self.z_num, vtype=GRB.BINARY, name="z")
        self.pl = self.model.addVars(1, self.pn.branch_num, lb=-GRB.INFINITY,  ub=GRB.INFINITY, vtype=GRB.CONTINUOUS,
                                     name="pl")

        for i in range(len(self.pn.gen)):
            var_name = "pv_gen[" + str(i) + "]"
            # x = self.model.addVar(lb=self.pn.gen[i][9], ub=self.pn.gen[i][8], vtype=GRB.CONTINUOUS, name=var_name)
            x = self.model.addVar(lb=0, ub=self.pn.gen[i][8], vtype=GRB.CONTINUOUS, name=var_name)
            self.pv_gen.append(x)

        for i in range(len(self.pn.load)):
            var_name = "pv_load[" + str(i) + "]"
            x = self.model.addVar(lb=0, ub=self.pn.load[i][2], vtype=GRB.CONTINUOUS, name=var_name)
            self.pv_load.append(x)

        self.theta = self.model.addVars(1, self.pn.bus_num, lb=-2 * math.pi, ub=2 * math.pi, vtype=GRB.CONTINUOUS,
                                        name="theta")

    def set_obj(self):
        obj = LinExpr()
        for i in range(self.y_num):
            obj += self.y[0, i]
        self.model.setObjective(obj, GRB.MINIMIZE)

    def set_constraints(self):
        """
        Constraint (2) in Model 2
        """
        max_flow = 2850
        constr2 = LinExpr()
        for i in range(self.pn.load_num):
            constr2 += self.pv_load[i]
        self.model.addConstr(constr2 == max_flow, name="Constraint 2")

        """
        Constraint (3) in Model 2
        """
        for i in range(self.node_num):
            if i not in list(self.damaged_node):
                con_name = "Constraint 3" + str(i)
                self.model.addConstr(self.y[0, i] == 1, name=con_name)

        """
        Constraint (4) in Model 1
        """
        for i in range(self.pn.bus_num):
            con_name = "Constraint 4" + str(i)
            self.model.addConstr(self.y[0, i] == self.z[0, i], name=con_name)

        """
        Constraint (5) in Model 1
        """
        # y order: bus gen load line
        for i in range(self.pn.gen_num):
            bus_num = self.pn.gen[i][0] - 1
            con_name = "Constraint 5 gen" + str(i)
            self.model.addGenConstrAnd(self.z[0, i + self.pn.bus_num],
                                       [self.y[0, i + self.pn.bus_num], self.y[0, bus_num]], name=con_name)

        for i in range(self.pn.load_num):
            bus_num = self.pn.load[i][1] - 1
            con_name = "Constraint 5 load " + str(i)
            self.model.addGenConstrAnd(self.z[0, i + self.pn.bus_num + self.pn.gen_num],
                                       [self.y[0, i + self.pn.bus_num + self.pn.gen_num], self.y[0, bus_num]],
                                       name=con_name)

        """
        Constraint (6) in Model 1
        """
        for i in range(self.pn.branch_num):
            bus_from = self.pn.branch[i][0] - 1
            bus_to = self.pn.branch[i][1] - 1
            con_name = "Constraint 6 " + str(i)
            self.model.addGenConstrAnd(self.z[0, i + self.pn.bus_num + self.pn.gen_num + self.pn.load_num],
                                       [self.y[0, i + self.pn.bus_num + self.pn.gen_num + self.pn.load_num],
                                        self.y[0, bus_from], self.y[0, bus_to]], name=con_name)

        """
        Constraint (7) in Model 1
        """
        for i in range(self.pn.bus_num):
            load_set = []
            for j in range(len(self.pn.load)):
                if self.pn.load[j][1] - 1 == i:
                    load_set.append(j)
            gen_set = []
            for j in range(len(self.pn.gen)):
                if self.pn.gen[j][0] - 1 == i:
                    gen_set.append(j)
            li_set = []
            lo_set = []
            for j in range(len(self.pn.branch)):
                if self.pn.branch[j][0] - 1 == i:
                    li_set.append(j)
                if self.pn.branch[j][1] - 1 == i:
                    lo_set.append(j)

            expr = LinExpr()
            for index in load_set:
                expr -= self.pv_load[index]
            for index in gen_set:
                expr += self.pv_gen[index]
            for index in li_set:
                expr += self.pl[0, index]
            for index in lo_set:
                expr -= self.pl[0, index]

            con_name = "Constraint 7 " + str(i)
            self.model.addConstr(expr == 0, name=con_name)

        """
        Constraint 8 in Model (1)
        """
        for i in range(self.pn.gen_num):
            con_name = "Constraint 8 gen " + str(i)
            self.model.addGenConstrIndicator(self.z[0, i + self.pn.bus_num], False,
                                             self.pv_gen[i] == 0, name=con_name)
        for i in range(self.pn.load_num):
            con_name = "Constraint 8 gen " + str(i)
            self.model.addGenConstrIndicator(self.z[0, i + self.pn.bus_num + self.pn.gen_num],
                                             False, self.pv_load[i] == 0, name=con_name)

        """
        Constraint 9 in Model (1)
        """
        for i in range(self.pn.branch_num):
            con_name = "Constraint 9 " + str(i)
            self.model.addGenConstrIndicator(self.z[0, i + self.pn.bus_num + self.pn.gen_num + self.pn.load_num],
                                             False, self.pl[0, i] == 0, name=con_name)

        """
        Constraint 10 and 11 in Model (1)
        """
        for i in range(self.pn.branch_num):
            if self.pn.branch[i][4] > 0:
                con_name = "Constraint 10 " + str(i)
                to_bus = self.pn.branch[i][1] - 1
                from_bus = self.pn.branch[i][0] - 1
                self.model.addGenConstrIndicator(self.z[0, i + self.pn.bus_num + self.pn.gen_num + self.pn.load_num],
                                                 True, self.pl[0, i] == self.pn.branch[i][4] * 100 *
                                                 (self.theta[0, to_bus] - self.theta[0, from_bus]), name=con_name)

    def optimize(self):
        self.model.optimize()
        # for v in self.model.getVars():
        #     print(v.varName, v.x)
        for i in range(self.y_num):
            var_name = "y[0," + str(i) + "]"
            var = self.model.getVarByName(var_name)
            self.y_value.append(var.x)
        # print(self.y_value)
        for i in range(len(self.y_value)):
            if self.y_value[i] == 1 and i in self.damaged_node:
                self.repair.append(i)
            else:
                self.not_repair.append(i)
        # print(self.repair)
        # print(self.not_repair)
        # print('Obj:', self.model.objVal)

    def set_damaged_node(self):
        for node in self.damaged_node:
            self.pn.branch[node][10] = 0

    def display_repair(self):
        print("Minimum Restoration Set:")
        # for i in self.repair:
        #     print(i)
        print(self.repair)


if __name__ == "__main__":
    pn = PN()
    # d_line = [0, 1]
    d_node = [1, 2]
    # d_node = []
    opf = MRSP(pn, d_node)
    opf.display_repair()
    # print(opf.pn.bus_num)
    # print(opf.pn.gen_num)
    # print(opf.pn.load_num)
else:
    print("power_network is implemented into another module.")
    