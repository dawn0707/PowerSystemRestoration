# encoding: utf-8
"""
@author: Chen Zhang
@file: pdrppccdt.py
@time: 2019/3/25 17:19
"""
from power_network import PN
from trans_network import TN
from mrsp import MRSP
from rop import ROP
from vehicle import VEHICLE
from gurobipy import *
import numpy as np


class PDRPPCCDT:

    def __init__(self, pn, tn, order, vehicle):
        self.pn = pn
        self.tn = tn
        self.order = order

        self.w_num = len(self.order)
        self.h_num = self.tn.base_num
        self.l_num = 2 * (self.w_num + self.h_num)

        self.vehicle = vehicle

        self.model = Model("PDRPPCCDT")
        self.node_num = self.pn.bus_num + self.pn.gen_num + self.pn.load_num
        self.item_num = self.node_num + self.pn.branch_num
        self.tn_node_num = 2 * (self.tn.base_num + self.tn.w_num)

        self.sigma = []
        self.ve = []
        self.load = []
        self.edt = []
        self.t = []
        self.s = []

        self.initialization()

    def initialization(self):
        self.set_variables()
        self.set_obj()
        self.set_constraints()
        self.optimize()

    def set_variables(self):
        # variable order:
        # W-    repair node
        # W+    stockpiled node
        # H+    base station start
        # H-    base station return
        self.sigma = self.model.addVars(1, self.l_num, lb=0, ub=self.w_num - 1, vtype=GRB.INTEGER, name="sigma")
        self.ve = self.model.addVars(1, self.l_num, lb=0, ub=self.vehicle.num - 1, vtype=GRB.INTEGER, name="ve")
        self.load = self.model.addVars(1, 2 * self.l_num, lb=0, ub=self.vehicle.capacity,
                                       vtype=GRB.INTEGER, name="load")
        self.edt = self.model.addVars(1, self.l_num, lb=0, ub=GRB.INFINITY, vtype=GRB.INTEGER, name="edt")
        self.t = self.model.addVars(self.l_num, self.l_num, vtype=GRB.INTEGER, name="t", lb=0, ub=GRB.INFINITY)
        self.s = self.model.addVars(1, self.l_num, vtype=GRB.INTEGER, name="s", lb=0, ub=GRB.INFINITY)

    def set_obj(self):
        obj = LinExpr()
        for i in range(self.w_num):
            obj += self.edt[0, i]
        self.model.setObjective(obj, GRB.MINIMIZE)

    def set_constraints(self):
        """
        Constraint for t
        """
        # W- - W-
        for i in range(self.w_num):
            i_index = self.order[i]
            for j in range(self.w_num):
                j_index = self.order[j]
                self.model.addConstr(self.t[i, j] == self.tn.travel_time[i_index][j_index])

        # W+ - W+
        for i in range(self.w_num):
            i_index = self.order[i] + self.tn.w_num
            for j in range(self.w_num):
                j_index = self.order[j] + self.tn.w_num
                self.model.addConstr(self.t[i + self.w_num, j + self.w_num] == self.tn.travel_time[i_index][j_index])

        # H+ - H+ and H- - H-
        for i in range(self.h_num * 2):
            i_index = self.tn.w_num * 2 + i
            for j in range(self.h_num * 2):
                j_index = self.tn.w_num * 2 + j
                self.model.addConstr(self.t[i + self.w])

        # W- - W+
        for i in range(self.w_num):
            i_index = self.order[i]
            for j in range(self.w_num):
                j_index = self.order[j] + self.tn.w_num
                self.model.addConstr(self.t[i, j + self.w_num] == self.tn.travel_time[i_index][j_index])
                self.model.addConstr(self.t[j + self.w_num, i] == self.tn.travel_time[j_index][i_index])

        for i in range(self.w_num):
            i_index = self.order[i]
            # W- - H+
            for j in range(self.h_num):
                j_index = self.tn.w_num * 2 + j
                self.model.addConstr(self.t[i, j + self.w_num * 2] == self.tn.travel_time[i_index][j_index])
                self.model.addConstr(self.t[j + self.w_num * 2, i] == self.tn.travel_time[j_index][i_index])
            # W- - H-
            for j in range(self.h_num):
                j_index = self.tn.w_num * 2 + self.h_num + j
                self.model.addConstr(self.t[i, j + self.w_num * 2 + self.h_num] ==
                                     self.tn.travel_time[i_index][j_index])
                self.model.addConstr(self.t[j + self.w_num * 2 + self.h_num, i] ==
                                     self.tn.travel_time[j_index][i_index])




        """
        Constraint (3) - (7)
        """
        for i in range(self.h_num):
            index = i + 2 * self.w_num

            con_name = "Constraint 3 " + str(i)
            self.model.addConstr(self.ve[0, index] == self.ve[0, self.sigma[0, index]], name=con_name)

            con_name = "Constraint 4 " + str(i)
            self.model.addConstr(self.load[0, index] == 0, name=con_name)

            con_name = "Constraint 5 " + str(i)
            self.model.addConstr(self.load[0, self.sigma[0, index]] == 0, name=con_name)

            con_name = "Constraint 6 " + str(i)
            self.model.addConstr(self.edt[0, index] == 0, name=con_name)

            con_name = "Constraint 7 " + str(i)

            # self.model.addConstr(self.edt[0, self.sigma[0, index]] >= self.t[index, self.sigma[0, index]] +
            #                      self.s[0, self.sigma[0, index]], name=con_name)

        """
        Constraint (8) - (10)
        """
        # for i in range(2 * (self.tn.w_num)):
        #     con_name = "Constraint 8" + str(i)
        #     self.model.addConstr(self.ve[0, i] == self.ve[0, self.sigma[0, i]], name=con_name)
        #
        #     con_name = "Constraint 9" + str(i)
        #     if i < self.tn.w_num:
        #         self.model.addConstr(self.load[0, self.sigma[0, i]] == self.load[0, i] - self.tn.)
        #     # else:

    def optimize(self):
        self.model.optimize()
        edt = []
        for i in range(self.w_num):
            var_name = "edt[0," + str(i) + "]"
            var = self.model.getVarByName(var_name)
            edt.append(var.x)
        print("edt:")
        print(edt)

    def display_t(self):
        t_value = []
        print("t value:")
        for i in range(self.l_num):
            value = []
            for j in range(self.l_num):
                var_name = "t[" + str(i) + "," + str(j) + "]"
                var = self.model.getVarByName(var_name)
                value.append(var.x)
            print(value)
            t_value.append(value)


if __name__ == "__main__":
    pn = PN()
    tn = TN(20, 2)

    d_node = [1, 2, 3, 4, 13, 14, 15, 17, ]
    # d_node = [0]
    stage1 = MRSP(pn, d_node)

    minimum_repair_set = stage1.repair
    # minimum_repair_set = [1, 14, 15]
    stage2 = ROP(pn, minimum_repair_set, d_node)


    stage1.display_repair()
    stage2.display_o()
    order = stage2.order

    vehicle = VEHICLE()

    stage3 = PDRPPCCDT(pn, tn, order, vehicle)
    stage3.display_t()
else:
    print("pdrppccdt is implemented into another module.")

