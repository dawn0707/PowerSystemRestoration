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

        self.sigma_x = []
        self.ve = []
        self.load = []
        self.edt = []
        self.t = []
        self.s = []
        self.d = []

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
        self.sigma_x = self.model.addVars(self.l_num, self.l_num, vtype=GRB.BINARY, name="sigma_x")
        self.ve = self.model.addVars(1, self.l_num, lb=0, ub=self.vehicle.num - 1, vtype=GRB.INTEGER, name="ve")
        self.load = self.model.addVars(1, self.l_num, lb=0, ub=self.vehicle.capacity,
                                       vtype=GRB.INTEGER, name="load")
        self.edt = self.model.addVars(1, self.l_num, lb=0, ub=GRB.INFINITY, vtype=GRB.INTEGER, name="edt")
        self.t = self.model.addVars(self.l_num, self.l_num, vtype=GRB.INTEGER, name="t", lb=0, ub=GRB.INFINITY)
        self.s = self.model.addVars(1, self.l_num, vtype=GRB.INTEGER, name="s", lb=0, ub=GRB.INFINITY)
        self.d = self.model.addVars(1, self.l_num, vtype=GRB.INTEGER, name="s", lb=0, ub=GRB.INFINITY)

    def set_obj(self):
        obj = LinExpr()
        for i in range(self.w_num):
            obj += self.edt[0, i]
        self.model.setObjective(obj, GRB.MINIMIZE)

    def set_constraints(self):
        self.set_t_variable_value()
        self.set_s_d_variable_value()

        """
        Constraint for sigma_x
        """
        for i in range(self.l_num - self.h_num):
            self.model.addConstr(self.sigma_x[i, i] == 0)
            self.model.addConstr(quicksum(self.sigma_x[i, j] for j in range(self.l_num)) == 1)

        for i in range(self.w_num * 2):
            self.model.addConstr(quicksum(self.sigma_x[j, i] for j in range(self.l_num)) == 1)
        for i in range(self.w_num * 2, self.w_num * 2 + self.h_num):
            self.model.addConstr(quicksum(self.sigma_x[i, j] for j in range(self.w_num * 2)) == 1)
        for i in range(self.w_num * 2 + self.h_num, self.l_num):
            for j in range(self.l_num):
                self.model.addConstr(self.sigma_x[i, j] == 0)

        """
        Vehicle of H+ node
        """
        for i in range(self.vehicle.num):
            self.model.addConstr(self.ve[0, i] == i)

        """
        Constraint (3) - (7)
        """
        for i in range(self.h_num):
            index = i + 2 * self.w_num

            con_name = "Constraint 3 " + str(i)
            for j in range(self.l_num):
                con_name = con_name + " " + str(j)
                self.model.addGenConstrIndicator(self.sigma_x[index, j], True,
                                                 self.ve[0, index] == self.ve[0, j], name=con_name)
            # self.model.addConstr(self.ve[0, index] == self.ve[0, self.sigma[0, index]], name=con_name)

            con_name = "Constraint 4 " + str(i)
            self.model.addConstr(self.load[0, index] == 0, name=con_name)

            con_name = "Constraint 5 " + str(i)
            for j in range(self.l_num):
                con_name = con_name + " " + str(j)
                self.model.addGenConstrIndicator(self.sigma_x[index, j], True, self.load[0, j] == 0, name=con_name)
            # self.model.addConstr(self.load[0, self.sigma[0, index]] == 0, name=con_name)

            con_name = "Constraint 6 " + str(i)
            self.model.addConstr(self.edt[0, index] == 0, name=con_name)

            con_name = "Constraint 7 " + str(i)
            for j in range(self.l_num):
                con_name = con_name + " " + str(j)
                self.model.addGenConstrIndicator(self.sigma_x[index, j], True,
                                                 self.edt[0, j] >= self.t[index, j] + self.s[0, j], name=con_name)
            # self.model.addConstr(self.edt[0, self.sigma[0, index]] == self.t[index, self.sigma[0, index]] +
            #                      self.s[0, self.sigma[0, index]], name=con_name)

        """
        Constraint (8) - (10)
        """
        for i in range(2 * self.w_num):
            con_name = "Constraint 8 " + str(i)
            for j in range(self.l_num):
                con_name = con_name + " " + str(j)
                self.model.addGenConstrIndicator(self.sigma_x[i, j], True,
                                                 self.ve[0, i] == self.ve[0, j], name=con_name)
            # self.model.addConstr(self.ve[0, i] == self.ve[0, self.sigma[0, i]], name=con_name)

            con_name = "Constraint 9 " + str(i)
            for j in range(self.l_num):
                con_name = con_name + " " + str(j)
                if i < self.w_num:
                    self.model.addGenConstrIndicator(self.sigma_x[i, j], True,
                                                     self.load[0, j] == self.load[0, i] - self.d[0, i], name=con_name)
            # self.model.addConstr(self.load[0, self.sigma[0, i]] == self.load[0, i] - self.d[0, i], name=con_name)
                else:
                    self.model.addGenConstrIndicator(self.sigma_x[i, j], True,
                                                     self.load[0, j] == self.load[0, i] + self.d[0, i], name=con_name)
            #     self.model.addConstr(self.load[0, self.sigma[0, i]] == self.load[0, i] + self.d[0, i], name=con_name)

            con_name = "Constraint 10 " + str(i)
            for j in range(self.l_num):
                con_name = con_name + " " + str(j)
                self.model.addGenConstrIndicator(self.sigma_x[i, j], True,
                                                 self.edt[0, j] >= self.s[0, j] + self.t[i, j] + self.edt[0, i],
                                                 name=con_name)
            # self.model.addConstr(self.edt[0, self.sigma[0, i]] >= self.s[0, self.sigma[0, i]] +
            #                      self.t[i, self.sigma[0, i]] + self.edt[0, i], name=con_name)

        """
        Constraint (11) - (12)
        """
        for i in range(self.w_num):
            con_name = "Constraint 11 " + str(i)
            self.model.addConstr(self.ve[0, i] == self.ve[0, i + self.w_num], name=con_name)
            con_name = "Constraint 12 " + str(i)
            self.model.addConstr(self.edt[0, i + self.w_num] <= self.edt[0, i], name=con_name)

        """
        Constraint (13)
        """
        for i in range(self.w_num - 1):
            con_name = "Constraint 13 " + str(i)
            self.model.addConstr(self.edt[0, i] <= self.edt[0, i + 1], name=con_name)

        self.sub_tour_elim()

    def sub_tour_elim(self):
        set_all = self.__private__subset()
        for s in set_all:
            expr = LinExpr()
            for i in s:
                for j in s:
                    expr += self.sigma_x[i, j]
            self.model.addConstr(expr <= len(s) - 1)


    def __private__subset(self):
        set_all = []
        for i in range(2 ** self.l_num):
            combo = []
            for j in range(self.l_num):
                if (i >> j) % 2 == 1:
                    combo.append(j)
            if len(combo) > 1:
                set_all.append(combo)
        return set_all

    def optimize(self):
        self.model.optimize()

    def display_edt(self):
        edt = []
        for i in range(self.l_num):
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

    def display_s(self):
        s_value = []
        print("s value:")
        for i in range(self.l_num):
            var_name = "s[0," + str(i) + "]"
            var = self.model.getVarByName(var_name)
            s_value.append(var.x)
        print(s_value)

    def display_load(self):
        load_value = []
        print("load value:")
        for i in range(self.l_num):
            var_name = "load[0," + str(i) + "]"
            var = self.model.getVarByName(var_name)
            load_value.append(var.x)
        print(load_value)

    def display_sigma(self):
        sigma_value = []
        print("sigma value:")
        for i in range(self.l_num):
            var_name = "sigma[0," + str(i) + "]"
            var = self.model.getVarByName(var_name)
            sigma_value.append(var.x)
        print(sigma_value)

    def display_sigma_x(self):
        sigma_x_value = []
        print("sigma x value:")
        for i in range(self.l_num):
            value = []
            for j in range(self.l_num):
                var_name = "sigma_x[" + str(i) + "," + str(j) + "]"
                var = self.model.getVarByName(var_name)
                value.append(var.x)
            print(value)
            sigma_x_value.append(value)

    def display_vehicle(self):
        ve_value = []
        print("vehicle value:")
        for i in range(self.l_num):
            var_name = "ve[0," + str(i) + "]"
            var = self.model.getVarByName(var_name)
            ve_value.append(var.x)
        print(ve_value)

    def set_t_variable_value(self):
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
                self.model.addConstr(self.t[i + self.w_num * 2, j + self.w_num * 2] ==
                                     self.tn.travel_time[i_index][j_index])

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

        # W+ - H+ and W+ - H-
        for i in range(self.w_num):
            i_index = self.tn.w_num + self.order[i]
            for j in range(2 * self.h_num):
                j_index = self.tn.w_num * 2 + j
                self.model.addConstr(self.t[self.w_num + i, self.w_num * 2 + j] ==
                                     self.tn.travel_time[i_index][j_index])
                self.model.addConstr(self.t[self.w_num * 2 + j, self.w_num + i] ==
                                     self.tn.travel_time[j_index][i_index])

    def set_s_d_variable_value(self):
        for i in range(self.l_num):
            if i < self.w_num:
                i_index_power = self.order[i]
                if i_index_power < self.pn.bus_num:
                    service_time = self.tn.bus_cost[i_index_power][2]
                    demand = self.tn.bus_cost[i_index_power][1]
                elif i_index_power < self.pn.bus_num + self.pn.gen_num:
                    service_time = self.tn.gen_cost[i_index_power - self.pn.bus_num][2]
                    demand = self.tn.gen_cost[i_index_power - self.pn.bus_num][1]
                else:
                    service_time = self.tn.load_cost[i_index_power - self.pn.bus_num - self.pn.gen_num][2]
                    demand = self.tn.load_cost[i_index_power - self.pn.bus_num - self.pn.gen_num][1]
            elif i < 2 * self.w_num:
                # assume the pickup time is 50
                service_time = 50
                demand = 150
            else:
                service_time = 0
                demand = 0
            self.model.addConstr(self.s[0, i] == service_time)
            self.model.addConstr(self.d[0, i] == demand)


if __name__ == "__main__":
    pn = PN()
    tn = TN(20, 2)

    d_node = [1, 2, 3, 4, 13, 14, 15, 17]
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
    # stage3.display_t()
    stage3.display_s()
    stage3.display_load()
    # stage3.display_sigma()
    stage3.display_sigma_x()
    stage3.display_vehicle()
    stage3.display_edt()
    # stage3.subset()
else:
    print("pdrppccdt is implemented into another module.")

