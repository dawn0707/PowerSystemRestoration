# encoding: utf-8
"""
@author: Chen Zhang
@author: Juncheng Li
@file: rop.py
@time: 2019/3/17 17:03
"""

from power_network import PN
from mrsp import MRSP
from gurobipy import *


class ROP:

    def __init__(self, pn, repair, damaged_node):
        self.pn = pn
        self.damaged_node = damaged_node
        self.repair = repair
        self.order = []

        self.flow = []
        self.o = []
        self.y = []
        self.z = []
        self.pl = []
        self.pv_gen = []
        self.pv_load = []
        self.theta = []

        self.model = Model("ROP")
        self.node_num = self.pn.bus_num + self.pn.gen_num + self.pn.load_num
        self.item_num = self.node_num + self.pn.branch_num
        self.r_num = len(self.repair)

        self.initialization()

    def initialization(self):
        self.set_damaged_node()
        self.set_variables()
        self.set_obj()
        self.set_constraints()
        self.optimize()

    def set_variables(self):
        self.flow = self.model.addVars(1, self.r_num, vtype=GRB.CONTINUOUS, name="flow")
        self.o = self.model.addVars(self.item_num, self.r_num, vtype=GRB.BINARY, name="o")
        self.y = self.model.addVars(self.item_num, self.r_num, vtype=GRB.BINARY, name="y")
        self.z = self.model.addVars(self.item_num, self.r_num, vtype=GRB.BINARY, name="z")

        # self.pl = self.model.addVars(self.pn.branch_num, self.r_num, vtype=GRB.CONTINUOUS, name="pl")
        for i in range(self.pn.branch_num):
            var_name = "pl[" + str(i) + "]"
            x = self.model.addVars(1, self.r_num, lb=0, ub=self.pn.branch[i][5], vtype=GRB.CONTINUOUS, name=var_name)
            self.pl.append(x)

        for i in range(len(self.pn.gen)):
            var_name = "pv_gen[" + str(i) + "]"
            # x = self.model.addVars(1, self.r_num, lb=self.pn.gen[i][9], ub=self.pn.gen[i][8],
            #                        vtype=GRB.CONTINUOUS, name=var_name)
            x = self.model.addVars(1, self.r_num, lb=0, ub=self.pn.gen[i][8],
                                   vtype=GRB.CONTINUOUS, name=var_name)
            self.pv_gen.append(x)
        for i in range(len(self.pn.load)):
            var_name = "pv_load[" + str(i) + "]"
            x = self.model.addVars(1, self.r_num, lb=0, ub=self.pn.load[i][2],
                                   vtype=GRB.CONTINUOUS, name=var_name)
            self.pv_load.append(x)

        self.theta = self.model.addVars(self.pn.bus_num, self.r_num, lb=-2 * math.pi, ub=2 * math.pi,
                                        vtype=GRB.CONTINUOUS, name="theta")

    def set_obj(self):
        max_flow = 1965.527380937
        obj = LinExpr()
        # obj += max_flow - self.flow[0, self.r_num - 1]
        for i in range(self.r_num):
            obj += (max_flow - self.flow[0, i])
        self.model.setObjective(obj, GRB.MINIMIZE)

    def set_constraints(self):
        self.model.addConstr(self.flow[0, self.r_num - 1] == 1965.527380937)
        """
        Constraint (2) in Model 3
        """
        for k in range(self.r_num):
            constr2 = LinExpr()
            con_name = "Constraint 2 " + str(k)
            for i in range(self.pn.load_num):
                constr2 += self.pv_load[i][0, k]
            self.model.addConstr(constr2 == self.flow[0, k], name=con_name)

        """
        Constraint (3) in Model 3
        """
        for k in range(self.r_num):
            constr3 = LinExpr()
            con_name = "Constraint 3 " + str(k)
            for r in self.repair:
                constr3 += self.o[r, k]
            print("k")
            print(k)
            self.model.addConstr(constr3 == k + 1, name=con_name)

        """
        Constraint (4) in Model 3
        """
        for r in range(self.r_num):
            index = self.repair[r]
            for k in range(self.r_num - 1):
                con_name = "Constraint 4 " + str(r) + " " + str(k)
                self.model.addConstr(self.o[index, k] <= self.o[index, k + 1], name=con_name)

        """
        Constraint (5) in Model 3
        """
        for index in self.damaged_node:
            for k in range(self.r_num):
                con_name = "Constraint 5 " + str(index) + " " + str(k)
                self.model.addConstr(self.y[index, k] <= self.o[index, k], name=con_name)

        """
        Constraint (6) in Model 3
        """
        for k in range(self.r_num):
            for i in range(self.item_num):
                if i not in self.damaged_node:
                    con_name = "Constraint 6 " + str(i) + " " + str(k)
                    self.model.addConstr(self.y[i, k] == 1, name=con_name)

        """
        Constraint (7) in Model 3
        """
        for k in range(self.r_num):
            for index in self.damaged_node:
                if index not in self.repair:
                    con_name = "Constraint 7 " + str(index) + " " + str(k)
                    self.model.addConstr(self.y[index, k] == 0, name=con_name)

        """
        Constraint (8) in Model 3
        """
        for k in range(self.r_num):
            for i in range(self.pn.bus_num):
                con_name = "Constraint 8 " + str() + " " + str(k)
                self.model.addConstr(self.y[i, k] == self.z[i, k], name=con_name)

        """
        Constraint (9) in Model 3
        """
        for k in range(self.r_num):
            for i in range(self.pn.gen_num):
                bus_num = self.pn.gen[i][0] - 1
                con_name = "Constraint 9 gen " + str(i) + " " + str(k)
                self.model.addGenConstrAnd(self.z[i + self.pn.bus_num, k],
                                           [self.y[i + self.pn.bus_num, k],
                                            self.y[bus_num, k]], name=con_name)

            for i in range(self.pn.load_num):
                bus_num = self.pn.load[i][1] - 1
                con_name = "Constraint 9 load " + str(i) + " " + str(k)
                self.model.addGenConstrAnd(self.z[i + self.pn.bus_num + self.pn.gen_num, k],
                                           [self.y[i + self.pn.bus_num + self.pn.gen_num, k],
                                            self.y[bus_num, k]], name=con_name)

        """
        Constraint (10) in Model 3
        """
        for i in range(self.pn.branch_num):
            bus_from = self.pn.branch[i][0]
            bus_to = self.pn.branch[i][1]
            for k in range(self.r_num):
                con_name = "Constraint 10 " + str(i) + " " + str(k)
                self.model.addGenConstrAnd(
                    self.z[i + self.pn.bus_num + self.pn.gen_num + self.pn.load_num, k],
                    [self.y[i + self.pn.bus_num + self.pn.gen_num + self.pn.load_num, k],
                     self.y[bus_from, k], self.y[bus_to, k]], name=con_name)

        """
        Constraint (11) in Model 3
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
                if self.pn.branch[j][0] == i:
                    li_set.append(j)
                if self.pn.branch[j][1] == i:
                    lo_set.append(j)

            for k in range(self.r_num):
                expr = LinExpr()
                for index in load_set:
                    expr -= self.pv_load[index][0, k]
                for index in gen_set:
                    expr += self.pv_gen[index][0, k]
                for index in li_set:
                    expr += self.pl[index][0, k]
                for index in lo_set:
                    expr -= self.pl[index][0, k]

                con_name = "Constraint 11 " + str(i) + " " + str(k)
                self.model.addConstr(expr == 0, name=con_name)

        """
        Constraint (12) in Model 3
        """

        for k in range(self.r_num):
            for i in range(self.pn.gen_num):
                con_name = "Constraint 12 gen " + str(i) + " " + str(k)
                # self.model.addConstr(self.pv_gen[i][0, k] <= self.pn.gen[i][8] * self.z[i + self.pn.bus_num, k],
                #                      name=con_name)
                self.model.addGenConstrIndicator(self.z[i + self.pn.bus_num, k], False,
                                                 self.pv_gen[i][0, k] == 0, name=con_name)

        for k in range(self.r_num):
            for i in range(self.pn.load_num):
                con_name = "Constraint 12 load " + str(i) + " " + str(k)
                # self.model.addConstr(self.pv_load[i][0, k] <= self.pn.load[i][2] *
                #                      self.z[i + self.pn.bus_num + self.pn.load_num, k], name=con_name)
                self.model.addGenConstrIndicator(self.z[i + self.pn.bus_num + self.pn.gen_num, k], False,
                                                 self.pv_load[i][0, k] == 0, name=con_name)

        for k in range(self.r_num):
            for i in range(self.pn.branch_num):
                con_name = "Constraint 9 " + str(i)
                self.model.addGenConstrIndicator(self.z[i + self.pn.bus_num + self.pn.gen_num + self.pn.load_num, k],
                                                 False, self.pl[i][0, k] == 0, name=con_name)

        """
        Constraint 13 and 14 in Model (3)
        """
        for i in range(self.pn.branch_num):
            if self.pn.branch[i][4] > 0:
                to_bus = self.pn.branch[i][1]
                from_bus = self.pn.branch[i][0]
                for k in range(self.r_num):
                    con_name = "Constraint 13 " + str(i) + " " + str(k)
                    self.model.addGenConstrIndicator(
                        self.z[i + self.pn.bus_num + self.pn.gen_num + self.pn.load_num, k],
                        True, self.pl[i][0, k] ==
                        self.pn.branch[i][4] * 100 * (self.theta[to_bus, k] - self.theta[from_bus, k]), name=con_name)

        # self.model.addConstr(self.o[0, 0] == 1)
        # self.model.addConstr(self.y[0, 0] == 1)
        # self.model.addConstr(self.z[0, 0] == 1)

    def optimize(self):
        self.model.optimize()
        # print('Obj:', self.model.objVal)
        flow_value = []
        for i in range(self.r_num):
            var_name = "flow[0," + str(i) + "]"
            var = self.model.getVarByName(var_name)
            flow_value.append(var.x)
        print(flow_value)

    def set_damaged_node(self):
        for node in self.damaged_node:
            self.pn.branch[node][10] = 0

    def display_o(self):
        o_value = []
        order_index = []
        for i in self.repair:
            value = []
            for j in range(self.r_num):
                var_name = "o[" + str(i) + "," + str(j) + "]"
                var = self.model.getVarByName(var_name)
                value.append(var.x)
            o_value.append(value)
            order_index.append(self.r_num - sum(value))
        self.order = [x for (y, x) in sorted(zip(order_index, self.repair))]
        print("o value:")
        print(o_value)
        print("order:")
        print(self.order)

    def display_z(self):
        z_value = []
        for i in range(self.item_num):
            var_name = "z[" + str(i) + "," + str(self.r_num - 1) + "]"
            var = self.model.getVarByName(var_name)
            z_value.append(var.x)
        print("z value:")
        print(z_value)

    def display_y(self):
        y_value = []
        for i in range(self.item_num):
            var_name = "y[" + str(i) + "," + str(self.r_num - 1) + "]"
            var = self.model.getVarByName(var_name)
            y_value.append(var.x)
        print("y value:")
        print(y_value)

    def get_sum_y(self):
        sum_y_value = 0
        for i in range(self.item_num):
            var_name = "y[" + str(i) + "," + str(self.r_num - 1) + "]"
            var = self.model.getVarByName(var_name)
            sum_y_value = sum_y_value + var.x
        return sum_y_value

    def display_pv_load(self):
        pv_load_value = []
        for i in range(self.pn.load_num):
            var_name = "pv_load[" + str(i) + "][0," + str(self.r_num - 1) + "]"
            var = self.model.getVarByName(var_name)
            pv_load_value.append(var.x)
        print("stage 2 pv load:")
        print(pv_load_value)

    def display_pv_gen(self):
        pv_gen_value = []
        for i in range(self.pn.gen_num):
            var_name = "pv_gen[" + str(i) + "][0," + str(self.r_num - 1) + "]"
            var = self.model.getVarByName(var_name)
            pv_gen_value.append(var.x)
        print("stage 2 pv gen:")
        print(pv_gen_value)

    def display_pl(self):
        pl_value = []
        for i in range(self.pn.branch_num):
            var_name = "pl[" + str(i) + "][0," + str(self.r_num - 1) + "]"
            var = self.model.getVarByName(var_name)
            pl_value.append(var.x)
        print("stage 2 pl:")
        print(pl_value)


if __name__ == "__main__":
    pn = PN()

    d_node = [1, 2, 3, 4, 13, 14, 15]
    # d_node = [0]
    stage1 = MRSP(pn, d_node)

    minimum_repair_set = stage1.repair
    # minimum_repair_set = [1, 14, 15]
    stage2 = ROP(pn, minimum_repair_set, d_node)
    stage1.display_repair()
    stage2.display_o()
    print(stage2.order)
    # y_sum = stage2.get_sum_y()
    # print("sum y:")
    # print(y_sum)
    # print(stage2.item_num)
    # stage1.display_z()
    # stage2.display_z()
    # stage1.display_y()
    # stage2.display_y()
    # stage1.display_pv_load()
    # stage2.display_pv_load()
    # stage1.display_pv_gen()
    # stage2.display_pv_gen()
    # stage1.display_pl()
    # stage2.display_pl()
    # print(stage2.pn.bus_num)
    # print(stage2.pn.gen_num)
    # print(stage2.pn.load_num)

else:
    print("rop is implemented into another module.")
