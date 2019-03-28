# encoding: utf-8
"""
@author: Chen Zhang
@file: pdrppccdt.py
@time: 2019/3/25 17:19
"""
from power_network import PN
from mrsp import MRSP
from rop import ROP
from vehicle import  VEHICLE
from gurobipy import *


class PDRPPCCDT:

    def __init__(self, pn, repair, order, vehicle):
        self.pn = pn
        self.repair = repair
        self.order = order

        self.vehicle = vehicle

        self.model = Model("PDRPPCCDT")
        self.node_num = self.pn.bus_num + self.pn.gen_num + self.pn.load_num
        self.item_num = self.node_num + self.pn.branch_num
        self.base_station_num = 2
        self.r_num = len(self.repair)

        self.sigma = []
        self.ve = []
        self.load = []
        self.edt = []

        self.initialization()

    def initialization(self):
        self.set_variables()
        self.set_obj()
        self.set_constraints()
        self.optimize()

    def set_variables(self):
        self.sigma = self.model.addVars(1, 2 * (self.item_num + self.base_station_num),
                                        lb=0, ub=self.r_num - 1, vtype=GRB.INTEGER, name="sigma")
        self.ve = self.model.addVars(1, 2 * (self.item_num + self.base_station_num),
                                     lb=0, ub=self.vehicle.num - 1, vtype=GRB.INTEGER, name="ve")
        self.load = self.model.addVars(1, 2 * (self.item_num + self.base_station_num),
                                       lb=0, ub=self.vehicle.capacity, vtype=GRB.INTEGER, name="load")
        self.edt = self.model.addVars(1, 2 * (self.item_num + self.base_station_num),
                                      lb=0, ub=GRB.INFINITY, vtype=GRB.INTEGER, name="edt")

    def set_obj(self):
        obj = LinExpr()
        for i in range(self.item_num):
            obj += self.edt[0, i]
        self.model.setObjective(obj, GRB.MINIMIZE)

    def set_constraints(self):


    def optimize(self):
        self.model.optimize()
        sig = []
        for i in range(self.r_num):
            var_name = "sigma[0," + str(i) + "]"
            var = self.model.getVarByName(var_name)
            sig.append(var.x)
        print("sig:")
        print(sig)

if __name__ == "__main__":
    pn = PN()

    d_node = [1, 2, 3, 4, 13, 14, 15, 17, ]
    # d_node = [0]
    stage1 = MRSP(pn, d_node)

    minimum_repair_set = stage1.repair
    # minimum_repair_set = [1, 14, 15]
    stage2 = ROP(pn, minimum_repair_set, d_node)
    order = stage2.order

    stage1.display_repair()
    stage2.display_o()

    vehicle = VEHICLE()

    stage3 = PDRPPCCDT(pn, minimum_repair_set, order, vehicle)
else:
    print("pdrppccdt is implemented into another module.")

