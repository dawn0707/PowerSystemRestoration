# encoding: utf-8
"""
@author: Chen Zhang
@file: data_access.py
@time: 2019/3/3 19:03
"""
import xlrd
import numpy as np


class PN:

    def __init__(self):
        self.bus = []
        self.branch = []
        self.gen = []
        self.load = []
        self.gen_cost = []

        self.bus_num = 0
        self.branch_num = 0
        self.gen_num = 0
        self.load_num = 0

        self.initialization()

    def initialization(self):
        path = 'data/bus.xlsx'
        self.__private__get_data_from_excel(path, self.bus)

        path = 'data/branch.xlsx'
        self.__private__get_data_from_excel(path, self.branch)

        path = 'data/gen.xlsx'
        self.__private__get_data_from_excel(path, self.gen)

        path = 'data/load.xlsx'
        self.__private__get_data_from_excel(path, self.load)

        path = 'data/gencost.xlsx'
        self.__private__get_data_from_excel(path, self.gen_cost)

        self.get_num()

    def get_num(self):
        self.bus_num = len(self.bus)
        self.branch_num = len(self.branch)
        self.gen_num = len(self.gen)
        self.load_num = len(self.load)

    def __private__get_data_from_excel(self, path, list):
        workbook = xlrd.open_workbook(path)
        table = workbook.sheets()[0]
        nrows = table.nrows
        for i in range(nrows):
            list.append(table.row_values(i))


if __name__ == "__main__":
    pn = PN()
    print(pn.bus_num)
else:
    print("power_network is implemented into another module.")
