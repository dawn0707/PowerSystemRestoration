# encoding: utf-8
"""
@author: Chen Zhang
@file: trans_network.py
@time: 2019/3/27 18:36
"""
import xlrd


class TN:

    def __init__(self, w_num, base_num):
        self.coupling = []
        self.bus_cost = []
        self.gen_cost = []
        self.load_cost = []
        self.travel_time = []

        self.w_num = w_num
        self.base_num = base_num

        self.initialization()

    def initialization(self):
        path = 'data/trans-power.xlsx'
        self.__private__get_data_from_excel(path)

    def __private__get_data_from_excel(self, path):
        workbook = xlrd.open_workbook(path)
        coupling_table = workbook.sheets()[0]
        coupling_nrows = coupling_table.nrows
        for i in range(coupling_nrows):
            self.coupling.append(coupling_table.row_values(i))

        bus_cost_table = workbook.sheets()[1]
        bus_cost_nrows = bus_cost_table.nrows
        for i in range(bus_cost_nrows):
            self.bus_cost.append(bus_cost_table.row_values(i))

        gen_cost_table = workbook.sheets()[2]
        gen_cost_nrows = gen_cost_table.nrows
        for i in range(gen_cost_nrows):
            self.gen_cost.append(gen_cost_table.row_values(i))

        load_cost_table = workbook.sheets()[3]
        load_cost_nrows = load_cost_table.nrows
        for i in range(load_cost_nrows):
            self.load_cost.append(load_cost_table.row_values(i))

        travel_time_table = workbook.sheets()[4]
        travel_time_nrows = travel_time_table.nrows
        for i in range(travel_time_nrows):
            self.travel_time.append(travel_time_table.row_values(i))

    def display_coupling(self):
        print("Coupling Relationship between TN and PN:")
        print(self.coupling)

    def display_cost(self):
        print("Bus Cost:")
        print(self.bus_cost)
        print("Generator Cost:")
        print(self.gen_cost)
        print("Load Cost:")
        print(self.load_cost)

    def display_travel_time(self):
        print("Travel Time:")
        print(self.travel_time)


if __name__ == "__main__":
    tn = TN(20, 2)
    tn.display_coupling()
    tn.display_cost()
    tn.display_travel_time()
