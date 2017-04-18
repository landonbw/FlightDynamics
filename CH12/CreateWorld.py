import numpy as np
import sys

sys.path.append('..')
import ParamFiles.AerosondeParameters as P

class CreateWorld(object):
    def __init__(self):
        self.map_width = P.map_width
        self.max_height = P.max_height
        self.num_blocks = P.num_blocks
        self.street_width = P.street_width
        self.building_width = None
        self.map_heights = None
        self.buildings_north = []
        self.buildings_east = []
        self.buildings = []

        self.block_width = self.map_width / self.num_blocks
        self.building_width = self.map_width / self.num_blocks * (1 - self.street_width)
        self.street_width = self.map_width / self.num_blocks * self.street_width
        self.map_heights = np.random.rand(self.num_blocks, self.num_blocks) * self.max_height
        for i in range(self.num_blocks):
            self.buildings_north.append(0.5 * self.map_width / self.num_blocks * (2 * i + 1))
        self.buildings_east = self.buildings_north
        self.calculate_buildings()

    def calculate_buildings(self):
        for x in range(self.num_blocks):
            for y in range(self.num_blocks):
                x1 = self.buildings_east[y] - self.building_width/2
                y1 = self.buildings_north[x] - self.building_width/2
                x2 = x1 + self.building_width
                y2 = y1 + self.building_width
                h = self.map_heights[x,y]
                self.buildings.append(Building(x1, y1, x2, y2, h))

    def get_height(self, y, x):
        if x >= self.map_width or y >= self.map_width:
            return 0
        x_count = int(x / self.block_width)
        y_count = int(y / self.block_width)
        x = x % self.block_width
        y = y % self.block_width
        if x < self.street_width/2 or self.block_width - x <= self.street_width/2 or y < self.street_width/2 or self.block_width - y <= self.street_width/2:
            height = 0
        else:
            height = self.map_heights[x_count, y_count]
        return height

class Building(object):
    def __init__(self, x1, y1, x2, y2, h):
        self.b1 = x1, -y1, 0
        self.b2 = x1, -y2, 0
        self.b3 = x2, -y1, 0
        self.b4 = x2, -y2, 0
        self.t1 = x1, -y1, h
        self.t2 = x1, -y2, h
        self.t3 = x2, -y1, h
        self.t4 = x2, -y2, h

        self.h = h



if __name__ == "__main__":
    a = CreateWorld()
    # output = open('outputTest.csv', 'w')
    # b1 = a.get_height(500,0)
    # b2 = a.get_height(500,50)
    # b3 = a.get_height(500,100)
    # b4 = a.get_height(500,150)
    # b5 = a.get_height(500,200)
    # b6 = a.get_height(500,250)
    # b7 = a.get_height(500,300)
    # b8 = a.get_height(500,350)
    # b9 = a.get_height(500,400)
    # b10 = a.get_height(500,450)
    # b11 = a.get_height(500,500)
    # c1 = a.get_height(500, 0)
    # c2 = a.get_height(500, 50)
    # c3 = a.get_height(500, 100)
    # c4 = a.get_height(500, 150)
    # c5 = a.get_height(500, 200)
    # c6 = a.get_height(500, 250)
    # c7 = a.get_height(500, 300)
    # c8 = a.get_height(500, 350)
    # c9 = a.get_height(500, 400)
    # c10 = a.get_height(500, 450)
    # c11 = a.get_height(500, 500)
    # c12 = a.get_height(500, 550)
    # c13 = a.get_height(500, 600)
    # c14 = a.get_height(500, 650)
    # c15 = a.get_height(500, 700)
    # c16 = a.get_height(500, 750)
    # c17 = a.get_height(500, 800)
    # c18 = a.get_height(500, 850)
    # c19 = a.get_height(500, 900)
    # c20 = a.get_height(500, 1000)
    # c21 = a.get_height(500, 1050)
    # c = 5
    # for i in range(0, a.map_width, 50):
    #     for j in range(0, a.map_width, 50):
    #         output.write(str(a.get_height(j,i)) + ",")
    #     output.write("\n")
    # output.close()
    c = 5