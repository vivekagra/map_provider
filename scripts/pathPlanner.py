import math
import numpy as np
import matplotlib.pyplot as plt

import rospy

from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped

class pathPlanner:
    def __init__(self):
        self.map = None
        rospy.init_node('path_planner')
        rospy.Subscriber('/map', OccupancyGrid ,self.readOccupancy)
        print("Yes")

    def getCSpace(self, mapdata):
        if(self.map==None):
            n = mapdata.info.width * mapdata.info.height
            inflatedGrid = [0]*n
            grid_pos = [0]*n

            for w in mapdata.data:
                x = w % mapdata.info.width
                y = (w - x)/mapdata.info.width
                grid_pos[w] = [x,y,0]
                self.edge_check(x,y,mapdata.info.width,mapdata.info.height,mapdata)
                yup1 = ((y+1)*mapdata.info.width)+x
                ydwn1 = ((y-1)*mapdata.info.width)+x
                if self.mapdata.data[w] >= objs:
                    inflatedGrid[w] = 100
                elif ((not edge_check.leftMost and self.mapdata.data[w-1] >= objs) or (not edge_check.rightMost and self.mapdata.data[w+1] >= objs)
                    or (not edge_check.top and self.mapdata.data[yup] >= objs) or (edge_check.bottom and self.mapdata.data[ydwn] >= objs)):
                    inflatedGrid[w] = 100

            grid_cell = GridCells()
            grid_cell.header.frame_id = "/map"
            grid_cell.cell_height = mapdata.info.height
            grid_cell.cell_width = mapdata.info.width
            grid_cell.cells = grid_pos
            self.map = grid_cell
            print(self.map)

    def readOccupancy(self, mapdata):
        print("Yes1")
        self.getCSpace(mapdata)


    def make_cspace(self, mapdata):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [[int8]]        The C-Space as a list of values from 0 to 100.
        """

        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        n = mapdata.info.width * mapdata.info.height
        inflatedGrid = [0]*n
        grid_pos = [0]*n

        ## Go through each cell in the occupancy grid
        for w in mapdata.data:
            x = w % mapdata.info.width
            y = (w - x)/mapdata.info.width
            grid_pos[w] = [x,y,0]
            self.edge_check(x,y,mapdata.info.width,mapdata.info.height,mapdata)
            yup1 = ((y+1)*mapdata.info.width)+x
            ydwn1 = ((y-1)*mapdata.info.width)+x
            if self.mapdata.data[w] >= objs:
                inflatedGrid[w] = 100
            elif ((not edge_check.leftMost and self.mapdata.data[w-1] >= objs) or (not edge_check.rightMost and self.mapdata.data[w+1] >= objs)
                or (not edge_check.top and self.mapdata.data[yup] >= objs) or (edge_check.bottom and self.mapdata.data[ydwn] >= objs)):
                inflatedGrid[w] = 100
        ## Inflate the obstacles where necessary
        # TODO
        ## Create a GridCells message and publish it
        msg_grid_cell = GridCells()
        # Fix header
        msg_grid_cell.Header.frame_id = "/map"
        msg_grid_cell.height = mapdata.info.height
        msg_grid_cell.width = mapdata.info.width
        msg_grid_cell.Point = grid_pos
        self.pub.publish(msg_grid_cell)
        # TODO
        ## Return the C-space
        #inflatedb grid
        return inflatedGrid
        #Go through all original map data and check for 100, if 100 make new
        # grid 100, if not check surrounding grids

    def run(self):
        print("Map Printed")
        rospy.spin()

if __name__=='__main__':
    pathPlanner().run()