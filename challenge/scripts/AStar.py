"""
Grid based A* algorithm planning adaptated to ROS

authors: Carlos Aunion Dominguez, Sergio Luzuriaga Rodriguez

"""

import math
import heapq
from node import Node
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt


show_animation = False # Put to false to omit the animation

class AStar:
    def __init__(self, costmap, obstacle_threshold):
        # Copy costmap metadata
        self.resolution = costmap.info.resolution
        self.min_x = costmap.info.origin.position.x
        self.min_y = costmap.info.origin.position.y
        self.x_width = costmap.info.width
        print("Width x: " + str(self.x_width))
        self.y_width = costmap.info.height
        print("Width y: " + str(self.y_width))
        # Calculate map maximum coordinates
        self.max_x = self.min_x + self.x_width * self.resolution
        self.max_y = self.min_y + self.y_width * self.resolution

        # print(self.min_x, self.min_y)
        # print(self.max_x, self.max_y)
        # print(self.x_width, self.y_width)

        # Set motion model
        self.motion = self.get_motion_model()

        # Initialize x and y iterators
        x = 0
        y = 0
        # Initialize obstacle map matrix on False
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]

        # Initialize number of obstacles variable
        obstacles = 0

        print("Creating obstacle map...")
        # Check costmap data for obstacles in map
        for value in costmap.data:
            # If costmap value is higher than the obstacle threshold, 
            #   the position is considered an obstacle
            if value > obstacle_threshold:
                obstacles += 1
                self.obstacle_map[x][y] = True
            # Update the iterators
            x += 1
            if x == self.x_width:
                x = 0
                y += 1
        print("Loaded %d obstacles"%(obstacles))


    def planning(self, sx, sy, gx, gy):
        """
            A* path search
            
            input:
                sx: start x position [m]
                sy: start y position [m]
                gx: goal x position [m]
                gx: goal x position [m]

            output:
                rx: x position list of the final path
                ry: y position list of the final path
        """  
        # Show animation
        if show_animation:
            plt.plot(sx, sy, "og")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")

        # Initialize init and goal node
        startNode = Node(self.calc_xy_index(sx, self.min_x),
                            self.calc_xy_index(sy, self.min_y), None)
        goalNode = Node(self.calc_xy_index(gx, self.min_x),
                        self.calc_xy_index(gy, self.min_y), None)

        # Check if init node and goal node are valid
        if not self.verify_node(startNode):
            print("Error: init not valid: ")
            return (0, 0)
        if not self.verify_node(goalNode):
            print("Error: goal not valid")
            return (0, 0)

        print("Calculating Path...")

        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(startNode)] = startNode

        # Loop until goal is found
        iteration = 0
        while True:
            iteration += 1
            # Get current node index
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            # Get current node
            currentNode = open_set[c_id]

            # Show animation
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_position(currentNode.x, self.min_x),
                        self.calc_position(currentNode.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)
            
            # Get lowest cost node in open set
            for index, item in open_set.items():
                if item.cost < currentNode.cost:
                    currentNode = item
                    c_id = index

            open_set.pop(c_id)
            closed_set[c_id] = currentNode

            """ Goal found """
            #print("Current Node: (" +str(currentNode.x) +", " + str(currentNode.y) +"), Goal Node: (" + str(goalNode.x)+", " + str(goalNode.y) +")")
            if currentNode == goalNode:
                # Initialize path varaibles
                xPath, yPath = [], []
                # Get current node
                current = currentNode
                # Initialize previous node variable
                previous = Node()
                # Add goal node to path
                xPath.append(self.calc_position(current.x, self.min_x))
                yPath.append(self.calc_position(current.y, self.min_y))
                # Update previous node variable
                previous = current
                # Get next node
                current = current.parent
                while current is not None:
                    # If node doesn't follow the same direction
                    if (current.movement != previous.movement) and not current.parent is None:
                        # Add node to path
                        xPath.append(self.calc_position(current.x, self.min_x))
                        yPath.append(self.calc_position(current.y, self.min_y))
                        # Update previous node variable
                        previous = current
                    elif current.parent is None:
                         # Always add start node
                        xPath.append(self.calc_position(current.x, self.min_x))
                        yPath.append(self.calc_position(current.y, self.min_y))
                    # Get next node
                    current = current.parent
                
                # Return path found on reverse order
                return xPath[::-1], yPath[::-1]

            # Add node to the closed set
            closed_set[c_id] = currentNode

            # Initialize children variable
            children = []

            # For each possible motion to take:
            for move_x, move_y, cost in self.get_motion_model():
                # Get note position
                node = Node(currentNode.x + move_x,
                            currentNode.y + move_y,
                            currentNode)
                
                # Get node index
                n_id = self.calc_index(node)

                # Check if node is already in the closed set
                if n_id in closed_set:
                    continue

                # Check if node is not valid
                if not self.verify_node(node):
                    continue

                # Update node cost
                node.cost = node.parent.cost + self.heuristic(node, goalNode) + cost
                # Update movement taken
                node.movement = [move_x, move_y]
                # Add node to children list
                children.append(node)
            
            # For each children
            for child in children:
                # Calculate child node index
                n_id = self.calc_index(child)

                # Add node to open set if not added already
                if n_id not in open_set:
                    open_set[n_id] = node # New node discovered
                else:
                    # If current path is better than the one stored, replace the node
                    if open_set[n_id].cost >= node.cost:
                        # Set this path the best until now
                        open_set[n_id] = node
                for index, open_node in open_set.items():
                    if child == open_node and child.cost > open_node.cost:
                        continue
            
                open_set[n_id] = child

        # Returns lists with x and y coordinates of the path
        return xPath[::-1], yPath[::-1]


    def calc_xy_index(self, position, minp):
        """ Calculate x or y index with given position """
        return round((position - minp) / self.resolution)

    def verify_node(self, node):
        """
            Check if the node is valid as path
        """
        # Get node position from given index
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        # Check if node is inside the map
        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        # Check if node is inside obstacle
        if self.obstacle_map[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_index(self, node):
        """ Calculate the index of a node on the matrix """
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)


    def calc_position(self, index, minp):
        """ Calculate position of a node on the map """
        return index * self.resolution + minp

    """ Euclidean distance heuristic """
    # def heuristic(self, start, goal):
    #     return math.sqrt(((start.x - goal.x) ** 2) + ((start.y - goal.y) ** 2))

    """ Manhattan distance heuristic """
    def heuristic(self, start, goal):
        return abs(start.x - goal.x) + abs(start.y - goal.y)

    @staticmethod
    def get_motion_model():
        """
            Defines the motion cost in each direction including diagonals
        """
        # Motion matrix: dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        return motion