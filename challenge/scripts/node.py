class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, x=-1, y=-1, parent=None, cost = 0):
        self.parent = parent
        
        self.x = x
        self.y = y
        self.cost = cost
        self.movement = []

    def __eq__(self, other):
        return (self.x == other.x) and (self.y == other.y)