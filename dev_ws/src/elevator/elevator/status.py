import constants

class LocalElevator:

    def __init__(self,id,floor=None,behaviour=None,direction=None,queue=None):
        self.id = id
        self.floor = {} #Key = Elevator ID || Val = Floor
        self.behaviour = {} #Key = Elevator ID || Val = Behaviour
        self.direction = {} #Key = Elevator ID || Val = Direction
        self.network = {} #Key = Elevator ID || Val = Network status
        self.queue = {} #Key = Elevator ID || Val = Order matrix

        if(queue == None):
            matrix = [[0 for b in range(constants.N_BUTTONS)] for f in range(constants.N_FLOORS)]
            self.queue[self.id] = matrix
        else:
            self.floor[self.id] = floor #Key = Elevator ID || Val = Floor
            self.behaviour[self.id] = behaviour #Key = Elevator ID || Val = Behaviour
            self.direction[self.id] = direction #Key = Elevator ID || Val = Direction
            self.queue[self.id] = queue #Key = Elevator ID || Val = Order matrix
