import constants


class SingleElevatorCopy:

    def __init__(self,id,floor,behaviour,direction,queue):
        self.id = id
        self.floor = {} #Key = Elevator ID || Val = Floor
        self.behaviour = {} #Key = Elevator ID || Val = Behaviour
        self.direction = {} #Key = Elevator ID || Val = Direction
        self.network = {} #Key = Elevator ID || Val = Network status
        self.queue = {} #Key = Elevator ID || Val = Order matrix
        self.unacknowledgedOrders = {} #key = Start time || Val =Elevator ID, floor, btn

        self.floor[self.id] = floor #Key = Elevator ID || Val = Floor
        self.behaviour[self.id] = behaviour #Key = Elevator ID || Val = Behaviour
        self.direction[self.id] = direction #Key = Elevator ID || Val = Direction
        self.queue[self.id] = queue #Key = Elevator ID || Val = Order matrix
