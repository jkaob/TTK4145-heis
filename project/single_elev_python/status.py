import constants
import socket
import os
class LocalElevator:

    def __init__(self,floor,bh,dir,queue):
        self.floor = {} #Key = Elevator ID || Val = Floor
        self.behaviour = {} #Key = Elevator ID || Val = Behaviour
        self.direction = {} #Key = Elevator ID || Val = Direction
        self.network = {} #Key = Elevator ID || Val = Network status
        self.queue = {} #Key = Elevator ID || Val = Order matrix

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        self.id = s.getsockname()[0][-3:]

        matrix = [[0 for b in range(constants.N_BUTTONS)] for f in range(constants.N_FLOORS)]
        self.queue[self.id] = matrix
