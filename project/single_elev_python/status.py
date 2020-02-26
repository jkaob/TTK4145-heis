import constants
import socket
import os
class LocalElevator:
    # id
    # behaviour = constants.IDLE
    # floor
    # direction
    # network
    # queue = {}
    #######
    # Vi burde vurdere å lage alle variabler til dict e.g:
    # id: behaviour
    # id: floor
    # id: direction
    # etc.
    # etc.
    #######

    def __init__(self):
        self.queue = {}
        #Finner egen ip og setter id til ip
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        self.id = s.getsockname()[0][-3:]

        self.behaviour = 1
        self.floor = 1
        self.direction = 1
        self.network = 1
        matrix = [[0 for b in range(constants.N_BUTTONS)] for f in range(constants.N_FLOORS)]
        self.queue[self.id] = matrix
