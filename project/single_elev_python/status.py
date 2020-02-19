import constants
import socket

class LocalElevator:
    id
    behaviour
    floor
    direction
    network
    queue = {}

    def __init__():
        hostname = socket.gethostname()
        self.id = socket.gethostbyname(hostname)[-3:]
        matrix = [[0 for b in range(N_BUTTONS)] for f in range(N_FLOORS)]
        Status.queue[self.id] = matrix
