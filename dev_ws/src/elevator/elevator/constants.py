#~ Environment
N_FLOORS            = 4  # Number of floors
N_BUTTONS           = 3  # Number of buttons

#~ Directions
DIRN_DOWN               = -1 # Down
DIRN_STOP               = 0  # Stop
DIRN_UP                 = 1  # Up

#~ Floor sensor
BETWEEN_FLOORS          = -1

#~ Buttons
BTN_HALL_UP             = 0  # Hall Up
BTN_HALL_DOWN           = 1  # Hall Down
BTN_CAB                 = 2  # Cab Button

#~ Behaviour
INIT                    = 0 # On initialization
IDLE                    = 1 # Idle on floor
DOOR_OPEN               = 2 # Doors Open  on floor
MOVING                  = 3 # Moving between floors

#~ Network status
OFFLINE                 = 0
ONLINE                  = 1

#~ Init mode
RESTART                 = 0
RECONNECT               = 1

#~ Time weighting in seconds
TIME_BETWEEN_FLOORS     = 5   # Seconds it takes to travel between floors
TIME_DOOR_OPEN          = 3.0 # Seconds door is kept open
TIME_ORDER_TIMEOUT      = 1   # Seconds before Order Confirmed-Timeout
TIME_EXEC_TIMEOUT       = 9   # Seconds before Mechanical Error-Timeout
TIME_HEARTBEAT_SEND     = 3   # How often you send harbeat message
TIME_HEARTBEAT_RECEIVE  = 8   # Seconds befor heartbeat is considered dead

#~ Message types
MSG_INIT                = 0
MSG_NEW_ORDER           = 1
MSG_ORDER_CONFIRMED     = 2
MSG_ORDER_EXECUTED      = 3
MSG_STATUS              = 4
MSG_NODE                = 5

#~ System
MODEL                   = 0
SIMULATOR               = 1
ELEV_MODE               = SIMULATOR
