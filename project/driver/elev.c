#include "elev.h"

#include <assert.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <pthread.h>

#include "channels.h"
#include "io.h"
#include "con_load.h"

#define MOTOR_SPEED 2800


static const int lamp_channel_matrix[N_FLOORS][N_BUTTONS] = {
    {LIGHT_UP1, LIGHT_DOWN1, LIGHT_COMMAND1},
    {LIGHT_UP2, LIGHT_DOWN2, LIGHT_COMMAND2},
    {LIGHT_UP3, LIGHT_DOWN3, LIGHT_COMMAND3},
    {LIGHT_UP4, LIGHT_DOWN4, LIGHT_COMMAND4},
};


static const int button_channel_matrix[N_FLOORS][N_BUTTONS] = {
    {BUTTON_UP1, BUTTON_DOWN1, BUTTON_COMMAND1},
    {BUTTON_UP2, BUTTON_DOWN2, BUTTON_COMMAND2},
    {BUTTON_UP3, BUTTON_DOWN3, BUTTON_COMMAND3},
    {BUTTON_UP4, BUTTON_DOWN4, BUTTON_COMMAND4},
};



static elev_type elevatorType = ET_Comedi;
static int sockfd;
static pthread_mutex_t sockmtx;

void elev_init(elev_type e) {
    elevatorType = e;
    switch(elevatorType) {
    case ET_Comedi:
        ;
        int init_success = io_init();
        assert(init_success && "Unable to initialize elevator hardware!");

        for(int f = 0; f < N_FLOORS; f++) {
            for(elev_button_type_t b = 0; b < N_BUTTONS; b++) {
                elev_set_button_lamp(b, f, 0);
            }
        }

        elev_set_stop_lamp(0);
        elev_set_door_open_lamp(0);
        elev_set_floor_indicator(0);
        break;

    case ET_Simulation:
        ;
        char ip[16] = {0};
        char port[8] = {0};
        con_load("simulator.con",
            con_val("com_ip",   ip,   "%s")
            con_val("com_port", port, "%s")
        )
        
        pthread_mutex_init(&sockmtx, NULL);
    
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        assert(sockfd != -1 && "Unable to set up socket");

        struct addrinfo hints = {
            .ai_family      = AF_UNSPEC, 
            .ai_socktype    = SOCK_STREAM, 
            .ai_protocol    = IPPROTO_TCP,
        };
        struct addrinfo* res;
        getaddrinfo(ip, port, &hints, &res);

        int fail = connect(sockfd, res->ai_addr, res->ai_addrlen);
        assert(fail == 0 && "Unable to connect to simulator server");

        freeaddrinfo(res);

        send(sockfd, (char[4]) {0}, 4, 0);

        break;
    }
}




void elev_set_motor_direction(elev_motor_direction_t dirn) {
    switch(elevatorType) {
    case ET_Comedi:
        if(dirn == 0) {
            io_write_analog(MOTOR, 0);
        } else if(dirn > 0) {
            io_clear_bit(MOTORDIR);
            io_write_analog(MOTOR, MOTOR_SPEED);
        } else if(dirn < 0) {
            io_set_bit(MOTORDIR);
            io_write_analog(MOTOR, MOTOR_SPEED);
        }
        break;
    case ET_Simulation:
        pthread_mutex_lock(&sockmtx);
        send(sockfd, (char[4]) {1, dirn}, 4, 0);
        pthread_mutex_unlock(&sockmtx);
        break;
    }
}


void elev_set_button_lamp(elev_button_type_t button, int floor, int value) {
    switch(elevatorType) {
    case ET_Comedi:
        assert(floor >= 0);
        assert(floor < N_FLOORS);
        assert(button >= 0);
        assert(button < N_BUTTONS);

        if(value) {
            io_set_bit(lamp_channel_matrix[floor][button]);
        } else {
            io_clear_bit(lamp_channel_matrix[floor][button]);
        }
        break;
    case ET_Simulation:
        pthread_mutex_lock(&sockmtx);
        send(sockfd, (char[4]) {2, button, floor, value}, 4, 0);
        pthread_mutex_unlock(&sockmtx);
        break;
    }
}


void elev_set_floor_indicator(int floor) {
    switch(elevatorType) {
    case ET_Comedi:
        assert(floor >= 0);
        assert(floor < N_FLOORS);

        // Binary encoding. One light must always be on.
        if(floor & 0x02) {
            io_set_bit(LIGHT_FLOOR_IND1);
        } else {
            io_clear_bit(LIGHT_FLOOR_IND1);
        }

        if(floor & 0x01) {
            io_set_bit(LIGHT_FLOOR_IND2);
        } else {
            io_clear_bit(LIGHT_FLOOR_IND2);
        }
        break;
    case ET_Simulation:
        pthread_mutex_lock(&sockmtx);
        send(sockfd, (char[4]) {3, floor}, 4, 0);
        pthread_mutex_unlock(&sockmtx);
        break;
    }
}


void elev_set_door_open_lamp(int value) {
    switch(elevatorType) {
    case ET_Comedi:
        if(value) {
            io_set_bit(LIGHT_DOOR_OPEN);
        } else {
            io_clear_bit(LIGHT_DOOR_OPEN);
        }
        break;
    case ET_Simulation:
        pthread_mutex_lock(&sockmtx);
        send(sockfd, (char[4]) {4, value}, 4, 0);
        pthread_mutex_unlock(&sockmtx);
        break;
    }
}


void elev_set_stop_lamp(int value) {
    switch(elevatorType) {
    case ET_Comedi:
        if(value) {
            io_set_bit(LIGHT_STOP);
        } else {
            io_clear_bit(LIGHT_STOP);
        }
        break;
    case ET_Simulation:
        pthread_mutex_lock(&sockmtx);
        send(sockfd, (char[4]) {5, value}, 4, 0);
        pthread_mutex_unlock(&sockmtx);
        break;
    }
}




int elev_get_button_signal(elev_button_type_t button, int floor) {
    switch(elevatorType) {
    case ET_Comedi:
        assert(floor >= 0);
        assert(floor < N_FLOORS);
        assert(button >= 0);
        assert(button < N_BUTTONS);

        return (io_read_bit(button_channel_matrix[floor][button]));
    case ET_Simulation:
        pthread_mutex_lock(&sockmtx);
        send(sockfd, (char[4]) {6, button, floor}, 4, 0);
        char buf[4];
        recv(sockfd, buf, 4, 0);
        pthread_mutex_unlock(&sockmtx);
        return buf[1];
    }
    return 0;
}


int elev_get_floor_sensor_signal(void) {
    switch(elevatorType) {
    case ET_Comedi:
        if(io_read_bit(SENSOR_FLOOR1)) {
            return 0;
        } else if(io_read_bit(SENSOR_FLOOR2)) {
            return 1;
        } else if(io_read_bit(SENSOR_FLOOR3)) {
            return 2;
        } else if(io_read_bit(SENSOR_FLOOR4)) {
            return 3;
        } else {
            return -1;
        }
    case ET_Simulation:
        pthread_mutex_lock(&sockmtx);
        send(sockfd, (char[4]) {7}, 4, 0);
        char buf[4];
        recv(sockfd, buf, 4, 0);
        pthread_mutex_unlock(&sockmtx);
        return buf[1] ? buf[2] : -1;
    }
    return 0;
}


int elev_get_stop_signal(void) {
    switch(elevatorType) {
    case ET_Comedi:
        return io_read_bit(STOP);
    case ET_Simulation:
        pthread_mutex_lock(&sockmtx);
        send(sockfd, (char[4]) {8}, 4, 0);
        char buf[4];
        recv(sockfd, buf, 4, 0);
        pthread_mutex_unlock(&sockmtx);
        return buf[1];
    }
    return 0;
}


int elev_get_obstruction_signal(void) {
    switch(elevatorType) {
    case ET_Comedi:
        return io_read_bit(OBSTRUCTION);
    case ET_Simulation:
        pthread_mutex_lock(&sockmtx);
        send(sockfd, (char[4]) {9}, 4, 0);
        char buf[4];
        recv(sockfd, buf, 4, 0);
        pthread_mutex_unlock(&sockmtx);
        return buf[1];
    }
    return 0;
}
