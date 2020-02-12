#include <comedilib.h>

#include "io.h"
#include "channels.h"


static comedi_t *it_g = NULL;


int io_init(void) {

    it_g = comedi_open("/dev/comedi0");

    if (it_g == NULL) {
        return 0;
    }

    int status = 0;
    for (int i = 0; i < 8; i++) {
        status |= comedi_dio_config(it_g, PORT_1_SUBDEVICE, i + PORT_1_CHANNEL_OFFSET, PORT_1_DIRECTION);
        status |= comedi_dio_config(it_g, PORT_2_SUBDEVICE, i + PORT_2_CHANNEL_OFFSET, PORT_2_DIRECTION);
        status |= comedi_dio_config(it_g, PORT_3_SUBDEVICE, i + PORT_3_CHANNEL_OFFSET, PORT_3_DIRECTION);
        status |= comedi_dio_config(it_g, PORT_4_SUBDEVICE, i + PORT_4_CHANNEL_OFFSET, PORT_4_DIRECTION);
    }

    return (status == 0);
}


void io_set_bit(int channel) {
    comedi_dio_write(it_g, channel >> 8, channel & 0xff, 1);
}


void io_clear_bit(int channel) {
    comedi_dio_write(it_g, channel >> 8, channel & 0xff, 0);
}


void io_write_analog(int channel, int value) {
    comedi_data_write(it_g, channel >> 8, channel & 0xff, 0, AREF_GROUND, value);
}


int io_read_bit(int channel) {
    unsigned int data = 0;
    comedi_dio_read(it_g, channel >> 8, channel & 0xff, &data);
    return (int)data;
}


int io_read_analog(int channel) {
    lsampl_t data = 0;
    comedi_data_read(it_g, channel >> 8, channel & 0xff, 0, AREF_GROUND, &data);
    return (int)data;
}



