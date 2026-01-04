#include <stdio.h>
//#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>      // For open
#include <unistd.h>     // For close, write
#include <termios.h>    // For serial config
#include "RFpacket.pb-c.h"       // Your generated proto header (includes descriptors)

// Assume your proto message is RFPacket; tweak if AntennaData
typedef struct {
    int fd;  // File descriptor for serial port
} RFHandle;

// Open serial port and configure
void* rf_open(const char* port, int baud) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("rf_open: Failed to open port");
        return NULL;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    options.c_cflag |= (CLOCAL | CREAD);  // Enable receiver, ignore modem lines
    options.c_cflag &= ~PARENB;            // No parity
    options.c_cflag &= ~CSTOPB;            // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;                // 8 data bits
    options.c_cflag &= ~CRTSCTS;           // No hardware flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);  // No software flow control
    options.c_oflag &= ~OPOST;             // Raw output
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Raw input
    tcsetattr(fd, TCSANOW, &options);

    RFHandle* handle = malloc(sizeof(RFHandle));
    if (!handle) {
        close(fd);
        return NULL;
    }
    handle->fd = fd;
    return handle;
}

// Write proto-encoded packet (caller encodes to bytes; we just write raw)
int rf_write_packet(void* h, const int* data, size_t len) {
    RFHandle* handle = (RFHandle*)h;
    if (!handle || handle->fd == -1) return -1;

    ssize_t written = write(handle->fd, data, len);
    if (written != (ssize_t)len) {
        perror("rf_write_packet: Partial or failed write");
        return -1;
    }
    tcdrain(handle->fd);  // Wait for transmit complete
    return 0;  // Success
}

// Close handle
void rf_close(void* h) {
    RFHandle* handle = (RFHandle*)h;
    if (handle && handle->fd != -1) {
        close(handle->fd);
    }
    free(handle);
}

//Below is potential implementation of .pb-c.c

// Optional: Encode helper if you want C to handle proto too (Julia can still do it)
//int rf_encode_and_write(void* h, const AntennaData* packet) {  // Tweak RFPacket to your struct name
//    size_t len;
//   int* buf = encode_array(&antenna_data__descriptor, packet, &len);  // Assuming descriptor
//    if (!buf) return -1;
//    int res = rf_write_packet(h, buf, len);
//    free(buf);
//    return res;
//}
