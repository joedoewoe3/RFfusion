#include <protobuf-c/protobuf-c.h>
#include <stdint.h>
#include <stdio.h>
//#include <stdint.h>  // Dupe, remove
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>      // For open
#include <unistd.h>     // For close, write
#include <termios.h>    // For serial config
#include "RFpacket.pb-c.h"  // From protoc --c_out (tweak if name differs)

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
int rf_write_packet(void* h, const uint8_t* data, size_t len) {  // Changed int* to uint8_t* for bytes
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

int rf_read_packet(void* h, uint8_t* data, size_t max_len) {
    RFHandle* handle = (RFHandle*)h;
    if (!handle || handle->fd == -1) return -1;

    size_t pos = 0;
    while (pos < max_len) {
        ssize_t r = read(handle->fd, data + pos, max_len - pos);
        if (r <= 0) {
            if (r < 0) perror("rf_read_packet: Read failed");
            return (int)pos > 0 ? (int)pos : r;
        }
        pos += (size_t)r;
    }
    return (int)pos;
}

// New: Encode and write RFPacket (static buf, no malloc; tweak MAX_PKT_LEN per proto max)
#define MAX_PKT_LEN 1200  // Tweak based on max proto size (e.g., firmware blobs)

int rf_encode_and_write(void* h, const RFPacket* packet) {
    // Cast to base msg for protobuf-c API
    const ProtobufCMessage* msg = (const ProtobufCMessage*)packet;

    // Get required len
    size_t len = protobuf_c_message_get_packed_size(msg);
    if (len == 0) {
        fprintf(stderr, "rf_encode_and_write: Zero-size packet\n");
        return -1;
    }

    uint8_t* buf;
    int use_malloc = 0;
    if (len > MAX_PKT_LEN) {
        buf = malloc(len);
        if (!buf) {
            perror("rf_encode_and_write: Malloc failed for large packet");
            return -1;
        }
        use_malloc = 1;
    } else {
        buf = (uint8_t*)alloca(len);  // Stack for small; or static array if preferred
    }

    // Pack to buf
    if (protobuf_c_message_pack(msg, buf) != len) {
        fprintf(stderr, "rf_encode_and_write: Packing failed\n");
        if (use_malloc) free(buf);
        return -1;
    }

    int res = rf_write_packet(h, buf, len);  // Write encoded bytes
    if (use_malloc) free(buf);
    return res;
}
