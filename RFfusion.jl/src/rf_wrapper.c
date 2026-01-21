#include <errno.h>
#include <protobuf-c/protobuf-c.h>
#include <stdint.h>
#include <stdio.h>
//#include <stdint.h>  // Dupe, remove
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>      // For open
#include <unistd.h>     // For close, write
#include <termios.h>    // For serial config
#include <sys/select.h>
#include <sys/time.h>
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
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        perror("tcsetattr failed");
        close(fd);
        return NULL;
    }

    RFHandle* handle = malloc(sizeof(RFHandle));
    if (!handle) {
        close(fd);
        return NULL;
    }
    handle->fd = fd;
    printf("rf_open success: fd=%d\n", fd);
    return handle;
}

// Write proto-encoded packet (caller encodes to bytes; we just write raw)
int rf_write_packet(void* h, const uint8_t* data, size_t len) {
    RFHandle* handle = (RFHandle*)h;
    if (!handle || handle->fd == -1) {
        fprintf(stderr, "rf_write_packet: Invalid handle/fd\n");
        return -1;
    }
    printf("rf_write_packet: fd=%d, len=%zu\n", handle->fd, len);
    ssize_t written = write(handle->fd, data, len);
    if (written != (ssize_t)len) {
        perror("rf_write_packet failed");
        return -1;
    }
    tcdrain(handle->fd);
    return 0;
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
    struct timeval start, now;
    gettimeofday(&start, NULL);  // Start time for overall timeout
    while (pos < max_len) {
        gettimeofday(&now, NULL);
        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec) / 1e6;
        if (elapsed >= 2.0) {
            return (int)pos > 0 ? (int)pos : 0;  // Partial or no data
        }

        // Prepare select
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(handle->fd, &fds);
        struct timeval tv = {0, 100000};  // 0.1s per select (adjustable)

        int ready = select(handle->fd + 1, &fds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;  // Signal interrupt: retry
            perror("rf_read_packet: Select failed");  // Real error
            return -1;
        } else if (ready == 0) {
            continue;  // Timeout: retry loop (overall time will catch)
        }

        if (FD_ISSET(handle->fd, &fds)) {

        ssize_t r = read(handle->fd, data + pos, max_len - pos);
        if (r > 0) {
            pos += (size_t)r;
        } else if (r == 0) {
            // EOF? Rare for serial, but break
            break;
        } else {  // r < 0
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;  // Shouldn't happen post-select, but safe
            }
            perror("rf_read_packet: Read failed");  // Real error
            return -1;

            }
        }

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
