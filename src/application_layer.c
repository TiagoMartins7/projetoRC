// RCOM 2026/2027
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#define CTRL_DATA  2
#define CTRL_START 1
#define CTRL_END   3
#define TYPE_SIZE  0
#define TYPE_NAME  1
#define DATA_HEADER_SIZE 4
#define MAX_FILE_NAME 255

static int buildControlPacket(unsigned char control, const char *name,
                              long size, unsigned char *packet) {
    if (!name || !packet || size < 0) return -1;
    size_t nameLen = strlen(name);
    if (nameLen == 0 || nameLen > MAX_FILE_NAME) return -1;

    /* Encode the file size in the minimum number of bytes (big-endian). */
    unsigned long value = (unsigned long)size;
    int sizeLen = 1;
    for (unsigned long n = value; n > 0xFF; n >>= 8) sizeLen++;
    if (1 + 2 + sizeLen + 2 + nameLen > MAX_PAYLOAD_SIZE) return -1;

    int i = 0;
    packet[i++] = control;
    packet[i++] = TYPE_SIZE;
    packet[i++] = (unsigned char)sizeLen;
    for (int shift = (sizeLen - 1) * 8; shift >= 0; shift -= 8)
        packet[i++] = (unsigned char)(value >> shift);
    packet[i++] = TYPE_NAME;
    packet[i++] = (unsigned char)nameLen;
    memcpy(packet + i, name, nameLen);
    i += (int)nameLen;
    return i;
}

static int parseControlPacket(const unsigned char *packet, int packetLen,
                              char *name, size_t nameCapacity, long *size) {
    if (!packet || packetLen < 1 || !name || nameCapacity == 0 || !size) return -1;
    if (packet[0] != CTRL_START && packet[0] != CTRL_END) return -1;

    int pos = 1, gotSize = 0, gotName = 0;
    unsigned long parsedSize = 0;
    name[0] = '\0';
    while (pos < packetLen) {
        if (packetLen - pos < 2) return -1;
        unsigned char type = packet[pos++];
        unsigned char len = packet[pos++];
        if (len > packetLen - pos) return -1;
        if (type == TYPE_SIZE) {
            if (gotSize || len == 0 || len > sizeof(long)) return -1;
            parsedSize = 0;
            for (int j = 0; j < len; j++) {
                if (parsedSize > ((unsigned long)LONG_MAX >> 8)) return -1;
                parsedSize = (parsedSize << 8) | packet[pos + j];
            }
            if (parsedSize > LONG_MAX) return -1;
            gotSize = 1;
        } else if (type == TYPE_NAME) {
            if (gotName || len == 0 || (size_t)len >= nameCapacity) return -1;
            memcpy(name, packet + pos, len);
            name[len] = '\0';
            gotName = 1;
        }
        pos += len;
    }
    if (!gotSize || !gotName) return -1;
    *size = (long)parsedSize;
    return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename) {
    if (!serialPort || !role || !filename) {
        fprintf(stderr, "Invalid application parameters\n");
        return;
    }

    LinkLayer params = {0};
    if (strlen(serialPort) >= sizeof(params.serialPort)) {
        fprintf(stderr, "Serial port name too long\n");
        return;
    }
    strcpy(params.serialPort, serialPort);
    params.baudRate = baudRate;
    params.nRetransmissions = nTries;
    params.timeout = timeout;

    if (strcmp(role, "tx") == 0) {
        /* Open the file first, so a missing file does not open a connection. */
        FILE *file = fopen(filename, "rb");
        if (!file) { perror("fopen (tx)"); return; }
        if (fseek(file, 0, SEEK_END) != 0) {
            perror("fseek"); fclose(file); return;
        }
        long fileSize = ftell(file);
        if (fileSize < 0 || fseek(file, 0, SEEK_SET) != 0) {
            perror("ftell/fseek"); fclose(file); return;
        }

        unsigned char controlPacket[MAX_PAYLOAD_SIZE];
        int controlLen = buildControlPacket(CTRL_START, filename, fileSize, controlPacket);
        if (controlLen < 0) {
            fprintf(stderr, "Invalid file name or control packet\n");
            fclose(file); return;
        }
        if (llOpenTx(params) < 0) { fclose(file); return; }

        int success = 1;
        if (llSend(controlPacket, controlLen) != controlLen) {
            fprintf(stderr, "Failed to send START\n");
            success = 0;
        }

        unsigned char dataPacket[MAX_PAYLOAD_SIZE];
        unsigned char sequence = 0;
        while (success) {
            size_t n = fread(dataPacket + DATA_HEADER_SIZE, 1,
                             MAX_PAYLOAD_SIZE - DATA_HEADER_SIZE, file);
            if (n == 0) {
                if (ferror(file)) { perror("fread"); success = 0; }
                break;
            }
            dataPacket[0] = CTRL_DATA;
            dataPacket[1] = sequence;
            dataPacket[2] = (unsigned char)(n >> 8);
            dataPacket[3] = (unsigned char)n;
            int packetLen = (int)n + DATA_HEADER_SIZE;
            if (llSend(dataPacket, packetLen) != packetLen) {
                fprintf(stderr, "Failed to send DATA packet %u\n", sequence);
                success = 0;
                break;
            }
            sequence++;
        }

        if (success) {
            controlLen = buildControlPacket(CTRL_END, filename, fileSize, controlPacket);
            if (controlLen < 0 || llSend(controlPacket, controlLen) != controlLen) {
                fprintf(stderr, "Failed to send END\n");
                success = 0;
            }
        }
        fclose(file);
        if (llCloseTx() < 0) success = 0;
        printf("Transmission %s\n", success ? "completed" : "failed");

    } else if (strcmp(role, "rx") == 0) {
        if (llOpenRx(params) < 0) return;

        unsigned char packet[MAX_PAYLOAD_SIZE];
        int packetLen = llReceive(packet);
        char sentName[MAX_FILE_NAME + 1];
        long expectedSize = 0;
        int success = packetLen > 0 && packet[0] == CTRL_START &&
                      parseControlPacket(packet, packetLen, sentName,
                                         sizeof(sentName), &expectedSize) == 0;
        if (!success) {
            fprintf(stderr, "Invalid START packet\n");
            /* Do not wait for DISC: the peer may no longer be communicating. */
            closeSerialPort();
            return;
        }
        printf("Receiving '%s' (%ld bytes) into '%s'\n",
               sentName, expectedSize, filename);

        FILE *file = fopen(filename, "wb");
        if (!file) {
            perror("fopen (rx)");
            closeSerialPort();
            return;
        }

        long total = 0;
        unsigned char expectedSequence = 0;
        int gotEnd = 0;
        while (success && !gotEnd) {
            packetLen = llReceive(packet);
            if (packetLen <= 0) { fprintf(stderr, "llReceive failed\n"); success = 0; break; }
            if (packet[0] == CTRL_DATA) {
                if (packetLen < DATA_HEADER_SIZE) { success = 0; break; }
                int dataSize = ((int)packet[2] << 8) | packet[3];
                if (dataSize != packetLen - DATA_HEADER_SIZE ||
                    packet[1] != expectedSequence ||
                    total > expectedSize || dataSize > expectedSize - total) {
                    fprintf(stderr, "Invalid DATA packet (sequence/length/size)\n");
                    success = 0;
                    break;
                }
                if (fwrite(packet + DATA_HEADER_SIZE, 1, (size_t)dataSize, file)
                    != (size_t)dataSize) {
                    perror("fwrite"); success = 0; break;
                }
                total += dataSize;
                expectedSequence++;
            } else if (packet[0] == CTRL_END) {
                char endName[MAX_FILE_NAME + 1];
                long endSize = 0;
                if (parseControlPacket(packet, packetLen, endName,
                                       sizeof(endName), &endSize) != 0 ||
                    endSize != expectedSize || strcmp(endName, sentName) != 0 ||
                    total != expectedSize) {
                    fprintf(stderr, "END packet or file size mismatch\n");
                    success = 0;
                }
                gotEnd = 1;
            } else {
                fprintf(stderr, "Unexpected application packet\n");
                success = 0;
            }
        }
        if (fclose(file) != 0) { perror("fclose"); success = 0; }
        if (success && gotEnd) {
            if (llCloseRx() < 0) success = 0;
        } else {
            /* An orderly DISC may never arrive after a failed transfer. */
            closeSerialPort();
        }
        if (!success && remove(filename) != 0)
            perror("remove incomplete received file");
        printf("Reception %s: %ld/%ld bytes\n",
               success && gotEnd ? "completed" : "failed", total, expectedSize);
    } else {
        fprintf(stderr, "Invalid role '%s' (use tx or rx)\n", role);
    }
}
