// RCOM 2026/2027
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CTRL_START 1
#define CTRL_DATA  2
#define CTRL_END   3

#define TYPE_SIZE  0
#define TYPE_NAME  1

// Builds a Control packet (START / END) using TLV format
static int buildControlPacket(unsigned char ctrlType, const char *filename, long fileSize, unsigned char *packet) {
    int idx = 0;
    packet[idx++] = ctrlType;

    // Determine minimal number of bytes required to encode fileSize
    int sizeBytes = 0;
    long tempSize = fileSize;
    do {
        sizeBytes++;
        tempSize >>= 8;
    } while (tempSize > 0);

    // Type 0: File Size (TLV)
    packet[idx++] = TYPE_SIZE;
    packet[idx++] = (unsigned char)sizeBytes;
    for (int i = sizeBytes - 1; i >= 0; i--) {
        packet[idx++] = (unsigned char)((fileSize >> (i * 8)) & 0xFF);
    }

    // Type 1: File Name (TLV)
    int nameLen = strlen(filename);
    packet[idx++] = TYPE_NAME;
    packet[idx++] = (unsigned char)nameLen;
    memcpy(packet + idx, filename, nameLen);
    idx += nameLen;

    return idx;
}

// Parses a Control packet (START / END) formatted with TLV
static int parseControlPacket(const unsigned char *packet, int packetLen, char *filename, long *fileSize) {
    if (packetLen < 1) return -1;

    int idx = 1; // Skip control field
    while (idx < packetLen) {
        unsigned char type = packet[idx++];
        if (idx >= packetLen) return -1;

        unsigned char len = packet[idx++];
        if (idx + len > packetLen) return -1;

        if (type == TYPE_SIZE) {
            long size = 0;
            for (int i = 0; i < len; i++) {
                size = (size << 8) | packet[idx++];
            }
            if (fileSize) *fileSize = size;
        } else if (type == TYPE_NAME) {
            if (filename) {
                memcpy(filename, packet + idx, len);
                filename[len] = '\0';
            }
            idx += len;
        } else {
            // Unknown TLV parameter type
            idx += len;
        }
    }
    return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strncpy(llParameters.serialPort, serialPort, sizeof(llParameters.serialPort) - 1);

    if (strcmp(role, "tx") == 0)
    {
        // Open connection as Transmitter
        if (llOpenTx(llParameters) < 0) {
            printf("Error: llOpenTx failed\n");
            return;
        }

        FILE *file = fopen(filename, "rb");
        if (!file) {
            perror("fopen");
            llCloseTx();
            return;
        }

        // Determine file size
        fseek(file, 0, SEEK_END);
        long fileSize = ftell(file);
        fseek(file, 0, SEEK_SET);

        printf("Sending file '%s' (%ld bytes)\n", filename, fileSize);

        // Send START packet
        unsigned char ctrlPacket[MAX_PAYLOAD_SIZE];
        int ctrlLen = buildControlPacket(CTRL_START, filename, fileSize, ctrlPacket);
        if (llSend(ctrlPacket, ctrlLen) < 0) {
            printf("Error sending START packet\n");
            fclose(file);
            llCloseTx();
            return;
        }

        // Send DATA packets (Header: C, L2, L1 -> 3 bytes)
        unsigned char fileBuf[MAX_PAYLOAD_SIZE - 3];
        unsigned char dataPacket[MAX_PAYLOAD_SIZE];
        size_t bytesRead;

        while ((bytesRead = fread(fileBuf, 1, sizeof(fileBuf), file)) > 0) {
            dataPacket[0] = CTRL_DATA;
            dataPacket[1] = (unsigned char)((bytesRead >> 8) & 0xFF); // L2
            dataPacket[2] = (unsigned char)(bytesRead & 0xFF);        // L1
            memcpy(dataPacket + 3, fileBuf, bytesRead);

            if (llSend(dataPacket, (int)bytesRead + 3) < 0) {
                printf("Error sending DATA packet\n");
                fclose(file);
                llCloseTx();
                return;
            }
        }

        fclose(file);

        // Send END packet
        ctrlLen = buildControlPacket(CTRL_END, filename, fileSize, ctrlPacket);
        if (llSend(ctrlPacket, ctrlLen) < 0) {
            printf("Error sending END packet\n");
            llCloseTx();
            return;
        }

        printf("File '%s' sent successfully\n", filename);
        llCloseTx();
    }
    else if (strcmp(role, "rx") == 0)
    {
        // Open connection as Receiver
        if (llOpenRx(llParameters) < 0) {
            printf("Error: llOpenRx failed\n");
            return;
        }

        unsigned char packet[MAX_PAYLOAD_SIZE];

        // Wait for START packet
        int packetLen = llReceive(packet);
        if (packetLen < 0 || packet[0] != CTRL_START) {
            printf("Error receiving START packet\n");
            llCloseRx();
            return;
        }

        long fileSize = 0;
        char rcvFilename[256] = {0};
        parseControlPacket(packet, packetLen, rcvFilename, &fileSize);
        printf("Receiving file '%s' (expected size: %ld bytes)\n", filename, fileSize);

        FILE *file = fopen(filename, "wb");
        if (!file) {
            perror("fopen");
            llCloseRx();
            return;
        }

        long totalReceived = 0;

        // Process incoming DATA and END packets
        while (1) {
            packetLen = llReceive(packet);
            if (packetLen < 0) {
                printf("Error receiving packet in llReceive\n");
                break;
            }

            if (packet[0] == CTRL_END) {
                long endFileSize = 0;
                parseControlPacket(packet, packetLen, NULL, &endFileSize);
                printf("Received END packet (reported size: %ld bytes)\n", endFileSize);
                break;
            }

            if (packet[0] == CTRL_DATA) {
                int dataSize = (packet[1] << 8) | packet[2]; // K = 256 * L2 + L1

                // Validate payload consistency with packet length
                if (dataSize != packetLen - 3) {
                    printf("Warning: Packet size mismatch (header: %d, received: %d)\n", dataSize, packetLen - 3);
                }

                fwrite(packet + 3, 1, dataSize, file);
                totalReceived += dataSize;
            }
        }

        fclose(file);
        printf("Received file saved to '%s' (%ld/%ld bytes received)\n", filename, totalReceived, fileSize);
        llCloseRx();
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}