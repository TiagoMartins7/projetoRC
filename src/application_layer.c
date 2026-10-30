// RCOM 2026/2027
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CTRL_DATA 2
#define CTRL_START 1
#define CTRL_END 3

#define TYPE_SIZE 0
#define TYPE_NAME 1


static int buildControlPacket(unsigned char ctrlType, const char *filename, long fileSize, unsigned char *packet) {
    int idx = 0;
    packet[idx++] = ctrlType;

    // Type 0: File Size
    packet[idx++] = TYPE_SIZE;
    packet[idx++] = sizeof(long);
    for (int i = (int)sizeof(long) - 1; i >= 0; i--) {
        packet[idx++] = (fileSize >> (i * 8)) & 0xFF;
    }

    // Type 1: File Name
    int nameLen = strlen(filename);
    packet[idx++] = TYPE_NAME;
    packet[idx++] = nameLen;
    memcpy(packet + idx, filename, nameLen);
    idx += nameLen;

    return idx;
}

static int parseControlPacket(const unsigned char *packet, int packetLen, char *filename, long *fileSize) {
    if (packetLen < 1) return -1;

    int idx = 1; // skip control byte
    while (idx < packetLen) {
        unsigned char type = packet[idx++];
        unsigned char len = packet[idx++];

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
            idx += len;
        }
    }
    return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // ----------------------------------------------------
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strncpy(llParameters.serialPort, serialPort, sizeof(llParameters.serialPort) - 1);

    if (strcmp(role, "tx") == 0)
    {
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
        printf("Sent START packet\n");

        // Send DATA packets
        unsigned char fileBuf[MAX_PAYLOAD_SIZE - 3];
        unsigned char dataPacket[MAX_PAYLOAD_SIZE];
        unsigned char seqNum = 0;
        size_t bytesRead;

        while ((bytesRead = fread(fileBuf, 1, sizeof(fileBuf), file)) > 0) {
            dataPacket[0] = CTRL_DATA;
            dataPacket[1] = (bytesRead >> 8) & 0xFF;
            dataPacket[2] = bytesRead & 0xFF;
            memcpy(dataPacket + 3, fileBuf, bytesRead);

            if (llSend(dataPacket, bytesRead + 3) < 0) {
                printf("Error sending DATA packet %d\n", seqNum);
                fclose(file);
                llCloseTx();
                return;
            }
            printf("Sent data packet %i with size %li\n",seqNum, bytesRead + 3);

            seqNum++;
        }
        
        
        // Send END packet
        ctrlLen = buildControlPacket(CTRL_END, filename, fileSize, ctrlPacket);
        if (llSend(ctrlPacket, ctrlLen) < 0) {
            printf("Error sending END packet\n");
            llCloseTx();
            return;
        }
        
        printf("File '%s' sent successfully\n", filename);
        fclose(file);
        llCloseTx();
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) < 0) {
            printf("Error: llOpenRx failed\n");
            return;
        }

        unsigned char packet[MAX_PAYLOAD_SIZE];
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
        int seqNum = 0;

        while (1) {
            packetLen = llReceive(packet);
            if (packetLen < 1) {
                printf("Error receiving packet in llReceive\n");
                break;
            }

            if (packet[0] == CTRL_END) {
                printf("Received END packet\n");
                break;
            } else if (packet[0] == CTRL_DATA) {
                if (packetLen < 3) {
                    fprintf(stderr, "Malformed DATA packet: too short\n");
                    break;
                }

                int dataSize = 256 * packet[1] + packet[2];
                if (dataSize != packetLen - 3 || totalReceived + dataSize > fileSize) {
                    fprintf(stderr, "Malformed DATA packet: invalid data length\n");
                    break;
                }

                if (fwrite(packet + 3, 1, (size_t)dataSize, file) != (size_t)dataSize) {
                    perror("fwrite");
                    break;
                }
                totalReceived += dataSize;
                seqNum++;
                printf("Received packet %i with size %i\n", seqNum, packetLen);
            }
        }

        fclose(file);
        if (totalReceived == fileSize) {
            printf("Received file saved to '%s' (%ld bytes)\n", filename, totalReceived);
        } else {
            fprintf(stderr, "File reception incomplete: received %ld of %ld bytes\n",
                    totalReceived, fileSize);
        }
        llCloseRx();
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}
