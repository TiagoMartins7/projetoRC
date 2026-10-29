// RCOM 2026/2027
// Link layer protocol implementation
//
// Integrated Stop-and-Wait Error Control Mechanism with State Machine

#define _POSIX_C_SOURCE 200809L

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

// Protocol-specific definitions kept here so link_layer.h remains unchanged.
#define FLAG 0x7E

typedef enum {
    SET = 0x03,
    UA = 0x07,
    DISC = 0x0B,
    RR = 0x05,
    REJ = 0x01,
    INFO_FRAME_0 = 0x00,
    INFO_FRAME_1 = 0x80
} ControlField;

typedef enum {
    A_SENDER = 0x03,
    A_RECEIVER = 0x01
} AddressField;

#define BUF_SIZE 2048
#define SUPERVISION_FRAME_SIZE 5


#define STUFF 0x7D
#define FLAG_STUFF 0x5E
#define STUFF_STUFF 0x5D


// Control Field values strictly following the project specification (Slide 10):
// RR0:  10101010 (0xAA) - Receiver ready to receive Information Frame 0
// RR1:  10101011 (0xAB) - Receiver ready to receive Information Frame 1
// REJ0: 01010100 (0x54) - Receiver rejects Information Frame 0
// REJ1: 01010101 (0x55) - Receiver rejects Information Frame 1
#define RR_0 0xAA
#define RR_1 0xAB
#define REJ_0 0x54
#define REJ_1 0x55

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC1_OK,
    RCV_DATA,
    STOP
} State;

// Link layer parameters
static LinkLayer llParams;

// Sequence numbers for Stop-and-Wait
static unsigned char txFrameNum = INFO_FRAME_0;      // 0x00 or 0x80
static unsigned char rxExpectedFrame = INFO_FRAME_0; // 0x00 or 0x80

// Alarm handling
static volatile sig_atomic_t alarmEnabled = FALSE;
static volatile sig_atomic_t alarmCount = 0;

// Statistics
static unsigned long numFramesSent = 0;
static unsigned long numFramesReceived = 0;
static unsigned long numDataFrames = 0;
static unsigned long numRejFrames = 0;
static struct timeval startTime;

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
}

static void configureAlarm() {
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1) {
        perror("sigaction");
        exit(1);
    }
}

static void printStatistics(int isTx) {
    struct timeval endTime;
    gettimeofday(&endTime, NULL);
    double totalTime = (endTime.tv_sec - startTime.tv_sec) + 
                       (endTime.tv_usec - startTime.tv_usec) / 1000000.0;
    if (totalTime <= 0.0) totalTime = 1e-6;

    printf("\n--- TRANSMISSION STATISTICS ---\n");
    if (isTx) {
        printf("Frames sent: %lu\n", numFramesSent);
        printf("Data frames sent: %lu\n", numDataFrames);
        printf("REJ frames received: %lu\n", numRejFrames);
    } else {
        printf("Frames received: %lu\n", numFramesReceived);
        printf("Data frames received: %lu\n", numDataFrames);
        printf("REJ frames sent: %lu\n", numRejFrames);
    }
    printf("Total time: %.3f seconds\n", totalTime);
    printf("Timeouts: %d\n", (int)alarmCount);
    if (totalTime > 0) {
        printf("Throughput: %.2f frames/second\n", 
               isTx ? (double)numFramesSent / totalTime : (double)numFramesReceived / totalTime);
    }
    printf("-------------------------------\n");
}

////////////////////////////////////////////////
// Helper functions
////////////////////////////////////////////////

int createFrameHeader(AddressField address, ControlField control, unsigned char* frame) {
    int i = 0;
    frame[i++] = FLAG;                // Start flag
    frame[i++] = address;             // Address field
    frame[i++] = control;             // Control field

    // BCC1 (stuffing)
    if ((frame[1] ^ frame[2]) == FLAG) {
        frame[i++] = STUFF;
        frame[i++] = FLAG_STUFF;
    } else if ((frame[1] ^ frame[2]) == STUFF) {
        frame[i++] = STUFF;
        frame[i++] = STUFF_STUFF;
    } else {
        frame[i++] = frame[1] ^ frame[2];
    }

    return i;
}

static unsigned int buildSFrame(AddressField address, ControlField control, unsigned char* frame) {
    unsigned int idx = createFrameHeader(address, control, frame);
    frame[idx++] = FLAG;
    return idx;
}

// Builds an Information frame with byte stuffing and BCC2
static unsigned int buildIFrame(AddressField address, unsigned char control, unsigned char *frame,
                       const unsigned char *data, int dataSize) {
    unsigned int idx = createFrameHeader(address, control, frame);
    // bcc2 calculation and data stuffing 
    unsigned char bcc2 = 0;
    for (int i = 0; i < dataSize; i++) {
        bcc2 ^= data[i];
        if (data[i] == FLAG) {
            frame[idx++] = STUFF;
            frame[idx++] = FLAG_STUFF;
        } else if (data[i] == STUFF) {
            frame[idx++] = STUFF;
            frame[idx++] = STUFF_STUFF;
        } else {
            frame[idx++] = data[i];
        }
    }

    // Byte stuffing for BCC2
    if (bcc2 == FLAG) {
        frame[idx++] = STUFF;
        frame[idx++] = FLAG_STUFF;
    } else if (bcc2 == STUFF) {
        frame[idx++] = STUFF;
        frame[idx++] = STUFF_STUFF;
    } else {
        frame[idx++] = bcc2;
    }

    frame[idx++] = FLAG; // End flag
    return idx;
}

// State Machine Frame Reader (handles supervision frames and destuffs I-frames)
int readFrame(unsigned char* buf) {
    unsigned char byte;
    unsigned int i = 0;
    State state = START;
    bool isIFrame = FALSE;
    alarmEnabled = TRUE;

    while (state != STOP && alarmEnabled) {
        int readResult = readByteSerialPort(&byte);
        if (readResult == 0)
            continue;
        if (readResult < 0) {
            /* SIGALRM may interrupt the serial read; let the caller handle timeout. */
            if (!alarmEnabled)
                break;
            return -1;
        }

        switch (state) {

            case START:
                i = 0;
                if (byte == FLAG) {
                    buf[i++] = byte;
                    state = FLAG_RCV;
                }
                break;

            case FLAG_RCV:
                if (byte == FLAG) {
                    break;
                } else if (byte == A_SENDER || byte == A_RECEIVER) {
                    buf[i++] = byte;
                    state = A_RCV;
                } else {
                    state = START;
                }
                break;

            case A_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    i = 1;
                    buf[0] = FLAG;
                } else if (byte == SET || byte == UA || byte == DISC ||
                           byte == RR_0 || byte == RR_1 ||
                           byte == REJ_0 || byte == REJ_1 ||
                           byte == INFO_FRAME_0 || byte == INFO_FRAME_1) {
                    buf[i++] = byte;
                    isIFrame = (byte == INFO_FRAME_0 || byte == INFO_FRAME_1);
                    state = C_RCV;
                } else {
                    state = START;
                }
                break;

            case C_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    i = 1;
                    buf[0] = FLAG;
                } else if (byte == STUFF) {
                    break;
                } else if (byte == STUFF_STUFF) {
                    buf[i++] = STUFF;
                } else if (byte == FLAG_STUFF) {
                    buf[i++] = FLAG;
                } else {
                    buf[i++] = byte;
                }

                if (buf[3] != buf[1] ^ buf[2]) {
                    // BCC1 error
                    return -1;
                }
                break;

            case BCC1_OK:

                // Supervision or unnumbered frame
                if (!isIFrame) {
                    if (byte == FLAG) {
                        buf[i++] = byte;
                        state = STOP;
                    } else {
                        state = START;
                    }
                    return 0;
                // Information frame
                } else {
                    state = RCV_DATA;
                }

                // No break here because we want it to also run RCV_DATA for destuffing the current byte

            case RCV_DATA: 

                // Destuffing
                if (byte == STUFF) {
                    // skip 0x7D
                    break;
                } else if (byte == FLAG_STUFF) {
                    buf[i++] = FLAG;
                } else if (byte == STUFF_STUFF) {
                    buf[i++] = STUFF;
                } else if (byte == FLAG) {
                    // End of data
                    buf[i++] = byte;
                    state = STOP;
                } else {
                    // Normal byte, no destuffing
                    buf[i++] = byte;
                }
                break;

                
            case STOP:
                if (isIFrame) {
                    // Calculate bcc2
                    unsigned char bcc2 = 0;
                    // Start in the second element of the data
                    for (int idx = 5; idx++; idx < i - 1) {
                        bcc2 = buf[idx - 1] ^ buf[idx];
                    }
                    if (bcc2 != buf[i-1]) {
                        return -2;
                    }
                }
                break;
            }
        }
        
    alarm(0);
    if (!alarmEnabled) return -3;
    return (int) i;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////


int llOpenTx(LinkLayer llParameters) {
    llParams = llParameters;
    gettimeofday(&startTime, NULL);

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0) {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened (Tx)\n", llParameters.serialPort);

    configureAlarm();
    alarmCount = 0;

    unsigned char setBuf[SUPERVISION_FRAME_SIZE + 1];
    unsigned char uaBuf[SUPERVISION_FRAME_SIZE + 1] = {0};
    int setBufSize = buildSFrame(A_SENDER, SET, setBuf);
    unsigned char attempts = 1;
    int maxAttempts = 1 + llParameters.nRetransmissions;

    while (attempts <= maxAttempts) {
        printf("Sending SET frame, attempt %d/%d\n",
               attempts, maxAttempts);

        int written = writeBytesSerialPort(setBuf, setBufSize);
        numFramesSent++;

        if (written != setBufSize) {
            perror("writeBytesSerialPort (SET)");
            closeSerialPort();
            return -1;
        }

        alarm(llParameters.timeout);
        int res = readFrame(uaBuf);

        if (res == -1) {
            // BCC1 error
            attempts++;
        } else if (res == -3) {
            // Timeout
            attempts++;
        } else if (res == SUPERVISION_FRAME_SIZE &&
            uaBuf[1] == A_SENDER &&
            uaBuf[2] == UA) {
            // Everything ok
            printf("Received UA, connection established\n");
            return 0;
        }
    }

    printf("Failed to establish connection in llOpenTx after %d attempts\n",
           maxAttempts);
    closeSerialPort();
    return -1;
}


int llOpenRx(LinkLayer llParameters) {
    llParams = llParameters;
    gettimeofday(&startTime, NULL);

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0) {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened (Rx)\n", llParameters.serialPort);

    configureAlarm();
    unsigned char buf[BUF_SIZE] = {0};

    while (TRUE) {

        int res = readFrame(buf);

        if (res == -1) {
            // BCC1 error
        } else if (res == SUPERVISION_FRAME_SIZE && buf[1] == A_SENDER && buf[2] == SET) {
            printf("Received SET, sending UA\n");
            int uaSize = buildSFrame(A_SENDER, UA, buf);
            int written = writeBytesSerialPort(buf, uaSize);
            numFramesSent++;
            if (written != uaSize) {
                perror("writeBytesSerialPort (UA)");
                closeSerialPort();
                break;
            }
            return 0;
        }
    }
    return -1;

}

////////////////////////////////////////////////
// LLSEND (Stop-and-Wait Error Control)
////////////////////////////////////////////////

int llSend(const unsigned char *buf, int bufSize) {
    if (buf == NULL || bufSize < 0 || bufSize > MAX_PAYLOAD_SIZE) {
        fprintf(stderr, "llSend: invalid payload or size (%d)\n", bufSize);
        return -1;
    }

    alarmCount = 0;

    unsigned char frame[2 * MAX_PAYLOAD_SIZE + 20];
    int frameSize = buildIFrame(A_SENDER, txFrameNum, frame, buf, bufSize);
    unsigned char expectedRR = (txFrameNum == INFO_FRAME_0) ? RR_1 : RR_0;
    unsigned char rejCtrl = (txFrameNum == INFO_FRAME_0) ? REJ_0 : REJ_1;
    unsigned char respBuf[BUF_SIZE] = {0};
    int maxAttempts = 1 + llParams.nRetransmissions;

    for (int attempts = 1; attempts <= maxAttempts; attempts++) {
        int written = writeBytesSerialPort(frame, frameSize);
        numFramesSent++;
        numDataFrames++;
        if (written != frameSize) {
            perror("writeBytesSerialPort (I frame)");
            return -1;
        }

        alarm(llParams.timeout);

        int bytesRead = readFrame(respBuf);
        

        if (bytesRead == -1) {
            //BCC1 error
            break;
        } else if (bytesRead == -3) {
            // Timeout
            break;
        } else if (bytesRead != SUPERVISION_FRAME_SIZE || respBuf[1] != A_SENDER) {
            break;
        }

        if (respBuf[2] == expectedRR) { 
            txFrameNum = (txFrameNum == INFO_FRAME_0) ? INFO_FRAME_1 : INFO_FRAME_0;
            return bufSize;
        }
        if (respBuf[2] == rejCtrl) {        
            numRejFrames++;
            printf("Received REJ; retransmitting frame (attempt %d/%d)\n", attempts + 1, maxAttempts);
            break;
        }
    }

    fprintf(stderr, "llSend failed after %d attempts\n", maxAttempts);
    return -1;
}

////////////////////////////////////////////////
// LLRECEIVE (Stop-and-Wait Error Control)
////////////////////////////////////////////////

int llReceive(unsigned char *packet) {
    if (packet == NULL) {
        fprintf(stderr, "llReceive: packet buffer is NULL\n");
        return -1;
    }

    alarmCount = 0;

    unsigned char frameBuf[BUF_SIZE];
    unsigned char respBuf[SUPERVISION_FRAME_SIZE];

    while (TRUE) {
        int res = readFrame(frameBuf);

        unsigned char rejCtrl = (rxExpectedFrame == INFO_FRAME_0) ? REJ_0 : REJ_1;
        unsigned char rrCtrl = (rxExpectedFrame == INFO_FRAME_0) ? RR_0 : RR_1;
        unsigned char duplicateCtrl = (rxExpectedFrame == INFO_FRAME_0) ? INFO_FRAME_1 : INFO_FRAME_0;

        if (res == -2) {
            /* BCC2 error: readFrame preserves the valid address/control fields on BCC2 errors. */
            if (frameBuf[2] == rxExpectedFrame) {
                int respSize = buildSFrame(A_SENDER, rejCtrl, respBuf);
                if (writeBytesSerialPort(respBuf, respSize) != respSize) break;
                numRejFrames++;
            }
            numFramesSent++;
            continue;
        } else if (res == 0 || frameBuf[1] != A_SENDER) {
            continue;
        } else {

            if (frameBuf[2] == duplicateCtrl) {
                int respSize = buildSFrame(A_SENDER, rrCtrl, respBuf);
                if (writeBytesSerialPort(respBuf, respSize) != respSize) break;
                numFramesSent++;
            } else if (frameBuf[2] == rxExpectedFrame) {
                int payloadLen = res - 6;
                if (payloadLen < 0 || payloadLen > MAX_PAYLOAD_SIZE) continue;

                memcpy(packet, frameBuf + 4, (size_t)payloadLen);
                rxExpectedFrame = (rxExpectedFrame == INFO_FRAME_0) ? INFO_FRAME_1 : INFO_FRAME_0;
                rrCtrl = (rxExpectedFrame == INFO_FRAME_0) ? RR_1 : RR_0;
                int respSize = buildSFrame(A_SENDER, rrCtrl, respBuf);
                if (writeBytesSerialPort(respBuf, respSize) != respSize) break;
                numDataFrames++;
                numFramesSent++;
                return payloadLen;
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

int llCloseTx() {
    unsigned char discBuf[SUPERVISION_FRAME_SIZE];
    unsigned char respBuf[BUF_SIZE];
    int discSize = buildSFrame(A_SENDER, DISC, discBuf);
    int maxAttempts = 1 + llParams.nRetransmissions;

    for (int attempts = 1; attempts <= maxAttempts; attempts++) {
        printf("Sending DISC frame (Tx), attempt %d/%d\n", attempts, maxAttempts);
        int written = writeBytesSerialPort(discBuf, discSize);
        if (written != discSize) {
            perror("writeBytesSerialPort (DISC)");
            closeSerialPort();
            return -1;
        }
        numFramesSent++;

        alarmEnabled = FALSE;
        alarm(llParams.timeout);
        while (!alarmEnabled) {
            int bytesRead = readFrame(respBuf);
            if (alarmEnabled)
                break;
            if (bytesRead < 0) {
                alarm(0);
                closeSerialPort();
                return -1;
            }
            if (bytesRead == SUPERVISION_FRAME_SIZE &&
                respBuf[1] == A_RECEIVER && respBuf[2] == DISC) {
                alarm(0);
                printf("Received DISC frame from Rx, sending UA\n");
                unsigned char uaBuf[SUPERVISION_FRAME_SIZE];
                int uaSize = buildSFrame(A_RECEIVER, UA, uaBuf);
                if (writeBytesSerialPort(uaBuf, uaSize) != uaSize)
                    perror("writeBytesSerialPort (UA)");
                else
                    numFramesSent++;
                sleep(1);
                closeSerialPort();
                printStatistics(TRUE);
                return 0;
            }
            /* Keep waiting for the expected DISC without resetting the timer. */
        }
        alarm(0);
    }

    fprintf(stderr, "llCloseTx failed after %d attempts\n", maxAttempts);
    closeSerialPort();
    return -1;
}

int llCloseRx() {
    unsigned char buf[BUF_SIZE];
    unsigned char discBuf[SUPERVISION_FRAME_SIZE];
    int discSize = buildSFrame(A_RECEIVER, DISC, discBuf);

    /* Wait for the first DISC from Tx. This phase has no timer by design. */
    while (TRUE) {
        int bytesRead = readFrame(buf);
        if (bytesRead < 0) {
            if (alarmEnabled)
                continue;
            perror("readFrame while waiting for initial DISC");
            closeSerialPort();
            return -1;
        }
        if (bytesRead == SUPERVISION_FRAME_SIZE && buf[1] == A_SENDER && buf[2] == DISC)
            break;
    }

    printf("Received DISC frame from Tx, sending DISC\n");
    int written = writeBytesSerialPort(discBuf, discSize);
    if (written != discSize) {
        perror("writeBytesSerialPort (Rx DISC)");
        closeSerialPort();
        return -1;
    }
    numFramesSent++;

    int maxAttempts = 1 + llParams.nRetransmissions;
    int attempts = 1; /* The first DISC sent above is attempt 1. */
    alarmEnabled = FALSE;
    alarm(llParams.timeout);

    while (TRUE) {
        if (alarmEnabled) {
            if (attempts >= maxAttempts)
                break;
            written = writeBytesSerialPort(discBuf, discSize);
            if (written != discSize) {
                alarm(0);
                perror("writeBytesSerialPort (Rx DISC retry)");
                closeSerialPort();
                return -1;
            }
            attempts++;
            numFramesSent++;
            alarmEnabled = FALSE;
            alarm(llParams.timeout);
            continue;
        }

        int bytesRead = readFrame(buf);
        if (alarmEnabled)
            continue;
        if (bytesRead < 0) {
            alarm(0);
            perror("readFrame while waiting for UA");
            closeSerialPort();
            return -1;
        }
        if (bytesRead != SUPERVISION_FRAME_SIZE)
            continue;

        if (buf[1] == A_RECEIVER && buf[2] == UA) {
            alarm(0);
            printf("Received UA frame from Tx\n");
            sleep(1);
            closeSerialPort();
            printStatistics(FALSE);
            return 0;
        }

        /* A repeated DISC means Tx did not receive our DISC; resend it, bounded by retries. */
        if (buf[1] == A_SENDER && buf[2] == DISC && attempts < maxAttempts) {
            written = writeBytesSerialPort(discBuf, discSize);
            if (written != discSize) {
                alarm(0);
                perror("writeBytesSerialPort (duplicate DISC response)");
                closeSerialPort();
                return -1;
            }
            attempts++;
            numFramesSent++;
            alarmEnabled = FALSE;
            alarm(llParams.timeout);
        }
        /* Unrelated frames do not reset the running timer. */
    }

    alarm(0);
    fprintf(stderr, "llCloseRx failed: timed out waiting for UA after %d attempts\n", attempts);
    closeSerialPort();
    printStatistics(FALSE);
    return -1;
}
