// RCOM 2026/2027
// Link layer protocol implementation
//
// Integrated Stop-and-Wait Error Control Mechanism with State Machine

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 2048
#define SUPERVISION_FRAME_SIZE 5

#define STUFF 0x7D
#define FLAG_STUFF 0x5E
#define STUFF_STUFF 0x5D

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC1_OK,
    STOP
} State;

// Control Field values strictly following the project specification (Slide 10):
// RR0:  10101010 (0xAA) - Receiver ready to receive Information Frame 0
// RR1:  10101011 (0xAB) - Receiver ready to receive Information Frame 1
// REJ0: 01010100 (0x54) - Receiver rejects Information Frame 0
// REJ1: 01010101 (0x55) - Receiver rejects Information Frame 1
#define RR_0 0xAA
#define RR_1 0xAB
#define REJ_0 0x54
#define REJ_1 0x55

// Link layer parameters
static LinkLayer llParams;

// Sequence numbers for Stop-and-Wait
static unsigned char txFrameNum = INFO_FRAME_0;      // 0x00 or 0x80
static unsigned char rxExpectedFrame = INFO_FRAME_0; // 0x00 or 0x80

// Alarm handling
int alarmEnabled = FALSE;
int alarmCount = 0;

// Statistics
static unsigned long numFramesSent = 0;
static unsigned long numFramesReceived = 0;
static unsigned long numDataFrames = 0;
static unsigned long numRejFrames = 0;
static unsigned long numTimeouts = 0;
static struct timeval startTime;

void alarmHandler(int signal) {
    alarmCount++;
    alarmEnabled = TRUE;
    numTimeouts++;
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
    printf("Timeouts: %lu\n", numTimeouts);
    if (totalTime > 0) {
        printf("Throughput: %.2f frames/second\n", 
               isTx ? (double)numFramesSent / totalTime : (double)numFramesReceived / totalTime);
    }
    printf("-------------------------------\n");
}

////////////////////////////////////////////////
// Helper functions
////////////////////////////////////////////////

int createFrame(AddressField address, ControlField control, unsigned char* frame) {
    frame[0] = FLAG;                // Start flag
    frame[1] = address;             // Address field
    frame[2] = control;             // Control field
    frame[3] = frame[1] ^ frame[2]; // BCC1
    frame[4] = FLAG;                // End flag
    return 5;
}

// Builds an Information frame with byte stuffing and BCC2
static int buildIFrame(AddressField address, unsigned char control, unsigned char *frame,
                       const unsigned char *data, int dataSize) {
    int idx = 0;
    frame[idx++] = FLAG;
    frame[idx++] = address;
    frame[idx++] = control;
    frame[idx++] = address ^ control; // BCC1

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

// createIFrame matching the exact 4-argument prototype in link_layer.h
int createIFrame(AddressField address, ControlField control, unsigned char* frame,
                 const unsigned char* data) {
    int dataSize = data ? strlen((const char*)data) : 0;
    return buildIFrame(address, control, frame, data, dataSize);
}

// State Machine Frame Reader (handles supervision frames and destuffs I-frames)
int readFrame(unsigned char* buf) {
    unsigned char byte;
    unsigned int i = 0;
    State state = START;
    int isIFrame = FALSE;
    int destuffNext = FALSE;
    int payloadStart = 4;

    while (state != STOP && !alarmEnabled) {
        if (readByteSerialPort(&byte) < 1)
            continue;

        switch (state) {
            case START:
                if (byte == FLAG) {
                    buf[0] = byte;
                    i = 1;
                    state = FLAG_RCV;
                    destuffNext = FALSE;
                    isIFrame = FALSE;
                }
                break;

            case FLAG_RCV:
                if (byte == FLAG) {
                    // Repeated FLAG: stay in FLAG_RCV
                    break;
                } else if (byte == A_SENDER || byte == A_RECEIVER) {
                    buf[i++] = byte;
                    state = A_RCV;
                } else {
                    state = START;
                    i = 0;
                    destuffNext = FALSE;
                    isIFrame = FALSE;
                }
                break;

            case A_RCV:
                if (byte == FLAG) {
                    // Unexpected FLAG: restart cleanly from FLAG_RCV
                    state = FLAG_RCV;
                    i = 1;
                    destuffNext = FALSE;
                    isIFrame = FALSE;
                } else if (byte == SET || byte == UA || byte == DISC ||
                           byte == RR_0 || byte == RR_1 ||
                           byte == REJ_0 || byte == REJ_1 ||
                           byte == INFO_FRAME_0 || byte == INFO_FRAME_1) {
                    buf[i++] = byte;
                    isIFrame = (byte == INFO_FRAME_0 || byte == INFO_FRAME_1);
                    state = C_RCV;
                } else {
                    state = START;
                    i = 0;
                    destuffNext = FALSE;
                    isIFrame = FALSE;
                }
                break;

            case C_RCV:
                if (byte == FLAG) {
                    // Unexpected FLAG: restart cleanly from FLAG_RCV
                    state = FLAG_RCV;
                    i = 1;
                    destuffNext = FALSE;
                    isIFrame = FALSE;
                } else if (byte == (buf[1] ^ buf[2])) {
                    buf[i++] = byte;
                    state = BCC1_OK;
                    payloadStart = i; // payload starts at index 4
                } else {
                    state = START;
                    i = 0;
                    destuffNext = FALSE;
                    isIFrame = FALSE;
                }
                break;

            case BCC1_OK:
                if (destuffNext) {
                    destuffNext = FALSE;
                    if (byte == FLAG_STUFF) {
                        buf[i++] = FLAG;
                    } else if (byte == STUFF_STUFF) {
                        buf[i++] = STUFF;
                    } else if (byte == FLAG) {
                        // Illegal escape followed by FLAG: restart cleanly
                        state = FLAG_RCV;
                        i = 1;
                        destuffNext = FALSE;
                        isIFrame = FALSE;
                        break;
                    } else {
                        // Invalid escape sequence
                        state = START;
                        i = 0;
                        destuffNext = FALSE;
                        isIFrame = FALSE;
                        break;
                    }

                    // Check if destuffed payload exceeds maximum allowed payload (+1 for BCC2)
                    if (isIFrame && (i - payloadStart) > (MAX_PAYLOAD_SIZE + 1)) {
                        state = START;
                        i = 0;
                        destuffNext = FALSE;
                        isIFrame = FALSE;
                    }
                } else if (byte == FLAG) {
                    if (!isIFrame) {
                        // Supervision frame complete (5 bytes: FLAG, A, C, BCC1, FLAG)
                        buf[i++] = byte;
                        state = STOP;
                    } else {
                        // Information frame ends with FLAG
                        if (i <= (unsigned int)payloadStart) {
                            state = START;
                            i = 0;
                            destuffNext = FALSE;
                            isIFrame = FALSE;
                            break;
                        }

                        int dataLen = (i - 1) - payloadStart; // exclude BCC2
                        if (dataLen < 0 || dataLen > MAX_PAYLOAD_SIZE) {
                            state = START;
                            i = 0;
                            destuffNext = FALSE;
                            isIFrame = FALSE;
                            break;
                        }

                        // Verify BCC2
                        unsigned char receivedBCC2 = buf[i - 1];
                        unsigned char calcBCC2 = 0;
                        for (int k = 0; k < dataLen; k++) {
                            calcBCC2 ^= buf[payloadStart + k];
                        }

                        if (calcBCC2 == receivedBCC2) {
                            buf[i++] = byte; // End FLAG
                            state = STOP;
                        } else {
                            // BCC2 error detected
                            numFramesReceived++;
                            return -2;
                        }
                    }
                } else {
                    if (!isIFrame) {
                        // Supervision frames must not have data bytes after BCC1
                        state = START;
                        i = 0;
                        destuffNext = FALSE;
                        isIFrame = FALSE;
                    } else {
                        if (byte == STUFF) {
                            destuffNext = TRUE;
                        } else {
                            buf[i++] = byte;
                            // Check if payload exceeds maximum allowed (+1 for BCC2)
                            if ((i - payloadStart) > (MAX_PAYLOAD_SIZE + 1)) {
                                state = START;
                                i = 0;
                                destuffNext = FALSE;
                                isIFrame = FALSE;
                            }
                        }
                    }
                }
                break;

            case STOP:
                break;
        }
    }

    if (state == STOP) {
        numFramesReceived++;
        return i;
    }

    return -1;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////

int llOpenTx(LinkLayer llParameters) {
    llParams = llParameters;
    txFrameNum = INFO_FRAME_0;
    gettimeofday(&startTime, NULL);

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0) {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened (Tx)\n", llParameters.serialPort);

    configureAlarm();

    unsigned char setBuf[BUF_SIZE] = {0};
    unsigned char uaBuf[BUF_SIZE] = {0};
    int setBufSize = createFrame(A_SENDER, SET, setBuf);

    int attempts = 0;
    int maxAttempts = 1 + llParameters.nRetransmissions;

    while (attempts < maxAttempts) {
        attempts++;
        printf("Sending SET frame, attempt %d/%d\n", attempts, maxAttempts);
        if (writeBytesSerialPort(setBuf, setBufSize) < 0) {
            perror("writeBytesSerialPort");
            closeSerialPort();
            return -1;
        }
        numFramesSent++;

        alarmEnabled = FALSE;
        alarm(llParameters.timeout);
        int res = readFrame(uaBuf);
        alarm(0);

        // Reply UA sent by Receiver must carry Address A_SENDER (0x03) per Slide 10 & 14
        if (res > 0 && uaBuf[1] == A_SENDER && uaBuf[2] == UA) {
            printf("Received UA, connection established\n");
            return 0;
        }
    }

    printf("Failed to establish connection in llOpenTx after %d attempts\n", attempts);
    closeSerialPort();
    return -1;
}

int llOpenRx(LinkLayer llParameters) {
    llParams = llParameters;
    rxExpectedFrame = INFO_FRAME_0;
    gettimeofday(&startTime, NULL);

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0) {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened (Rx)\n", llParameters.serialPort);

    configureAlarm();

    unsigned char buf[BUF_SIZE] = {0};

    while (TRUE) {
        alarmEnabled = FALSE;
        int res = readFrame(buf);
        if (res > 0 && buf[1] == A_SENDER && buf[2] == SET) {
            printf("Received SET, sending UA\n");
            // Reply sent by Rx uses A_SENDER (0x03) according to Slide 10 & 14
            int bufSize = createFrame(A_SENDER, UA, buf);
            if (writeBytesSerialPort(buf, bufSize) < 0) {
                perror("writeBytesSerialPort");
                closeSerialPort();
                return -1;
            }
            numFramesSent++;
            return 0;
        }
    }

    return -1;
}

////////////////////////////////////////////////
// LLSEND (Stop-and-Wait Error Control)
////////////////////////////////////////////////

int llSend(const unsigned char *buf, int bufSize) {
    if (bufSize < 0 || bufSize > MAX_PAYLOAD_SIZE) {
        printf("Payload size %d exceeds MAX_PAYLOAD_SIZE\n", bufSize);
        return -1;
    }

    unsigned char frame[2 * MAX_PAYLOAD_SIZE + 20];
    int frameSize = buildIFrame(A_SENDER, txFrameNum, frame, buf, bufSize);

    // Expected RR and REJ according to Slide 10:
    // If txFrameNum == INFO_FRAME_0 (0x00), expecting RR1 (0xAB) or REJ0 (0x54)
    // If txFrameNum == INFO_FRAME_1 (0x80), expecting RR0 (0xAA) or REJ1 (0x55)
    unsigned char expectedRR = (txFrameNum == INFO_FRAME_0) ? RR_1 : RR_0;
    unsigned char rejCtrl = (txFrameNum == INFO_FRAME_0) ? REJ_0 : REJ_1;

    unsigned char respBuf[BUF_SIZE] = {0};
    int attempts = 0;
    int maxAttempts = 1 + llParams.nRetransmissions;

    while (attempts < maxAttempts) {
        attempts++;
        if (writeBytesSerialPort(frame, frameSize) < 0) {
            perror("writeBytesSerialPort");
            return -1;
        }
        numFramesSent++;
        numDataFrames++;

        alarmEnabled = FALSE;
        alarm(llParams.timeout);
        int bytesRead = readFrame(respBuf);
        alarm(0);

        // Replies sent by Receiver have Address A_SENDER (0x03) per Slide 10 & 14
        if (bytesRead == 5 && respBuf[1] == A_SENDER) {
            if (respBuf[2] == expectedRR) {
                // Positive acknowledgment: toggle sequence number for next frame
                txFrameNum = (txFrameNum == INFO_FRAME_0) ? INFO_FRAME_1 : INFO_FRAME_0;
                return bufSize;
            } else if (respBuf[2] == rejCtrl) {
                // Negative acknowledgment: retransmit (attempts already incremented)
                numRejFrames++;
                printf("Received REJ frame, retransmitting (attempt %d/%d)...\n", attempts + 1, maxAttempts);
                continue;
            }
        }
    }

    printf("llSend failed after maximum attempts (%d)\n", maxAttempts);
    return -1;
}

////////////////////////////////////////////////
// LLRECEIVE (Stop-and-Wait Error Control)
////////////////////////////////////////////////

int llReceive(unsigned char *packet) {
    unsigned char frameBuf[BUF_SIZE];
    unsigned char respBuf[SUPERVISION_FRAME_SIZE];

    unsigned char duplicateCtrl = (rxExpectedFrame == INFO_FRAME_0) ? INFO_FRAME_1 : INFO_FRAME_0;
    unsigned char rrForCurrentExpected = (rxExpectedFrame == INFO_FRAME_0) ? RR_0 : RR_1;

    while (TRUE) {
        alarmEnabled = FALSE;
        int res = readFrame(frameBuf);

        if (res == -2) {
            // BCC2 checksum error on data field
            if (frameBuf[1] != A_SENDER)
                continue;

            if (frameBuf[2] == rxExpectedFrame) {
                // Corrupted new frame -> send REJ for this expected frame (Slide 15)
                unsigned char rejCtrl = (rxExpectedFrame == INFO_FRAME_0) ? REJ_0 : REJ_1;
                int respSize = createFrame(A_SENDER, rejCtrl, respBuf);
                if (writeBytesSerialPort(respBuf, respSize) > 0) {
                    numFramesSent++;
                }
                numRejFrames++;
            } else if (frameBuf[2] == duplicateCtrl) {
                // Corrupted duplicate frame -> confirm with RR for next expected frame (Slide 15)
                int respSize = createFrame(A_SENDER, rrForCurrentExpected, respBuf);
                if (writeBytesSerialPort(respBuf, respSize) > 0) {
                    numFramesSent++;
                }
            }
            continue;
        }

        if (res <= 0 || frameBuf[1] != A_SENDER)
            continue;

        // Duplicate frame received -> resend RR for current expected frame so transmitter moves forward (Slide 15)
        if (frameBuf[2] == duplicateCtrl) {
            int respSize = createFrame(A_SENDER, rrForCurrentExpected, respBuf);
            if (writeBytesSerialPort(respBuf, respSize) > 0) {
                numFramesSent++;
            }
            continue;
        }

        // Correct expected frame received
        if (frameBuf[2] == rxExpectedFrame) {
            numDataFrames++;

            int payloadLen = res - 6; // Subtract FLAG, A, C, BCC1, BCC2, FLAG
            if (payloadLen < 0 || payloadLen > MAX_PAYLOAD_SIZE)
                continue; // Reject oversized frame
            memcpy(packet, frameBuf + 4, payloadLen);

            // Toggle expected sequence number
            rxExpectedFrame = (rxExpectedFrame == INFO_FRAME_0) ? INFO_FRAME_1 : INFO_FRAME_0;
            // Send RR for the new expected frame number
            unsigned char nextRR = (rxExpectedFrame == INFO_FRAME_0) ? RR_0 : RR_1;

            int respSize = createFrame(A_SENDER, nextRR, respBuf);
            if (writeBytesSerialPort(respBuf, respSize) > 0) {
                numFramesSent++;
            }

            return payloadLen;
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
    int discSize = createFrame(A_SENDER, DISC, discBuf);

    int attempts = 0;
    int maxAttempts = 1 + llParams.nRetransmissions;

    while (attempts < maxAttempts) {
        attempts++;
        printf("Sending DISC frame (Tx), attempt %d/%d\n", attempts, maxAttempts);
        if (writeBytesSerialPort(discBuf, discSize) < 0) {
            perror("writeBytesSerialPort");
            closeSerialPort();
            return -1;
        }
        numFramesSent++;

        alarmEnabled = FALSE;
        alarm(llParams.timeout);
        int bytesRead = readFrame(respBuf);
        alarm(0);

        // DISC from Rx is a command sent by Rx, so Address is A_RECEIVER (0x01) per Slide 10 & 14
        if (bytesRead > 0 && respBuf[1] == A_RECEIVER && respBuf[2] == DISC) {
            printf("Received DISC frame from Rx, sending UA\n");
            unsigned char uaBuf[SUPERVISION_FRAME_SIZE];
            // Reply sent by Tx uses A_RECEIVER (0x01) per Slide 10 & 14
            int uaSize = createFrame(A_RECEIVER, UA, uaBuf);
            if (writeBytesSerialPort(uaBuf, uaSize) < 0) {
                perror("writeBytesSerialPort");
            } else {
                numFramesSent++;
            }

            sleep(1);
            closeSerialPort();
            printStatistics(TRUE);
            return 0;
        }
    }

    printf("llCloseTx failed after %d attempts\n", attempts);
    closeSerialPort();
    return -1;
}

int llCloseRx() {
    unsigned char buf[BUF_SIZE];
    unsigned char discBuf[SUPERVISION_FRAME_SIZE];
    // Command sent by Rx uses A_RECEIVER (0x01) according to Slide 10 & 14
    int discSize = createFrame(A_RECEIVER, DISC, discBuf);

    // Wait for DISC from Tx (A_SENDER = 0x03)
    while (TRUE) {
        alarmEnabled = FALSE;
        int bytesRead = readFrame(buf);
        if (bytesRead > 0 && buf[1] == A_SENDER && buf[2] == DISC) {
            printf("Received DISC frame from Tx, sending DISC\n");
            if (writeBytesSerialPort(discBuf, discSize) < 0) {
                perror("writeBytesSerialPort");
                closeSerialPort();
                return -1;
            }
            numFramesSent++;
            break;
        }
    }

    // Wait for UA reply from Tx (A_RECEIVER = 0x01) with timer protection and retransmission of DISC
    int attempts = 0;
    int maxAttempts = 1 + llParams.nRetransmissions;

    while (attempts < maxAttempts) {
        attempts++;
        alarmEnabled = FALSE;
        alarm(llParams.timeout);
        int bytesRead = readFrame(buf);
        alarm(0);

        if (bytesRead > 0) {
            if (buf[1] == A_RECEIVER && buf[2] == UA) {
                printf("Received UA frame from Tx\n");
                sleep(1);
                closeSerialPort();
                printStatistics(FALSE);
                return 0;
            }
            if (buf[1] == A_SENDER && buf[2] == DISC) {
                printf("Received duplicate DISC from Tx, resending DISC\n");
                writeBytesSerialPort(discBuf, discSize);
                numFramesSent++;
            }
        }

        if (alarmEnabled) {
            printf("Timeout waiting for UA in llCloseRx, resending DISC (attempt %d/%d)\n", attempts + 1, maxAttempts);
            if (attempts < maxAttempts) {
                writeBytesSerialPort(discBuf, discSize);
                numFramesSent++;
            }
        }
    }

    // Return failure (-1) if attempts exhausted without UA
    printf("llCloseRx failed: timed out waiting for UA\n");
    closeSerialPort();
    printStatistics(FALSE);
    return -1;
}
