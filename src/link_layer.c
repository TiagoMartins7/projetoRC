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
#define BUF_SIZE 256
#define SUPERVISION_FRAME_SIZE 5

#define STUFF 0x7D
#define FLAG_STUFF 0x5E
#define STUFF_STUFF 0x5D

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

int createIFrame(AddressField address, ControlField control, unsigned char* frame,
                 const unsigned char* data) {
    // Standard signature helper
    return createFrame(address, control, frame);
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
                }
                break;

            case FLAG_RCV:
                if (byte == FLAG) {
                    // Stay int this state
                    break;
                } else if (byte == A_SENDER || byte == A_RECEIVER) {
                    // Valid address field, continue
                    buf[i++] = byte;
                    state = A_RCV;
                } else{
                    // Invalid frame
                    state = START;
                    i = 0;
                }
                break;

            case A_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    i = 1;
                } else if (byte == SET || byte == UA || byte == DISC ||
                           byte == RR || byte == (0x80 | RR) ||
                           byte == REJ || byte == (0x80 | REJ) ||
                           byte == INFO_FRAME_0 || byte == INFO_FRAME_1) {
                    buf[i++] = byte;
                    isIFrame = (byte == INFO_FRAME_0 || byte == INFO_FRAME_1);
                    state = C_RCV;
                } else {
                    state = START;
                    i = 0;
                }
                break;

            case C_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    i = 1;
                } else if (byte == (buf[1] ^ buf[2])) {
                    buf[i++] = byte;
                    state = BCC1_OK;
                    payloadStart = i; // payload starts at index 4
                } else {
                    state = START;
                    i = 0;
                }
                break;

            case BCC1_OK:
                if (byte == FLAG) {
                    if (!isIFrame) {
                        // Supervision frame complete (5 bytes)
                        buf[i++] = byte;
                        state = STOP;
                    } else{
                        // Information frame ends with FLAG
                        if (i <= (unsigned int)payloadStart) {
                            state = START;
                            i = 0;
                            break;
                        }

                        // Last received byte was BCC2
                        unsigned char receivedBCC2 = buf[i - 1];
                        int dataLen = (i - 1) - payloadStart;

                        // Check BCC2
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
                        state = START;
                        i = 0;
                    } else {
                        // Handle byte destuffing
                        if (byte == STUFF) {
                            destuffNext = TRUE;
                        } else if (destuffNext) {
                            if (byte == FLAG_STUFF) {
                                buf[i++] = FLAG;
                            } else if (byte == STUFF_STUFF) {
                                buf[i++] = STUFF;
                            } else {
                                state = START;
                                i = 0;
                            }
                            destuffNext = FALSE;
                        } else {
                            buf[i++] = byte;
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

    alarmCount = 0;
    while(alarmCount < llParameters.nRetransmissions) {
        printf("Sending SET frame, attempt %d\n", alarmCount + 1);
        writeBytesSerialPort(setBuf, setBufSize);
        numFramesSent++;

        alarm(llParameters.timeout);
        int res = readFrame(uaBuf);
        alarm(0);
        alarmEnabled = FALSE;

        if (res > 0 && uaBuf[1] == A_RECEIVER && uaBuf[2] == UA) {
            alarm(0);
            printf("Received UA, connection established\n");
            return 0;
        }
    }
    alarm(0);

    printf("Failed to establish connection in llOpenTx\n");
    closeSerialPort();
    return -1;
}

int llOpenRx(LinkLayer llParameters) {
    llParams = llParameters;
    gettimeofday(&startTime, NULL);

    if(openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0) {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened (Rx)\n", llParameters.serialPort);

    unsigned char buf[BUF_SIZE] = {0};

    while (TRUE){
        alarmEnabled = FALSE;
        int res = readFrame(buf);
        if (res > 0 && buf[1] == A_SENDER && buf[2] == SET) {
            printf("Received SET, sending UA\n");
            int bufSize = createFrame(A_RECEIVER, UA, buf);
            writeBytesSerialPort(buf, bufSize);
            numFramesSent++;
            return 0;
        }
    }

    return -1;
}

////////////////////////////////////////////////
// LLSEND 
////////////////////////////////////////////////

int llSend(const unsigned char *buf, int bufSize){
    if(bufSize< 0 ||  > MAX_PAYLOAD_SIZE) {
        printf("Payload size %d exceeds MAX_PAYLOAD_SIZE\n", bufSize);
        return -1;
    }

    unsigned char frame[2 * MAX_PAYLOAD_SIZE + 20];
    int frameSize = buildIFrame(A_SENDER, txFrameNum, frame, buf, bufSize);

    // Expected RR for current sequence number
    // If txFrameNum == 0x00, expecting RR1 (0x85)
    // If txFrameNum == 0x80, expecting RR0 (0x05)
    unsigned char expectedRR = (txFrameNum == INFO_FRAME_0) ? (0x80 | RR) : RR;
    unsigned char rejCtrl = (txFrameNum == INFO_FRAME_0) ? REJ : (0x80 | REJ);

    alarmCount = 0;
    unsigned char respBuf[BUF_SIZE] = {0};

    int maxAttempts = llParams.nRetransmissions +1;
    int attemps = 0;

    while(alarmCount < llParams.nRetransmissions) {
        attemps++;
        alarmEnabled = FALSE;
        writeBytesSerialPort(frame, frameSize);
        numFramesSent++;
        numDataFrames++;

        alarm(llParams.timeout);
        int bytesRead = readFrame(respBuf);
        alarm(0);
        alarmEnabled = FALSE;

        if(bytesRead==5 && respBuf[1] == A_RECEIVER) {
            if(respBuf[2] == expectedRR){
                alarm(0);
                // Toggle sequence number for next frame
                txFrameNum = (txFrameNum == INFO_FRAME_0) ? INFO_FRAME_1 : INFO_FRAME_0;
                return bufSize;
            }else if (respBuf[2] == rejCtrl){
                alarm(0);
                numRejFrames++;
                printf("Received REJ frame, retransmitting...\n");
                continue;
            }
        }
    }

    alarm(0);
    printf("llSend failed after maximum retransmissions\n");
    return -1;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////

int llReceive(unsigned char *packet) {
    unsigned char frameBuf[2 * MAX_PAYLOAD_SIZE + 20];
    unsigned char respBuf[SUPERVISION_FRAME_SIZE];

    unsigned char duplicateCtrl = (rxExpectedFrame == INFO_FRAME_0) ? INFO_FRAME_1 : INFO_FRAME_0;
    unsigned char rrForCurrentExpected = (rxExpectedFrame == INFO_FRAME_0) ? RR : (0x80 | RR);

    while (TRUE) {
        alarmEnabled = FALSE;
        int res = readFrame(frameBuf);

        
    if(res == -2){
        // BCC2 error: discard the data and check the sequence number.
        // BCC1 was already validated by readFrame().

        if(frameBuf[2] == rxExpectedFrame) {
            // New frame with corrupted data: send REJ.
            unsigned char rejCtrl =
                (rxExpectedFrame == INFO_FRAME_0)
                    ? REJ
                    : (0x80 | REJ);

            int respSize = createFrame(A_RECEIVER, rejCtrl, respBuf);
            writeBytesSerialPort(respBuf, respSize);

            numRejFrames++;
            numFramesSent++;

            }else if(frameBuf[2] == duplicateCtrl) {
                // Duplicate frame with corrupted data: send RR
                // indicating the next frame expected.
                int respSize =
                    createFrame(A_RECEIVER, rrForCurrentExpected, respBuf);

                writeBytesSerialPort(respBuf, respSize);
                numFramesSent++;
            }
            continue;
        }

        if(res <= 0 || frameBuf[1] != A_SENDER)
            continue;

        // Duplicate frame received: resend RR so transmitter moves forward
        if(frameBuf[2] == duplicateCtrl) {
            int respSize = createFrame(A_RECEIVER, rrForCurrentExpected, respBuf);
            writeBytesSerialPort(respBuf, respSize);
            numFramesSent++;
            continue;
        }

        // Correct expected frame received
        if(frameBuf[2] == rxExpectedFrame) {
            numDataFrames++;

            // Payload starts at index 4, length is total - 4 (header) - 1 (BCC2) - 1 (FLAG)
            int payloadLen = res - 6;
            if(payloadLen <0 || payloadLen > MAX_PAYLOAD_SIZE)
                continue;
            memcpy(packet, frameBuf + 4, payloadLen);

            // Toggle expected sequence number
            rxExpectedFrame = (rxExpectedFrame == INFO_FRAME_0) ? INFO_FRAME_1 : INFO_FRAME_0;
            unsigned char nextRR = (rxExpectedFrame == INFO_FRAME_0) ? RR : (0x80 | RR);

            int respSize = createFrame(A_RECEIVER, nextRR, respBuf);
            writeBytesSerialPort(respBuf, respSize);
            numFramesSent++;

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

    alarmCount = 0;
    while(alarmCount < llParams.nRetransmissions){
        alarmEnabled = FALSE;
        printf("Sending DISC frame (Tx)\n");
        writeBytesSerialPort(discBuf, discSize);
        numFramesSent++;

        alarm(llParams.timeout);
        int bytesRead = readFrame(respBuf);
        alarmEnabled = FALSE;

        if(bytesRead > 0 && respBuf[1] == A_RECEIVER && respBuf[2] == DISC) {
            alarm(0);
            printf("Received DISC frame from Rx, sending UA\n");
            unsigned char uaBuf[SUPERVISION_FRAME_SIZE];
            int uaSize = createFrame(A_RECEIVER, UA, uaBuf);
            writeBytesSerialPort(uaBuf, uaSize);
            numFramesSent++;

            sleep(1);
            closeSerialPort();
            printStatistics(TRUE);
            return 0;
        }
    }

    alarm(0);
    closeSerialPort();
    return -1;
}

int llCloseRx() {
    unsigned char buf[BUF_SIZE];
    unsigned char discBuf[SUPERVISION_FRAME_SIZE];

    // Wait for DISC from Tx
    while (TRUE) {
        alarmEnabled = FALSE;
        int bytesRead = readFrame(buf);
        if(bytesRead > 0 && buf[1] == A_SENDER && buf[2] == DISC) {
            printf("Received DISC frame from Tx, sending DISC\n");
            int discSize = createFrame(A_RECEIVER, DISC, discBuf);
            writeBytesSerialPort(discBuf, discSize);
            numFramesSent++;
            break;
        }
    }

    // Wait for UA from Tx
    alarmCount = 0;
    while(alarmCount < llParams.nRetransmissions) {
        alarmEnabled = FALSE;
        alarm(llParams.timeout);
        int bytesRead = readFrame(buf);
        alarmEnabled = FALSE;

        if (bytesRead > 0 && buf[1] == A_RECEIVER && buf[2] == UA) {
            alarm(0);
            printf("Received UA frame from Tx\n");
            sleep(1);
            closeSerialPort();
            printStatistics(FALSE);
            return 0;
        }
    }

    alarm(0);
    closeSerialPort();
    printStatistics(FALSE);
    return 0;
}
