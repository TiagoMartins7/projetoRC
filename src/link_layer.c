// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256


int alarmEnabled = FALSE;
int alarmCount = 0;


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters) {

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0) {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened\n", llParameters.serialPort);


    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }


    unsigned char setBuf[BUF_SIZE] = {0};
    unsigned char uaBuf[BUF_SIZE] = {0};
    int setBufSize = createFrame(A_SENDER, SET, setBuf);

    while (alarmCount < llParameters.nRetransmissions) {
        printf("Sending SET frame, attempt %d\n", alarmCount + 1);
        writeBytesSerialPort(setBuf, setBufSize);

        alarm(llParameters.timeout); // Set the alarm for timeout
        readFrame(uaBuf);  // Read the response frame from the receiver
        alarmEnabled = FALSE;

        if (uaBuf[1] == A_RECEIVER && uaBuf[2] == UA) {
            printf("Received UA, connection established\n");
            break;
        }
    }
    alarm(0); // Cancel the alarm
    
    if (closeSerialPort() < 0) {
        perror("closeSerialPort");
        return -1;
    }
    printf("Serial port %s closed\n", llParameters.serialPort);
    return 0;
}

int llOpenRx(LinkLayer llParameters) {

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0) {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened\n", llParameters.serialPort);

    unsigned char buf[BUF_SIZE] = {0};
    
    readFrame(buf);

    if (buf[1] == A_SENDER && buf[2] == SET) {
        printf("Received SET, sending UA\n");
        int bufSize = createFrame(A_RECEIVER, UA, buf);
        int bytes = writeBytesSerialPort(buf, bufSize);
        printf("%d bytes written to serial port\n", bytes);
    } else {
        printf("Received unexpected frame. Closing connection.\n");
    }

    if (closeSerialPort() < 0) {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    // TODO: Implement this function

    return 0;
}

int llCloseRx()
{
    // TODO: Implement this function

    return 0;
}



////////////////////////////////////////////////
// Helper functions
////////////////////////////////////////////////
int createFrame(AddressField address, ControlField control, unsigned char* frame) {

    frame[0] = FLAG; // Start flag
    frame[1] = address; // Address field
    frame[2] = control; // Control field
    frame[3] = frame[1] ^ frame[2]; // BCC1
    frame[4] = FLAG; // End flag

    return 5;
}

int createIFrame(AddressField address, ControlField control, unsigned char* frame,
                const unsigned char* data) {

    int bufSize = createFrame(address, control, frame);
    int i = 0;

    // TODO

    frame[bufSize + i] = FLAG; // End flag

    return 5 + i;
}


int readFrame(unsigned char* buf) {

    unsigned char byte;
    unsigned int i = 0;
    State state = START;


    while (state != STOP && !alarmEnabled) {

        if (readByteSerialPort(&byte) < 1)
            continue;
        
        switch (state) {

            case START:
                i = 0;
                if (byte != FLAG) 
                    break;

                buf[i++] = byte;
                state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if (byte == FLAG) {
                    // Stay in this state
                    break;
                } else if (byte == A_SENDER || byte == A_RECEIVER) {
                    // Valid address field, continue
                    buf[i++] = byte;
                    state = A_RCV;
                } else {
                    // Invalid frame
                    state = START;
                    i = 0;
                }
                break;

            case A_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    break;
                } else if (byte == SET || byte == UA || byte == DISC) {
                    // Valid control field, continue
                    buf[i++] = byte;
                    state = C_RCV;
                } else {
                    // Invalid frame
                    state = START;
                    i = 0;
                }
                break;

            case C_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    break;
                } else if (byte == (buf[1] ^ buf[2])) {
                    // Valid BCC1, continue
                    buf[i++] = byte;
                    state = BCC1_OK;
                } else {
                    // Invalid frame
                    state = START;
                    i = 0;
                }

            case BCC1_OK:
            
                if (byte != FLAG) {
                    state = START;
                    i = 0;
                } else {
                    // Frame complete 
                    buf[i++] = byte;
                    state = STOP;
                }
                break;

            case STOP:
                // Should not reach here
                break;

        }
    }
    printf("Total bytes received: %d\n", i);
    return i;
}


void alarmHandler(int signal){
    alarmCount++;
    alarmEnabled = TRUE;
}