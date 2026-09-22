// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256
#define MAX_ATTEMPTS 3
#define TIMEOUT 3 // seconds



////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened\n", llParameters.serialPort);

    unsigned char buf[BUF_SIZE] = {0};
    int bufSize = createFrame(A_SENDER, SET, buf);
    int bytes = writeBytesSerialPort(buf, bufSize);
    printf("%d bytes written to serial port\n", bytes);

    sleep(1); // Wait for the receiver to process the SET frame


    unsigned int nBytesBuf = 0;
    unsigned char byte;
    volatile int insideFrame = FALSE;
    volatile int STOP = FALSE;

    while (STOP == FALSE)
    {
        bytes = readByteSerialPort(&byte);

        if (bytes < 1)
            continue;

        buf[nBytesBuf++] = byte;

        if (byte == FLAG)
        {
            if (insideFrame)
            {
                STOP = TRUE;
                insideFrame = FALSE;
            }
            else
            {
                insideFrame = TRUE;
            }
        }
    }
    if (buf[1] == A_RECEIVER && buf[2] == UA)
    {
        printf("Received UA, connection established\n");
    }

    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    unsigned char buf[BUF_SIZE] = {0};
    unsigned char byte;
    int bytes = 0;
    volatile int STOP = FALSE;
    volatile int insideFrame = FALSE;
    int nBytesBuf = 0;

    while (STOP == FALSE)
    {
        bytes = readByteSerialPort(&byte);

        if (bytes < 1)
            continue;

        buf[nBytesBuf++] = byte;
        if (byte == FLAG)
        {
            if (insideFrame)
            {
                insideFrame = FALSE;
                STOP = TRUE;
            }
            else
            {
                insideFrame = TRUE;
            }
        }
    }

    if (buf[1] == A_SENDER && buf[2] == SET)
    {
        printf("Total bytes received: %d\n", nBytesBuf);
        printf("Received SET, sending UA\n");
        int frameSize = createFrame(A_RECEIVER, UA, buf);
        bytes = writeBytesSerialPort(buf, frameSize);
        printf("%d bytes written to serial port\n", bytes);
    }
    else
    {
        printf("Received unexpected frame. Closing connection.\n");
        if (closeSerialPort() < 0)
        {
            perror("closeSerialPort");
            return -1;
        }
        return -1;
    }

    if (closeSerialPort() < 0)
    {
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




// Helper functions

int createFrame(AddressField address, ControlField control,
                unsigned char* frame)
{
    frame[0] = FLAG; // Start flag
    frame[1] = address; // Address field
    frame[2] = control; // Control field
    frame[3] = frame[1] ^ frame[2]; // BCC1
    frame[4] = FLAG; // End flag


    return 5;
}