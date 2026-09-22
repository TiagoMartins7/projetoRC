// RCOM 2026/2027
//
// Link layer header.
// DO NOT CHANGE THIS FILE

#ifndef LINK_LAYER_H
#define LINK_LAYER_H

typedef struct
{
    char serialPort[50];  // Serial port device (e.g., "/dev/ttyS0" or "/tmp/ttyS10")
    int baudRate;         // Speed of the transmission
    int nRetransmissions; // Number of retries in case of failure
    int timeout;          // Retransmission timeout value in seconds
} LinkLayer;


typedef enum
{
    SET = 0x03,
    UA = 0x07,
    DISC = 0x0B,
    RR = 0x05,
    REJ = 0x01,
    INFO_FRAME_0 = 0x00,
    INFO_FRAME_1 = 0x80
} ControlField;

typedef enum
{
    A_SENDER = 0x03,
    A_RECEIVER = 0x01
} AddressField;

// Size of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer.
#define MAX_PAYLOAD_SIZE 1000

// MISC
#define FALSE 0
#define TRUE 1
#define FLAG 0x7E
#define ALT_FLAG 0x7D 0x5E

/**
 * Open the link layer connection as a transmitter (Tx) using the parameters
 * passed as argument.
 *
 * @param llParameters The link layer parameters.
 * @return 0 on success or -1 on error.
 */
int llOpenTx(LinkLayer llParameters);

/**
 * Open the link layer connection as a receiver (Rx) using the parameters
 * passed as argument.
 *
 * @param llParameters The link layer parameters.
 * @return 0 on success or -1 on error.
 */
int llOpenRx(LinkLayer llParameters);

/**
 * Send data in buf with size bufSize.
 *
 * @param buf The buffer containing the data to send.
 * @param bufSize The size of the buffer.
 * @return number of chars written, or -1 on error.
 */
int llSend(const unsigned char *buf, int bufSize);

/**
 * Receive data in packet.
 *
 * @param packet The buffer to store the received data.
 * @return number of chars read, or -1 on error.
 */
int llReceive(unsigned char *packet);

/**
 * Close the previously opened connection as a transmitter (Tx) and
 * print the transmission statistics in the console.
 *
 * @return 0 on success or -1 on error.
 */
int llCloseTx();

/**
 * Close the previously opened connection as a receiver (Rx) and
 * print the transmission statistics in the console.
 *
 * @return 0 on success or -1 on error.
 */
int llCloseRx();


// Helper functions

int createFrame(AddressField address, ControlField control,
                unsigned char* frame);

int createIFrame(AddressField address, ControlField control,
                unsigned char* frame, const unsigned char* data);


#endif // LINK_LAYER_H
