// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

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
    strcpy(llParameters.serialPort, serialPort);

    if (strcmp(role, "tx") == 0)
    {
        if(llOpenTx(llParameters)==0){


        }
        else{
            printf("Error transmitting");
            return 0;
        }
    }
    else if (strcmp(role, "rx") == 0)
    {
        llOpenRx(llParameters);
        if(llReceive(llparameters)==0){
            int check=0,endloop=0;

            unsigned char packet[MAX_PAYLOAD_SIZE];
            int packetSize = llReceive(packet);
            

            if( packetSize>0){
                while(endloop<packetSize){
                    
                }

                if( packet[0]==1){
                    check=1
                    //its a start packet
                    if(packet[1]==0){
                        FILE *filereceived = fopen(packet[1], "mode");
                    }
                }
                
                else if(packet[0]==3 && check==1){
                    //its the finnal paaccket
                    //parsing logic
                }

                else if(packet[0]==2){

                }

                else{printf("ERROR ON RECEIVING"; return 0;)}
            }

        }
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}
