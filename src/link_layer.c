// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include "alarm_sigaction.h"

#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256
#define FRAME_SIZE 5

//BYTES
#define FLAG 0x7E
#define A_TX 0x03
#define A_RX 0x01
#define C_SET 0x03
#define C_UA 0x07  

//estados leitura frame

typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
} State_Machine;

//FUNÇÕES AUXILIARES

//criar frame
void buildFrame(unsigned char *buf, unsigned char adress, unsigned char control){

    buf[0] = FLAG;
    buf[1] = adress;
    buf[2] = control;
    buf[3] = adress^control;
    buf[4] = FLAG;

}

//ler frame
int readFrame(unsigned char *buf){
    unsigned char byte;
    State_Machine state = START;

    while(state != STOP){
        int read = readByteSerialPort(&byte);
        if(read < 0){
            perror("Could not read byte from serial port");
            return -1;
        }
        if (read == 0){
            continue; // No byte read, continue to next iteration
        }
        switch(state){
            case START:
                if(byte == FLAG){
                    buf[0] = byte;
                    state = FLAG_RCV;
                }
                break;
            case FLAG_RCV:
                if(byte == A_TX || byte == A_RX){
                    buf[1] = byte;
                    state = A_RCV;
                }else if(byte != FLAG){
                    state = START;
                }
                break;
            case A_RCV:
                if(byte == C_SET || byte == C_UA){
                    buf[2] = byte;
                    state = C_RCV;
                }else if(byte == FLAG){
                    state = FLAG_RCV;
                }else{
                    state = START;
                }
                break;
            case C_RCV:
                if(byte == (buf[1]^buf[2])){
                    buf[3] = byte;
                    state = BCC_OK;
                }else if(byte == FLAG){
                    state = FLAG_RCV;
                }else{
                    state = START;
                }
                break;
            case BCC_OK:
                if(byte == FLAG){
                    buf[4] = byte;
                    state = STOP;
                }else{
                    state = START;
                }
                break;
            default:
                break;
        }
    }
    return 0;
}

int alarmEnabled = FALSE;
int alarmCount = 0;

int alarmConfig(){
    struct sigaction act = {0};

    act.sa_handler = &alarmHandler;

    if (sigaction(SIGALRM, &act, NULL) == -1){
        perror("sigaction");
        exit(1);
    }

    printf("Alarm configured\n");

    return 0;
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    // ----------------------------------------------------
    // This example code shows how to open the serial port and send a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    if (alarmConfig() < 0) return -1;

    // Create string to send
    unsigned char buf[FRAME_SIZE] = {0};


    for (int i = 0; i < 4; i++){
    // Enviar SET
    buildFrame(buf, A_TX, C_SET);
    writeBytesSerialPort(buf, FRAME_SIZE);

    printf("SET sent. Attempt %d\n", i + 1);

    // Ativar alarme
    alarmEnabled = TRUE;
    alarm(3);

    // Esperar pela resposta
    if (readFrame(buf) == 0)
    {
        // Recebeu uma frame antes do timeout
        alarm(0); // cancela o alarme

        if (buf[1] == A_RX && buf[2] == C_UA)
        {
            printf("UA received. Connection established.\n");
            return 0;
        }
    }

    // Se chegou aqui, não recebeu UA corretamente
    if (!alarmEnabled)
    {
        printf("Timeout. Retrying...\n");
    }
}


    // Close serial port
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
    // ----------------------------------------------------
    // This example code shows how to open the serial port and receive a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Read from serial port until checked full frame.

    //criar o buf onde se vai escrever o frame
    unsigned char buf[FRAME_SIZE] = {0};

    //ler o frame enviado por tx

    if(readFrame(buf) == 0 && buf[1] == A_TX){
        printf("SET received.\n");

        buildFrame(buf, A_RX, C_UA);
        int bytes = writeBytesSerialPort(buf, FRAME_SIZE);
        printf("%d bytes written to serial port\n", bytes);
    }

    // Wait until all bytes have been written to the serial port
    sleep(1);

    // Close serial port
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
