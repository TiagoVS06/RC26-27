#define _POSIX_C_SOURCE 199309L // para o clock_gettime
// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include "alarm_sigaction.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

// MISC
#define FRAME_SIZE 5

#define MAX_FRAME_SIZE (2 * (MAX_PAYLOAD_SIZE + 1) + 5)


//BYTES
#define FLAG 0x7E
#define ESC 0x7D
#define ESC_XOR 0x20 // 0x7E -> 7D 5E ; 0x7D -> 7D 5D

#define C_SET 0x03
#define C_UA 0x07  
#define C_DISC 0x0B
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_I0 0x00
#define C_I1 0x80
#define A_CMD_TX 0x03   // comandos do Tx / respostas do Rx
#define A_CMD_RX 0x01   // comandos do Rx / respostas do Tx

#define C_RR(n) ((n) ? C_RR1 : C_RR0)
#define C_REJ(n) ((n) ? C_REJ1 : C_REJ0)
#define C_I(n) ((n) ? C_I1 : C_I0)


//estados leitura frame

typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    DATA,
    STOP
} State_Machine;

// estatísticas
typedef struct
{
    // Tx
    int iFramesSent;       // tramas I novas
    int retransmissions;   // retransmissões (I, SET, DISC)
    int timeouts;
    int rejReceived;
    // Rx
    int iFramesAccepted;   // tramas I aceites
    int duplicates;
    int rejSent;
    int headerErrors;      // BCC1 errado
    // Tx e Rx
    long bytes;            // bytes de dados (sem stuffing)
} Statistics;

static LinkLayer ll;                 // parâmetros do llOpen
static unsigned char txNs = 0;       // Tx: Ns da próxima trama I
static unsigned char rxExpected = 0; // Rx: Ns que espera receber
static Statistics stats;
static int isTx = 0;
static struct timespec tStart, tEnd; // tempo da transferência

volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;
//FUNÇÕES AUXILIARES

//criar frame
void buildFrame(unsigned char *buf, unsigned char address, unsigned char control){

    buf[0] = FLAG;
    buf[1] = address;
    buf[2] = control;
    buf[3] = address^control;
    buf[4] = FLAG;

}
// enviar trama S/U
static int sendSU(unsigned char address, unsigned char control)
{
    unsigned char f[FRAME_SIZE];
    buildFrame(f, address, control);
    return writeBytesSerialPort(f, FRAME_SIZE);
}
 
// desligar o alarme
static void stopAlarm()
{
    alarm(0);
    alarmEnabled = FALSE;
}

// timeout do Rx caso o Tx deixe de enviar (2x o tempo das tentativas do Tx)
static int rxInactivityTimeout()
{
    int maxTxTime = (MAX_FRAME_SIZE * 10) / ll.baudRate + 1;
    return 2 * (ll.nRetransmissions + 1) * (ll.timeout + maxTxTime);
}

// BCC2 = XOR dos dados (antes do stuffing)
static unsigned char calcBCC2(const unsigned char *data, int n)
{
    unsigned char bcc = 0;
    for (int i = 0; i < n; i++)
        bcc ^= data[i];
    return bcc;
}

// stuffing: 0x7E -> 7D 5E, 0x7D -> 7D 5D
static int stuffBytes(const unsigned char *in, int n, unsigned char *out)
{
    int j = 0;
    for (int i = 0; i < n; i++)
    {
        if (in[i] == FLAG || in[i] == ESC)
        {
            out[j++] = ESC;
            out[j++] = in[i] ^ ESC_XOR;
        }
        else
        {
            out[j++] = in[i];
        }
    }
    return j;
}

// destuffing, devolve -1 se o escape for inválido
static int destuffBytes(const unsigned char *in, int n, unsigned char *out)
{
    int j = 0;
    for (int i = 0; i < n; i++)
    {
        if (in[i] == ESC)
        {
            if (i + 1 >= n)
                return -1; // ESC no fim
            unsigned char next = in[++i];
            if (next != (FLAG ^ ESC_XOR) && next != (ESC ^ ESC_XOR))
                return -1; // escape inválido
            out[j++] = next ^ ESC_XOR;
        }
        else
        {
            out[j++] = in[i];
        }
    }
    return j;
}

static void printStatistics()
{
    double time = (tEnd.tv_sec - tStart.tv_sec) + (tEnd.tv_nsec - tStart.tv_nsec) / 1e9;
    double R = time > 0 ? stats.bytes * 8 / time : 0; // bit/s

    printf("\n---------- PROTOCOL STATISTICS ----------\n");
    printf("Transfer time:              %.3f s\n", time);
    printf("Data bytes:                 %ld\n", stats.bytes);
    printf("R (bit/s):                  %.1f\n", R);
    printf("S = R/C:                    %.4f\n", R / ll.baudRate);
    if (isTx)
    {
        printf("I frames sent (new):        %d\n", stats.iFramesSent);
        printf("Retransmissions:            %d\n", stats.retransmissions);
        printf("Timeouts:                   %d\n", stats.timeouts);
        printf("REJ received:               %d\n", stats.rejReceived);
    }
    else
    {
        printf("I frames accepted:          %d\n", stats.iFramesAccepted);
        printf("Duplicate I frames:         %d\n", stats.duplicates);
        printf("REJ sent:                   %d\n", stats.rejSent);
        printf("Frames with header error:   %d\n", stats.headerErrors);
        printf("DISC retransmissions:       %d\n", stats.retransmissions);
        printf("Timeouts:                   %d\n", stats.timeouts);
    }
    printf("-----------------------------------------\n\n");
}

//ler frame S/U
int readFrame(unsigned char *buf, int useTimeout){
    unsigned char byte;
    State_Machine state = START;

    while(state != STOP){
        //timeout
        if (useTimeout && !alarmEnabled) {
            return -1;
        }

        int read = readByteSerialPort(&byte);
        if(read < 0){
            if (errno != EINTR) // EINTR = foi o alarme, não é erro
                perror("Could not read byte from serial port");
            return -1;
        }
        if (read == 0){
            continue;
        }
        switch(state){
            case START:
                if(byte == FLAG){
                    buf[0] = byte;
                    state = FLAG_RCV;
                }
                break;
            case FLAG_RCV:
                if(byte == A_CMD_TX || byte == A_CMD_RX){
                    buf[1] = byte;
                    state = A_RCV;
                }else if(byte != FLAG){
                    state = START;
                }
                break;
            case A_RCV:
                if(byte == C_SET || byte == C_UA || byte == C_DISC || byte == C_REJ0 || byte == C_REJ1 || byte == C_RR0 || byte == C_RR1){
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

// ler trama do Tx (I, SET ou DISC), devolve o C ou -1
// nas tramas I o raw fica com dados + BCC2 ainda com stuffing
static int readTxFrame(unsigned char *raw, int *rawLen)
{
    static int lastWasFlag = 0; // a flag final da anterior pode ser a inicial desta
    unsigned char byte, a = 0, c = 0;
    State_Machine state = lastWasFlag ? FLAG_RCV : START;
    int len = 0;
    lastWasFlag = 0;

    while (1)
    {
        int r = readByteSerialPort(&byte);
        if (r < 0)
        {
            if (errno != EINTR)
                perror("Could not read byte from serial port");
            return -1;
        }
        if (r == 0)
            continue;
 
        switch (state)
        {
        case START:
            if (byte == FLAG)
                state = FLAG_RCV;
            break;
        case FLAG_RCV:
            if (byte == A_CMD_TX)
            {
                a = byte;
                state = A_RCV;
            }
            else if (byte != FLAG)
                state = START;
            break;
        case A_RCV:
            if (byte == C_I0 || byte == C_I1 || byte == C_SET || byte == C_DISC)
            {
                c = byte;
                state = C_RCV;
            }
            else if (byte == FLAG)
                state = FLAG_RCV;
            else
                state = START;
            
            break;
        case C_RCV:
            if (byte == (a ^ c))
            {
                len = 0;
                state = (c == C_I0 || c == C_I1) ? DATA : BCC_OK;
            }
            else if (byte == FLAG)
                state = FLAG_RCV;
            else
            {
                stats.headerErrors++;
                state = START; // BCC1 errado, ignora
            }
            break;
        case BCC_OK: // SET ou DISC: só falta a flag
            if (byte == FLAG)
            {
                *rawLen = 0;
                lastWasFlag = 1;
                return c;
            }
            state = START;
            break;
        case DATA:
            if (byte == FLAG)
            {
                *rawLen = len;
                lastWasFlag = 1;
                return c;
            }
            if (len >= MAX_FRAME_SIZE)
            {
                state = START; // demasiado grande
                break;
            }
            raw[len++] = byte;
            break;
        default:
            break;
        }
    }
}



int alarmConfig(){
    struct sigaction act = {0};

    act.sa_handler = &alarmHandler;

    if (sigaction(SIGALRM, &act, NULL) == -1){
        perror("sigaction");
        return -1;
    }

    printf("Alarm configured\n");

    return 0;
}


// LLOPEN
int llOpenTx(LinkLayer llParameters)
{


        ll = llParameters;
        isTx = 1;
        txNs = 0;
        memset(&stats, 0, sizeof(stats));


    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    if (alarmConfig() < 0) {
        closeSerialPort();
        return -1;
    }

    unsigned char buf[FRAME_SIZE] = {0};


    for (int i = 0; i <= ll.nRetransmissions; i++){
       if (i > 0) stats.retransmissions++;
       // Enviar SET
       sendSU(A_CMD_TX, C_SET);

       printf("SET sent. Attempt %d\n", i + 1);

       // Ativar alarme
       alarmEnabled = TRUE;
       alarm(ll.timeout);

       // Esperar pelo UA
     while (readFrame(buf, 1) == 0)
     {
        if (buf[1] == A_CMD_TX && buf[2] == C_UA){

            stopAlarm();

            printf("UA received. Connection established.\n");
            clock_gettime(CLOCK_MONOTONIC, &tStart);
            return 0;
        }
        printf("Received frame is not UA.\n");
    }

    // não recebeu UA
    if (!alarmEnabled)
    {
        stats.timeouts++;
        printf("Timeout. Retrying...\n");
    }
}
    stopAlarm();

    printf("Failed to establish connection after all attempts.\n");
    closeSerialPort();

    return -1;
}

int llOpenRx(LinkLayer llParameters)
{
    ll = llParameters;
    isTx = 0;
    rxExpected = 0;
    memset(&stats, 0, sizeof(stats));


    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    if (alarmConfig() < 0) {
        closeSerialPort();
        return -1;
    }

    

    unsigned char buf[FRAME_SIZE] = {0};

    //esperar pelo SET

    while(readFrame(buf, 0) == 0){
        if(buf[1] == A_CMD_TX && buf[2] == C_SET){
            printf("SET received.\n");

            int bytes = sendSU(A_CMD_TX, C_UA);
            printf("%d bytes written to serial port\n", bytes);
            clock_gettime(CLOCK_MONOTONIC, &tStart);
            return 0;
        }
    }

    closeSerialPort();
    return -1;
}

// LLSEND
int llSend(const unsigned char *buf, int bufSize)
{
        if (buf == NULL || bufSize <= 0 || bufSize > MAX_PAYLOAD_SIZE)
        return -1;
 
    // trama I: F A C BCC1 dados BCC2 F (dados e BCC2 com stuffing)
    unsigned char frame[MAX_FRAME_SIZE];
    unsigned char bcc2 = calcBCC2(buf, bufSize); 
    int len = 0;
 
    frame[len++] = FLAG;
    frame[len++] = A_CMD_TX;
    frame[len++] = C_I(txNs);
    frame[len++] = A_CMD_TX ^ C_I(txNs);
    len += stuffBytes(buf, bufSize, frame + len);
    len += stuffBytes(&bcc2, 1, frame + len);     
    frame[len++] = FLAG;
 
    // tempo da trama a sair (10 bits por byte), soma-se ao timeout
    int txTime = (len * 10) / ll.baudRate + 1;
    int nSent = 0;

    // enviar e esperar pelo RR
    for (int attempt = 0; attempt <= ll.nRetransmissions; attempt++)
    {
        if (nSent == 0) stats.iFramesSent++;
        else stats.retransmissions++;
        nSent++;

        if (writeBytesSerialPort(frame, len) < 0){
            stopAlarm();
            return -1;
        }
        printf("I(Ns=%d) sent (%d bytes). Attempt %d\n", txNs, len, attempt + 1);
 
        alarmEnabled = TRUE;
        alarm(ll.timeout + txTime);
 
        while (1)
        {
            unsigned char resp[FRAME_SIZE];
 
            if (readFrame(resp, 1) < 0)
            {
                stats.timeouts++;
                printf("Timeout waiting for RR.\n");
                break; // reenviar
            }
 
            if (resp[1] != A_CMD_TX)
                continue; // não é resposta do Rx
 
            if (resp[2] == C_RR(txNs ^ 1))
            {
                // RR(Ns+1): trama aceite
                stopAlarm();
                printf("RR%d received.\n", txNs ^ 1);
                txNs ^= 1;
                stats.bytes += bufSize;
                return bufSize;
            }
 
            if (resp[2] == C_REJ(txNs))
            {
                // REJ: reenvia logo
                stopAlarm();
                stats.rejReceived++;
                printf("REJ%d received.\n", txNs);
                break; // REJ conta como tentativa (reenvia na próxima iteração)
            }
 
        }
    }
 
    stopAlarm();
    printf("llSend: max retransmissions reached.\n");
    return -1;


    
}

// LLRECEIVE
int llReceive(unsigned char *packet)
{
    unsigned char raw[MAX_FRAME_SIZE];
    unsigned char data[MAX_FRAME_SIZE];
    int rawLen;
 
    while (1)
    {
        // para não ficar preso se o Tx desistir
        alarmEnabled = TRUE;
        alarm(rxInactivityTimeout());
        int c = readTxFrame(raw, &rawLen);
        stopAlarm();
        if (c < 0)
        {
            printf("llReceive: no frame received from Tx (inactivity timeout).\n");
            return -1;
        }
 
        if (c == C_SET)
        {
            // o UA perdeu-se, reenviar
            printf("SET received again. Resending UA.\n");
            sendSU(A_CMD_TX, C_UA);
            continue;
        }
 
        if (c == C_DISC)
        {
            printf("DISC received during llReceive.\n");
            return -1;
        }
 
        // trama I (BCC1 já verificado)
        int ns = (c == C_I1);
 
        // destuffing e verificar BCC2
        int n = destuffBytes(raw, rawLen, data);
        int ok = n >= 2 && (n - 1) <= MAX_PAYLOAD_SIZE &&
                 calcBCC2(data, n - 1) == data[n - 1]; // último byte = BCC2
 
        // duplicado: descartar e mandar RR
        if (ns != rxExpected)
        {
            stats.duplicates++;
            printf("Duplicate I(Ns=%d). Sending RR%d.\n", ns, rxExpected);
            sendSU(A_CMD_TX, C_RR(rxExpected));
            continue;
        }
 
        // erro nos dados: REJ
        if (!ok)
        {
            stats.rejSent++;
            printf("BCC2 error in I(Ns=%d). Sending REJ%d.\n", ns, ns);
            sendSU(A_CMD_TX, C_REJ(ns));
            continue;
        }
 
        // trama nova e certa
        int packetSize = n - 1;
        memcpy(packet, data, packetSize);
        rxExpected ^= 1;
        stats.iFramesAccepted++;
        stats.bytes += packetSize;
        sendSU(A_CMD_TX, C_RR(rxExpected));
        printf("I(Ns=%d) accepted (%d bytes). Sending RR%d.\n", ns, packetSize, rxExpected);
        return packetSize;
    }

}

// LLCLOSE
int llCloseTx()
{
    unsigned char buf[FRAME_SIZE] = {0};

    clock_gettime(CLOCK_MONOTONIC, &tEnd);

    for (int i = 0; i <= ll.nRetransmissions; i++){
    
    if (i > 0) stats.retransmissions++;

    // Enviar DISC
    sendSU(A_CMD_TX, C_DISC);

    printf("DISC sent. Attempt %d\n", i + 1);

    // Ativar alarme
    alarmEnabled = TRUE;
    alarm(ll.timeout);

    // Esperar pelo DISC do Rx
    while (readFrame(buf, 1) == 0)
    {
        if (buf[1] == A_CMD_RX && buf[2] == C_DISC){

            stopAlarm();

            printf("DISC received from RX.\n");

            sendSU(A_CMD_RX, C_UA);
            
            printf("UA sent.\n");

            sleep(1);

            printStatistics();
            
            if (closeSerialPort() < 0) {
        
                perror("closeSerialPort");

                return -1;
            }

            return 0;
        }
        printf("Received frame is not DISC.\n");
    }

    // não recebeu DISC
    if (!alarmEnabled)
    {
        stats.timeouts++;
        printf("Timeout. Retrying...\n");
    }
}
    stopAlarm();
    printf("Failed to disconnect after all attempts.\n");
    printStatistics();
    closeSerialPort();
    return -1;
}

int llCloseRx()
{
    unsigned char raw[MAX_FRAME_SIZE];
    unsigned char buf[FRAME_SIZE] = {0};
    int rawLen;
 
    // esperar pelo DISC (se o último RR se perdeu ainda pode vir a trama I repetida)
    clock_gettime(CLOCK_MONOTONIC, &tEnd);

    while (1)
    {
        // para não ficar preso se o Tx desistir
        alarmEnabled = TRUE;
        alarm(rxInactivityTimeout());
        int c = readTxFrame(raw, &rawLen);
        stopAlarm();
        if (c < 0) {
            printf("llCloseRx: DISC not received (inactivity timeout).\n");
            printStatistics();
            closeSerialPort();
            return -1;
        }
 
        if (c == C_DISC)
            break;
 
        if (c == C_I0 || c == C_I1)
        {
            stats.duplicates++;
            printf("Duplicate I frame while closing. Sending RR%d.\n", rxExpected);
            sendSU(A_CMD_TX, C_RR(rxExpected));
        }
        // SET: ignorar
    }
    printf("DISC received from TX.\n");
 
    // enviar DISC e esperar pelo UA
    for (int i = 0; i <= ll.nRetransmissions; i++)
    {
        if (i > 0) stats.retransmissions++;
 
        sendSU(A_CMD_RX, C_DISC);
        printf("DISC sent. Attempt %d\n", i + 1);
 
        alarmEnabled = TRUE;
        alarm(ll.timeout);
 
        while (1)
        {
            if (readFrame(buf, 1) < 0)
            {
                stats.timeouts++;
                printf("Timeout waiting for UA.\n");
                break; // reenviar DISC
            }
 
            if (buf[1] == A_CMD_RX && buf[2] == C_UA)
            {
                stopAlarm();
                printf("UA received from TX.\n");
 
                printStatistics();
 
                if (closeSerialPort() < 0) {
                    perror("closeSerialPort");
                    return -1;
                }
                return 0;
            }
 
            if (buf[1] == A_CMD_TX && buf[2] == C_DISC)
            {
                // o Tx não recebeu o nosso DISC
                stopAlarm();
                printf("DISC received again from TX.\n");
                break;
            }
        }
    }
 
    // o UA não chegou
    stopAlarm();
    printf("Failed to close connection: UA not received.\n");
    printStatistics();
    closeSerialPort();
    return -1;
}