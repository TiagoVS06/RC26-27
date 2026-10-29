#define _POSIX_SOURCE 1 // POSIX compliant source
// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include "alarm_sigaction.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h> 

// MISC
#define BUF_SIZE 256
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

// ESTATÍSTICAS DO PROTOCOLO
typedef struct
{
    // Tx
    int iFramesSent;       // tramas I novas enviadas
    int retransmissions;   // retransmissões (I, SET ou DISC)
    int timeouts;          // nº de vezes que o temporizador expirou
    int rejReceived;       // REJ recebidos
    // Rx
    int iFramesAccepted;   // tramas I novas aceites e entregues à aplicação
    int duplicates;        // tramas I duplicadas recebidas
    int rejSent;           // REJ enviados (erro no BCC2)
    int headerErrors;      // tramas ignoradas por BCC1 errado
} Statistics;

static LinkLayer ll;                 // parâmetros guardados no llOpen (timeout, nRetransmissions)
static unsigned char txNs = 0;       // Tx: número (0/1) da próxima trama I a enviar
static unsigned char rxExpected = 0; // Rx: número (0/1) da trama I que espera receber
static Statistics stats;
static int isTx = 0;

volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;
//FUNÇÕES AUXILIARES

//criar frame
void buildFrame(unsigned char *buf, unsigned char adress, unsigned char control){

    buf[0] = FLAG;
    buf[1] = adress;
    buf[2] = control;
    buf[3] = adress^control;
    buf[4] = FLAG;

}
// construir e enviar trama S/U
static int sendSU(unsigned char address, unsigned char control)
{
    unsigned char f[FRAME_SIZE];
    buildFrame(f, address, control);
    return writeBytesSerialPort(f, FRAME_SIZE);
}
 
// desligar o alarme (usado em todas as saídas das funções que o ativam)
static void stopAlarm()
{
    alarm(0);
    alarmEnabled = FALSE;
}

// BCC2 = XOR de todos os bytes de dados (calculado SEM stuffing)
static unsigned char calcBCC2(const unsigned char *data, int n)
{
    unsigned char bcc = 0;
    for (int i = 0; i < n; i++)
        bcc ^= data[i];
    return bcc;
}

// Byte stuffing: 0x7E -> 0x7D 0x5E ; 0x7D -> 0x7D 0x5D.
// Devolve o nº de bytes escritos em out (no máximo 2*n).
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

// Destuffing: 0x7D 0x5E -> 0x7E ; 0x7D 0x5D -> 0x7D.
// Devolve o nº de bytes escritos em out, ou -1 se a sequência de escape for inválida.
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
    printf("\n---------- PROTOCOL STATISTICS ----------\n");
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
        //verificar se passaram 3 segundos
        if (useTimeout && !alarmEnabled) {
            return -1;
        }

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

// Ler frame enviada pelo Tx durante a transferência (I, SET ou DISC).
// Devolve o campo C do frama (ou -1 em erro).
// Para tramas I, raw fica com os bytes ENTRE o BCC1 e a flag final (dados + BCC2, ainda com stuffing)
// e *rawLen com o nº desses bytes. Para SET/DISC, *rawLen = 0.
static int readTxFrame(unsigned char *raw, int *rawLen)
{
    unsigned char byte, a = 0, c = 0;
    State_Machine state = START;
    int len = 0;
 
    while (1)
    {
        int r = readByteSerialPort(&byte);
        if (r < 0)
        {
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
                state = START; // BCC1 errado: ignorar a trama, sem resposta
            }
            break;
        case BCC_OK: // SET ou DISC: só falta a flag
            if (byte == FLAG)
            {
                *rawLen = 0;
                return c;
            }
            state = START;
            break;
        case DATA:
            if (byte == FLAG)
            {
                *rawLen = len;
                return c;
            }
            if (len >= MAX_FRAME_SIZE)
            {
                state = START; // trama demasiado grande: descartar
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


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
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

    // Create string to send
    unsigned char buf[FRAME_SIZE] = {0};


    for (int i = 0; i <= ll.nRetransmissions; i++){
       if (i > 0) stats.retransmissions++;
       // Enviar SET
       sendSU(A_CMD_TX, C_SET);

       printf("SET sent. Attempt %d\n", i + 1);

       // Ativar alarme
       alarmEnabled = TRUE;
       alarm(ll.timeout);

       // Esperar pela resposta
     if (readFrame(buf, 1) == 0)
     {
        if (buf[1] == A_CMD_TX && buf[2] == C_UA){

            stopAlarm();

            printf("UA received. Connection established.\n");
            return 0;
        }
        printf("Received frame is not UA.\n");
    }

    // Se chegou aqui, não recebeu UA corretamente
    if (!alarmEnabled)
    {
        stats.timeouts++;
        printf("Timeout. Retrying...\n");
    }
}
    stopAlarm();

    printf("Failed to establish connection after all attempts.\n");
    closeSerialPort();

    return -1; // Failed to establish connection after all attempts
}

int llOpenRx(LinkLayer llParameters)
{
    // ----------------------------------------------------
    // This example code shows how to open the serial port and receive a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

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

    

    //criar o buf onde se vai escrever o frame
    unsigned char buf[FRAME_SIZE] = {0};

    //ler o frame enviado por tx

    if(readFrame(buf, 0) == 0 && buf[1] == A_CMD_TX && buf[2] == C_SET){
        printf("SET received.\n");

        int bytes = sendSU(A_CMD_TX, C_UA);
        printf("%d bytes written to serial port\n", bytes);
        return 0;
    }

    closeSerialPort();
    return -1;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
        if (buf == NULL || bufSize <= 0 || bufSize > MAX_PAYLOAD_SIZE)
        return -1;
 
    // 1. Montar a trama I: F A C BCC1 | dados+BCC2 com stuffing | F
    unsigned char frame[MAX_FRAME_SIZE];
    unsigned char bcc2 = calcBCC2(buf, bufSize); 
    int len = 0;
 
    frame[len++] = FLAG;
    frame[len++] = A_CMD_TX;
    frame[len++] = C_I(txNs);
    frame[len++] = A_CMD_TX ^ C_I(txNs);
    len += stuffBytes(buf, bufSize, frame + len); // stuffing dos dados
    len += stuffBytes(&bcc2, 1, frame + len);     
    frame[len++] = FLAG;
 
    // 2. Enviar e esperar confirmação 
    for (int attempt = 0; attempt <= ll.nRetransmissions; attempt++)
    {
        if (attempt == 0) stats.iFramesSent++;
        else stats.retransmissions++;

        if (writeBytesSerialPort(frame, len) < 0){
            stopAlarm();
            return -1;
        }
        printf("I(Ns=%d) sent (%d bytes). Attempt %d\n", txNs, len, attempt + 1);
 
        alarmEnabled = TRUE;
        alarm(ll.timeout);
 
        // Esperar até: RR certo (sucesso), REJ (reenviar já) ou timeout (reenviar)
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
                // RR com o número da PRÓXIMA trama: a atual foi aceite
                stopAlarm();
                printf("RR%d received.\n", txNs ^ 1);
                txNs ^= 1;
                return len;
            }
 
            if (resp[2] == C_REJ(txNs))
            {
                // REJ: retransmitir imediatamente, sem esperar pelo timeout
                stopAlarm();
                stats.rejReceived++;
                printf("REJ%d received.\n", txNs);
                break;
            }
 
            // Outro frame,  ignorar e continuar à espera
        }
    }
 
    stopAlarm();
    printf("llSend: max retransmissions reached.\n");
    return -1;


    
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    unsigned char raw[MAX_FRAME_SIZE];
    unsigned char data[MAX_FRAME_SIZE];
    int rawLen;
 
    while (1)
    {
        int c = readTxFrame(raw, &rawLen);
        if (c < 0)
            return -1;
 
        if (c == C_SET)
        {
            // O UA do llOpen perdeu-se e o Tx reenviou o SET: responder outra vez
            printf("SET received again. Resending UA.\n");
            sendSU(A_CMD_TX, C_UA);
            continue;
        }
 
        if (c == C_DISC)
        {
            // Não devia acontecer: a aplicação só chama llReceive até receber o END
            printf("DISC received during llReceive.\n");
            return -1;
        }
 
        // frame I com header válido (o BCC1 já foi verificado na máquina de estados)
        int ns = (c == C_I1);
 
        // 1. Destuffing PRIMEIRO, depois verificar BCC2
        int n = destuffBytes(raw, rawLen, data);
        int ok = n >= 2 && (n - 1) <= MAX_PAYLOAD_SIZE &&
                 calcBCC2(data, n - 1) == data[n - 1]; // último byte = BCC2
 
        // 2. Duplicado (com ou sem erro nos dados): descartar e confirmar com RR
        if (ns != rxExpected)
        {
            stats.duplicates++;
            printf("Duplicate I(Ns=%d). Sending RR%d.\n", ns, rxExpected);
            sendSU(A_CMD_TX, C_RR(rxExpected));
            continue;
        }
 
        // 3. Trama nova com erro nos dados: pedir retransmissão
        if (!ok)
        {
            stats.rejSent++;
            printf("BCC2 error in I(Ns=%d). Sending REJ%d.\n", ns, ns);
            sendSU(A_CMD_TX, C_REJ(ns));
            continue;
        }
 
        // 4. Trama nova e correta: entregar à aplicação e confirmar
        int packetSize = n - 1;
        memcpy(packet, data, packetSize);
        rxExpected ^= 1;
        stats.iFramesAccepted++;
        sendSU(A_CMD_TX, C_RR(rxExpected));
        printf("I(Ns=%d) accepted (%d bytes). Sending RR%d.\n", ns, packetSize, rxExpected);
        return packetSize;
    }

}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    unsigned char buf[FRAME_SIZE] = {0};

    for (int i = 0; i <= ll.nRetransmissions; i++){
    
    if (i > 0) stats.retransmissions++;

    // Enviar DISC
    sendSU(A_CMD_TX, C_DISC);

    printf("DISC sent. Attempt %d\n", i + 1);

    // Ativar alarme
    alarmEnabled = TRUE;
    alarm(ll.timeout);

    // Esperar pela resposta
    if (readFrame(buf, 1) == 0)
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

    // Se chegou aqui, não recebeu DISC corretamente
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
 
    // 1. Esperar pelo DISC do Tx.
    // Se o último RR se perdeu, o Tx ainda reenvia a última trama I: é um duplicado,
    // por isso confirmamos outra vez com RR (sem isto, o Tx esgotava as tentativas).
    while (1)
    {
        int c = readTxFrame(raw, &rawLen);
        if (c < 0) {
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
 
    // 2. Enviar DISC (comando do Rx -> protegido por temporizador) e esperar pelo UA
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
                // O Tx não recebeu o nosso DISC e reenviou o dele: reenviar já
                stopAlarm();
                printf("DISC received again from TX.\n");
                break;
            }
        }
    }
 
    // O UA final nunca chegou (provavelmente perdeu-se e o Tx já fechou)
    stopAlarm();
    printf("Failed to close connection: UA not received.\n");
    printStatistics();
    closeSerialPort();
    return -1;
}