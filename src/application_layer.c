// RCOM 2026/2027
//
// Application layer protocol implementation
 
#include "application_layer.h"
#include "link_layer.h"
 
#include <stdio.h>
#include <string.h>
 
// CAMPO C DOS PACOTES
#define C_START 1
#define C_DATA 2
#define C_END 3
 
// CAMPO T DOS PARÂMETROS (TLV) DOS PACOTES DE CONTROLO
#define T_FILE_SIZE 0
#define T_FILE_NAME 1
 
// Pacote de dados: C L2 L1 + fragmento -> fragmento máximo = payload máximo - 3
#define DATA_HEADER_SIZE 3
#define MAX_FRAGMENT_SIZE (MAX_PAYLOAD_SIZE - DATA_HEADER_SIZE)
 
#define MAX_NAME_SIZE 255 // L tem 1 byte -> V tem no máximo 255 bytes
 

// FUNÇÕES AUXILIARES - PACOTES
 
// Construir pacote de controlo (START ou END): C | T=0 L V(tamanho) | T=1 L V(nome)
// Devolve o tamanho do pacote.
static int buildControlPacket(unsigned char c, long fileSize, const char *fileName, unsigned char *packet)
{
    int len = 0;
    packet[len++] = c;
 
    // TLV do tamanho do ficheiro
    unsigned char sizeBytes[sizeof(long)];
    int nSize = 0;
    long tmp = fileSize;
    do
    {
        sizeBytes[nSize++] = tmp & 0xFF; // byte menos significativo primeiro
        tmp >>= 8;
    } while (tmp > 0);
 
    packet[len++] = T_FILE_SIZE;
    packet[len++] = nSize;
    for (int i = nSize - 1; i >= 0; i--) // escrever do mais significativo para o menos
        packet[len++] = sizeBytes[i];
 
    // TLV do nome do ficheiro 
    int nameLen = strlen(fileName);
    if (nameLen > MAX_NAME_SIZE)
        nameLen = MAX_NAME_SIZE;
 
    packet[len++] = T_FILE_NAME;
    packet[len++] = nameLen;
    memcpy(packet + len, fileName, nameLen);
    len += nameLen;
 
    return len;
}
 
// Ler pacote de controlo: percorre os TLV e extrai tamanho e nome.
// Devolve 0 se o pacote estiver bem formado e tiver o tamanho, -1 caso contrário.
static int parseControlPacket(const unsigned char *packet, int size, long *fileSize, char *fileName)
{
    int hasSize = 0;
    int i = 1; // saltar o campo C
    fileName[0] = '\0';
 
    while (i < size)
    {
        if (i + 2 > size)
            return -1; // faltam T e L
 
        unsigned char t = packet[i++];
        unsigned char l = packet[i++];
 
        if (i + l > size)
            return -1; // V maior do que o que resta do pacote
 
        if (t == T_FILE_SIZE)
        {
            if (l == 0 || l > sizeof(long))
                return -1;
            *fileSize = 0;
            for (int k = 0; k < l; k++)
                *fileSize = (*fileSize << 8) | packet[i + k]; 
            hasSize = 1;
        }
        else if (t == T_FILE_NAME)
        {
            memcpy(fileName, packet + i, l);
            fileName[l] = '\0';
        }
        // outros T: ignorar
 
        i += l;
    }
 
    return hasSize ? 0 : -1;
}
 
// Construir pacote de dados: C=2 | L2 | L1 | P1..PK. Devolve o tamanho do pacote.
static int buildDataPacket(const unsigned char *data, int dataSize, unsigned char *packet)
{
    packet[0] = C_DATA;
    packet[1] = (dataSize >> 8) & 0xFF; // L2 (mais significativo)
    packet[2] = dataSize & 0xFF;        // L1 (menos significativo)
    memcpy(packet + DATA_HEADER_SIZE, data, dataSize);
    return dataSize + DATA_HEADER_SIZE;
}
 
// TRANSMISSOR

// Envia START, os pacotes de dados e END. Devolve 0 em sucesso, -1 em erro.
static int sendFile(const char *filename)
{
    FILE *file = fopen(filename, "rb");
    if (file == NULL)
    {
        perror("fopen");
        return -1;
    }
 
    // Tamanho do ficheiro
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);
    printf("File %s opened (%ld bytes)\n", filename, fileSize);
 
    unsigned char packet[MAX_PAYLOAD_SIZE];
 
    // START
    int len = buildControlPacket(C_START, fileSize, filename, packet);
    if (llSend(packet, len) < 0)
    {
        printf("Failed to send START packet.\n");
        fclose(file);
        return -1;
    }
    printf("START packet sent.\n");
 
    // Pacotes de dados até ao fim do ficheiro
    unsigned char fragment[MAX_FRAGMENT_SIZE];
    long totalSent = 0;
    int nPackets = 0;
    int n;
 
    while ((n = fread(fragment, 1, MAX_FRAGMENT_SIZE, file)) > 0)
    {
        len = buildDataPacket(fragment, n, packet);
        if (llSend(packet, len) < 0)
        {
            printf("Failed to send data packet %d.\n", nPackets);
            fclose(file);
            return -1;
        }
        totalSent += n;
        nPackets++;
        printf("Data packet %d sent (%d bytes, %ld/%ld)\n", nPackets, n, totalSent, fileSize);
    }
 
    if (ferror(file))
    {
        perror("fread");
        fclose(file);
        return -1;
    }
    fclose(file);
 
    // END (repete a informação do START)
    len = buildControlPacket(C_END, fileSize, filename, packet);
    if (llSend(packet, len) < 0)
    {
        printf("Failed to send END packet.\n");
        return -1;
    }
    printf("END packet sent. File transfer complete.\n");
 
    return 0;
}
 
// RECETOR

// Recebe START, os pacotes de dados e END, e escreve o ficheiro em filename.
// 0 sucesso, -1 erro.
static int receiveFile(const char *filename)
{
    unsigned char packet[MAX_PAYLOAD_SIZE];
    long fileSize = 0;
    char fileName[MAX_NAME_SIZE + 1];
    int size;
 
    // Esperar o START
    while (1)
    {
        size = llReceive(packet);
        if (size < 0)
        {
            printf("Failed to receive START packet.\n");
            return -1;
        }
        if (size > 0 && packet[0] == C_START)
            break;
        printf("Unexpected packet before START. Ignoring.\n");
    }
 
    if (parseControlPacket(packet, size, &fileSize, fileName) < 0)
    {
        printf("Invalid START packet.\n");
        return -1;
    }
    printf("START received: file \"%s\", %ld bytes\n", fileName, fileSize);
 
    // Escrevemos no nome dado na linha de comandos, para não
    // apagar o original quando Tx e Rx estão no mesmo PC.
    FILE *file = fopen(filename, "wb");
    if (file == NULL)
    {
        perror("fopen");
        return -1;
    }
 
    //Pacotes de dados até ao END
    long totalReceived = 0;
    int nPackets = 0;
 
    while (1)
    {
        size = llReceive(packet);
        if (size < 0)
        {
            printf("Failed to receive packet.\n");
            fclose(file);
            return -1;
        }
        if (size == 0)
            continue;
 
        if (packet[0] == C_DATA)
        {
            if (size < DATA_HEADER_SIZE)
            {
                printf("Data packet too short. Ignoring.\n");
                continue;
            }
 
            int k = 256 * packet[1] + packet[2]; // K = 256 * L2 + L1
            if (k != size - DATA_HEADER_SIZE)
            {
                // verificação adicional de integridade
                printf("Data packet size mismatch (K=%d, received %d). Ignoring.\n", k, size - DATA_HEADER_SIZE);
                continue;
            }
 
            if (fwrite(packet + DATA_HEADER_SIZE, 1, k, file) != (size_t)k)
            {
                perror("fwrite");
                fclose(file);
                return -1;
            }
            totalReceived += k;
            nPackets++;
            printf("Data packet %d received (%d bytes, %ld/%ld)\n", nPackets, k, totalReceived, fileSize);
        }
        else if (packet[0] == C_END)
        {
            break;
        }
        else
        {
            printf("Unexpected packet (C=%d). Ignoring.\n", packet[0]);
        }
    }
 
    fclose(file);
 
    // Verificar o END e a integridade do ficheiro
    long endSize = 0;
    char endName[MAX_NAME_SIZE + 1];
 
    if (parseControlPacket(packet, size, &endSize, endName) < 0)
    {
        printf("Invalid END packet.\n");
        return -1;
    }
    printf("END received.\n");
 
    if (endSize != fileSize || strcmp(endName, fileName) != 0)
    {
        printf("END does not match START.\n");
        return -1;
    }
 
    if (totalReceived != fileSize)
    {
        printf("File size mismatch: expected %ld bytes, received %ld.\n", fileSize, totalReceived);
        return -1;
    }
 
    printf("File received successfully (%ld bytes) and saved as %s\n", totalReceived, filename);
    return 0;
}
 

// APPLICATION LAYER

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    // snprintf em vez de strcpy
    snprintf(llParameters.serialPort, sizeof(llParameters.serialPort), "%s", serialPort);
 
    if (strcmp(role, "tx") == 0)
    {
        if (llOpenTx(llParameters) < 0)
        {
            printf("Failed to open connection (Tx).\n");
            return;
        }
 
        if (sendFile(filename) < 0)
            printf("File transfer failed.\n");
 
        // fechar sempre a ligação
        if (llCloseTx() < 0)
            printf("Failed to close connection (Tx).\n");
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) < 0)
        {
            printf("Failed to open connection (Rx).\n");
            return;
        }
 
        if (receiveFile(filename) < 0)
        {
            printf("File reception failed.\n");
            remove(filename); // não deixar um ficheiro incompleto
        }
 
        if (llCloseRx() < 0)
            printf("Failed to close connection (Rx).\n");
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
    }
}
 