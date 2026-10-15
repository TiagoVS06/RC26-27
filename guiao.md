# Guião — RCOM Lab 1 (Protocolo de Ligação de Dados)

Guião de trabalho para acabar a camada de ligação (`link_layer.c`).
Os valores marcados com **⚠ confirmar** devem ser verificados no guia do lab antes de os usarem.

---

## 0. Estado atual

| Função | Estado |
|---|---|
| `buildFrame` | ✅ Feito |
| `readFrame` (tramas S/U, com `useTimeout`) | ✅ Feito (falta aceitar RR/REJ) |
| `llOpenTx` / `llOpenRx` | ✅ Feito |
| `llCloseTx` / `llCloseRx` | ✅ Feito |
| `llSend` | ⬜ Por fazer |
| `llReceive` | ⬜ Por fazer |
| Correções para a entrega | ⬜ Ver secção 8 |

---

## 1. Conceitos que vamos usar

### 1.1 Campo A (tabela do guia, pág. 11)

| Valor | Significado |
|---|---|
| `0x03` (`A_CMD_TX`) | Comandos enviados pelo Tx **ou** respostas enviadas pelo Rx |
| `0x01` (`A_CMD_RX`) | Comandos enviados pelo Rx **ou** respostas enviadas pelo Tx |

Aplicado às tramas novas:

| Trama | Quem envia | Tipo | A |
|---|---|---|---|
| I (dados) | Tx | comando | `0x03` |
| RR / REJ | Rx | resposta | `0x03` |

### 1.2 Trama de informação (I)

```
F | A | C | BCC1 | D1 ... Dn | BCC2 | F
```

- **C** = `0x00` (trama 0) ou `0x80` (trama 1). É o N(s), um contador módulo 2.
- **BCC1** = A XOR C
- **BCC2** = D1 XOR D2 XOR ... XOR Dn

### 1.3 Tramas de supervisão novas

| Trama | C | Significado |
|---|---|---|
| RR0 | `0xAA` ⚠ confirmar | "Recebi bem, manda a trama 0" |
| RR1 | `0xAB` ⚠ confirmar | "Recebi bem, manda a trama 1" |
| REJ0 | `0x54` ⚠ confirmar | "A trama 0 chegou com erro, reenvia" |
| REJ1 | `0x55` ⚠ confirmar | "A trama 1 chegou com erro, reenvia" |

> O número no RR é a **próxima** trama que o Rx espera.
> Depois de enviar a trama 0, a confirmação é **RR1**.

### 1.4 Stop-and-Wait

O Tx envia uma trama e só passa à seguinte quando recebe o RR certo.
Se houver timeout ou REJ, retransmite a mesma trama.

### 1.5 Byte stuffing ⚠ confirmar valores

Aplica-se aos **dados e ao BCC2**, nunca às FLAGs nem ao cabeçalho.

| Byte original | Enviado como |
|---|---|
| `0x7E` (FLAG) | `0x7D 0x5E` |
| `0x7D` (ESC) | `0x7D 0x5D` |

Regra: o byte é substituído por `ESC` seguido de `byte XOR 0x20`.
No destuffing, quando aparece `0x7D`, o byte seguinte é `byte XOR 0x20`.

**Ordem certa:**
- **Tx:** calcular o BCC2 sobre os dados originais → fazer stuffing a dados + BCC2.
- **Rx:** fazer destuffing → separar o BCC2 (último byte) → verificar o BCC2 sobre os dados.

---

## 2. Passo 1 — Defines e variáveis globais

### 2.1 Defines a acrescentar

```c
#define ESC      0x7D
#define C_I0     0x00
#define C_I1     0x80
#define C_RR0    0xAA   // ⚠ confirmar
#define C_RR1    0xAB   // ⚠ confirmar
#define C_REJ0   0x54   // ⚠ confirmar
#define C_REJ1   0x55   // ⚠ confirmar
```

### 2.2 Variáveis globais

```c
static int txSequence = 0;       // N(s) do Tx: número da próxima trama a enviar (0 ou 1)
static int rxExpected = 0;       // número da trama que o Rx espera receber (0 ou 1)
static LinkLayer connectionParams; // guardado no llopen (ver secção 8)
```

### 2.3 Tamanho do buffer da trama

No pior caso, o stuffing duplica cada byte de dados e o BCC2:

```
tamanho máximo = 4 (F, A, C, BCC1) + 2 × (MAX_PAYLOAD_SIZE + 1) + 1 (F)
```

Ver o valor de `MAX_PAYLOAD_SIZE` no `link_layer.h`.

---

## 3. Passo 2 — Funções auxiliares

Fazer e testar estas funções **antes** do `llSend`.

### 3.1 `calcBCC2`

```c
unsigned char calcBCC2(const unsigned char *data, int size);
```
- Começa com `bcc = 0` e faz XOR byte a byte.

### 3.2 `stuffing`

```c
int stuffing(const unsigned char *in, int inSize, unsigned char *out);
```
- Percorre `in`. Se o byte for `FLAG` ou `ESC`, escreve `ESC` e depois `byte ^ 0x20`. Caso contrário, copia o byte.
- Devolve o número de bytes escritos em `out`.

### 3.3 `destuffing`

```c
int destuffing(const unsigned char *in, int inSize, unsigned char *out);
```
- Se encontrar `ESC`, o byte seguinte passa a `byte ^ 0x20` (e salta os dois).
- Devolve o tamanho final.

### 3.4 Teste rápido (num `main` à parte ou temporário)

```
entrada:  41 7E 42 7D 43
stuffing: 41 7D 5E 42 7D 5D 43
destuffing do resultado → tem de dar 41 7E 42 7D 43
```

Confirmar também que `destuffing(stuffing(x)) == x` para alguns arrays aleatórios.

---

## 4. Passo 3 — `readFrame` aceitar RR/REJ

No estado `A_RCV`, acrescentar os novos C:

```c
if (byte == C_SET || byte == C_UA || byte == C_DISC ||
    byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1)
```

O `readFrame` continua a servir só para tramas de 5 bytes (supervisão).
As tramas I precisam de uma função própria (Passo 5).

---

## 5. Passo 4 — `llSend` (lado Tx)

### 5.1 Algoritmo

1. **Montar a trama:**
   - `frame[0] = FLAG`
   - `frame[1] = A_CMD_TX`
   - `frame[2] = txSequence == 0 ? C_I0 : C_I1`
   - `frame[3] = frame[1] ^ frame[2]`
   - calcular `bcc2 = calcBCC2(buf, bufSize)`
   - juntar dados + bcc2 num buffer temporário e fazer `stuffing` para `frame + 4`
   - última posição = `FLAG`
2. **Loop de tentativas** (igual ao `llOpenTx`):
   - `writeBytesSerialPort(frame, frameSize)`
   - `alarmEnabled = TRUE; alarm(timeout);`
   - `readFrame(resp, 1)`
3. **Analisar a resposta:**

| Resposta | Ação |
|---|---|
| RR com o número **seguinte** (`txSequence ^ 1`) | `alarm(0)`, `txSequence ^= 1`, devolver `bufSize` |
| RR com o mesmo número | Ignorar (resposta antiga) e continuar à espera / retransmitir |
| REJ com o número atual | Retransmitir já (não conta como timeout, mas conta como tentativa) |
| Timeout (`readFrame` devolve -1) | Retransmitir |

4. **Tentativas esgotadas:** devolver `-1`.

### 5.2 Checklist do `llSend`

- [ ] A = `0x03` (comando do Tx)
- [ ] C alterna entre `0x00` e `0x80` só depois de RR certo
- [ ] BCC2 calculado **antes** do stuffing
- [ ] BCC2 também leva stuffing
- [ ] FLAGs não levam stuffing
- [ ] Usa o timeout e o número de retransmissões do `LinkLayer`
- [ ] Devolve o número de bytes de dados enviados (não o tamanho da trama)

---

## 6. Passo 5 — `llReceive` (lado Rx)

### 6.1 Máquina de estados para tramas I

Igual à de supervisão até ao BCC1. Depois:

```
START → FLAG_RCV → A_RCV → C_RCV → BCC1_OK → DATA → (FLAG) → STOP
```

- Em `A_RCV`, aceitar `C_I0`, `C_I1` e também `C_SET` / `C_DISC` (ver 6.3).
- Em `DATA`, guardar todos os bytes até aparecer uma `FLAG`.
- Se o BCC1 estiver errado → voltar a `START` (o cabeçalho não é fiável, ignora-se).
- Proteger contra overflow: se o número de bytes passar o tamanho máximo, descartar a trama.

### 6.2 Depois de ler a trama

1. `destuffing` dos bytes de `DATA`.
2. O último byte é o BCC2. O resto são os dados.
3. Comparar `calcBCC2(dados)` com o BCC2 recebido.

### 6.3 Casos a tratar

`N` = número da trama recebida (0 se C = `0x00`, 1 se C = `0x80`).

| O que chega | Ação | Devolve |
|---|---|---|
| Trama I, `N == rxExpected`, BCC2 certo | Copiar dados para `packet`, enviar RR(`N^1`), `rxExpected ^= 1` | nº de bytes |
| Trama I, `N == rxExpected`, BCC2 errado | Enviar REJ(`N`) | continua a ler |
| Trama I, `N != rxExpected` (repetida) | Enviar RR(`rxExpected`), descartar dados | continua a ler |
| BCC1 errado | Ignorar | continua a ler |
| SET (o UA do llopen perdeu-se) | Reenviar UA | continua a ler |
| DISC | O Tx quer fechar — decidir como sinalizar à aplicação (ver nota) | ⚠ decidir |

> **Nota sobre DISC no `llReceive`:** a aplicação normalmente sabe que acabou pelo pacote de controlo END e chama o `llCloseRx`. Se aparecer um DISC dentro do `llReceive`, é porque algo correu mal. Combinem em grupo o que fazer (por ex.: devolver um valor especial). Vejam se o guia diz alguma coisa sobre isto.

### 6.4 Checklist do `llReceive`

- [ ] RR/REJ enviados com A = `0x03` (resposta do Rx)
- [ ] RR leva o número da **próxima** trama
- [ ] Repetidas são confirmadas com RR mas **não** entregues à aplicação
- [ ] BCC2 errado numa repetida → RR (não REJ)
- [ ] Destuffing feito antes de verificar o BCC2
- [ ] `rxExpected` só alterna quando a trama é aceite

---

## 7. Passo 6 — Testes

Correr por esta ordem, sempre com Tx e Rx em terminais/PCs separados.

### 7.1 Funções auxiliares
- [ ] Teste do stuffing/destuffing (secção 3.4)

### 7.2 Protocolo sem erros
- [ ] `llopen` → 1 `llSend` → `llclose`
- [ ] Vários `llSend` seguidos (confirmar que C alterna 0x00, 0x80, 0x00, ...)
- [ ] Ficheiro completo (penguin.gif ou o que o guia indicar), comparar o ficheiro recebido com o original (`cmp` ou `diff`)

### 7.3 Protocolo com falhas
- [ ] Arrancar só o Tx → 4 tentativas e falha
- [ ] Arrancar o Rx atrasado → liga-se na 2.ª/3.ª tentativa
- [ ] Desligar o cabo a meio da transferência e voltar a ligar → a transferência continua
- [ ] Ruído / erros simulados (se o lab tiver essa opção) → REJs e retransmissões, ficheiro final correto

---

## 8. Correções para a entrega (fazer no fim)

### Obrigatórias
- [ ] **Usar os parâmetros do `LinkLayer`** em vez de `4` e `3` fixos.
  - Guardar `llParameters` em `connectionParams` no `llOpenTx` e no `llOpenRx`.
  - Usar `connectionParams.nRetransmissions` e `connectionParams.timeout` no `llOpenTx`, `llCloseTx` e `llSend`.
  - ⚠ confirmar no guia se `nRetransmissions` é o total de tentativas ou só as retransmissões (total = n + 1).

### Recomendadas (robustez)
- [ ] `llOpenRx`: ler em `while` até chegar um SET, em vez de falhar à primeira trama inesperada.
- [ ] `llCloseRx`: ler em `while` até chegar DISC; depois, à espera do UA, se chegar outro DISC, reenviar o DISC.
- [ ] (Opcional) Timeout no `llCloseRx` à espera do UA final — exige `alarmConfig()` também no Rx.
- [ ] Ignorar o erro de leitura quando `errno == EINTR` (é o alarme a interromper o `read`, não um erro real).

### Limpeza
- [ ] Remover `A_TX` / `A_RX` e usar só `A_CMD_TX` / `A_CMD_RX`.
- [ ] Mover `#define _POSIX_SOURCE 1` para antes dos `#include` (ou remover).
- [ ] Remover `BUF_SIZE` se não for usado.
- [ ] Não imprimir "Timeout. Retrying..." na última tentativa.
- [ ] Verificar o retorno de `writeBytesSerialPort`.
- [ ] Ver se o guia pede estatísticas no `llclose` (nº de tramas, retransmissões, timeouts, tempo total).

---

## 9. Checklist final antes de entregar

- [ ] Compila sem warnings (`-Wall`)
- [ ] Todos os testes da secção 7 passam
- [ ] Endereços A de todas as tramas conferidos com a tabela da secção 1.1
- [ ] Valores marcados com ⚠ confirmados no guia
- [ ] Código comentado e sem prints de debug a mais
