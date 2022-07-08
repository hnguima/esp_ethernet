
#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#include <stdint.h>
#include <stddef.h>

typedef struct {
  uint8_t head;            /* indice do byte a ser escrito no buffer */
  uint8_t tail;            /* indice do byte a ser lido do buffer */
  uint8_t count;           /* número de bytes não lidos no buffer */
  uint8_t bitmask;         /* máscara representando o maior índice possível */
  unsigned char* buffer;  /* ponteiro para o início do buffer */
} ringbuffer;

/**
 * Inicializa estrutura de dados do buffer
 * @param rb estrutura de dados do buffer circular
 * @param buffer ponteiro para o início do buffer
 * @param size tamanho alocado para o buffer (deve ser uma potência de 2!)
 */
void ringbuffer_init(ringbuffer* rb, unsigned char* buffer, size_t size);

/**
 * Retorna tamanho do buffer alocado para o buffer circular
 * @param rb estrutura de dados do buffer circular
 */
size_t ringbuffer_get_size(ringbuffer* rb);

/**
 * Escreve um pacote no buffer circular
 * @param rb estrutura de dados do buffer circular
 * @param packet ponteiro para o início do pacote ser escrito no buffer
 * @param size tamanho do pacote a ser escrito no buffer
 * @retval 0 pacote escrito com sucesso
 * @retval -1 falha na escrita (ponto de leitura seria sobrescrito)
 */
int_fast8_t ringbuffer_write_packet(ringbuffer* rb, unsigned char* packet, size_t size);

/**
 * Escreve um byte no buffer circular
 * @param rb estrutura de dados do buffer circular
 * @param byte byte a ser escrito no buffer
 * @retval 0 byte escrito com sucesso
 * @retval -1 falha na escrita (buffer já está cheio)
 */
int_fast8_t ringbuffer_write_byte(ringbuffer* rb, unsigned char byte);

/**
 * Retorna o próximo byte do buffer circular
 * @param rb estrutura de dados do buffer circular
 * @param byte byte a ser lido do buffer
 * @retval 0 byte lido com sucesso
 * @retval -1 falha na leitura (não há mais bytes a serem lidos)
 */
int_fast8_t ringbuffer_read_byte(ringbuffer* rb, unsigned char* byte);

/**
 * Limpa o buffer circular
 * @param rb estrutura de dados do buffer circular
 * @retval 0 byte lido com sucesso
 * @retval -1 falha na leitura (não há mais bytes a serem lidos)
 */
int_fast8_t ringbuffer_flush(ringbuffer *rb);

#endif
