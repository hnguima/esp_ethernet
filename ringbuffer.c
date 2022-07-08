
#include "ringbuffer.h"

void ringbuffer_init(ringbuffer *rb, unsigned char *buffer, size_t size)
{
  rb->head = 0;
  rb->tail = 0;
  rb->count = 0;
  rb->bitmask = size - 1;
  rb->buffer = buffer;
}

size_t ringbuffer_get_size(ringbuffer *rb)
{
  return rb->bitmask + 1;
}

int_fast8_t ringbuffer_write_byte(ringbuffer *rb, unsigned char byte)
{
  /* Se o buffer já estiver cheio retorna sem escrever o byte */
  if (rb->count == rb->bitmask + 1)
    return -1;

  rb->buffer[rb->head] = byte;
  /* Como o tamanho do buffer é uma potência de 2, a operação AND com a máscara
  * causa o "loop" do índice caso o mesmo ultrapasse o fim */
  rb->head++;
  rb->head = rb->head & rb->bitmask;
  rb->count++;
  return 0;
}

int_fast8_t ringbuffer_read_byte(ringbuffer *rb, unsigned char *byte)
{
  /* Verifica se ainda há elementos a serem lidos do buffer */
  if (rb->count)
  {
    rb->count--;
    *byte = rb->buffer[rb->tail];
    /* Como o tamanho do buffer é uma potência de 2, a operação AND com a máscara
     * causa o "loop" do índice caso o mesmo ultrapasse o fim */
    rb->tail++;
    rb->tail = rb->tail & rb->bitmask;
    return 0;
  }
  else
    return -1;
}

int_fast8_t ringbuffer_write_packet(ringbuffer *rb, unsigned char *packet, size_t size)
{
  size_t i = 0;
  /* Se pelo tamanho do pacote fosse ocorrer um buffer overrun, retorna sem
   * escrever nada */
  if (rb->count + size > rb->bitmask + 1)
    return -1;
  else
  {
    for (i = 0; i < size; i++)
    {
      rb->buffer[rb->head] = packet[i];

      rb->head++;
      rb->head = rb->head & rb->bitmask;
    }
    rb->count += size;
  }
  return 0;
}

int_fast8_t ringbuffer_flush(ringbuffer *rb)
{

  rb->tail = rb->head;
  rb->count = 0;

  return 0;
}
