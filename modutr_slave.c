#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "modutr_slave.h"
#include "esp_log.h"

#ifdef CONFIG_MULTIPOINT

/** Tamanho destinado ao cabeçalho do pacote (STX, NMN, UCDid, IDP e LEN) */
#define HEADER_SIZE 9
/** Tamanho destinado ao fim do pacote (ETX e CRC) */
#define TAIL_SIZE 3

#else

/** Tamanho destinado ao cabeçalho do pacote (STX, IDP e LEN) */
#define HEADER_SIZE 4
/** Tamanho destinado ao fim do pacote (CRC) */
#define TAIL_SIZE 2

#endif

/**
 * Efetua reset do interpretador de pacotes recebidos
 * @param modutr_rx estrutura de dados de recepção do protocolo ModUTR
 */
void reset_rx_packet_decoder(modutr_rx_t *modutr_rx)
{
  modutr_rx->rx_state = RxdSTART01;
  modutr_rx->rx_status &= ~RXD_PROCESSING_PACKET;
  modutr_rx->app_buffer_ptr = modutr_rx->app_buffer;
  modutr_rx->accum_crc = 0x0000;
}

/**
 * Retorna CRC de um buffer inteiro
 * @param buffer ponteiro para o buffer cujos elementos serão considerados no cálculo do CRC
 * @param size tamanho do buffer cujos elementos serão considerados no cálculo do CRC
 */
uint16_t packet_crc(unsigned char *buffer, size_t size)
{
  unsigned char *byte_ptr = buffer;
  uint16_t accum_crc = 0;
  uint16_t H, L, R, X;

  while (byte_ptr < (buffer + size))
  {
    H = accum_crc >> 8 & 0xFF;
    L = accum_crc & 0xFF;
    R = *byte_ptr++ ^ L;
    X = (R ^ ((R << 4) & 0x00ff));
    accum_crc = H ^ ((X << 8) ^ (X << 3) ^ (X >> 4));
  }
  return accum_crc;
}

/**
 * Calcula CRC cumulativo byte a byte
 * @param byte byte a ser utilizado no cálculo do CRC
 * @param accum_crc ponteiro para variável com o CRC acumulado
 */
void byte_crc(unsigned char byte, uint16_t *accum_crc)
{
  uint16_t H, L, R, X;

  H = (*accum_crc >> 8) & 0xFF;
  L = (*accum_crc) & 0xFF;
  R = byte ^ L;
  X = (R ^ ((R << 4) & 0x00ff));
  *accum_crc = H ^ ((X << 8) ^ (X << 3) ^ (X >> 4));
}

/**
 * Interpreta progressivamente o pacote ModUTR recebido
 * @param modutr ponteiro para estrutura do protocolo ModUTR
 * @param modutr_rx ponteiro para estrutura de recepção do protocolo ModUTR
 * @retval INVALID_PACKET pacote inválido (deve ser rejeitado)
 * @retval IN_PROGRESS interpretação do pacote progredindo com sucesso
 * @retval VALID_PACKET interpretação finalizada com pacote válido
 */
int_fast8_t rx_packet_decode(modutr_t *modutr, modutr_rx_t *modutr_rx)
{
  switch (modutr_rx->rx_state)
  {
  case RxdSTART01:
    if (modutr_rx->current_byte == MOD_UTR_STXD)
    {
      byte_crc(modutr_rx->current_byte, &modutr_rx->accum_crc);
#ifdef CONFIG_MULTIPOINT
      modutr_rx->rx_state = RxdNMN;
#else
      modutr_rx->rx_state = RxdIDP;
#endif
    }
    break;

#ifdef CONFIG_MULTIPOINT
  case RxdNMN:
    if (modutr_rx->current_byte == MOD_UTR_NMN)
    {
      byte_crc(modutr_rx->current_byte, &modutr_rx->accum_crc);
      modutr_rx->rx_state = RxdUCDidMMSB;
    }
    else
    {
      modutr_rx->invalid_nmn_count++;
      ESP_LOGE("rx_err", "nmn %x", modutr_rx->current_byte);
      return INVALID_PACKET;
    }
    break;

  case RxdUCDidMMSB:
    modutr_rx->UCDidTmp = (unsigned long)modutr_rx->current_byte << 24;
    byte_crc(modutr_rx->current_byte, &modutr_rx->accum_crc);
    modutr_rx->rx_state = RxdUCDidMSB;
    break;

  case RxdUCDidMSB:
    modutr_rx->UCDidTmp |= (unsigned long)modutr_rx->current_byte << 16;
    byte_crc(modutr_rx->current_byte, &modutr_rx->accum_crc);
    modutr_rx->rx_state = RxdUCDidLSB;
    break;

  case RxdUCDidLSB:
    modutr_rx->UCDidTmp |= (unsigned long)modutr_rx->current_byte << 8;
    byte_crc(modutr_rx->current_byte, &modutr_rx->accum_crc);
    modutr_rx->rx_state = RxdUCDidLLSB;
    break;

  case RxdUCDidLLSB:
    modutr_rx->UCDidTmp |= modutr_rx->current_byte;
    byte_crc(modutr_rx->current_byte, &modutr_rx->accum_crc);
    if (modutr_rx->UCDidTmp != modutr->slave_id)
    {
      modutr_rx->invalid_address++;
      ESP_LOGE("rx_err", "id");
      return INVALID_PACKET;
    }
    modutr_rx->rx_state = RxdIDP;
    break;
#endif
  case RxdIDP:
    modutr_rx->idp = modutr_rx->current_byte;
    byte_crc(modutr_rx->current_byte, &modutr_rx->accum_crc);
    modutr_rx->rx_state = RxdLENMSB;
    break;

  case RxdLENMSB:
    /* Atribui parte alta do campo LEN */
    modutr_rx->len = modutr_rx->current_byte << 8;
    byte_crc(modutr_rx->current_byte, &modutr_rx->accum_crc);
    modutr_rx->rx_state = RxdLENLSB;
    break;

  case RxdLENLSB:
    /* Atribui parte baixa do campo LEN */
    modutr_rx->len |= modutr_rx->current_byte;
    byte_crc(modutr_rx->current_byte, &modutr_rx->accum_crc);
    /* Se o tamanho do corpo do pacote for maior que o tamanho do buffer de
       * aplicação (buffsize - HEADER_SIZE - TAIL_SIZE porque o buffer de
       * aplicação não inclui os campos de cabeçalho) o campo LEN provavelmente
       * foi corrompido, devendo o pacote ser descartado */
    if (modutr_rx->len > (modutr_rx->buffsize - HEADER_SIZE - TAIL_SIZE))
    {
      modutr_rx->invalid_size_count++;
      ESP_LOGE("rx_err", "len");
      return INVALID_PACKET;
    }
    if (modutr_rx->len)
      modutr_rx->rx_state = RxdBODY;
    else
#ifdef CONFIG_MULTIPOINT
      modutr_rx->rx_state = RxdETXD;
#else
      modutr_rx->rx_state = RxdCRCMSB;
#endif
    break;

  case RxdBODY:
    *modutr_rx->app_buffer_ptr++ = modutr_rx->current_byte;
    byte_crc(modutr_rx->current_byte, &modutr_rx->accum_crc);
    /* NOTE: A subtração do tamanho do header se refere à exclusão do
       * cabeçalho para que apenas o tamanho do BODY seja considerado.*/
    if ((modutr_rx->app_buffer_ptr - modutr_rx->app_buffer) >= modutr_rx->len)
#ifdef CONFIG_MULTIPOINT
      modutr_rx->rx_state = RxdETXD;
#else
      modutr_rx->rx_state = RxdCRCMSB;
#endif
    break;

#ifdef CONFIG_MULTIPOINT
  case RxdETXD:
    if (modutr_rx->current_byte == MOD_UTR_ETXD)
    {
      byte_crc(modutr_rx->current_byte, &modutr_rx->accum_crc);
      modutr_rx->rx_state = RxdCRCMSB;
    }
    else
    {
      modutr_rx->invalid_end_count++;
      ESP_LOGE("rx_err", "etdx");
      return INVALID_PACKET;
    }
    break;
#endif

  case RxdCRCMSB:
    if (modutr_rx->current_byte == ((modutr_rx->accum_crc >> 8) & 0xFF))
      modutr_rx->rx_state = RxdCRCLSB;
    else
    {
      modutr_rx->invalid_crc_count++;
      ESP_LOGE("rx_err", "crc msb");
      return INVALID_PACKET;
    }
    break;

  case RxdCRCLSB:
    if (modutr_rx->current_byte == (modutr_rx->accum_crc & 0xFF))
      return VALID_PACKET;
    else
    {
      modutr_rx->invalid_crc_count++;
      ESP_LOGE("rx_err", "crc lsb");
      return INVALID_PACKET;
    }
    break;
  }
  /* Informa que se está aguardando um novo fragmento */
  return IN_PROGRESS;
}

/**
 * Reseta montador de pacotes a serem transmitidos
 * @param modutr_tx estrutura de dados de transmissão do protocolo ModUTR
 */
void reset_tx_packet_encoder(modutr_tx_t *modutr_tx)
{
  /* Inicializa os ponteiros de byte atual e de corpo (body do pacote)
   * respectivamente para o início do buffer e início do body */
  modutr_tx->byte_ptr = modutr_tx->buffer;
  modutr_tx->body_byte_ptr = modutr_tx->buffer + HEADER_SIZE;
}

/**
 * Monta pacote ModUTR a ser transmitido
 * @param modutr_tx estrutura de dados de transmissão do protocolo ModUTR
 * @retval VALID_PACKET pacote montado com sucesso
 * @retval INVALID_PACKET falha na montagem do pacote
 * @note Esta função só pode ser chamada DEPOIS de reset_tx_packet_encoder
 */
int_fast8_t tx_packet_encode(modutr_t *modutr, modutr_tx_t *modutr_tx)
{
  unsigned short LEN = 0;
  unsigned short CRC = 0;

  /* STX */
  *modutr_tx->byte_ptr++ = MOD_UTR_STXD;
#ifdef CONFIG_MULTIPOINT
  /* NMN */
  *modutr_tx->byte_ptr++ = MOD_UTR_NMN;

  /* UCDidMMSB */
  *modutr_tx->byte_ptr++ = (modutr->slave_id >> 24) & 0xFF;

  /* UCDidMSB */
  *modutr_tx->byte_ptr++ = (modutr->slave_id >> 16) & 0xFF;

  /* UCDidLSB */
  *modutr_tx->byte_ptr++ = (modutr->slave_id >> 8) & 0xFF;

  /* UCDidLLSB */
  *modutr_tx->byte_ptr++ = modutr->slave_id & 0xFF;
#endif
  /* idp */
  *modutr_tx->byte_ptr++ = modutr_tx->idp;

  /* LENMSB */
  LEN = modutr_tx->body_byte_ptr - (modutr_tx->buffer + HEADER_SIZE);
  *modutr_tx->byte_ptr++ = (LEN >> 8) & 0xFF;

  /* LENLSB */
  *modutr_tx->byte_ptr++ = LEN & 0xFF;

  /* BODY */
  if (LEN)
  {
    /* Se o tamanho do body mais os cabeçalhos for maior que o
     * buffer de transmissão, retorna com status de erro */
    if ((LEN + HEADER_SIZE + TAIL_SIZE) > modutr_tx->buffsize)
      return INVALID_PACKET;
    /* Incrementa ponteiro do buffer para logo após o fim do BODY, pois o mesmo
     * já foi preenchido pela função que define o pacote a ser enviado */
    modutr_tx->byte_ptr += LEN;
  }
#ifdef CONFIG_MULTIPOINT
  /* ETX */
  *modutr_tx->byte_ptr++ = MOD_UTR_ETXD;
#endif

  /* CRCMSB */
#ifdef CONFIG_MULTIPOINT
  /* No cálculo do CRC abaixo o "+1" é para contar apenas com o campo ETX,
   * pois TAIL_SIZE inclui também o CRC */
  CRC = packet_crc(modutr_tx->buffer, HEADER_SIZE + LEN + 1);
#else
  CRC = packet_crc(modutr_tx->buffer, HEADER_SIZE + LEN);
#endif
  *modutr_tx->byte_ptr++ = (CRC >> 8) & 0xFF;

  /* CRCLSB */
  *modutr_tx->byte_ptr++ = CRC & 0xFF;

  return VALID_PACKET;
}

/**
 * Envia pacote montado no buffer de transmissão
 * @param modutr estrutura de dados do protocolo ModUTR
 * @param modutr_tx estrutura de dados de transmissão do protocolo ModUTR
 */
void send_packet(modutr_t *modutr, modutr_tx_t *modutr_tx)
{
  int timeout;

  /* O tamanho do pacote completo é o endereço do ponteiro menos o início do buffer */
  size_t size = modutr_tx->byte_ptr - modutr_tx->buffer;
  /* Se a callback está configurada, envia pacote */
  modutr->callbacks.send_packet(modutr_tx->buffer, size, modutr->user_ptr);
  /* Arma timer com timeout para recepção de uma resposta */
  timeout = modutr->comm_failure_timeout;
  modutr->callbacks.set_comm_timer(timeout, modutr->user_ptr);
}

/**
 * Envia pacote que não envolve preenchimento de um BODY
 * @param modutr estrutura de dados do protocolo ModUTR
 * @param modutr_tx estrutura de dados de transmissão do protocolo ModUTR
 * @param idp identificador de função do pacote
 */
void send_null_body_packet(modutr_t *modutr, modutr_tx_t *modutr_tx, unsigned char idp)
{
  reset_tx_packet_encoder(modutr_tx);
  modutr_tx->idp = idp;
  /* Procede à montagem e envio do pacote */
  if (tx_packet_encode(modutr, modutr_tx) == VALID_PACKET)
    send_packet(modutr, modutr_tx);
}

/**
 * Executa ação correspondente ao pacote recebido, devolvendo pacote de resposta
 * @param modutr estrutura de dados do protocolo ModUTR
 * @param modutr_rx estrutura de dados de recepção do protocolo ModUTR
 * @param modutr_tx estrutura de dados de transmissão do protocolo ModUTR
 */
void rx_process_action(modutr_t *modutr, modutr_rx_t *modutr_rx, modutr_tx_t *modutr_tx)
{
  if ((modutr_rx->idp == MOD_UTR_OP_TC) &&
      (modutr->callbacks.on_telecommand_action))
  {
    uint8_t action = modutr_rx->app_buffer[0];
    uint8_t point = modutr_rx->app_buffer[1];
    tc_action_response_t response = modutr->callbacks.on_telecommand_action(point, action, modutr->user_ptr);
    /* Prepara resposta */
    reset_tx_packet_encoder(modutr_tx);
    modutr_tx->idp = MOD_UTR_RESP_OP_TC;
    *modutr_tx->body_byte_ptr++ = response;
    *modutr_tx->body_byte_ptr++ = point;
    /* Procede à montagem e envio do pacote */
    if (tx_packet_encode(modutr, modutr_tx) == VALID_PACKET)
      send_packet(modutr, modutr_tx);
  }
  else if ((modutr_rx->idp == MOD_UTR_LEITURA_ESTADO_TC) &&
           modutr->callbacks.on_telecommand_state_read)
  {
    uint8_t initial_point = modutr_rx->app_buffer[0];
    uint8_t num_points = modutr_rx->app_buffer[1];
    /* Prepara resposta */
    reset_tx_packet_encoder(modutr_tx);
    modutr_tx->idp = MOD_UTR_RESP_ESTADO_TC;
    *modutr_tx->body_byte_ptr++ = initial_point;
    *modutr_tx->body_byte_ptr++ = num_points;
    modutr->callbacks.on_telecommand_state_read(initial_point, num_points,
                                                modutr_tx->body_byte_ptr, modutr->user_ptr);
    /* Atualiza ponteiro do corpo para contabilizar os valores preenchidos pela
    * aplicação na callback */
    modutr_tx->body_byte_ptr += num_points;
    /* Procede à montagem e envio do pacote */
    if (tx_packet_encode(modutr, modutr_tx) == VALID_PACKET)
      send_packet(modutr, modutr_tx);
  }
  else if ((modutr_rx->idp == MOD_UTR_CONFIG_TC) &&
           (modutr->callbacks.on_telecommand_config_read || modutr->callbacks.on_telecommand_config))
  {
    if ((modutr_rx->len == 1) || (modutr_rx->len == 4))
    {
      uint8_t point = modutr_rx->app_buffer[0];
      tc_config_t config = (tc_config_t)0;
      uint16_t time_ms = 0;
      /* Se o comprimento do BODY é 1 trata-se de requisição de leitura de configuração do TC
       * (aplicação retornará valores lidos através dos ponteiros fornecidos) */
      if ((modutr_rx->len == 1) && (modutr->callbacks.on_telecommand_config_read))
        modutr->callbacks.on_telecommand_config_read(point, &config, &time_ms, modutr->user_ptr);
      /* Se o comprimento do BODY é 4 trata-se de requisição para configurar um TC */
      else if ((modutr_rx->len == 4) && (modutr->callbacks.on_telecommand_config))
      {
        config = (tc_config_t)modutr_rx->app_buffer[1];
        time_ms = modutr_rx->app_buffer[2] << 8 | modutr_rx->app_buffer[3];
        modutr->callbacks.on_telecommand_config(point, config, time_ms, modutr->user_ptr);
      }

      /* Prepara resposta */
      reset_tx_packet_encoder(modutr_tx);
      modutr_tx->idp = MOD_UTR_RESP_CONFIG_TC;
      *modutr_tx->body_byte_ptr++ = point;
      *modutr_tx->body_byte_ptr++ = config;
      *modutr_tx->body_byte_ptr++ = time_ms >> 8;
      *modutr_tx->body_byte_ptr++ = time_ms & 0xff;
      /* Procede à montagem e envio do pacote */
      if (tx_packet_encode(modutr, modutr_tx) == VALID_PACKET)
        send_packet(modutr, modutr_tx);
    }
    /* Requisição inválida (LEN não é 1 nem 4), envia NACK */
    else
      send_null_body_packet(modutr, modutr_tx, MOD_UTR_NACK);
  }
  else if ((modutr_rx->idp == MOD_UTR_LEITURA_ESTADO_TS) &&
           modutr->callbacks.on_telesignal_state_read)
  {
    uint8_t initial_point = modutr_rx->app_buffer[0];
    uint8_t num_points = modutr_rx->app_buffer[1];
    /* Prepara resposta */
    reset_tx_packet_encoder(modutr_tx);
    modutr_tx->idp = MOD_UTR_RESP_ESTADO_TS;
    *modutr_tx->body_byte_ptr++ = initial_point;
    *modutr_tx->body_byte_ptr++ = num_points;
    modutr->callbacks.on_telesignal_state_read(initial_point, num_points,
                                               modutr_tx->body_byte_ptr, modutr->user_ptr);
    /* Atualiza ponteiro do corpo para contabilizar os valores preenchidos pela
    * aplicação na callback */
    modutr_tx->body_byte_ptr += num_points;
    /* Procede à montagem e envio do pacote */
    if (tx_packet_encode(modutr, modutr_tx) == VALID_PACKET)
      send_packet(modutr, modutr_tx);
  }
  else if ((modutr_rx->idp == MOD_UTR_CONFIG_TS) &&
           (modutr->callbacks.on_telesignal_config_read || modutr->callbacks.on_telesignal_config))
  {
    if ((modutr_rx->len == 1) || (modutr_rx->len == 4))
    {
      uint8_t point = modutr_rx->app_buffer[0];
      ts_config_t config = (ts_config_t)0;
      uint16_t debounce_ms = 0;
      /* Se o comprimento do BODY é 1 trata-se de requisição de leitura de configuração do TS
       * (aplicação retornará valores lidos através dos ponteiros fornecidos) */
      if ((modutr_rx->len == 1) && modutr->callbacks.on_telesignal_config_read)
        modutr->callbacks.on_telesignal_config_read(point, &config, &debounce_ms, modutr->user_ptr);
      /* Se o comprimento do BODY é 4 trata-se de requisição para configurar um TS */
      else if ((modutr_rx->len == 4) && modutr->callbacks.on_telesignal_config)
      {
        config = (ts_config_t)modutr_rx->app_buffer[1];
        debounce_ms = modutr_rx->app_buffer[2] << 8 | modutr_rx->app_buffer[3];
        modutr->callbacks.on_telesignal_config(point, config, debounce_ms, modutr->user_ptr);
      }
      /* Prepara resposta */
      reset_tx_packet_encoder(modutr_tx);
      modutr_tx->idp = MOD_UTR_RESP_CONFIG_TS;
      *modutr_tx->body_byte_ptr++ = point;
      *modutr_tx->body_byte_ptr++ = config;
      *modutr_tx->body_byte_ptr++ = debounce_ms >> 8;
      *modutr_tx->body_byte_ptr++ = debounce_ms & 0xff;
      /* Procede à montagem e envio do pacote */
      if (tx_packet_encode(modutr, modutr_tx) == VALID_PACKET)
        send_packet(modutr, modutr_tx);
    }
  }
  else if ((modutr_rx->idp == MOD_UTR_LEITURA_TM) &&
           modutr->callbacks.on_telemeasurement_read)
  {
    uint8_t point = modutr_rx->app_buffer[0];
    uint8_t num_points = modutr_rx->app_buffer[1];
    /* Prepara resposta */
    reset_tx_packet_encoder(modutr_tx);
    modutr_tx->idp = MOD_UTR_RESP_LEITURA_TM;
    *modutr_tx->body_byte_ptr++ = point;
    *modutr_tx->body_byte_ptr++ = num_points;
    modutr->callbacks.on_telemeasurement_read(point, num_points,
                                              (float *)modutr_tx->body_byte_ptr,
                                              modutr->user_ptr);
    /* Atualiza ponteiro do corpo para contabilizar os valores preenchidos pela
    * aplicação na callback (note que cada elemento é um float de 4 bytes) */
    modutr_tx->body_byte_ptr += (num_points * 4);
    /* Procede à montagem e envio do pacote */
    if (tx_packet_encode(modutr, modutr_tx) == VALID_PACKET)
      send_packet(modutr, modutr_tx);
  }
  else if ((modutr_rx->idp == MOD_UTR_CFG_RTC) && modutr->callbacks.on_set_rtc)
  {
    timestamp_t timestamp;
    timestamp.high = modutr_rx->app_buffer[0] << 8 | modutr_rx->app_buffer[1];
    timestamp.low = (uint32_t)modutr_rx->app_buffer[2] << 24 |
                    (uint32_t)modutr_rx->app_buffer[3] << 16 |
                    modutr_rx->app_buffer[4] << 8 |
                    modutr_rx->app_buffer[5];
    modutr->callbacks.on_set_rtc(&timestamp, modutr->user_ptr);
    send_null_body_packet(modutr, modutr_tx, MOD_UTR_ACK);
  }
  else if ((modutr_rx->idp == MOD_UTR_PEDIDO_LEITURA_RTC) &&
           modutr->callbacks.on_read_rtc)
  {
    timestamp_t timestamp; // = {0,0};
    timestamp.high = 0;
    timestamp.low = 0;
    modutr->callbacks.on_read_rtc(&timestamp, modutr->user_ptr);
    /* Prepara resposta */
    reset_tx_packet_encoder(modutr_tx);
    modutr_tx->idp = MOD_UTR_RESP_LEITURA_RTC;
    *modutr_tx->body_byte_ptr++ = timestamp.high >> 8;
    *modutr_tx->body_byte_ptr++ = timestamp.high & 0xff;
    *modutr_tx->body_byte_ptr++ = timestamp.low >> 24;
    *modutr_tx->body_byte_ptr++ = timestamp.low >> 16;
    *modutr_tx->body_byte_ptr++ = timestamp.low >> 8;
    *modutr_tx->body_byte_ptr++ = timestamp.low & 0xff;
    /* Procede à montagem e envio do pacote */
    if (tx_packet_encode(modutr, modutr_tx) == VALID_PACKET)
      send_packet(modutr, modutr_tx);
  }
  else if ((modutr_rx->idp == MOD_UTR_PEDIDO_LEITURA_VS_FW) &&
           modutr->callbacks.on_read_fw_version)
  {
    uint8_t ver = 0;
    uint8_t rev = 0;
    modutr->callbacks.on_read_fw_version(&ver, &rev, modutr->user_ptr);
    /* Prepara resposta */
    reset_tx_packet_encoder(modutr_tx);
    modutr_tx->idp = MOD_UTR_RESP_LEITURA_VS_FW;
    *modutr_tx->body_byte_ptr++ = ver;
    *modutr_tx->body_byte_ptr++ = rev;
    /* Procede à montagem e envio do pacote */
    if (tx_packet_encode(modutr, modutr_tx) == VALID_PACKET)
      send_packet(modutr, modutr_tx);
  }
  else if ((modutr_rx->idp == MOD_UTR_PREPARE_FW_UPGRADE) &&
           modutr->callbacks.on_prepare_fw_upgrade)
  {
    modutr->callbacks.on_prepare_fw_upgrade(modutr->user_ptr);
    /* NOTE: Implementação modificada para simplesmente retornar
     * um ACK comum assim que a callback retornar. Discutir com
     * o pessoal, conforme nota deixada na especificação */
    send_null_body_packet(modutr, modutr_tx, MOD_UTR_ACK);
  }
  else if ((modutr_rx->idp == MOD_UTR_FW_UPDATE_BLOCK) &&
           modutr->callbacks.on_recv_fw_block)
  {
    uint16_t block_num = modutr_rx->app_buffer[0] << 8 | modutr_rx->app_buffer[1];
    uint16_t blocks_total = modutr_rx->app_buffer[2] << 8 | modutr_rx->app_buffer[3];
    modutr->callbacks.on_recv_fw_block(block_num, blocks_total,
                                       modutr_rx->app_buffer + 4, modutr->user_ptr);
    /* Prepara resposta */
    reset_tx_packet_encoder(modutr_tx);
    modutr_tx->idp = MOD_UTR_FW_UPDATE_BLOCK_ACK;
    *modutr_tx->body_byte_ptr++ = block_num >> 8;
    *modutr_tx->body_byte_ptr++ = block_num & 0xff;
    /* Procede à montagem e envio do pacote */
    if (tx_packet_encode(modutr, modutr_tx) == VALID_PACKET)
      send_packet(modutr, modutr_tx);
  }
  else if ((modutr_rx->idp == MOD_UTR_RESET) && modutr->callbacks.on_reset_request)
  {
    modutr->callbacks.on_reset_request(modutr->user_ptr);
    send_null_body_packet(modutr, modutr_tx, MOD_UTR_RESP_RESET);
  }
#ifdef MULTIPOINT_MODUTR
  else if (modutr_rx->idp == MOD_UTR_ADDR_CFG)
  {
    modutr->slave_id = (uint32_t)modutr_rx->app_buffer[0] << 24 |
                       (uint32_t)modutr_rx->app_buffer[1] << 16 |
                       modutr_rx->app_buffer[2] << 8 |
                       modutr_rx->app_buffer[3] & 0xff;
    modutr->callbacks.on_slave_id_update(modutr->slave_id, modutr->user_ptr);
    /* Envia ACK confirmando mudança do endereço */
    send_null_body_packet(modutr, modutr_tx, MOD_UTR_ACK);
  }
#endif
  else if ((modutr_rx->idp == MOD_UTR_EVENTO) && modutr->callbacks.on_events_request)
  {
    uint16_t num_events, i;
    event_t *events = malloc(sizeof(event_t));
    modutr->callbacks.on_events_request(&num_events, events, modutr->user_ptr);
    /* Prepara resposta */
    reset_tx_packet_encoder(modutr_tx);
    modutr_tx->idp = MOD_UTR_RESP_EVENTO;
    *modutr_tx->body_byte_ptr++ = num_events;
    for (i = 0; i < num_events; i++)
    {
      *modutr_tx->body_byte_ptr++ = events[i].event_index;
      *modutr_tx->body_byte_ptr++ = events[i].event_state;
      *modutr_tx->body_byte_ptr++ = events[i].rtc_high >> 8;
      *modutr_tx->body_byte_ptr++ = events[i].rtc_high & 0xff;
      *modutr_tx->body_byte_ptr++ = events[i].rtc_low >> 24;
      *modutr_tx->body_byte_ptr++ = events[i].rtc_low >> 16;
      *modutr_tx->body_byte_ptr++ = events[i].rtc_low >> 8;
      *modutr_tx->body_byte_ptr++ = events[i].rtc_low & 0xff;
    }
    /* Procede à montagem e envio do pacote */
    if (tx_packet_encode(modutr, modutr_tx) == VALID_PACKET)
      send_packet(modutr, modutr_tx);
    free(events);
  }
  /* Reinicia máquina de estados de recepção */
  reset_rx_packet_decoder(modutr_rx);
}

void modutr_slave_on_comm_timeout(modutr_t *modutr)
{
  modutr->comm_failure = 1;
  modutr->callbacks.on_comm_link_status_change(modutr->comm_failure, modutr->user_ptr);
}

void modutr_slave_on_rx_fragment_timeout(modutr_rx_t *modutr_rx)
{
  if (modutr_rx->rx_status & RXD_PROCESSING_PACKET)
  {
    modutr_rx->invalid_timed_out_frag_count++;
    modutr_rx->invalid_packet_count++;
    reset_rx_packet_decoder(modutr_rx);
  }
}

int_fast8_t modutr_slave_recv_packet(modutr_t *modutr, modutr_rx_t *modutr_rx,
                                     modutr_tx_t *modutr_tx, unsigned char *packet,
                                     size_t size)
{
  int_fast8_t status;

  /* Copia fragmento recebido para o buffer */
  if (ringbuffer_write_packet(&modutr_rx->ring_buffer, packet, size) == -1)
  {
    modutr_rx->dropped_fragment_count++;
    return -1;
  }

  /* Habilita processamento do fragmento */
  modutr_rx->rx_status |= RXD_PROCESSING_PACKET;

  status = IN_PROGRESS;
  /* Processa conteúdo do buffer até onde o mesmo foi preenchido */
  while (ringbuffer_read_byte(&modutr_rx->ring_buffer, &modutr_rx->current_byte) == 0)
  {

    status = rx_packet_decode(modutr, modutr_rx);

    if (status != IN_PROGRESS)
      break;
  }

  /* Toma ações correspondentes ao status de interpretação do fragmento recebido */
  switch (status)
  {
  case IN_PROGRESS:
    ESP_LOGE("mdb_slv", "state: in progress");
    /* Se o processamento do pacote ainda não terminou, deve-se resetar o timer
       * com o timeout de espera máxima por um novo fragmento */
    modutr->callbacks.set_inter_fragment_timer(-1, modutr->user_ptr);
    modutr->callbacks.set_inter_fragment_timer(modutr->inter_fragment_timeout, modutr->user_ptr);
    break;

  case VALID_PACKET:
    // ESP_LOGE("mdb_slv", "state: valid packet");
    // ESP_LOG_BUFFER_HEX("buffer tx", modutr_tx->buffer, modutr_tx->body_byte_ptr - modutr_tx->buffer);
    /* Se estava com status de falha de comunicação, a recepção de um pacote
       * válido indica que ela voltou ao normal */
    if (modutr->comm_failure)
    {
      modutr->comm_failure = 0;
      modutr->callbacks.on_comm_link_status_change(modutr->comm_failure, modutr->user_ptr);
    }
    /* Cancela timers de espera por novo fragmento e timeout de comunicação */
    modutr->callbacks.set_inter_fragment_timer(-1, modutr->user_ptr);
    modutr->callbacks.set_comm_timer(-1, modutr->user_ptr);
    /* Contabiliza pacote válido e processa ação referente ao mesmo */
    modutr_rx->valid_packet_count++;
    rx_process_action(modutr, modutr_rx, modutr_tx);
    break;

  case INVALID_PACKET:
    ESP_LOGE("mdb_slv", "state: invalid");
    /* Cancela apenas o timer de espera por um novo fragmento. O de timeout
       * de comunicação deve ser resetado apenas com um pacote válido */
    modutr->callbacks.set_inter_fragment_timer(-1, modutr->user_ptr);
    modutr_rx->invalid_packet_count++;
    reset_rx_packet_decoder(modutr_rx);
    break;
  }
  return 0;
}

void send_ID_packet(modutr_t *modutr, modutr_tx_t *modutr_tx)
{
  reset_tx_packet_encoder(modutr_tx);

  modutr_tx->idp = MOD_UTR_SLAVE_ID;
  // modutr_tx->body_byte_ptr = malloc(sizeof(uint8_t) * 2);
  *(modutr_tx->body_byte_ptr++) = (modutr->slave_id >> 8) & 0xFF;
  *(modutr_tx->body_byte_ptr++) = modutr->slave_id & 0xFF;
  /* Procede à montagem e envio do pacote */
  if (tx_packet_encode(modutr, modutr_tx) == VALID_PACKET)
    send_packet(modutr, modutr_tx);
}

int_fast8_t modutr_slave_init_io(modutr_tx_t *modutr_tx, modutr_rx_t *modutr_rx,
                                 unsigned char *tx_buffer, size_t tx_buffsize,
                                 unsigned char *rx_buffer, size_t rx_buffsize,
                                 unsigned char *rx_app_buffer)
{
  if (tx_buffsize < (HEADER_SIZE + TX_MAX_APP_BODY_LEN + TAIL_SIZE) ||
      (rx_buffsize < (HEADER_SIZE + RX_MAX_APP_BODY_LEN + TAIL_SIZE)))
    return -1;

  /* Zera todos os campos das estruturas */
  // memset(modutr_tx, 0, sizeof(modutr_tx_t));
  // memset(modutr_rx, 0, sizeof(modutr_rx_t));

  /* Inicializa ring buffer para recepção com base no maior pacote de resposta esperado */
  modutr_rx->buffsize = rx_buffsize;
  ringbuffer_init(&modutr_rx->ring_buffer, rx_buffer, rx_buffsize);

  /* Inicializa buffer de camada de aplicação para recepção */
  modutr_rx->app_buffer = rx_app_buffer;
  memset(rx_app_buffer, 0, RX_MAX_APP_BODY_LEN);

  /* Inicializa buffer de transmissão */
  modutr_tx->buffsize = tx_buffsize;
  memset(tx_buffer, 0, tx_buffsize);

  reset_rx_packet_decoder(modutr_rx);

  return 0;
}

void modutr_slave_init_device(modutr_t *modutr, uint8_t num_ts, uint8_t num_tc, uint8_t num_tm,
                              uint16_t comm_failure_timeout, uint16_t inter_fragment_timeout,
                              void *user_ptr)
{
  /* Zera todos os campos da estrutura */
  memset(modutr, 0, sizeof(modutr_t));

  modutr->num_ts = num_ts;
  modutr->num_tc = num_tc;
  modutr->num_tm = num_tm;
  modutr->comm_failure_timeout = comm_failure_timeout;
  modutr->inter_fragment_timeout = inter_fragment_timeout;
  /* Inicializa ponteiro de dados de usuário */
  modutr->user_ptr = user_ptr;
}

void modutr_slave_init_callbacks(modutr_callbacks_t *CB,
                                 void (*send_packet)(unsigned char *packet, size_t size, void *user_ptr),
                                 void (*set_inter_fragment_timer)(int16_t timeout, void *user_ptr),
                                 void (*set_comm_timer)(int16_t timeout, void *user_ptr),
                                 void (*on_comm_link_status_change)(uint8_t comm_failure, void *user_ptr),
#ifdef CONFIG_MULTIPOINT
                                 void (*on_slave_id_update)(uint32_t slave_id, void *user_ptr),
#endif
                                 tc_action_response_t (*on_telecommand_action)(uint8_t point, unsigned char action,
                                                                               void *user_ptr),
                                 void (*on_telecommand_state_read)(uint8_t initial_point, uint8_t num_points,
                                                                   unsigned char *point_value_array, void *user_ptr),
                                 void (*on_telecommand_config)(uint8_t point, tc_config_t config, uint16_t time_ms,
                                                               void *user_ptr),
                                 void (*on_telecommand_config_read)(uint8_t point, tc_config_t *config,
                                                                    uint16_t *time_ms, void *user_ptr),
                                 void (*on_telesignal_state_read)(uint8_t initial_point, uint8_t num_points,
                                                                  unsigned char *point_value_array, void *user_ptr),
                                 void (*on_telesignal_config)(uint8_t point, ts_config_t config, uint16_t debounce_ms,
                                                              void *user_ptr),
                                 void (*on_telesignal_config_read)(uint8_t point, ts_config_t *config, uint16_t *debounce_ms,
                                                                   void *user_ptr),
                                 void (*on_telemeasurement_read)(uint8_t initial_point, uint8_t num_points,
                                                                 float *point_value_array, void *user_ptr),
                                 void (*on_set_rtc)(const timestamp_t *timestamp, void *user_ptr),
                                 void (*on_read_rtc)(timestamp_t *timestamp, void *user_ptr),
                                 void (*on_read_fw_version)(uint8_t *ver, uint8_t *rev, void *user_ptr),
                                 void (*on_prepare_fw_upgrade)(void *user_ptr),
                                 void (*on_recv_fw_block)(uint16_t block_number, uint16_t total_blocks,
                                                          unsigned char *block, void *user_ptr),
                                 void (*on_reset_request)(void *user_ptr),
                                 void (*on_events_request)(uint16_t *events_total, event_t *events,
                                                           void *user_ptr))
{
  if (send_packet)
    CB->send_packet = send_packet;
#if defined(__GNUC__) && !defined(EMBEDDED_SYSTEM)
  else
  {
    printf("The 'send_packet' callback can't be NULL!\n");
    abort();
  }
#endif

  if (set_inter_fragment_timer)
    CB->set_inter_fragment_timer = set_inter_fragment_timer;
#if defined(__GNUC__) && !defined(EMBEDDED_SYSTEM)
  else
  {
    printf("The 'set_inter_fragment_timer' callback can't be NULL!\n");
    abort();
  }
#endif

  if (set_comm_timer)
    CB->set_comm_timer = set_comm_timer;
#if defined(__GNUC__) && !defined(EMBEDDED_SYSTEM)
  else
  {
    printf("The 'set_comm_timer' callback can't be NULL!\n");
    abort();
  }
#endif

  /*
  if (on_comm_link_status_change)
    CB->on_comm_link_status_change = on_comm_link_status_change;
#if defined(__GNUC__) && !defined(EMBEDDED_SYSTEM)
  else {
    printf("The 'on_comm_link_status_change' callback can't be NULL!\n");
    abort();
  }
#endif
*/
  CB->on_comm_link_status_change = on_comm_link_status_change;

#ifdef CONFIG_MULTIPOINT
  if (on_slave_id_update)
    CB->on_slave_id_update = on_slave_id_update;
#if defined(__GNUC__) && !defined(EMBEDDED_SYSTEM)
  else
  {
    printf("The 'on_slave_id_update' callback can't be NULL!\n");
    abort();
  }
#endif
#endif

  CB->on_telecommand_action = on_telecommand_action;
  CB->on_telecommand_state_read = on_telecommand_state_read;
  CB->on_telecommand_config = on_telecommand_config;
  CB->on_telecommand_config_read = on_telecommand_config_read;
  CB->on_telesignal_state_read = on_telesignal_state_read;
  CB->on_telesignal_config = on_telesignal_config;
  CB->on_telesignal_config_read = on_telesignal_config_read;
  CB->on_telemeasurement_read = on_telemeasurement_read;
  CB->on_set_rtc = on_set_rtc;
  CB->on_read_rtc = on_read_rtc;
  CB->on_read_fw_version = on_read_fw_version;
  CB->on_prepare_fw_upgrade = on_prepare_fw_upgrade;
  CB->on_recv_fw_block = on_recv_fw_block;
  CB->on_reset_request = on_reset_request;
  CB->on_events_request = on_events_request;
}
