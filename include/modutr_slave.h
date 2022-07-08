
#ifndef _MODUTR_SLAVE_H_
#define _MODUTR_SLAVE_H_

#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "typedefs_modutr.h"

/*
 * Tamanho máximo do body de um pacote de requisição (pacote recebido pelo slave)
 * e de resposta (pacote transmitido pelo slave)
 * NOTE: Ao se adicionar novas requisições ao protocolo estas definições DEVEM
 * ser alteradas para refletirem o maior tamanho de corpo possível dentre os
 * pacotes de requisições a serem recebidas e respostas a serem transmitidas
 */
/* Atualmente este é maior body recebido (update de firmware) */
#define RX_MAX_APP_BODY_LEN 132
/* Atualmente este é o maior body transmitido (pacote com 28 eventos) */
#define TX_MAX_APP_BODY_LEN 227

/**
 * Inicializa estrutura de dados do dispositivo modutr slave
 * @param modutr ponteiro para estrutura de dados do protocolo ModUTR
 * @param num_ts número de telessinais presentes no slave
 * @param num_tc número de telecomandos presentes no slave
 * @param comm_failure_timeout timeout para recepção de requisições ModUTR do mestre (s)
 * @param inter_fragment_timeout timeout em ms para recepção do próximo fragmento
 * de um pacote modutr (deve ser estimado de acordo com a velocidade do meio de
 * transmissão e tamanho do maior pacote usualmente recebido) (ms)
 * @param user_ptr ponteiro definido pelo usuário
 */
void modutr_slave_init_device(modutr_t *modutr, uint8_t num_ts, uint8_t num_tc, uint8_t num_tm,
                              uint16_t comm_failure_timeout, uint16_t inter_fragment_timeout,
                              void* user_ptr);

/**
 * Inicializa estruturas de dados necessárias à recepção e transmissão de
 * pacotes modutr
 * @param modutr_tx estrutura de dados de recepção do protocolo ModUTR
 * @param modutr_rx estrutura de dados de recepção do protocolo ModUTR
 * @param tx_buffsize tamanho do buffer de transmissão
 * @param tx_buffer ponteiro para o buffer de transmissão
 * @param rx_buffsize tamanho do buffer de recepção (deve ser uma potência de 2!)
 * @param rx_buffer ponteiro para o buffer de recepção
 * @param rx_app_buffer buffer para corpo dos pacotes recebidos (camada de aplicação)
 * @retval 0 inicialização terminada com sucesso
 * @retval -1 inicialização abortada por tamanho de buffers insuficiente
 */
int_fast8_t modutr_slave_init_io(modutr_tx_t* modutr_tx, modutr_rx_t* modutr_rx,
                                 unsigned char* tx_buffer, size_t tx_buffsize,
                                 unsigned char* rx_buffer, size_t rx_buffsize,
                                 unsigned char* rx_app_buffer);

/**
 * Verifica se havia um pacote recebido sendo processado, caso no qual ele é
 * descartado como inválido pelo fragmento seguinte não ter chegado a tempo
 * @param modutr_rx estrutura de dados de recepção do protocolo ModUTR
 *
 * @note esta função deve ser chamada pelo mecanismo de timer externo programado
 * para expirar quando decorrido o tempo de timeout para recebimento de um novo
 * fragmento
 */
void modutr_slave_on_rx_fragment_timeout(modutr_rx_t* modutr_rx);

/**
 * Função a ser chamada pelo mecanismo de timer externo programado para expirar
 * quando decorrido o tempo de timeout sem comunicação com o mestre
 * @param modutr estrutura de dados do protocolo ModUTR
 *
 * @note esta função deve ser chamada pelo mecanismo de timer externo programado
 * para expirar quando decorrido o tempo de timeout para comunicação com o mestre
 */
void modutr_slave_on_comm_timeout(modutr_t* modutr);

/**
 * Copia o pacote ou fragmento de pacote modutr recebido pelo dispositivo de
 * comunicação para o buffer de recepção da biblioteca
 * @param modutr estrutura de dados do protocolo ModUTR
 * @param modutr_rx estrutura de dados de recepção do protocolo ModUTR
 * @param modutr_tx estrutura de dados de recepção do protocolo ModUTR
 * @param packet endereço do buffer com o fragmento recebido
 * @param size tamanho do fragmento
 * @retval 0 fragmento buferizado com sucesso
 * @retval -1 buffer cheio, pacote ignorado
 */
int_fast8_t modutr_slave_recv_packet(modutr_t* modutr, modutr_rx_t* modutr_rx,
                                     modutr_tx_t* modutr_tx, unsigned char* packet,
                                     size_t size);

/**
 * Envia pacote que não envolve preenchimento de um BODY
 * @param modutr estrutura de dados do protocolo ModUTR
 * @param modutr_tx estrutura de dados de transmissão do protocolo ModUTR
 * @param idp identificador de função do pacote
 */
void send_ID_packet(modutr_t *modutr, modutr_tx_t *modutr_tx);

/**
 * Registra callbacks do protocolo ModUTR
 * @note com exceção das callbacks 'send_packet', 'set_inter_fragment_timer',
 * 'set_comm_timer', 'on_comm_link_status_change' e 'on_slave_id_update' (esta
 * apenas existente no modo MULTIPONTO), a aplicação não precisa registrar as
 * outras se não forem necessárias. Neste caso, basta passar NULL como parâmetro
 * que a biblioteca nunca as chamará.
 */
void modutr_slave_init_callbacks(modutr_callbacks_t *CB,
                                 void (*send_packet)(unsigned char *packet, size_t size, void *user_ptr),
                                 void (*set_inter_fragment_timer)(int16_t timeout, void *user_ptr),
                                 void (*set_comm_timer)(int16_t timeout, void *user_ptr),
                                 void (*on_comm_link_status_change)(uint8_t comm_failure, void *user_ptr),
#ifdef CONFIG_MULTIPOINT
                                 void (*on_slave_id_update)(uint32_t modutr_id, void *user_ptr),
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
                                 void (*on_recv_fw_block)(uint16_t block_num, uint16_t total_blocks,
                                                          unsigned char *block, void *user_ptr),
                                 void (*on_reset_request)(void *user_ptr),
                                 void (*on_events_request)(uint16_t *events_total, event_t *events,
                                                           void *user_ptr));

#endif
