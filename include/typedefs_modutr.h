#ifndef TYPEDEFSMODUTR_H
#define TYPEDEFSMODUTR_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
//#include <timestamp.h>
// #include <time.h>
#include "ringbuffer.h"

typedef struct {
  uint32_t high;
  uint32_t low;
} timestamp_t;

// #define CONFIG_MULTIPOINT

/* Constantes modutr */
#define MOD_UTR_STXD        0x02

#ifdef CONFIG_MULTIPOINT
#define MOD_UTR_NMN         0x00
#define MOD_UTR_ETXD        0x03
#endif

/* Identificadores de operações MODUTR (IDP) */
#define MOD_UTR_ACK                    'A'
#define MOD_UTR_NACK                   'N'

#define MOD_UTR_OP_TC                  'K'
#define MOD_UTR_RESP_OP_TC             'k'
#define MOD_UTR_LEITURA_ESTADO_TC      'M'
#define MOD_UTR_RESP_ESTADO_TC         'm'
#define MOD_UTR_CONFIG_TC              'G'
#define MOD_UTR_RESP_CONFIG_TC         'g'

#define MOD_UTR_LEITURA_ESTADO_TS      'L'
#define MOD_UTR_RESP_ESTADO_TS         'l'
#define MOD_UTR_CONFIG_TS              'D'
#define MOD_UTR_RESP_CONFIG_TS         'd'

#define MOD_UTR_LEITURA_TM             'T'
#define MOD_UTR_RESP_LEITURA_TM        't'

#define MOD_UTR_CFG_RTC                'Q'

#define MOD_UTR_PEDIDO_LEITURA_RTC     'R'
#define MOD_UTR_RESP_LEITURA_RTC       'r'

#define MOD_UTR_PEDIDO_LEITURA_VS_FW   'S'
#define MOD_UTR_RESP_LEITURA_VS_FW     's'

#define MOD_UTR_PREPARE_FW_UPGRADE     'E'

#define MOD_UTR_FW_UPDATE_BLOCK        'U'
#define MOD_UTR_FW_UPDATE_BLOCK_ACK    'u'

#ifdef CONFIG_MULTIPOINT
#define MOD_UTR_SLAVE_ID               '*'
#define MOD_UTR_ADDR_CFG               '@'
#endif

#define MOD_UTR_EVENTO                 '!'
#define MOD_UTR_RESP_EVENTO            'a'

#define MOD_UTR_RESET                  'Z'
#define MOD_UTR_RESP_RESET             'z'


/** Códigos de erro e retorno */
#define ERROR_WAITING_FOR_RESPONSE                         1

#define INVALID_PACKET -1
#define IN_PROGRESS 0
#define VALID_PACKET 1

/* ---------------------------------------------------------------------------*/

typedef enum
{
  RXD_PROCESSING_PACKET    = 0x0001  /* Flag para indicar que há um fragmento sendo processado */
}TRxdStatus;

typedef enum
{
  RxdSTART01,
#ifdef CONFIG_MULTIPOINT
  RxdNMN,
  RxdUCDidMMSB,
  RxdUCDidMSB,
  RxdUCDidLSB,
  RxdUCDidLLSB,
#endif
  RxdIDP,
  RxdLENMSB,
  RxdLENLSB,
  RxdBODY,
#ifdef CONFIG_MULTIPOINT
  RxdETXD,
#endif
  RxdCRCMSB,
  RxdCRCLSB
} modutr_rx_state_t;

typedef struct
{
  uint8_t event_id;
  uint8_t event_index;
  uint8_t event_state;
  uint16_t rtc_high;
  uint32_t rtc_low;
} event_t;

/*TODO: Há categorias redundantes ou inúteis aqui, verificar e otimizar */
typedef enum {
  TC_READY,
  OP_ON_INEXISTENT_TC,
  TC_ACTIVATED,
  TC_DEACTIVATED,
  TC_INEXISTENT,
  TC_UNCONFIGURED,
  TC_DISABLED
} tc_action_response_t;


/*TODO: Há categorias redundantes ou inúteis comparando-se com o enum acima; otimizar */
/** Estados possíveis para um telecomando */
typedef enum {
  STATE_TC_ACTIVATED,
  STATE_TC_DEACTIVATED,
  STATE_TC_INEXISTENT,
  STATE_TC_UNCONFIGURED,
  STATE_TC_DISABLED
} tc_states_t;

/** Configurações possíveis para os telecomandos */
typedef enum {
  TC_MONOSTABLE_ACTIVE_LOW,
  TC_MONOSTABLE_ACTIVE_HIGH,
  TC_BISTABLE_ACTIVE_LOW,
  TC_BISTABLE_ACTIVE_HIGH
} tc_config_t;

/** Estados possíveis para um telessinal */
typedef enum {
  STATE_TS_INEXISTENT,
  STATE_TS_DISABLED,
  STATE_TS_ACTIVE,
  STATE_TS_INACTIVE
} ts_states_t;

/** Configurações possíveis para um telessinal */
typedef enum {
  TS_UNINHIBITED_ALARM_ACTIVE_LOW,
  TS_UNINHIBITED_ALARM_ACTIVE_HIGH,
  TS_INHIBITED_ALARM_ACTIVE_LOW,
  TS_INHIBITED_ALARM_ACTIVE_HIGH
} ts_config_t;

typedef struct
{
  /**
   * Callback da aplicação destinada a enviar um pacote por um canal de comunicação
   * @param packet ponteiro para o pacote a ser enviado
   * @param size tamanho do pacote a ser enviado
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   */
  void (*send_packet)(unsigned char* packet, size_t size, void* user_ptr);

  /**
   * Callback da aplicação destinada a programar ou cancelar um timer para
   * contabilizar o timeout entre fragmentos de pacotes recebidos
   * @param timeout tempo a ser programado no timer ou -1 se o mesmo deve ser cancelado
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   */
  void (*set_inter_fragment_timer)(int16_t timeout, void* user_ptr);

  /**
   * Callback da aplicação destinada a programar ou cancelar um timer para
   * contabilizar o timeout de comunicação
   * @param timeout tempo a ser programado no timer ou -1 se o mesmo deve ser cancelado
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   */
  void (*set_comm_timer)(int16_t timeout, void* user_ptr);

#ifdef CONFIG_MULTIPOINT
  /**
   * Callback da aplicação a ser chamada informando o novo endereço modutr do equipamento
   * @param slave_id novo endereço no protocolo modutr
   */
  void (*on_slave_id_update)(uint32_t slave_id, void* user_ptr);
#endif

  /**
   * Callback da aplicação destinada a acionar um telecomando
   * @param point telecomando a ser acionado
   * @param action ação a ser tomada sobre o telecomando
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   * @return resposta de operação do telecomando
   * @note a resposta de operação do telecomando deve ser enviada através do
   * próprio valor de retorno da callback, devendo ser uma dentre as especificadas
   * no 'enum' tc_action_response_t
   */
  tc_action_response_t (*on_telecommand_action)(uint8_t point, unsigned char action,
                                                void* user_ptr);

  /**
   * Callback da aplicação destinada a ler os estados dos telecomandos
   * @param initial_point ponto inicial
   * @param num_points número de pontos a serem lidos
   * @param point_value_array ponteiro para o buffer a ser preenchido com os estados
   * dos telecomandos especificados (em sequência ascendente)
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   * @note os estados lidos devem ser reportados no vetor 'point_value_array', sendo o
   * pacote de resposta automaticamente enviado assim que a callback retornar
   */
  void (*on_telecommand_state_read)(uint8_t initial_point, uint8_t num_points,
                                    unsigned char* point_value_array, void* user_ptr);

  /**
   * Callback da aplicação destinada a configurar um telecomando
   * @param point telecomando a ser configurado
   * @param config configuração do telecomando
   * @param time_ms tempo de atuação do telecomando em ms (para tipo monoestável)
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   * @note se o telecomando for do tipo biestável o dado de tempo é irrelevante
   * para a aplicação, podendo ser ignorado
   */
  void (*on_telecommand_config)(uint8_t point, tc_config_t config, uint16_t time_ms,
                                void* user_ptr);

  /**
   * Callback da aplicação destinada a ler a configuração de um telecomando
   * @param point telecomando cuja configuração deve ser retornada
   * @param config configuração do telecomando
   * @param time_ms tempo de atuação do telecomando em ms (para tipo monoestável)
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   * @return configuração do telecomando
   */
  void (*on_telecommand_config_read)(uint8_t point, tc_config_t* config, uint16_t* time_ms,
                                     void* user_ptr);

  /**
   * Callback da aplicação destinada a ler o estado de um ou mais telessinais
   * @param initial_point ponto inicial
   * @param num_points número de pontos a serem lidos
   * @param point_value_array ponteiro para o buffer a ser preenchido com os estados
   * dos telessinais especificados (em sequência ascendente)
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   * @return configuração do telecomando
   */
  void (*on_telesignal_state_read)(uint8_t initial_point, uint8_t num_points,
                                   unsigned char* point_value_array, void* user_ptr);

  /**
   * Callback da aplicação destinada a configurar um telessinal
   * @param point telessinal a ser configurado
   * @param config configuração do telessinal
   * @param debounce_ms tempo para confirmação do estado do telessinal após mudança
   * de estado da entrada digital associada (em ms)
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   */
  void (*on_telesignal_config)(uint8_t point, ts_config_t config, uint16_t debounce_ms,
                               void* user_ptr);

  /**
   * Callback da aplicação destinada a ler a configuração de um telessinal
   * @param point telessinal cuja configuração deve ser retornada
   * @param config configuração do telessinal
   * @param debounce_ms tempo de atuação do telecomando em ms (para tipo monoestável)
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   */
  void (*on_telesignal_config_read)(uint8_t point, ts_config_t* config,
                                    uint16_t* debounce_ms, void* user_ptr);

  /**
   * Callback da aplicação destinada a ler o valor de uma ou mais telemedidas
   * @param initial_point ponto inicial
   * @param num_points número de pontos a serem lidos
   * @param point_value_array ponteiro para o buffer a ser preenchido com os valores
   * das telemedidas especificadas (em sequência ascendente)
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   * @return configuração do telecomando
   */
  void (*on_telemeasurement_read)(uint8_t initial_point, uint8_t num_points,
                                  float* point_value_array, void* user_ptr);

  /**
   * Callback da aplicação destinada a setar o valor do RTC lido do
   * dispositivo slave
   * @param rtc_high parte alta do RTC
   * @param rtc_low parte baixa do RTC
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   * @note o tempo é contabilizado como milisegundos desde 01/01/1970
   */
  void (*on_set_rtc)(const timestamp_t* timestamp, void* user_ptr);
  
  /**
   * Callback da aplicação destinada a informar o valor do RTC lido do
   * dispositivo slave
   * @param rtc_high parte alta do RTC
   * @param rtc_low parte baixa do RTC
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   * @note o tempo é contabilizado como milisegundos desde 01/01/1970
   */
  void (*on_read_rtc)(timestamp_t* timestamp, void* user_ptr);

  /**
   * Callback da aplicação destinada a informar a versão de firmware do equipamento
   * @param ver versão de firmware
   * @param rev revisão da versão de firmware
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   * @note a aplicação deve retornar a revisão e revisão do firmware através dos
   * ponteiros fornecidos
   */
  void (*on_read_fw_version)(uint8_t* ver, uint8_t* rev, void* user_ptr);

  /**
   * Callback da aplicação destinada a preparar o equipamento para recebimento
   * de novo firmware
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   * @note a ação a ser tomada depende do dispositivo, pode ser criar um arquivo,
   * apagar uma memória flash, etc.
   */
  void (*on_prepare_fw_upgrade)(void* user_ptr);

  /**
   * Callback da aplicação destinada a salvar um bloco de upgrade de firmware
   * @param block_number índice do bloco
   * @param total_blocks número total de blocos no arquivo
   * @param block ponteiro para o buffer com o bloco de upgrade de firmware
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   */
  void (*on_recv_fw_block)(uint16_t block_number, uint16_t total_blocks,
                           unsigned char* block, void* user_ptr);

  /**
   * Callback da aplicação destinada a efetuar o reset do equipamento
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   */
  void (*on_reset_request)(void* user_ptr);

  /**
   * Callback da aplicação destinada a retornar eventos ainda não enviados ao mestre
   * @param num_events número de eventos a serem enviados ao mestre
   * @param events buffer de eventos a serem enviados
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   */
  void (*on_events_request)(uint16_t* events_total, event_t* events, void* user_ptr);

  /**
   * Callback para informar à aplicação o status do link de comunicação
   * @param comm_failure flag indicando falha de comunicação
   * @param user_ptr Ponteiro para dado genérico (p/ identificar instância, por ex.)
   */
  void (*on_comm_link_status_change)(uint8_t comm_failure, void* user_ptr);
} modutr_callbacks_t;

typedef struct
{
  /** Array de callbacks do protocolo modutr */
  modutr_callbacks_t callbacks;
  /** Número de telessinais presentes no slave */
  uint8_t num_ts;
  /** Número de telecomandos presentes no slave */
  uint8_t num_tc;
  /** Número de telemedidas presentes no slave */
  uint8_t num_tm;
  /** Timeout para recebimento de uma resposta válida a uma requisição modutr */
  uint16_t comm_failure_timeout;
  /** Timeout para recebimento de um novo fragmento de pacote modutr */
  uint32_t inter_fragment_timeout;
  /** Flag para indicar falha de comunicação com o slave */
  unsigned char comm_failure;
#ifdef CONFIG_MULTIPOINT
  uint32_t slave_id;
#endif
  /** Ponteiro para dado genérico (p/ identificar instância, por ex.) */
  void *user_ptr;
} modutr_t;

typedef struct
{
  modutr_rx_state_t rx_state;
  uint16_t rx_status;
  unsigned char idp;
  uint16_t len;
#ifdef CONFIG_MULTIPOINT
  uint32_t UCDidTmp;
#endif
  uint16_t accum_crc;
  uint32_t dropped_fragment_count;  /**< Número de fragmentos descartados devido ao buffer estar cheio */
  uint32_t valid_packet_count;      /**< Número de pacotes modutr recebidos e válidos */
  uint32_t invalid_packet_count;    /**< Número de pacotes descartados por serem inválidos */
#ifdef CONFIG_MULTIPOINT
  uint32_t invalid_nmn_count;       /**< Número de pacotes com NMN incorreto */
  uint32_t invalid_address;         /**< Número de pacotes com endereço incorreto */
#endif
  uint32_t invalid_idp;             /**< Número de pacotes inválidos pelo IDP recebido não casar com o da requisição */
  uint32_t invalid_size_count;      /**< Número de pacotes inválidos por tamanho maior que o buffer */
  uint32_t invalid_timed_out_frag_count;/**< Número de pacotes descartados por timeout de fragmento */
#ifdef CONFIG_MULTIPOINT
  uint32_t invalid_end_count;           /**< Número de pacotes inválidos por ETX incorreto */
#endif
  uint32_t invalid_crc_count;           /**< Número de pacotes inválidos por CRC incorreto */
  uint16_t buffsize;                    /**< Tamanho do buffer de recebimento */
  ringbuffer ring_buffer;               /**< Ring buffer de pacotes recebidos */
  unsigned char current_byte;           /**< Ponteiro para o byte do pacote recebido sendo processado */
  unsigned char* app_buffer;            /**< Buffer para pacote de camada de aplicação */
  unsigned char* app_buffer_ptr;        /**< Ponteiro para o byte atual da camada de aplicação */
} modutr_rx_t;

typedef struct
{
  unsigned char idp;
  uint16_t len;
  uint16_t buffsize;
  unsigned char* buffer;        /**< Buffer para pacote completo a ser transmitido */
  unsigned char* byte_ptr;      /**< Ponteiro para o byte corrente no pacote a ser transmitido */
  unsigned char* body_byte_ptr; /**< Ponteiro para o byte corrente no corpo (BODY) do pacote */
} modutr_tx_t;

#endif
