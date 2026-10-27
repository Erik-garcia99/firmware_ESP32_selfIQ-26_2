/* *********************************************
 *              selfIQ 2026-2
 *
 * @autor Erik Garcia Chavez
 * @date octuber 2026
 * @assginature : proyecto de carrera
 * @teacher : Jose Isabel Garcia Rocha
 *
 ************************************************* */

#ifndef GLOBAL_H
#define GLOBAL_H

// macros
#include <esp_err.h>
#include <stdint.h>
#define TRUE 1
#define FALSE 0

#define HEADER 0xABCD
#define ACK 0x5433
#define MAX_DATA 100 // el tamanio maximo que aceptara el protocolo en data.
#define SSID_LEN_MAX 32
#define PSWD_LEN_MAX 63
#define PSWD_LEN_MIN 8
#define TEXT_TERMINATOR_SIZE 1U
#define IPV4_TEXT_SIZE 16U /* 15 caracteres IPv4 + terminador */

#define UUID_TEXT_LEN 36U
#define UUID_TEXT_SIZE (UUID_TEXT_LEN + TEXT_TERMINATOR_SIZE)
#define UUID_DASH_1 8U
#define UUID_DASH_2 13U
#define UUID_DASH_3 18U
#define UUID_DASH_4 23U


#define CRC32_BITS_PER_BYTE 8U
#define CRC32_POLYNOMIAL 0xEDB88320U
#define CRC32_INITIAL_VALUE 0xFFFFFFFFU
#define CRC32_FINAL_XOR 0xFFFFFFFFU
#define CRC32_FORMAT_MARKER UINT64_C(0x53495101)
#define CRC32_FORMAT_SHIFT 32U
#define CRC32_LEGACY_VALUE_LIMIT UINT64_C(0x100000000)
#define CRC32_PORT_HIGH_SHIFT 8U
#define CRC32_PORT_LOW_MASK 0xFFU

#define MAIN_LOOP_DELAY_MS 1000U


// namespace propio en NVS donde guardamos nuestros datos y sus checksums
#define NVS_NS "selfiq_cfg"
#define KEY_WIFI_SUM "wifi_sum" // checksum de SSID + pswd
#define KEY_WIFI_SSID "wifi_ssid"
#define KEY_WIFI_PSWD "wifi_pswd"
#define KEY_TCP_IP "tcp_ip"
#define KEY_TCP_PORT "tcp_port"
#define KEY_TCP_SUM "tcp_sum" // checksum de IP + PORT

// sera el tipo que operacion que ser, mantenemos ACK y NACK para mas
// infromacion sobre si llegaron las peticiones o no
// interno
typedef enum {
  // L:S
  OP_ACK = 0,    // ACK
  OP_NACK,       // NACK
  OPReqCredWifi, // la ESP quiere solcitar las credenicales WIFI del lugar
  OPReqBroker, // la ESP quiere solciitar la ip del broker mosquitto para MQTT.
} TYPE_OP_TYPE_T;
extern TYPE_OP_TYPE_T eOp_type;

// esta informacion va en el frame.
//  accion que queremos haacer entre la ESP32 y la RPI, por el momento solo se
//  tiene para hacer logi= 0x0Fn
typedef enum {
  action_none = 0x00,
  eReqWifi, // soliciar credeniclaes WIFI a la RPI.
  eRespWifi,
  eReqBroker,
  eRespBroker,
  keepAlive,
} ACTION_T;

extern ACTION_T eAction;

typedef enum {
  eNoType = 0x00,
  eWifi,
  eBroker,
  eRespACK = 0x0F,
} TYPE_FRAME_T;
extern TYPE_FRAME_T eType_frame;

// este enum funciona para la estrucutra interna de data, diferenciar y
// identifizar que trozo de la aprte de data correcponde a que
typedef enum {
  eDataSSID = 0x69, // para las 2 tipo de redes sera este mimso id para inidcar
                    // el inicio
  eDataPSWD = 0x70, // igual
  eDataBrokerIP = 0x76,   // IPv4: cuatro bytes binarios
  eDataBrokerPort = 0x77, // uint16 en orden de red
} DATA_FIELD_T;

extern DATA_FIELD_T eData_field_t;

typedef enum {
  eWIFI_CRED,
  eTCP_CRED,
} CRED_UPDATE_T;
extern CRED_UPDATE_T eCredSave;

/*************************************************************
 * @brief establece la estrucutra del frame entre peticiones y respuesta de ESP
 * con rasbery pi. para peticiones de credenicales de WIFI y otros proceso que
 * se pueda emplear.
 *
 * @member header: cabecera del frame indicando que se esta enviando,
 *    --> lo cuales puede ser:
 *          Peticion : 0x0
 *          Error (NACK) : 0xCAFE
 * @member len : bytes restantes: TYPE/ACTION + DATA; excluye HEADER y LEN
 * @member action : que accion queremos hacer con la rpi cunado esta en mood AP
 *    --> puede ser, del enum < action >
 * @member type_frame :
 * @member type_red : si la red es privada o de empresa
 *   --> para empresa se requiere SSID, user y pswd del usuario
 *   --> para red privada solo SSID y pswd de la red
 * @member ssid : nombre de la red
 * @member pswd : contrasenia de la red o contrasenia de la red
 * @member user_network: usuario de la empresa
 *
 *********************************************************/

typedef struct {
  uint16_t header; // >> head of frame
  uint8_t len;     // >> lenght of frame
  TYPE_FRAME_T type_frame : 4;
  ACTION_T action : 4;
  uint8_t data[MAX_DATA];
} TYPE_FORMAT_REQUEST_T;

extern TYPE_FORMAT_REQUEST_T format_reques;

typedef struct {
  TYPE_OP_TYPE_T op_type;               // que operacion vamos a relaizar
  TYPE_FORMAT_REQUEST_T format_request; // trametos la trama a enviar
} TYPE_SEND_INFO_T;

extern TYPE_SEND_INFO_T send_info;

// en el caso de estar en modo OFFLINE durantre el periodo en el cual no se esta
// envinado datos dado que se intenta conectar. entonces crearemos una lista
// enlazada como un buffer para guardar los datos cuando ya nos conectemeos
// entonces empezamos a mandar los datos de la lista enlazada. --> COLA, tiene
// que ser una COLA enlazada.

// funcion global
//  calve - valor - SSID - PWSD / IP-PORT
uint64_t checksum(const char *key, const void *anchor, uint8_t identificator);

// funciones que usare para guardar crecnuales en flash, (WIFI lo hace cons us
// propias funciones)
esp_err_t save_credentials(uint8_t identifier);
esp_err_t load_cred(uint8_t identifier);
esp_err_t erease_cred_nvs(uint8_t identifier);

#endif
