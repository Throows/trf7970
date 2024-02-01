#ifndef TRF79XXA_H
#define TRF79XXA_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TRF_SUCCESS                         0
#define TRF_ERROR                           -1
#define TRF_TIMEOUT                         -2
#define TRF_INVALID_PARAM                   -3
typedef char trf79xxa_error_t;

#define	TRF79xxA_VERSION	                70

/* -- Address and command Format                       
                Address       ; Command
B7    :            0          ;    1
B6    :    0: Read - 1: Write ;    0
B5    :    1: Continuous Mode ;    0
B4-B0 :         Address       ; Command
*/

// -- Commands --       
#define CMD_TRF_IDLE                        0x00
#define CMD_TRF_INIT                        0x03
#define CMD_TRF_RF_AVOIDANCE                0x04
#define CMD_TRF_RF_AVOIDANCE_RESP           0x05
#define CMD_TRF_RF_AVOIDANCE_RESP1          0x06
#define CMD_TRF_RESET_FIFO                  0x0F
#define CMD_TRF_TRANSMISSION_NO_CRC         0x10
#define CMD_TRF_TRANSMISSION_CRC            0x11
#define CMD_TRF_TRANSMISSION_NO_CRC_DELAY   0x12
#define CMD_TRF_TRANSMISSION_CRC_DELAY      0x13
#define CMD_TRF_EOF_NEXT_TRANSMIT           0x14
#define CMD_TRF_BLOCK_RECEIVER              0x16
#define CMD_TRF_ENABLE_RECEIVER             0x17
#define CMD_TRF_INTERNAL_RF                 0x18
#define CMD_TRF_EXTERNAL_RF                 0x19

// -- Registers --
#define REG_CHIP_STATUS                     0x00
#define REG_ISO_CONTROL                     0x01
#define REG_14443B_TX_OPTIONS               0x02
#define REG_14443A_BIT_RATE                 0x03
#define REG_TX_TIMER_HB_CONTROL             0x04
#define REG_TX_TIMER_LB_CONTROL             0x05
#define REG_TX_PULSE_LENGTH_CONTROL         0x06
#define REG_RX_NO_RESPONSE_TIME             0x07
#define REG_RX_RESPONSE_TIME                0x08
#define REG_MODULATOR_SYS_CONTROL           0x09
#define REG_RX_SETTINGS                     0x0A
#define REG_IO_CONTROL                      0x0B
#define REG_SPECIAL_FUNC_ONE                0x10
#define REG_SPECIAL_FUNC_TWO                0x11
#define REG_FIFO_IRQ_LEVEL                  0x14
#define REG_NFC_LOW_FIELD_LEVEL             0x16
#define REG_NFC_ID                          0x17
#define REG_NFC_TARGET_DETECTION_LEVEL      0x18
#define REG_NFC_TARGET_PROTOCOL             0x19
#define REG_IRQ_STATUS                      0x0C
#define REG_COLLISION_INTERRUPT_STATUS      0x0D
#define REG_COLLISION_POSITION              0x0E
#define REG_RSSI_OSC_STATUS                 0x0F
// Skip test Ram registers
#define REG_FIFO_STATUS                     0x1C
#define REG_TX_LEN_BYTE_1                   0x1D
#define REG_TX_LEN_BYTE_2                   0x1E
#define REG_FIFO_IO_REGISTER                0x1F

// -- TRF device descriptor --
#define TRF_PROTOCOL_14443A                 0x01
#define TRF_PROTOCOL_14443A_NDEF            0x02
#define TRF_PROTOCOL_14443B                 0x04
#define TRF_PROTOCOL_15693                  0x08
#define TRF_PROTOCOL_FELICIA                0x10
#define TRF_PROTOCOL_ALL                    (TRF_PROTOCOL_14443A | TRF_PROTOCOL_14443A_NDEF | TRF_PROTOCOL_14443B | TRF_PROTOCOL_15693 | TRF_PROTOCOL_FELICIA)        
typedef char trf79xxa_protocols_t;  // OR of above

typedef void (*trf79xxa_write_byte_cb)(uint8_t data, void* user_data);
typedef void (*trf79xxa_read_byte_cb)(uint8_t* data, void* user_data);

typedef enum {
    TRF_3V_FULL_POWER = 0x00,
    TRF_3V_HALF_POWER = 0x10,
    TRF_5V_FULL_POWER = 0x01,
    TRF_5V_HALF_POWER = 0x11,
} trf79xxa_power_t;

typedef enum {
    STATE_IDLE = 0,
    STATE_RX,
} trf79xxa_state_t;

typedef struct {
    trf79xxa_state_t state;
    trf79xxa_protocols_t protocols;
    trf79xxa_power_t power;
    uint8_t iso_control;
    trf79xxa_write_byte_cb write_cb;
    trf79xxa_read_byte_cb read_cb;
    void* user_data;                    // Some pointer to the SPI device ? Null by default
} trf79xxa_dev_t;


trf79xxa_error_t trf79xxa_init(trf79xxa_dev_t* dev, trf79xxa_power_t power, trf79xxa_protocols_t protocols, trf79xxa_write_byte_cb write_cb, trf79xxa_read_byte_cb read_cb);
trf79xxa_error_t trf79xxa_reset(trf79xxa_dev_t* dev);
trf79xxa_error_t trf79xxa_reset_fifo(trf79xxa_dev_t* dev);
trf79xxa_error_t trf79xxa_set_rf(trf79xxa_dev_t* dev, bool status);

trf79xxa_error_t trf79xxa_read_register(trf79xxa_dev_t* dev, uint8_t reg, uint8_t* value, uint8_t length);
trf79xxa_error_t trf79xxa_write_register(trf79xxa_dev_t* dev, uint8_t reg, uint8_t* value, uint8_t length);
trf79xxa_error_t trf79xxa_send_cmd(trf79xxa_dev_t* dev, uint8_t cmd);



#ifdef __cplusplus
}
#endif

#endif /* TRF79XXA_H */