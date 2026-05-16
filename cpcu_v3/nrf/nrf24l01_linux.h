/**
 *  @file   nrf24l01_linux.h
 *  @brief  NRF24L01+ Linux SPI driver API — handle, init, receive, power control.
 */

#ifndef NRF24L01_LINUX_H
#define NRF24L01_LINUX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*============= CONFIGURATION CONSTANTS ======================*/

#define NRF_ADDR_WIDTH          5
#define NRF_PAYLOAD_SIZE        32
#define NRF_TX_TIMEOUT_MS       20

/*============= SPI COMMANDS =================================*/

#define NRF_CMD_R_REGISTER      0x00
#define NRF_CMD_W_REGISTER      0x20
#define NRF_CMD_R_RX_PAYLOAD    0x61
#define NRF_CMD_W_TX_PAYLOAD    0xA0
#define NRF_CMD_FLUSH_TX        0xE1
#define NRF_CMD_FLUSH_RX        0xE0
#define NRF_CMD_NOP             0xFF

/*============= REGISTER MAP =================================*/

#define NRF_REG_CONFIG          0x00
#define NRF_REG_EN_AA           0x01
#define NRF_REG_EN_ADDR         0x02
#define NRF_REG_SETUP_AW        0x03
#define NRF_REG_SETUP_RETR      0x04
#define NRF_REG_RF_CH           0x05
#define NRF_REG_RF_SETUP        0x06
#define NRF_REG_STATUS          0x07
#define NRF_REG_OBSERVE_TX      0x08
#define NRF_REG_RPD             0x09
#define NRF_REG_RX_ADDR_P0      0x0A
#define NRF_REG_TX_ADDR         0x10
#define NRF_REG_RX_PW_P0        0x11
#define NRF_REG_FIFO_STATUS     0x17
#define NRF_REG_FEATURE         0x1D

/*============= STATUS BITS ==================================*/

#define NRF_STATUS_RX_DR            (1 << 6)
#define NRF_STATUS_TX_DS            (1 << 5)
#define NRF_STATUS_MAX_RT           (1 << 4)
#define NRF_STATUS_RX_P_NO_MASK     0x0E
#define NRF_STATUS_RX_FIFO_EMPTY    0x0E

/*============= CONFIG BITS ==================================*/

#define NRF_CONFIG_MASK_RX_DR       (1 << 6)
#define NRF_CONFIG_MASK_TX_DS       (1 << 5)
#define NRF_CONFIG_MASK_MAX_RT      (1 << 4)
#define NRF_CONFIG_EN_CRC           (1 << 3)
#define NRF_CONFIG_CRCO             (1 << 2)
#define NRF_CONFIG_PWR_UP           (1 << 1)
#define NRF_CONFIG_PRIM_RX          (1 << 0)

/*============= RF_SETUP BITS ================================*/

#define NRF_RF_DR_HIGH          (1 << 3)
#define NRF_RF_PWR_0dBm         (0x03 << 1)
#define NRF_RF_LNA_HCURR        (1 << 0)

/*============= STATUS RETURN ================================*/

typedef enum
{
    NRF_OK = 0,
    NRF_ERR_SPI,
    NRF_ERR_GPIO,
    NRF_ERR_NOT_DETECTED,
    NRF_ERR_RX_EMPTY,
    NRF_ERR_TX_TIMEOUT,
    NRF_ERR_TX_MAX_RT,
} NRF_Status;

/*============= DRIVER HANDLE ================================*/

typedef struct
{
    int         spi_fd;                 /* /dev/spidev0.0 */
    int         ce_chip_fd;             /* /dev/gpiochipN */
    int         ce_line_fd;             /* gpiod line request for CE */
    uint32_t    spi_speed;              /* SPI clock Hz */
    uint8_t     channel;                /* RF Channel 0-125 */
    uint8_t     addr[NRF_ADDR_WIDTH];   /* RX/TX Address */
    uint8_t     last_status;            /* Last Status register read */
} NRF_Handle;

/*============= API ==========================================*/

NRF_Status  NRF_Init(NRF_Handle *h, 
                const char *spi_dev, uint32_t spi_speed,
                int gpio_chip_fd, uint32_t ce_gpio,
                uint8_t channel, const uint8_t addr[NRF_ADDR_WIDTH]);

void        NRF_Close(NRF_Handle *h);

/* Register Access */
uint8_t     NRF_ReadReg(NRF_Handle *h, uint8_t reg);
void        NRF_WriteReg(NRF_Handle *h, uint8_t reg, uint8_t val);
void        NRF_ReadRegMulti(NRF_Handle *h, uint8_t reg, uint8_t *buf, uint8_t len);
void        NRF_WriteRegMulti(NRF_Handle *h, uint8_t reg, const uint8_t *buf, uint8_t len);

/* RX Operations */
bool        NRF_DataAvailable(NRF_Handle *h);
NRF_Status  NRF_ReadPayload(NRF_Handle *h, uint8_t *buf);

/* Utilities */
uint8_t     NRF_GetStatus(NRF_Handle *h);
void        NRF_ClearIRQ(NRF_Handle *h, uint8_t flags);
void        NRF_FlushRX(NRF_Handle *h);
void        NRF_FlushTX(NRF_Handle *h);
void        NRF_PowerDown(NRF_Handle *h);

/* CE Pin Control */
void        NRF_CE_High(NRF_Handle *h);
void        NRF_CE_Low(NRF_Handle *h);

#ifdef __cplusplus
}
#endif

#endif /* NRF24L01_LINUX_H */

