/**
 *  @file       nrf24l01.h
 *  @brief      Unified NRF24L01+ driver for STM32 HAL — TX and RX roles
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.1
 *  @details
 *              Single driver serving both sides of the InfiniTech wireless link:
 *
 *                  BSAU (STM32L432KC) — Transmitter:   #define NRF_ROLE_TX
 *                  CPCU (STM32H755ZI) — Receiver:      #define NRF_ROLE_RX
 *
 *              Exactly one role must be defined BEFORE including this header.
 *              Common code (SPI access, pin control, register map, init)
 *              is always compiled; TX-only and RX-only sections use #if
 *              to keep flash usage tight on resource-constrained parts.
 *
 *              v2.1 changes:
 *                  - RF_SETUP: 250 kbps → 2 Mbps (RF_DR_HIGH instead of RF_DR_LOW)
 *                  - SETUP_RETR: ARD 1500 µs → 500 µs (0x5F → 0x1F)
 *                  - TX timeout: 75 ms → 20 ms (worst case at 2 Mbps = 12.1 ms)
 *                  - CE pulse in NRF_Transmit now uses DWT->CYCCNT for a
 *                    calibrated µs delay (was a volatile-loop ~5-7 µs, below
 *                    datasheet minimum Tpece2csn ≥ 10 µs)
 */

#ifndef NRF24L01_H
#define NRF24L01_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*============= ROLE VALIDATION ================================================================*/
/*
 *  Exactly one of NRF_ROLE_TX or NRF_ROLE_RX must be defined BEFORE
 *  including this header. If neither or both are defined, compilation
 *  fails with a descriptive error.
 */

#if defined(NRF_ROLE_TX) && defined(NRF_ROLE_RX)
    #error "Define exactly one of NRF_ROLE_TX or NRF_ROLE_RX, not both."
#endif

#if !defined(NRF_ROLE_TX) && !defined(NRF_ROLE_RX)
    #error "Define exactly one of NRF_ROLE_TX or NRF_ROLE_RX."
#endif

/*============= HAL INCLUDE ====================================================================*/

#include "main.h"

/*============= DRIVER CONFIGURATION ===========================================================*/

#define NRF_ADDR_WIDTH              5               /* 3, 4, or 5 bytes */
#define NRF_PAYLOAD_SIZE            32              /* Fixed payload: 1-32 bytes */
#define NRF_SPI_TIMEOUT_MS          15              /* Per-transaction timeout */

/*============= SPI COMMANDS ===================================================================*/

#define NRF_CMD_R_REGISTER          0x00            /* OR with 5-bit register address */
#define NRF_CMD_W_REGISTER          0x20            /* OR with 5-bit register address */
#define NRF_CMD_R_RX_PAYLOAD        0x61            /* Read RX payload */
#define NRF_CMD_W_TX_PAYLOAD        0xA0            /* Write TX payload */
#define NRF_CMD_FLUSH_TX            0xE1
#define NRF_CMD_FLUSH_RX            0xE0
#define NRF_CMD_REUSE_TX_PL         0xE3
#define NRF_CMD_NOP                 0xFF

/*============= REGISTER MAP ===================================================================*/

#define NRF_REG_CONFIG              0x00            /* Configuration Register */
#define NRF_REG_EN_AA               0x01            /* Auto-acknowledgment */
#define NRF_REG_EN_RXADDR           0x02            /* Enabled RX pipes */
#define NRF_REG_SETUP_AW            0x03            /* Address width */
#define NRF_REG_SETUP_RETR          0x04            /* Auto-retransmit */
#define NRF_REG_RF_CH               0x05            /* RF channel (0-125) */
#define NRF_REG_RF_SETUP            0x06            /* Data rate, TX power */
#define NRF_REG_STATUS              0x07            /* Status Register */
#define NRF_REG_OBSERVE_TX          0x08            /* TX-side: lost/retransmit counters */
#define NRF_REG_RX_ADDR_P0          0x0A            /* Receive address data pipe 0 (5 B) */
#define NRF_REG_RX_ADDR_P1          0x0B            /* Receive address data pipe 1 (5 B) */
#define NRF_REG_TX_ADDR             0x10            /* Transmit address (5 B) */
#define NRF_REG_RX_PW_P0            0x11            /* RX payload width pipe 0 */
#define NRF_REG_RX_PW_P1            0x12            /* RX payload width pipe 1 */
#define NRF_REG_FIFO_STATUS         0x17            /* FIFO Status Register */
#define NRF_REG_DYNPD               0x1C            /* Dynamic payload length */
#define NRF_REG_FEATURE             0x1D            /* Feature Register */

/*============= STATUS REGISTER BIT MASKS ======================================================*/

#define NRF_STATUS_RX_DR            (1 << 6)        /* Data ready in RX FIFO */
#define NRF_STATUS_TX_DS            (1 << 5)        /* TX data sent */
#define NRF_STATUS_MAX_RT           (1 << 4)        /* Max retransmits reached */
#define NRF_STATUS_RX_P_NO_MASK     0x0E            /* Pipe number for available payload */
#define NRF_STATUS_RX_FIFO_EMPTY    0x0E            /* When RX_P_NO = 0b111 */

/*============= FIFO_STATUS REGISTER BIT MASKS =================================================*/

#define NRF_FIFO_TX_FULL            (1 << 5)
#define NRF_FIFO_TX_EMPTY           (1 << 4)

/*============= CONFIG REGISTER BIT MASKS ======================================================*/

#define NRF_CONFIG_MASK_RX_DR       (1 << 6)        /* Mask RX_DR on IRQ pin */
#define NRF_CONFIG_MASK_TX_DS       (1 << 5)        /* Mask TX_DS on IRQ pin */
#define NRF_CONFIG_MASK_MAX_RT      (1 << 4)        /* Mask MAX_RT on IRQ pin */
#define NRF_CONFIG_EN_CRC           (1 << 3)        /* 0=Disable CRC, 1=Enable CRC */
#define NRF_CONFIG_CRCO             (1 << 2)        /* 0=1-byte CRC, 1=2-byte CRC */
#define NRF_CONFIG_PWR_UP           (1 << 1)        /* 0=Power down, 1=Power up */
#define NRF_CONFIG_PRIM_RX          (1 << 0)        /* 1=RX mode, 0=TX mode */

/*============= RF_SETUP REGISTER BIT MASKS ====================================================*/

#define NRF_RF_SETUP_RF_DR_HIGH     (1 << 3)        /* 0=1 Mbps, 1=2 Mbps */
#define NRF_RF_SETUP_RF_DR_LOW      (1 << 5)        /* 1=250 kbps (with DR_HIGH=0) */
#define NRF_RF_SETUP_RF_PWR_0       (0x00 << 1)     /* -18 dBm */
#define NRF_RF_SETUP_RF_PWR_1       (0x01 << 1)     /* -12 dBm */
#define NRF_RF_SETUP_RF_PWR_2       (0x02 << 1)     /*  -6 dBm */
#define NRF_RF_SETUP_RF_PWR_3       (0x03 << 1)     /*   0 dBm (max) */

/*============= DRIVER TYPES ===================================================================*/

typedef enum
{
    NRF_OK                  =   0,
    NRF_ERR_SPI,                                    /* HAL SPI call failed */
    NRF_ERR_NOT_DETECTED,                           /* Register readback mismatch */
    NRF_ERR_RX_EMPTY,                               /* No payload available */
    NRF_ERR_TX_FIFO_FULL,
    NRF_ERR_TX_MAX_RT,                              /* Max retransmits reached */
    NRF_ERR_TX_TIMEOUT
} NRF_Status;

typedef struct
{
    SPI_HandleTypeDef   *hspi;
    uint8_t              channel;                   /* RF channel 0-125 */
    uint8_t              rx_addr[NRF_ADDR_WIDTH];   /* This node's address */
    uint8_t              last_status;               /* Last STATUS register read */
} NRF_Handle;

/*============= PIN CONTROL (inline helpers) ===================================================*/

static inline void  csn_low   (void) { HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET); }
static inline void  csn_high  (void) { HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);   }
static inline void  ce_low    (void) { HAL_GPIO_WritePin(NRF_CE_GPIO_Port,  NRF_CE_Pin,  GPIO_PIN_RESET); }
static inline void  ce_high   (void) { HAL_GPIO_WritePin(NRF_CE_GPIO_Port,  NRF_CE_Pin,  GPIO_PIN_SET);   }

/*============= COMMON API =====================================================================*/

NRF_Status  NRF_Init            (NRF_Handle *hnrf, SPI_HandleTypeDef *hspi,
                                 uint8_t channel, const uint8_t addr[NRF_ADDR_WIDTH]);
uint8_t     NRF_GetStatus       (NRF_Handle *hnrf);
void        NRF_ClearIRQ        (NRF_Handle *hnrf, uint8_t flags);
void        NRF_FlushRX         (NRF_Handle *hnrf);
void        NRF_FlushTX         (NRF_Handle *hnrf);
void        NRF_PowerDown       (NRF_Handle *hnrf);
void        NRF_PowerUp         (NRF_Handle *hnrf);

/*-------------- Low-level register access (for debugging) -------------------------------------*/

uint8_t     NRF_ReadReg         (NRF_Handle *hnrf, uint8_t reg);
void        NRF_WriteReg        (NRF_Handle *hnrf, uint8_t reg, uint8_t val);
void        NRF_ReadRegMulti    (NRF_Handle *hnrf, uint8_t reg, uint8_t *buf, uint8_t len);
void        NRF_WriteRegMulti   (NRF_Handle *hnrf, uint8_t reg, const uint8_t *buf, uint8_t len);

/*============= RX-ONLY API ====================================================================*/

#if defined(NRF_ROLE_RX)

bool        NRF_DataAvailable   (NRF_Handle *hnrf);
NRF_Status  NRF_ReadPayload     (NRF_Handle *hnrf, uint8_t *buf);

#endif  /* NRF_ROLE_RX */

/*============= TX-ONLY API ====================================================================*/

#if defined(NRF_ROLE_TX)

NRF_Status  NRF_Transmit        (NRF_Handle *hnrf, const uint8_t *data);
NRF_Status  NRF_TransmitNoBlock (NRF_Handle *hnrf, const uint8_t *data);
NRF_Status  NRF_TransmitPoll    (NRF_Handle *hnrf);
void        NRF_GetTxStats      (NRF_Handle *hnrf, uint8_t *lost_pkts, uint8_t *retx_pkts);

#endif  /* NRF_ROLE_TX */

/*==============================================================================================*/

#ifdef __cplusplus
}
#endif

#endif /* NRF24L01_H */
