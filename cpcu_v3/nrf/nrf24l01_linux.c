/**
 *  @file   nrf24l01_linux.c
 *  @brief  NRF24L01+ Linux SPI driver — init, receive, status, power control.
 *
 *  Operates the NRF24L01+ in PRX (receive) mode via /dev/spidevX.Y.
 *  GPIO CE pin is managed through the Linux GPIO character device.
 *  Provides busy-poll packet reception, FIFO flush, IRQ clear, and
 *  power-down for clean shutdown and recovery.
 */

#include "nrf24l01_linux.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

/*============= INTERNAL: Timing =====================================================*/

static void delay_ms(uint32_t ms)
{
    struct timespec ts  =   { .tv_sec = ms / 1000, .tv_nsec = (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* Reserved for future register-level timing tweaks (e.g. powerup wait,
 * PA settle). Currently unused — NRF24L01+ datasheet timings are all
 * comfortably > 1ms so delay_ms suffices. Kept rather than deleted
 * because the moment you need µs-resolution waits, you'll want this
 * exact wrapper back. */
__attribute__((unused))
static void delay_us(uint32_t us)
{
    struct timespec ts  =   { .tv_sec = 0, .tv_nsec = us * 1000L };
    nanosleep(&ts, NULL); 
}

/*============= INTERNAL: SPI Transfer ===============================================*/

static int spi_transfer(NRF_Handle *h, const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
{
    struct spi_ioc_transfer xfer;
    memset(&xfer, 0, sizeof(xfer));

    xfer.tx_buf         =   (unsigned long)tx_buf;
    xfer.rx_buf         =   (unsigned long)rx_buf;
    xfer.len            =   len;
    xfer.speed_hz       =   h->spi_speed;
    xfer.bits_per_word  =   8;

    if( ioctl(h->spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0 )
    {
        return -1;
    }

    return 0;
}

/*============= INTERNAL: GPIO =======================================================*/

static int gpio_request_output(int chip_fd, uint32_t offset, int initial)
{
    struct gpio_v2_line_request req;
    memset(&req, 0, sizeof(req));

    req.offsets[0]      =   offset;
    req.num_lines       =   1;
    req.config.flags    =   GPIO_V2_LINE_FLAG_OUTPUT;
    snprintf(req.consumer, sizeof(req.consumer), "nrf_ce");

    if( ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0 )
    {
        return -1;
    }

    struct gpio_v2_line_values vals =   { .bits = initial ? 1ULL : 0ULL, .mask = 1ULL };
    ioctl(req.fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals);

    return req.fd;
}

static void gpio_set(int line_fd, int value)
{
    struct gpio_v2_line_values vals =   { .bits = value ? 1ULL : 0ULL, .mask = 1ULL };
    ioctl(line_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals);
}

/*============= CE PIN ===============================================================*/

void NRF_CE_High(NRF_Handle *h)     { gpio_set(h->ce_line_fd, 1); }
void NRF_CE_Low(NRF_Handle *h)      { gpio_set(h->ce_line_fd, 0); }

/*============= REGISTER ACCESS ======================================================*/

uint8_t NRF_ReadReg(NRF_Handle *h, uint8_t reg)
{
    uint8_t tx[2]   =   { NRF_CMD_R_REGISTER | (reg & 0x1F), 0x00 };
    uint8_t rx[2]   =   { 0 };

    spi_transfer(h, tx, rx, 2);
    h->last_status  =   rx[0];

    return rx[1];
}

void NRF_WriteReg(NRF_Handle *h, uint8_t reg, uint8_t val)
{
    uint8_t tx[2]   =   { NRF_CMD_W_REGISTER | (reg & 0x1F), val };
    uint8_t rx[2]   =   { 0 };

    spi_transfer(h, tx, rx, 2);
    h->last_status  =   rx[0];
}

void NRF_ReadRegMulti(NRF_Handle *h, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[1 + NRF_ADDR_WIDTH];
    uint8_t rx[1 + NRF_ADDR_WIDTH];

    memset(tx, 0, sizeof(tx));
    tx[0]   =   NRF_CMD_R_REGISTER | (reg & 0x1F);

    spi_transfer(h, tx, rx, 1 + len);
    h->last_status  =   rx[0];
    memcpy(buf, &rx[1], len);
}

void NRF_WriteRegMulti(NRF_Handle *h, uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t tx[1 + NRF_ADDR_WIDTH];
    uint8_t rx[1 + NRF_ADDR_WIDTH];

    tx[0]   =   NRF_CMD_W_REGISTER | (reg & 0x1F);
    memcpy(&tx[1], buf, len);

    spi_transfer(h, tx, rx, 1 + len);
    h->last_status  =   rx[0];
}

/*============= UTILITY ==============================================================*/

uint8_t NRF_GetStatus(NRF_Handle *h)
{
    uint8_t tx      =   NRF_CMD_NOP;
    uint8_t rx      =   0;

    spi_transfer(h, &tx, &rx, 1);
    h->last_status  =   rx;

    return rx;
}

void NRF_ClearIRQ(NRF_Handle *h, uint8_t flags)
{
    NRF_WriteReg(h, NRF_REG_STATUS, flags);
}

void NRF_FlushRX(NRF_Handle *h)
{
    uint8_t tx  =   NRF_CMD_FLUSH_RX;
    uint8_t rx  =   0;
    spi_transfer(h, &tx, &rx, 1);
}

void NRF_FlushTX(NRF_Handle *h)
{
    uint8_t tx  =   NRF_CMD_FLUSH_TX;
    uint8_t rx  =   0;
    spi_transfer(h, &tx, &rx, 1);
}

void NRF_PowerDown(NRF_Handle *h)
{
    NRF_CE_Low(h);
    uint8_t cfg =   NRF_ReadReg(h, NRF_REG_CONFIG);
    NRF_WriteReg(h, NRF_REG_CONFIG, cfg & ~NRF_CONFIG_PWR_UP);   
}

/*============= NRF_Init (PRX Mode) ==================================================*/

NRF_Status NRF_Init(NRF_Handle *h,
                const char *spi_dev, uint32_t spi_speed,
                int gpio_chip_fd, uint32_t ce_gpio,
                uint8_t channel, const uint8_t addr[NRF_ADDR_WIDTH])
{
    /* Clear Handle — sizeof(*h) not sizeof(h)! */
    memset(h, 0, sizeof(*h));

    /*----- Configuration -----*/
    h->channel      =   channel;
    h->spi_speed    =   spi_speed;
    h->ce_line_fd   =   -1;
    h->spi_fd       =   -1;
    
    memcpy(h->addr, addr, NRF_ADDR_WIDTH);

    /* Open SPI */
    h->spi_fd       =   open(spi_dev, O_RDWR);
    if(h->spi_fd < 0)
    {
        fprintf(stderr, "[NRF] SPI open %s: %s\n", spi_dev, strerror(errno));
        return NRF_ERR_SPI;
    }
    
    uint8_t mode    =   0;
    uint8_t bits    =   8;

    ioctl(h->spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(h->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(h->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);

    /* Request CE GPIO */
    h->ce_chip_fd   =   gpio_chip_fd;
    h->ce_line_fd   =   gpio_request_output(gpio_chip_fd, ce_gpio, 0);
    if(h->ce_line_fd < 0)
    {
        fprintf(stderr, "[NRF] CE GPIO %u request failed\n", ce_gpio);
        close(h->spi_fd);
        h->spi_fd   =   -1;
        return NRF_ERR_GPIO;
    }

    NRF_CE_Low(h);
    delay_ms(5);

    /* Chip Detection */
    uint8_t aw_val  =   NRF_ADDR_WIDTH - 2;
    NRF_WriteReg(h, NRF_REG_SETUP_AW, aw_val);
    if(NRF_ReadReg(h, NRF_REG_SETUP_AW) != aw_val)
    {
        fprintf(stderr, "[NRF] Chip is not detected (SETUP_AW readback has failed)\n");
        close(h->spi_fd);
        close(h->ce_line_fd);
        h->spi_fd       =   -1;
        h->ce_line_fd   =   -1;
        return NRF_ERR_NOT_DETECTED;
    }

    /* Radio Parameters */
    NRF_WriteReg(h, NRF_REG_RF_CH, channel);
    NRF_WriteReg(h, NRF_REG_RF_SETUP, NRF_RF_DR_HIGH | NRF_RF_PWR_0dBm | NRF_RF_LNA_HCURR);
    NRF_WriteReg(h, NRF_REG_EN_AA, 0x01);
    NRF_WriteReg(h, NRF_REG_EN_ADDR, 0x01);
    NRF_WriteReg(h, NRF_REG_RX_PW_P0, NRF_PAYLOAD_SIZE);
    NRF_WriteReg(h, NRF_REG_SETUP_RETR, 0x1F);
    
    /* RX address on pipe 0 */
    NRF_WriteRegMulti(h, NRF_REG_RX_ADDR_P0, addr, NRF_ADDR_WIDTH);

    /* Clear FIFOs and IRQ Flags */
    NRF_FlushRX(h);
    NRF_FlushTX(h);
    NRF_ClearIRQ(h, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);

    /* Config: PRX mode with CRC-16 */
    uint8_t cfg =   NRF_CONFIG_EN_CRC
                  | NRF_CONFIG_CRCO
                  | NRF_CONFIG_PWR_UP
                  | NRF_CONFIG_PRIM_RX
                  | NRF_CONFIG_MASK_TX_DS
                  | NRF_CONFIG_MASK_MAX_RT;

    NRF_WriteReg(h, NRF_REG_CONFIG, cfg);
    delay_ms(2);

    /* Start Listening */
    NRF_CE_High(h);

    printf("[NRF] Init OK: ch=%u | speed=%u Hz | addr=%02X:%02X:%02X:%02X:%02X\n",
            channel, spi_speed, addr[0], addr[1], addr[2], addr[3], addr[4]);

    return NRF_OK;
}

/*============= NRF_Close ==================================================================*/

void NRF_Close(NRF_Handle *h)
{
    NRF_CE_Low(h);
    NRF_PowerDown(h);

    if(h->ce_line_fd >= 0)  { close(h->ce_line_fd);    h->ce_line_fd   =   -1; }
    if(h->spi_fd >= 0)      { close(h->spi_fd);        h->spi_fd       =   -1; }
}

/*============= RX Operations ==============================================================*/

bool NRF_DataAvailable(NRF_Handle *h)
{
    uint8_t status  =   NRF_GetStatus(h);

    /* Check RX_DR flag first (set by hardware on successful reception) */
    if(status & NRF_STATUS_RX_DR)
    {
        return true;
    }

    /* 
     * Also check RX_P_NO field (bits 3:1) — if not 0b111 (0x0E),
     * there is data in one of the pipes even without RX_DR set
     * (can happen if IRQ was already cleared but FIFO not empty) 
     */
    return ( (status & NRF_STATUS_RX_P_NO_MASK) != NRF_STATUS_RX_FIFO_EMPTY );
}

NRF_Status NRF_ReadPayload(NRF_Handle *h, uint8_t *buf)
{
    if(!NRF_DataAvailable(h))
    {
        return NRF_ERR_RX_EMPTY;
    }

    uint8_t tx[1 + NRF_PAYLOAD_SIZE];
    uint8_t rx[1 + NRF_PAYLOAD_SIZE];

    memset(tx, 0xFF, sizeof(tx));
    tx[0]   =   NRF_CMD_R_RX_PAYLOAD;

    spi_transfer(h, tx, rx, 1 + NRF_PAYLOAD_SIZE);
    h->last_status  =   rx[0];

    memcpy(buf, &rx[1], NRF_PAYLOAD_SIZE);

    NRF_ClearIRQ(h, NRF_STATUS_RX_DR);

    return NRF_OK;
}

/*==========================================================================================*/

