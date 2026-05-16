/**
 *  @file   cpcu_pca9685.c
 *  @brief  PCA9685 I2C PWM servo driver — init, set pulse, safety clamp.
 *
 *  Communicates with the PCA9685 16-channel PWM controller at 400 kHz I2C.
 *  Configures 50 Hz PWM frequency, converts microsecond pulse widths to
 *  12-bit ON/OFF register values. Per-servo hardware limits are enforced
 *  by PCA_SafetyClamp() before any write reaches the bus.
 */

#include "cpcu_pca9685.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

/*============= REGISTER ACCESS =====================================================*/

PCA_Status PCA_WriteReg(PCA_Handle *p, uint8_t reg, uint8_t val)
{
    uint8_t buf[2]  =   { reg, val };

    if( write(p->i2c_fd, buf, 2) != 2 )
    {
        fprintf(stderr, "[PCA] Write reg 0x%02X failed: %s\n", reg, strerror(errno));
        return PCA_ERR_I2C_WRITE;
    }

    return PCA_OK;
}

PCA_Status PCA_ReadReg(PCA_Handle *p, uint8_t reg, uint8_t *val)
{
    /* Send register address */
    if( write(p->i2c_fd, &reg, 1) != 1 )
    {
        fprintf(stderr, "[PCA] Read reg 0x%02X addr-write failed: %s\n", reg, strerror(errno));
        return PCA_ERR_I2C_READ;
    }

    /* Read register value */
    if( read(p->i2c_fd, val, 1) != 1 )
    {
        fprintf(stderr, "[PCA] Read reg 0x%02X data-read failed: %s\n", reg, strerror(errno));
        return PCA_ERR_I2C_READ;
    }

    return PCA_OK;
}

static PCA_Status pca_write_4(PCA_Handle *p, uint8_t start_reg, const uint8_t data[4])
{
    uint8_t buf[5]  =   { start_reg, data[0], data[1], data[2], data[3] };

    if( write(p->i2c_fd, buf, 5) != 5 )
    {
        return PCA_ERR_I2C_WRITE;
    }

    return PCA_OK;
}

/*============= PCA_Init ============================================================*/

PCA_Status PCA_Init(PCA_Handle *p, const char *i2c_dev, uint8_t addr)
{
    /* Load servo safety limits and channel map */
    const uint16_t  mins[]  =   PCA_SERVO_MIN_US;
    const uint16_t  maxs[]  =   PCA_SERVO_MAX_US;
    const uint8_t   chans[] =   PCA_SERVO_CHANNEL;
    memcpy(p->servo_min,     mins,  sizeof(mins));
    memcpy(p->servo_max,     maxs,  sizeof(maxs));
    memcpy(p->servo_channel, chans, sizeof(chans));

    p->addr     =   addr;
    p->i2c_fd   =   -1;

    /* Open I2C bus */
    p->i2c_fd   =   open(i2c_dev, O_RDWR);
    if(p->i2c_fd < 0)
    {
        fprintf(stderr, "[PCA] Cannot open %s: %s\n", i2c_dev, strerror(errno));
        return PCA_ERR_I2C_OPEN;
    }

    if( ioctl(p->i2c_fd, I2C_SLAVE, addr) < 0 )
    {
        fprintf(stderr, "[PCA] ioctl I2C_SLAVE 0x%02X: %s\n", addr, strerror(errno));
        close(p->i2c_fd);
        p->i2c_fd   =   -1;
        return PCA_ERR_I2C_SLAVE;
    }

    PCA_Status status;

    /* Step 1: Sleep (oscillator off — required before prescaler write) */
    status  =   PCA_WriteReg(p, PCA_REG_MODE1, PCA_MODE1_SLEEP);
    if(status != PCA_OK) goto fail;

    /* Step 2: Compute and write prescaler (only writable in sleep mode) */
    p->prescaler    =   (uint8_t)( roundf( (float)(PCA_OSC_CLOCK_HZ)
                                         / (float)(PCA_RESOLUTION * PCA_SERVO_FREQ_HZ) ) - 1.0f );
    status  =   PCA_WriteReg(p, PCA_REG_PRE_SCALE, p->prescaler);
    if(status != PCA_OK) goto fail;

    /* Step 3: Wake up with auto-increment */
    status  =   PCA_WriteReg(p, PCA_REG_MODE1, PCA_MODE1_AI | PCA_MODE1_ALLCALL);
    if(status != PCA_OK) goto fail;

    /* Step 4: Oscillator startup delay (datasheet §7.3.1 note [2]) */
    usleep(500);

    /* Step 5: Restart */
    uint8_t mode1_val   =   0;
    status  =   PCA_ReadReg(p, PCA_REG_MODE1, &mode1_val);
    if(status != PCA_OK) goto fail;
    status  =   PCA_WriteReg(p, PCA_REG_MODE1, mode1_val | PCA_MODE1_RESTART);
    if(status != PCA_OK) goto fail;

    /* Step 6: Totem-Pole output */
    status  =   PCA_WriteReg(p, PCA_REG_MODE2, PCA_MODE2_OUTDRV);
    if(status != PCA_OK) goto fail;

    /* Verify: Check chip is alive */
    status  =   PCA_ReadReg(p, PCA_REG_MODE1, &mode1_val);
    if( !(mode1_val & PCA_MODE1_AI) )
    {
        fprintf(stderr, "[PCA] MODE1 verify failed: got 0x%02X, expected AI bit set\n", mode1_val);
        goto fail;
    }

    /* Step 7: Set Servos to neutral */
    PCA_SetAllNeutral(p);

    float actual_freq   =   (float)(PCA_OSC_CLOCK_HZ)
                          / (float)(PCA_RESOLUTION * ( (uint32_t)(p->prescaler) + 1 ));

    printf("[PCA] Init OK: addr=0x%02X | prescaler=%u | freq=%.1f Hz\n",
            addr, p->prescaler, actual_freq);
    printf("[PCA] Servo map (logical -> PCA channel): "
           "S0=%u S1=%u S2=%u S3=%u S4=%u S5=%u\n",
           p->servo_channel[0], p->servo_channel[1], p->servo_channel[2],
           p->servo_channel[3], p->servo_channel[4], p->servo_channel[5]);

    return PCA_OK;

fail:
    close(p->i2c_fd);
    p->i2c_fd   =   -1;
    return PCA_ERR_NOT_DETECTED;
}

/*============= PCA_Close ============================================================*/

void PCA_Close(PCA_Handle *p)
{
    if(p->i2c_fd < 0) return;

    PCA_AllOff(p);
    close(p->i2c_fd);
    p->i2c_fd   =   -1;

    printf("[PCA] Closed\n");
}

/*============= PWM Control ==========================================================*/

/*  Raw PCA channel access — caller passes 0..15. Used by pca_testbench
 *  for register-level poking and by the logical wrappers below. */
PCA_Status PCA_SetPWM(PCA_Handle *p, uint8_t channel, uint16_t on, uint16_t off)
{
    if(channel >= PCA_CHANNEL_COUNT)
    {
        return PCA_ERR_CHANNEL;
    }

    uint8_t data[4];
    data[0] =   (uint8_t)( on & 0xFF );
    data[1] =   (uint8_t)( (on >> 8) & 0x1F );
    data[2] =   (uint8_t)( off & 0xFF );
    data[3] =   (uint8_t)( (off >> 8) & 0x1F );

    return pca_write_4(p, PCA_REG_LEDn_ON_L(channel), data);
}

/* logical_idx is 0..PCA_SERVO_COUNT-1.
 *  Translates to physical PCA channel via servo_channel[]. */
PCA_Status PCA_SetServo(PCA_Handle *p, uint8_t logical_idx, uint16_t pulse_us)
{
    if(logical_idx >= PCA_SERVO_COUNT)
    {
        return PCA_ERR_CHANNEL;
    }

    uint8_t  pca_channel    =   p->servo_channel[logical_idx];
    uint16_t counts         =   PCA_PulseToCounts(pulse_us);

    return PCA_SetPWM(p, pca_channel, 0, counts);
}

void PCA_SetAllServos(PCA_Handle *p, const uint16_t pulse_us[PCA_SERVO_COUNT])
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        PCA_SetServo(p, (uint8_t)i, pulse_us[i]);
    }
}

void PCA_SetAllNeutral(PCA_Handle *p)
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        PCA_SetServo(p, (uint8_t)i, PCA_SERVO_NEUTRAL);
    }
}

/*  PCA_AllOff hits the global ALL_LED_OFF register, which is per-chip,
 *  not per-channel — turns off ALL 16 outputs in one transaction.
 *  Logical/physical mapping is irrelevant here. */
void PCA_AllOff(PCA_Handle *p)
{
    PCA_WriteReg(p, PCA_REG_ALL_LED_OFF_H, PCA_LED_FULL_OFF_BIT);
}

/*============= Safety ===============================================================*/

void PCA_SafetyClamp(PCA_Handle *p, uint16_t pulse_us[PCA_SERVO_COUNT])
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        if(pulse_us[i] < p->servo_min[i])   pulse_us[i] =   p->servo_min[i];
        if(pulse_us[i] > p->servo_max[i])   pulse_us[i] =   p->servo_max[i];
    }
}

/*====================================================================================*/

