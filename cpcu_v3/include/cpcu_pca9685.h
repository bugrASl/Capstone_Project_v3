/**
 *  @file   cpcu_pca9685.h
 *  @brief  PCA9685 PWM servo driver API — per-servo limits, pulse conversion.
 */

#ifndef CPCU_PCA9685_H
#define CPCU_PCA9685_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*============= DEVICE CONFIGURATION ===========================*/

#define PCA_I2C_ADDR_BASE       0x40
#define PCA_OSC_CLOCK_HZ        25000000
#define PCA_CHANNEL_COUNT       16
#define PCA_RESOLUTION          4096

/*============= SERVO CONFIGURATION ============================*/

#define PCA_SERVO_COUNT         6
#define PCA_SERVO_FREQ_HZ       50
#define PCA_SERVO_PERIOD_US     20000
#define PCA_SERVO_NEUTRAL       1500

/*  Logical servo index -> physical PCA9685 output channel.
 *  Indexed by [S0, S1, S2, S3, S4, S5] = [Base, Upper, Last, Joint-1,
 *  Joint-2, Gripper]. Edit ONLY when cables are physically rerouted on
 *  the breakout. */
#define PCA_SERVO_CHANNEL       { 0, 1, 11, 8, 5, 4 }

/*  Per-servo pulse-width limits, indexed by logical servo index.
 *  Empirical, derived from EE493 Arduino testbench. Edit if a new
 *  servo's mechanical travel differs from these values. */
#define PCA_SERVO_MIN_US        { 498, 1074, 1074, 1001, 1001,  976}
#define PCA_SERVO_MAX_US        {2500, 1953, 1953, 2002, 2002, 1733}

/*============= REGISTER MAP ===================================*/

#define PCA_REG_MODE1           0x00
#define PCA_REG_MODE2           0x01
#define PCA_REG_SUBADR1         0x02
#define PCA_REG_SUBADR2         0x03
#define PCA_REG_SUBADR3         0x04
#define PCA_REG_ALLCALLADR      0x05
#define PCA_REG_LED0_ON_L       0x06
#define PCA_REG_LED0_ON_H       0x07
#define PCA_REG_LED0_OFF_L      0x08
#define PCA_REG_LED0_OFF_H      0x09

#define PCA_REG_LEDn_ON_L(n)    (0x06 + (4 * (n)))
#define PCA_REG_LEDn_ON_H(n)    (0x07 + (4 * (n)))
#define PCA_REG_LEDn_OFF_L(n)   (0x08 + (4 * (n)))
#define PCA_REG_LEDn_OFF_H(n)   (0x09 + (4 * (n)))

#define PCA_REG_ALL_LED_ON_L    0xFA
#define PCA_REG_ALL_LED_ON_H    0xFB
#define PCA_REG_ALL_LED_OFF_L   0xFC
#define PCA_REG_ALL_LED_OFF_H   0xFD
#define PCA_REG_PRE_SCALE       0xFE

/*============= MODE1 REGISTER BITS ============================*/

#define PCA_MODE1_RESTART       (1 << 7)
#define PCA_MODE1_EXTCLK        (1 << 6)
#define PCA_MODE1_AI            (1 << 5)
#define PCA_MODE1_SLEEP         (1 << 4)
#define PCA_MODE1_SUB1          (1 << 3)
#define PCA_MODE1_SUB2          (1 << 2)
#define PCA_MODE1_SUB3          (1 << 1)
#define PCA_MODE1_ALLCALL       (1 << 0)

/*============= MODE2 REGISTER BITS ============================*/

#define PCA_MODE2_INVRT         (1 << 4)
#define PCA_MODE2_OCH           (1 << 3)
#define PCA_MODE2_OUTDRV        (1 << 2)
#define PCA_MODE2_OUTNE1        (1 << 1)
#define PCA_MODE2_OUTNE0        (1 << 0)

/*============= FULL ON / FULL OFF CONTROL =====================*/

#define PCA_LED_FULL_ON_BIT     (1 << 4)
#define PCA_LED_FULL_OFF_BIT    (1 << 4)

/*============= STATUS RETURN ==================================*/

typedef enum
{
    PCA_OK = 0,
    PCA_ERR_I2C_OPEN,
    PCA_ERR_I2C_SLAVE,
    PCA_ERR_I2C_WRITE,
    PCA_ERR_I2C_READ,
    PCA_ERR_NOT_DETECTED,
    PCA_ERR_CHANNEL,
} PCA_Status;

/*============= DRIVER HANDLE ==================================*/

typedef struct
{
    int         i2c_fd;
    uint8_t     addr;
    uint8_t     prescaler;
    uint16_t    servo_min[PCA_SERVO_COUNT];
    uint16_t    servo_max[PCA_SERVO_COUNT];
    uint8_t     servo_channel[PCA_SERVO_COUNT];     /* logical -> physical map */
} PCA_Handle;

/*============= API ============================================*/

PCA_Status  PCA_Init(PCA_Handle *p, const char *i2c_dev, uint8_t addr);
void        PCA_Close(PCA_Handle *p);

/* Register Access — these stay raw (PCA channel 0..15) */
PCA_Status  PCA_WriteReg(PCA_Handle *p, uint8_t reg, uint8_t val);
PCA_Status  PCA_ReadReg(PCA_Handle *p, uint8_t reg, uint8_t *val);
PCA_Status  PCA_SetPWM(PCA_Handle *p, uint8_t channel, uint16_t on, uint16_t off);

/* Logical Servo Control — `logical_idx` is 0..PCA_SERVO_COUNT-1.
 * Driver translates to the physical PCA channel via servo_channel[]. */
PCA_Status  PCA_SetServo(PCA_Handle *p, uint8_t logical_idx, uint16_t pulse_us);

void        PCA_SetAllServos(PCA_Handle *p, const uint16_t pulse_us[PCA_SERVO_COUNT]);
void        PCA_SetAllNeutral(PCA_Handle *p);
void        PCA_AllOff(PCA_Handle *p);

/* Safety */
void        PCA_SafetyClamp(PCA_Handle *p, uint16_t pulse_us[PCA_SERVO_COUNT]);

/* Conversion Utilities */
static inline uint16_t PCA_PulseToCounts(uint16_t pulse_us)
{
    return (uint16_t)( (uint32_t)(pulse_us) * PCA_RESOLUTION / PCA_SERVO_PERIOD_US );
}

static inline uint16_t PCA_CountsToPulse(uint16_t counts)
{
    return (uint16_t)( (uint32_t)(counts) * PCA_SERVO_PERIOD_US / PCA_RESOLUTION );
}

#ifdef __cplusplus
}
#endif

#endif /* CPCU_PCA9685_H */

