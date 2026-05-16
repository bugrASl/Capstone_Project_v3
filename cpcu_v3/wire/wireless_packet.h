/**
 *  @file       wireless_packet.h
 *  @brief      Wireless packet structure and 12-bit compression codec
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.1
 *  @details
 *                          Wire Layout (32-Byte, Fixed Payload)
 *              ────────────────────────────────────────────────────────────────
 *              Byte    Field           Size    Description
 *              ────────────────────────────────────────────────────────────────
 *              [0]     seq             1 B     Packet sequence number (0-255, wraps)
 *              [1]     flags           1 B     Status flags + 2-bit battery level
 *              [2]     tx_retry        1 B     ARC_CNT: retransmits for prev packet
 *              [3]     pkt_loss        1 B     PLOS_CNT: cumulative lost packets
 *              [4-5]   timestamp       2 B     TIM2 µs counter, little-endian
 *              [6-7]   vbat_raw        2 B     12-bit battery ADC, high-nibble aligned
 *              [8-19]  sample[0]       12 B    8 channels × 12-bit ADC, packed
 *              [20-31] sample[1]       12 B    8 channels × 12-bit ADC, packed
 *              ────────────────────────────────────────────────────────────────
 *              Total:  32 B
 *
 *                          12-bit Packing (two values → three bytes)
 *              ────────────────────────────────────────────────────────────────
 *              byte[0] =   A[7:0]
 *              byte[1] =   A[11:8] | (B[3:0] << 4)
 *              byte[2] =   B[11:4]
 *
 *                          vbat_raw Encoding
 *              ────────────────────────────────────────────────────────────────
 *              byte[0] =   vbat_raw[11:4]
 *              byte[1] =   (vbat_raw[3:0] << 4)
 */

#ifndef WIRELESS_PACKET_H
#define WIRELESS_PACKET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*============= CONFIGURABLE CONSTANTS =========================================================*/

#define WL_PAYLOAD_SIZE             32
#define WL_NUM_CHANNELS             8
#define WL_SAMPLES_PER_PACKET       2

/*============= BYTE OFFSETS ===================================================================*/

#define WL_OFF_SEQ                  0
#define WL_OFF_FLAGS                1
#define WL_OFF_TX_RETRY             2
#define WL_OFF_PKT_LOSS             3
#define WL_OFF_TIMESTAMP            4
#define WL_OFF_VBAT_RAW             6
#define WL_OFF_SAMPLES              8

/*============= DERIVED CONSTANTS ==============================================================*/

#define WL_BYTES_PER_SAMPLE         ( (WL_NUM_CHANNELS / 2) * 3 )
#define WL_OFF_SAMPLE(s)            ( WL_OFF_SAMPLES + ((s) * WL_BYTES_PER_SAMPLE) )

/*============= COMPILE-TIME ASSERTS ===========================================================*/

_Static_assert(
    WL_OFF_SAMPLES + (WL_SAMPLES_PER_PACKET * WL_BYTES_PER_SAMPLE) == WL_PAYLOAD_SIZE,
    "Packet layout does not fill 32-byte payload"
);

/*============= FLAG BYTE ======================================================================*/
/*
 *  bit 7     FIRST_PACKET    Session-first packet after boot; resets expected_seq on CPCU
 *  bit 6     CLIPPING        Any channel saturated (0x000 or 0xFFF) in this packet
 *  bit 5     ELEC_OFF        Electrode contact impedance too high
 *  bit 4     ADC_OVRN        ADC overrun detected (firmware bug indicator)
 *  bit 3     TX_SAT          TX_FIFO was full when packet submitted
 *  bit 2     CAL             Calibration frame (not user data)
 *  bit [1:0] BATT_LVL        2-bit battery level: OK / LOW / CRIT / CHARG
 */

#define WL_FLAG_FIRST_PACKET        (1u << 7)
#define WL_FLAG_CLIPPING            (1u << 6)
#define WL_FLAG_ELEC_OFF            (1u << 5)
#define WL_FLAG_ADC_OVRN            (1u << 4)
#define WL_FLAG_TX_SAT              (1u << 3)
#define WL_FLAG_CAL                 (1u << 2)

/*-------------- Battery level field (2-bit) ---------------------------------------------------*/

#define WL_BATT_MASK                (0x03u)
#define WL_BATT_OK                  (0x00u)
#define WL_BATT_LOW                 (0x01u)
#define WL_BATT_CRIT                (0x02u)
#define WL_BATT_CHARG               (0x03u)

/** Write battery level into flags byte (preserves flag bits). */
#define WL_BATT_SET(flags, level)   ( ((flags) & ~WL_BATT_MASK) | ((level) & WL_BATT_MASK) )

/** Extract battery level from flags byte. */
#define WL_BATT_GET(flags)          ( (flags) & WL_BATT_MASK )

/*============= VBAT_RAW HELPERS ===============================================================*/
/**
 *  Encoding (high-nibble aligned, two bytes):
 *      byte[0] =   val[11:4]
 *      byte[1] =   (val[3:0] << 4)
 */

/** Pack 12-bit vbat into two wire bytes. */
#define WL_VBAT_ENCODE(out, val)                                                        \
    do                                                                                  \
    {                                                                                   \
        (out)[0]    =   (uint8_t)( ((val) >> 4) & 0xFF );                               \
        (out)[1]    =   (uint8_t)( ((val) & 0x0F) << 4 );                               \
    }                                                                                   \
    while (0)

/** Unpack 12-bit vbat from two wire bytes. */
#define WL_VBAT_DECODE(in)                                                              \
    ( (uint16_t)( (uint16_t)((in)[0]) << 4 ) |                                          \
      (uint16_t)( ((in)[1] >> 4) & 0x0F ) )

/*============= STRUCTURES =====================================================================*/

typedef struct
{
    uint16_t    ch[WL_NUM_CHANNELS];
} WL_SampleSet;

typedef struct
{
    uint8_t         seq;
    uint8_t         flags;
    uint8_t         tx_retry;
    uint8_t         pkt_loss;
    uint16_t        timestamp;
    uint16_t        vbat_raw;
    WL_SampleSet    samples[WL_SAMPLES_PER_PACKET];
} WL_Packet;

/*============= API ============================================================================*/

void    WL_Pack  (const WL_Packet *in, uint8_t *out);
void    WL_Unpack(const uint8_t *in,   WL_Packet *out);

/*==============================================================================================*/

#ifdef __cplusplus
}
#endif

#endif /* WIRELESS_PACKET_H */
