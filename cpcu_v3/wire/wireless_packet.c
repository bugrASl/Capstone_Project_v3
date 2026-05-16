/**
 *  @file       wireless_packet.c
 *  @brief      Wireless packet codec — 8 channels, metadata, 32-byte payload
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.1
 */

#include "wireless_packet.h"

#include <string.h>

/*============= WL_Pack ========================================================================*/

void WL_Pack(const WL_Packet *in, uint8_t *out)
{
    out[WL_OFF_SEQ]             =   in->seq;
    out[WL_OFF_FLAGS]           =   in->flags;
    out[WL_OFF_TX_RETRY]        =   in->tx_retry;
    out[WL_OFF_PKT_LOSS]        =   in->pkt_loss;
    out[WL_OFF_TIMESTAMP]       =   (uint8_t)( in->timestamp & 0xFF );
    out[WL_OFF_TIMESTAMP + 1]   =   (uint8_t)( (in->timestamp >> 8) & 0xFF );

    WL_VBAT_ENCODE(&out[WL_OFF_VBAT_RAW], in->vbat_raw);

    for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
    {
        int base                =   WL_OFF_SAMPLE(s);

        /* Pack channel pairs: c steps by 2, each pair → 3 bytes */
        for (int c = 0; c < WL_NUM_CHANNELS; c += 2)
        {
            uint16_t a          =   in->samples[s].ch[c]     & 0x0FFF;
            uint16_t b          =   in->samples[s].ch[c + 1] & 0x0FFF;

            out[base]           =   (uint8_t)(  a       & 0xFF );
            out[base + 1]       =   (uint8_t)( ((a >> 8) & 0x0F) | ((b << 4) & 0xF0) );
            out[base + 2]       =   (uint8_t)( (b >> 4) & 0xFF );

            base               +=   3;
        }
    }
}

/*============= WL_Unpack ======================================================================*/

void WL_Unpack(const uint8_t *in, WL_Packet *out)
{
    out->seq                    =   in[WL_OFF_SEQ];
    out->flags                  =   in[WL_OFF_FLAGS];
    out->tx_retry               =   in[WL_OFF_TX_RETRY];
    out->pkt_loss               =   in[WL_OFF_PKT_LOSS];
    out->timestamp              =   (uint16_t)(  in[WL_OFF_TIMESTAMP]       ) |
                                    (uint16_t)( (in[WL_OFF_TIMESTAMP + 1]) << 8 );

    out->vbat_raw               =   WL_VBAT_DECODE(&in[WL_OFF_VBAT_RAW]);

    for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
    {
        int base                =   WL_OFF_SAMPLE(s);

        /* Unpack channel pairs: c steps by 2, each pair ← 3 bytes */
        for (int c = 0; c < WL_NUM_CHANNELS; c += 2)
        {
            uint8_t x0          =   in[base];
            uint8_t x1          =   in[base + 1];
            uint8_t x2          =   in[base + 2];

            out->samples[s].ch[c]       =   (uint16_t)(  x0        ) |
                                            (uint16_t)( (x1 & 0x0F) << 8 );
            out->samples[s].ch[c + 1]   =   (uint16_t)( (x1 >> 4) & 0x0F ) |
                                            (uint16_t)(  x2 << 4 );

            base               +=   3;
        }
    }
}

/*==============================================================================================*/
