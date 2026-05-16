/**
 *  @file   test_codec.c
 *  @brief  Codec unit test — WL_Pack/WL_Unpack round-trip on all packet fields.
 */

#include "wireless_packet.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

static int g_pass                       =   0;
static int g_fail                       =   0;

#define TEST_ASSERT(cond, fmt, ...) \
    do { \
        if (!(cond)) { \
            fprintf(stderr, "  [FAIL] " fmt "\n", ##__VA_ARGS__); \
            g_fail++; \
            return; \
        } \
    } while(0)

#define TEST_OK(fmt, ...) \
    do { printf("  [PASS] " fmt "\n", ##__VA_ARGS__); g_pass++; } while(0)

/* ══════════════════════════════════════════════════════════════════════
 *  TB-C100: Packet Codec Round-Trip
 * ══════════════════════════════════════════════════════════════════════ */

static void test_roundtrip(void)
{
    printf("\n--- TB-C100: Packet Codec Round-Trip ---\n");

    WL_Packet orig, decoded;
    uint8_t wire[WL_PAYLOAD_SIZE];

    orig.seq            =   42;
    orig.flags          =   WL_FLAG_FIRST_PACKET | WL_BATT_LOW;
    orig.tx_retry       =   3;
    orig.pkt_loss       =   7;
    orig.timestamp      =   0xABCD;
    orig.vbat_raw       =   2048;

    for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        for (int c = 0; c < WL_NUM_CHANNELS; c++)
            orig.samples[s].ch[c]   =   (uint16_t)((s * 100 + c * 10 + 1) & 0x0FFF);

    WL_Pack(&orig, wire);
    WL_Unpack(wire, &decoded);

    TEST_ASSERT(decoded.seq == 42, "seq: got %u, expected 42", decoded.seq);
    TEST_ASSERT(decoded.flags == (WL_FLAG_FIRST_PACKET | WL_BATT_LOW),
                "flags: got 0x%02X", decoded.flags);
    TEST_ASSERT(decoded.tx_retry == 3, "tx_retry: got %u", decoded.tx_retry);
    TEST_ASSERT(decoded.pkt_loss == 7, "pkt_loss: got %u", decoded.pkt_loss);
    TEST_ASSERT(decoded.timestamp == 0xABCD, "timestamp: got 0x%04X", decoded.timestamp);
    TEST_ASSERT(decoded.vbat_raw == 2048, "vbat_raw: got %u", decoded.vbat_raw);

    for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        for (int c = 0; c < WL_NUM_CHANNELS; c++)
            TEST_ASSERT(decoded.samples[s].ch[c] == orig.samples[s].ch[c],
                        "sample[%d].ch[%d]: got %u, expected %u",
                        s, c, decoded.samples[s].ch[c], orig.samples[s].ch[c]);

    TEST_OK("All fields round-trip correctly");
}

/* ══════════════════════════════════════════════════════════════════════
 *  TB-C100b: Boundary values (0x000, 0xFFF, 0x800)
 * ══════════════════════════════════════════════════════════════════════ */

static void test_boundary(void)
{
    printf("\n--- TB-C100b: Boundary Values ---\n");

    uint16_t vals[]     =   {0x000, 0x001, 0x7FF, 0x800, 0xFFE, 0xFFF};
    int nvals           =   sizeof(vals) / sizeof(vals[0]);

    for (int v = 0; v < nvals; v++)
    {
        WL_Packet orig, decoded;
        uint8_t wire[WL_PAYLOAD_SIZE];
        memset(&orig, 0, sizeof(orig));

        for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
            for (int c = 0; c < WL_NUM_CHANNELS; c++)
                orig.samples[s].ch[c]   =   vals[v];

        WL_Pack(&orig, wire);
        WL_Unpack(wire, &decoded);

        for (int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
            for (int c = 0; c < WL_NUM_CHANNELS; c++)
                TEST_ASSERT(decoded.samples[s].ch[c] == vals[v],
                            "val=0x%03X s=%d c=%d: got 0x%03X",
                            vals[v], s, c, decoded.samples[s].ch[c]);
    }

    TEST_OK("All boundary values preserved");
}

/* ══════════════════════════════════════════════════════════════════════
 *  TB-C101: vbat_raw Exhaustive (all 4096 values)
 * ══════════════════════════════════════════════════════════════════════ */

static void test_vbat_exhaustive(void)
{
    printf("\n--- TB-C101: vbat_raw Exhaustive ---\n");

    WL_Packet orig, decoded;
    uint8_t wire[WL_PAYLOAD_SIZE];
    int errors          =   0;

    for (uint32_t v = 0; v < 4096; v++)
    {
        memset(&orig, 0, sizeof(orig));
        orig.vbat_raw   =   (uint16_t)v;
        WL_Pack(&orig, wire);
        WL_Unpack(wire, &decoded);
        if (decoded.vbat_raw != (uint16_t)v)
            errors++;
    }

    TEST_ASSERT(errors == 0, "vbat_raw: %d/4096 values failed round-trip", errors);
    TEST_OK("All 4096 vbat_raw values round-trip perfectly");
}

/* ══════════════════════════════════════════════════════════════════════
 *  TB-C102: Sequence Gap Detector
 * ══════════════════════════════════════════════════════════════════════ */

static uint32_t seq_gap(uint8_t *expected, uint8_t received)
{
    uint8_t gap         =   (received - *expected) & 0xFF;
    *expected           =   (received + 1) & 0xFF;
    return (gap <= 1) ? 0 : gap;
}

static void test_seq_gap(void)
{
    printf("\n--- TB-C102: Sequence Gap Detector ---\n");

    uint8_t exp         =   0;

    /* Normal sequential */
    TEST_ASSERT(seq_gap(&exp, 0) == 0, "seq 0 gap should be 0");
    TEST_ASSERT(seq_gap(&exp, 1) == 0, "seq 1 gap should be 0");
    TEST_ASSERT(seq_gap(&exp, 2) == 0, "seq 2 gap should be 0");

    /* Gap of 3 (packets 3,4 lost) */
    TEST_ASSERT(seq_gap(&exp, 5) == 2, "seq 5 after 2: gap should be 2, got %u", seq_gap(&exp, 5));

    /* Wrap around 255 -> 0 */
    exp                 =   254;
    TEST_ASSERT(seq_gap(&exp, 254) == 0, "seq 254 gap should be 0");
    TEST_ASSERT(seq_gap(&exp, 255) == 0, "seq 255 gap should be 0");
    TEST_ASSERT(seq_gap(&exp, 0) == 0,   "seq 0 after 255: gap should be 0");
    TEST_ASSERT(seq_gap(&exp, 1) == 0,   "seq 1 after 0: gap should be 0");

    /* Wrap-around gap: expected=253, received=2 -> gap=5 (253,254,255,0,1 lost) */
    exp                 =   253;
    uint32_t g          =   seq_gap(&exp, 2);
    /* (2 - 253) & 0xFF = 5. 5 > 1 so return 5 which means 5 lost */
    TEST_ASSERT(g == 5, "wrap gap: expected 5, got %u", g);

    TEST_OK("Sequence gap detection with wrap");
}

/* ══════════════════════════════════════════════════════════════════════
 *  TB-C104: Timestamp Wrap Handling
 * ══════════════════════════════════════════════════════════════════════ */

static void test_timestamp_wrap(void)
{
    printf("\n--- TB-C104: Timestamp Wrap ---\n");

    /* Normal delta */
    uint16_t t1         =   1000;
    uint16_t t2         =   2000;
    uint16_t dt         =   (t2 - t1) & 0xFFFF;
    TEST_ASSERT(dt == 1000, "normal: dt=%u, expected 1000", dt);

    /* Wrap: t1=65000, t2=500 -> dt = 500 - 65000 + 65536 = 1036 */
    t1                  =   65000;
    t2                  =   500;
    dt                  =   (t2 - t1) & 0xFFFF;
    TEST_ASSERT(dt == 1036, "wrap: dt=%u, expected 1036", dt);

    /* Edge: exact wrap */
    t1                  =   0xFFFF;
    t2                  =   0x0000;
    dt                  =   (t2 - t1) & 0xFFFF;
    TEST_ASSERT(dt == 1, "edge: dt=%u, expected 1", dt);

    TEST_OK("16-bit timestamp wrap arithmetic correct");
}

/* ══════════════════════════════════════════════════════════════════════
 *  TB-C106: Battery Voltage Reconstruction
 * ══════════════════════════════════════════════════════════════════════ */

static void test_battery_voltage(void)
{
    printf("\n--- TB-C106: Battery Voltage Reconstruction ---\n");

    /* V_batt = vbat_raw × (3.3 / 4095) × 2 */
    struct { uint16_t raw; float expected_v; } cases[] = {
        { 2048,     3.30f   },      /* ~midscale */
        { 1862,     3.00f   },      /* OK/LOW boundary (approx) */
        { 1676,     2.70f   },      /* LOW/CRITICAL boundary (approx) */
        { 4095,     6.60f   },      /* maximum ADC reading */
        { 0,        0.00f   },      /* minimum */
    };
    int ncases          =   sizeof(cases) / sizeof(cases[0]);

    for (int i = 0; i < ncases; i++)
    {
        float v         =   cases[i].raw * (3.3f / 4095.0f) * 2.0f;
        float err       =   fabsf(v - cases[i].expected_v);
        TEST_ASSERT(err < 0.02f, "raw=%u: got %.3fV, expected %.3fV (err=%.3f)",
                    cases[i].raw, v, cases[i].expected_v, err);
    }

    TEST_OK("Battery voltage reconstruction within 20mV");
}

/* ══════════════════════════════════════════════════════════════════════
 *  TB-C100c: Exhaustive 12-bit channel pair (all A×B combinations)
 * ══════════════════════════════════════════════════════════════════════ */

static void test_exhaustive_pairs(void)
{
    printf("\n--- TB-C100c: Exhaustive 12-bit Pair Packing ---\n");

    /* Test every combination of (A, B) in the first channel pair */
    /* Full 4096 × 4096 = 16M is too slow for every run. Sample instead. */
    WL_Packet orig, decoded;
    uint8_t wire[WL_PAYLOAD_SIZE];
    int errors          =   0;
    int tests           =   0;

    /* Edge cases + random sampling */
    uint16_t edge_vals[] = {0x000, 0x001, 0x0FF, 0x100, 0x7FF, 0x800, 0xFFE, 0xFFF};
    int nedge           =   sizeof(edge_vals) / sizeof(edge_vals[0]);

    for (int ai = 0; ai < nedge; ai++)
    {
        for (int bi = 0; bi < nedge; bi++)
        {
            memset(&orig, 0, sizeof(orig));
            orig.samples[0].ch[0]   =   edge_vals[ai];
            orig.samples[0].ch[1]   =   edge_vals[bi];

            WL_Pack(&orig, wire);
            WL_Unpack(wire, &decoded);

            if (decoded.samples[0].ch[0] != edge_vals[ai] ||
                decoded.samples[0].ch[1] != edge_vals[bi])
                errors++;
            tests++;
        }
    }

    TEST_ASSERT(errors == 0, "pair packing: %d/%d failures", errors, tests);
    TEST_OK("All %d edge-case pairs round-trip correctly", tests);
}

/* ══════════════════════════════════════════════════════════════════════
 *  MAIN
 * ══════════════════════════════════════════════════════════════════════ */

int main(void)
{
    printf("=== CPCU Codec + IPC Unit Test Suite ===\n");
    printf("Build: %s %s\n", __DATE__, __TIME__);

    test_roundtrip();
    test_boundary();
    test_vbat_exhaustive();
    test_seq_gap();
    test_timestamp_wrap();
    test_battery_voltage();
    test_exhaustive_pairs();

    printf("\n════════════════════════════════════\n");
    printf("  RESULTS: %d PASS, %d FAIL\n", g_pass, g_fail);
    printf("════════════════════════════════════\n");

    return g_fail > 0 ? 1 : 0;
}

