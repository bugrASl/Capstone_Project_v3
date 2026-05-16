/**
 *  @file       nrf24l01_test.c
 *  @brief      On-target self-test suite for NRF24L01+ driver
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.1
 *  @details    Each test is non-destructive: the radio state is restored
 *              after each sub-test so the chip remains operational for the
 *              following tests and for normal application traffic.
 *
 *              All tests use the LOG macro for output. The board prefix
 *              [BSAU] or [CPCU] is inserted automatically by log.h based
 *              on LOG_BOARD_xxx.
 *
 *              v2.1 changes:
 *                  - NRF_Test_Registers: expected RF_SETUP value was
 *                    `RF_DR_LOW | RF_PWR_3` (= 0x26, v2.0 @ 250 kbps).
 *                    v2.1 driver programs `RF_DR_HIGH | RF_PWR_3`
 *                    (= 0x0E, 2 Mbps). Also expected SETUP_RETR was 0x5F
 *                    (ARD=1500 µs, v2.0). v2.1 driver programs 0x1F
 *                    (ARD=500 µs). Without these fixes, a correctly-
 *                    running v2.1 radio would spuriously fail the audit.
 *                  - NRF_Test_TX: commented timeout updated from 75 ms
 *                    to 20 ms to match the v2.1 NRF_Transmit timeout.
 *                  - Style polish: 98-char banners, Allman braces in all
 *                    single-statement blocks, column-aligned assignments,
 *                    @version / @date headers.
 */

#include "nrf24l01_test.h"
#include "log.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/*============= PRIVATE: Test assertion helper =================================================*/
/*
 *  Lightweight, logs inline, returns bool. Defined locally so
 *  nrf24l01_test.c is self-contained.
 */
static bool nrf_assert(bool condition, const char *name, const char *fmt, ...)
{
    char    detail[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(detail, sizeof(detail), fmt, args);
    va_end(args);

    if (condition)
    {
        LOG("NRF", name, "PASS", "%s", detail);
    }
    else
    {
        LOG("NRF", name, "FAIL", "%s", detail);
    }

    return condition;
}

/*============= TEST 1: SPI Loopback ===========================================================
 *
 *  Strategy:
 *      SETUP_AW (reg 0x03) is writable with values 0x01-0x03.
 *      Write a different valid value, read it back, then restore.
 *      Values: 0x01 = 3 bytes, 0x02 = 4 bytes, 0x03 = 5 bytes.
 *==============================================================================================*/
TestResult NRF_Test_SPI(NRF_Handle *hnrf)
{
    LOG("NRF", "Test_SPI", "RUN", "SPI write/readback on SETUP_AW");

    bool    pass                        =   true;
    uint8_t saved                       =   NRF_ReadReg(hnrf, NRF_REG_SETUP_AW);

    /* Write */
    uint8_t test_val                    =   (saved == 0x01) ? 0x02 : 0x01;
    NRF_WriteReg(hnrf, NRF_REG_SETUP_AW, test_val);
    uint8_t readback                    =   NRF_ReadReg(hnrf, NRF_REG_SETUP_AW);

    pass                               &=   nrf_assert(readback == test_val, "SPI_Loopback",
                                                       "wrote=0x%02X, read=0x%02X",
                                                       test_val, readback);

    /* Restore */
    NRF_WriteReg(hnrf, NRF_REG_SETUP_AW, saved);
    uint8_t restored                    =   NRF_ReadReg(hnrf, NRF_REG_SETUP_AW);

    pass                               &=   nrf_assert(restored == saved, "SPI_Restore",
                                                       "restored=0x%02X, expected=0x%02X",
                                                       restored, saved);

    return pass ? TEST_PASS : TEST_FAIL;
}

/*============= TEST 2: Register Audit =========================================================
 *
 *  Strategy:
 *      Read back every register NRF_Init() programmed and compare against
 *      expected values. Uses a mask per register because some fields are
 *      reserved or role-dependent.
 *
 *  v2.1 expected values (authoritative — see nrf24l01.c § NRF_Init):
 *      RF_SETUP     = RF_DR_HIGH | RF_PWR_3  (0x0E: 2 Mbps, 0 dBm)
 *      SETUP_RETR   = 0x1F                   (ARD=500 µs, ARC=15)
 *      EN_AA        = 0x01                   (auto-ACK pipe 0 only)
 *      EN_RXADDR    = 0x01                   (pipe 0 enabled)
 *      RX_PW_P0     = NRF_PAYLOAD_SIZE       (32)
 *==============================================================================================*/

static bool check_reg(NRF_Handle *hnrf, const uint8_t reg, const uint8_t expected,
                      const uint8_t mask, const char *name)
{
    uint8_t actual                      =   NRF_ReadReg(hnrf, reg);
    uint8_t masked_actual               =   actual   & mask;
    uint8_t masked_expected             =   expected & mask;

    return nrf_assert(masked_expected == masked_actual, "Reg_Check",
                      "%s:    got=0x%02X, expected=0x%02X, mask=0x%02X",
                      name, masked_actual, masked_expected, mask);
}

TestResult NRF_Test_Registers(NRF_Handle *hnrf)
{
    LOG("NRF", "Test_Registers", "RUN", "Configuration check on post-init registers");

    bool pass                           =   true;

    /* SETUP_AW: (NRF_ADDR_WIDTH - 2) = 0x03 for 5-byte addr. Bits [1:0]. */
    pass                               &=   check_reg(hnrf, NRF_REG_SETUP_AW,
                                                      NRF_ADDR_WIDTH - 2, 0x03, "SETUP_AW");

    /* RF_CH: bits [6:0] */
    pass                               &=   check_reg(hnrf, NRF_REG_RF_CH,
                                                      hnrf->channel, 0x7F, "RF_CH");

    /*
     *  RF_SETUP: v2.1 programs RF_DR_HIGH=1, RF_DR_LOW=0, RF_PWR=0x03
     *  -> 2 Mbps at 0 dBm. Mask 0x2E covers bits 5 (RF_DR_LOW),
     *  3 (RF_DR_HIGH), and 2:1 (RF_PWR). Bit 0 is reserved on NRF24L01+
     *  so we don't check it (driver leaves it 0, the plain NRF24L01 had
     *  it as LNA_HCURR).
     */
    pass                               &=   check_reg(hnrf, NRF_REG_RF_SETUP,
                                                      NRF_RF_SETUP_RF_DR_HIGH |
                                                      NRF_RF_SETUP_RF_PWR_3,
                                                      0x2E, "RF_SETUP");

    /* EN_AA: auto-ACK pipe 0 only */
    pass                               &=   check_reg(hnrf, NRF_REG_EN_AA,
                                                      0x01, 0x3F, "EN_AA");

    /* EN_RXADDR: pipe 0 enabled */
    pass                               &=   check_reg(hnrf, NRF_REG_EN_RXADDR,
                                                      0x01, 0x3F, "EN_RXADDR");

    /* RX_PW_P0: payload width */
    pass                               &=   check_reg(hnrf, NRF_REG_RX_PW_P0,
                                                      NRF_PAYLOAD_SIZE, 0x3F, "RX_PW_P0");

    /*
     *  SETUP_RETR: v2.1 programs ARD=500 µs (0x1), ARC=15 (0xF) -> 0x1F.
     *  This is the minimum ARD permitted at 2 Mbps with 32-byte payload
     *  and 2-byte CRC (datasheet table 19).
     */
    pass                               &=   check_reg(hnrf, NRF_REG_SETUP_RETR,
                                                      0x1F, 0xFF, "SETUP_RETR");

    /* CONFIG: role-dependent */
    uint8_t expected_config             =   NRF_CONFIG_EN_CRC
                                          | NRF_CONFIG_CRCO
                                          | NRF_CONFIG_PWR_UP;
#if defined(NRF_ROLE_RX)
    expected_config                    |=   NRF_CONFIG_PRIM_RX
                                          | NRF_CONFIG_MASK_TX_DS
                                          | NRF_CONFIG_MASK_MAX_RT;
#else
    expected_config                    |=   NRF_CONFIG_MASK_RX_DR;
#endif
    pass                               &=   check_reg(hnrf, NRF_REG_CONFIG,
                                                      expected_config, 0x7F, "CONFIG");

    return pass ? TEST_PASS : TEST_FAIL;
}

/*============= TEST 3: Address Verification ===================================================
 *
 *  Strategy:
 *      Read back the 5-byte address from RX_ADDR_P0 (and TX_ADDR for TX).
 *      Addresses are stored LSByte-first on SPI, so memcmp against the
 *      same byte order as programmed.
 *==============================================================================================*/
TestResult NRF_Test_Address(NRF_Handle *hnrf, const uint8_t addr[NRF_ADDR_WIDTH])
{
    LOG("NRF", "Test_Addr", "RUN", "Verify PIPE_0 address");

    bool pass                           =   true;

    /* Read RX_ADDR_P0 */
    uint8_t readback[NRF_ADDR_WIDTH];
    NRF_ReadRegMulti(hnrf, NRF_REG_RX_ADDR_P0, readback, NRF_ADDR_WIDTH);

    bool rx_match                       =   (memcmp(readback, addr, NRF_ADDR_WIDTH) == 0);
    pass                               &=   nrf_assert(rx_match, "Addr_RX_P0",
                                                       "got=%02X:%02X:%02X:%02X:%02X "
                                                       "exp=%02X:%02X:%02X:%02X:%02X",
                                                       readback[0], readback[1], readback[2],
                                                       readback[3], readback[4],
                                                       addr[0], addr[1], addr[2],
                                                       addr[3], addr[4]);
#if defined(NRF_ROLE_TX)
    /* Read TX_ADDR */
    NRF_ReadRegMulti(hnrf, NRF_REG_TX_ADDR, readback, NRF_ADDR_WIDTH);

    bool tx_match                       =   (memcmp(readback, addr, NRF_ADDR_WIDTH) == 0);
    pass                               &=   nrf_assert(tx_match, "Addr_TX",
                                                       "got=%02X:%02X:%02X:%02X:%02X "
                                                       "exp=%02X:%02X:%02X:%02X:%02X",
                                                       readback[0], readback[1], readback[2],
                                                       readback[3], readback[4],
                                                       addr[0], addr[1], addr[2],
                                                       addr[3], addr[4]);
#endif

    return pass ? TEST_PASS : TEST_FAIL;
}

/*============= TEST 4: FIFO Exercise ==========================================================
 *
 *  Strategy:
 *      Verify FIFOs are empty, write a dummy payload, verify non-empty,
 *      flush, verify empty again. Non-destructive: no air time.
 *==============================================================================================*/
TestResult NRF_Test_FIFO(NRF_Handle *hnrf)
{
    LOG("NRF", "Test_FIFO", "RUN", "Verify FIFO status write/flush");

    bool pass                           =   true;

    /* Enter standby mode for safe access */
#if defined(NRF_ROLE_RX)
    ce_low();
    HAL_Delay(1);
#endif

    /* FIFOs empty at start */
    uint8_t fifo                        =   NRF_ReadReg(hnrf, NRF_REG_FIFO_STATUS);
    pass                               &=   nrf_assert((fifo & NRF_FIFO_TX_EMPTY) != 0,
                                                       "FIFO_InitEmpty",
                                                       "TX_EMPTY=%u (expect 1)",
                                                       (fifo >> 4) & 1);
    pass                               &=   nrf_assert((fifo & 0x01) != 0,
                                                       "FIFO_RxEmpty",
                                                       "RX_EMPTY=%u (expect 1)",
                                                       fifo & 1);

    /* Write a payload */
    uint8_t payload[NRF_PAYLOAD_SIZE];
    memset(payload, 0xAA, NRF_PAYLOAD_SIZE);

    uint8_t cmd                         =   NRF_CMD_W_TX_PAYLOAD;
    csn_low();
    HAL_SPI_Transmit(hnrf->hspi, &cmd, 1,                           NRF_SPI_TIMEOUT_MS);
    HAL_SPI_Transmit(hnrf->hspi, payload, NRF_PAYLOAD_SIZE,         NRF_SPI_TIMEOUT_MS);
    csn_high();

    /* TX-FIFO not empty after write */
    fifo                                =   NRF_ReadReg(hnrf, NRF_REG_FIFO_STATUS);
    pass                               &=   nrf_assert((fifo & NRF_FIFO_TX_EMPTY) == 0,
                                                       "FIFO_NonEmpty",
                                                       "TX_EMPTY=%u (post-write, expect 0)",
                                                       (fifo >> 4) & 1);

    /* Flush */
    NRF_FlushTX(hnrf);

    /* FIFOs empty after flush */
    fifo                                =   NRF_ReadReg(hnrf, NRF_REG_FIFO_STATUS);
    pass                               &=   nrf_assert((fifo & NRF_FIFO_TX_EMPTY) != 0,
                                                       "FIFO_Flushed",
                                                       "TX_EMPTY=%u after flush (expect 1)",
                                                       (fifo >> 4) & 1);

    /* Restore RX CE state */
#if defined(NRF_ROLE_RX)
    ce_high();
#endif

    return pass ? TEST_PASS : TEST_FAIL;
}

/*============= TEST 5: Power Cycle ============================================================
 *
 *  Strategy:
 *      1. Read current CONFIG to save state.
 *      2. NRF_PowerDown() -> verify PWR_UP = 0.
 *      3. NRF_PowerUp()   -> verify PWR_UP = 1.
 *      4. Verify CONFIG matches the saved state (role bits preserved).
 *
 *      Tests the power control path end-to-end: CONFIG register write,
 *      oscillator startup timing, and CE restoration. Non-destructive —
 *      restores original state on exit.
 *==============================================================================================*/
TestResult NRF_Test_PowerCycle(NRF_Handle *hnrf)
{
    LOG("NRF", "Test_PwrCycle", "RUN", "Power down -> verify -> power up -> verify");

    bool pass                           =   true;

    /* Save current CONFIG */
    uint8_t saved_config                =   NRF_ReadReg(hnrf, NRF_REG_CONFIG);

    /* Power down */
    NRF_PowerDown(hnrf);
    uint8_t cfg_down                    =   NRF_ReadReg(hnrf, NRF_REG_CONFIG);
    pass                               &=   nrf_assert((cfg_down & NRF_CONFIG_PWR_UP) == 0,
                                                       "PwrDown",
                                                       "PWR_UP=%u after PowerDown (expect 0)",
                                                       (cfg_down >> 1) & 1);

    /* SPI still works in power-down (datasheet: SPI accessible in all states) */
    uint8_t ch_check                    =   NRF_ReadReg(hnrf, NRF_REG_RF_CH);
    pass                               &=   nrf_assert(ch_check == hnrf->channel, "PwrDown_SPI",
                                                       "RF_CH=0x%02X while powered down "
                                                       "(expect 0x%02X)",
                                                       ch_check, hnrf->channel);

    /* Power up (includes 2 ms oscillator startup + CE restore for RX) */
    NRF_PowerUp(hnrf);
    uint8_t cfg_up                      =   NRF_ReadReg(hnrf, NRF_REG_CONFIG);
    pass                               &=   nrf_assert((cfg_up & NRF_CONFIG_PWR_UP) != 0,
                                                       "PwrUp",
                                                       "PWR_UP=%u after PowerUp (expect 1)",
                                                       (cfg_up >> 1) & 1);

    /* CONFIG matches original — role bits, CRC, IRQ masks all preserved */
    pass                               &=   nrf_assert(cfg_up == saved_config, "PwrRestore",
                                                       "CONFIG=0x%02X after cycle "
                                                       "(expect 0x%02X)",
                                                       cfg_up, saved_config);

    return pass ? TEST_PASS : TEST_FAIL;
}

/*============= TEST 6: NRF_Test_TX ============================================================
 *
 *  Strategy:
 *      Transmit a test packet with Enhanced ShockBurst (auto-ACK).
 *      NRF_Transmit internally polls STATUS for TX_DS or MAX_RT.
 *
 *      ┌──────────────────────────────────────────────────────────┐
 *      │ STATUS bit │ Meaning               │ Test verdict        │
 *      ├────────────┼───────────────────────┼─────────────────────┤
 *      │ TX_DS=1    │ ACK received from PRX │ PASS (link alive)   │
 *      │ MAX_RT=1   │ 15 retries exhausted  │ PASS (RF SM works)  │
 *      │ neither    │ timeout (20 ms, v2.1) │ FAIL (HW fault)     │
 *      └──────────────────────────────────────────────────────────┘
 *
 *      Why MAX_RT = PASS:
 *          The chip performed the full PTX flowchart from the datasheet:
 *          loaded payload -> PLL lock -> TX -> RX for ACK -> timeout ->
 *          retransmit × 15 -> set MAX_RT. Every block in the RF chain
 *          ran. The only thing missing is the other radio on the air.
 *==============================================================================================*/
#if defined(NRF_ROLE_TX)

TestResult NRF_Test_TX(NRF_Handle *hnrf)
{
    LOG("NRF", "Test_TX", "RUN", "Transmit test packet: checking TX_DS/MAX_RT");

    bool pass                           =   true;

    /* Recognizable test payload */
    uint8_t test_payload[NRF_PAYLOAD_SIZE];
    memset(test_payload, 0, NRF_PAYLOAD_SIZE);
    test_payload[0]                     =   0xAB;
    test_payload[1]                     =   0xCD;
    test_payload[2]                     =   0xEF;
    test_payload[3]                     =   0x7F;

    /* Blocking transmit — handles CE pulse and polling internally */
    NRF_Status result                   =   NRF_Transmit(hnrf, test_payload);

    /* Read STATUS and OBSERVE_TX for diagnostics */
    uint8_t status                      =   NRF_GetStatus(hnrf);
    uint8_t lost, retx;
    NRF_GetTxStats(hnrf, &lost, &retx);

    if (result == NRF_OK)
    {
        pass                           &=   nrf_assert(true, "TX_Result",
                                                       "TX_DS=1 — ACK received "
                                                       "(PLOS=%u ARC=%u)",
                                                       lost, retx);
    }
    else if (result == NRF_ERR_TX_MAX_RT)
    {
        pass                           &=   nrf_assert(true, "TX_Result",
                                                       "MAX_RT=1 — no receiver, state "
                                                       "machine OK (PLOS=%u ARC=%u)",
                                                       lost, retx);
    }
    else if (result == NRF_ERR_TX_TIMEOUT)
    {
        pass                           &=   nrf_assert(false, "TX_Result",
                                                       "TIMEOUT — neither TX_DS nor MAX_RT "
                                                       "in 20 ms. STATUS=0x%02X",
                                                       status);
    }
    else
    {
        pass                           &=   nrf_assert(false, "TX_Result",
                                                       "UNEXPECTED ERROR=%d STATUS=0x%02X",
                                                       result, status);
    }

    LOG("NRF", "TX_Diag", "INFO",
        "STATUS=0x%02X (RX_DR=%u TX_DS=%u MAX_RT=%u) OBSERVE=PLOS:%u ARC:%u",
        status,
        (status >> 6) & 1,
        (status >> 5) & 1,
        (status >> 4) & 1,
        lost, retx);

    return pass ? TEST_PASS : TEST_FAIL;
}

#endif  /* NRF_ROLE_TX */

/*============= TEST 7: NRF_Test_RX ============================================================
 *
 *  Strategy:
 *      Phase A — Config check (always):
 *          Verify CONFIG has PRIM_RX=1, PWR_UP=1, CE is high.
 *
 *      Phase B — Live packet reception (if NRF_TEST_RX_TIMEOUT_MS > 0):
 *          Clear stale flags, flush RX FIFO, poll STATUS for RX_DR.
 *          When RX_DR fires, verify pipe number and read payload.
 *==============================================================================================*/
#if defined(NRF_ROLE_RX)

TestResult NRF_Test_RX(NRF_Handle *hnrf)
{
    LOG("NRF", "Test_RX", "RUN", "Checking RX config + waiting for packet");

    bool pass                           =   true;

    /*-------------- Phase A: Configuration check ---------------------------------------------*/
    uint8_t config                      =   NRF_ReadReg(hnrf, NRF_REG_CONFIG);
    pass                               &=   nrf_assert((config & NRF_CONFIG_PWR_UP) != 0,
                                                       "RX_PwrUp",
                                                       "PWR_UP=%u (expect 1)",
                                                       (config >> 1) & 1);
    pass                               &=   nrf_assert((config & NRF_CONFIG_PRIM_RX) != 0,
                                                       "RX_PrimRx",
                                                       "PRIM_RX=%u (expect 1)",
                                                       config & 1);
    GPIO_PinState ce_state              =   HAL_GPIO_ReadPin(NRF_CE_GPIO_Port, NRF_CE_Pin);
    pass                               &=   nrf_assert(ce_state == GPIO_PIN_SET,
                                                       "RX_CE",
                                                       "CE=%u (expect 1)",
                                                       ce_state);

    /*-------------- Phase B: Live packet reception --------------------------------------------*/
#if NRF_TEST_RX_TIMEOUT_MS > 0

    LOG("NRF", "Test_RX", "INFO",
        "Waiting up to %u ms for packet from BSAU...", NRF_TEST_RX_TIMEOUT_MS);

    /* Start clean: clear pending IRQ flags and flush RX FIFO */
    NRF_ClearIRQ(hnrf, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    NRF_FlushRX(hnrf);

    /* Poll STATUS for RX_DR */
    bool        pkt_received            =   false;
    uint8_t     rx_status               =   0;
    uint32_t    start                   =   HAL_GetTick();

    while ((HAL_GetTick() - start) < NRF_TEST_RX_TIMEOUT_MS)
    {
        rx_status                       =   NRF_GetStatus(hnrf);

        if (rx_status & NRF_STATUS_RX_DR)
        {
            pkt_received                =   true;
            break;
        }
    }

    if (!pkt_received)
    {
        pass                           &=   nrf_assert(false, "RX_Packet",
                                                       "no RX_DR after %u ms — is BSAU "
                                                       "transmitting? STATUS=0x%02X",
                                                       NRF_TEST_RX_TIMEOUT_MS, rx_status);
        return pass ? TEST_PASS : TEST_FAIL;
    }

    /* RX_DR fired — verify pipe number */
    uint8_t pipe                        =   (rx_status & NRF_STATUS_RX_P_NO_MASK) >> 1;
    pass                               &=   nrf_assert(pipe == 0, "RX_Pipe",
                                                       "RX_P_NO=%u (expect 0)", pipe);

    /* Read the payload */
    uint8_t payload[NRF_PAYLOAD_SIZE];
    NRF_Status read_result              =   NRF_ReadPayload(hnrf, payload);
    pass                               &=   nrf_assert(read_result == NRF_OK, "RX_ReadPayload",
                                                       "NRF_ReadPayload returned %d "
                                                       "(expect 0=OK)",
                                                       read_result);

    /* Clear RX_DR */
    NRF_ClearIRQ(hnrf, NRF_STATUS_RX_DR);

    /* Post-read diagnostics */
    uint8_t post_status                 =   NRF_GetStatus(hnrf);
    uint8_t fifo                        =   NRF_ReadReg(hnrf, NRF_REG_FIFO_STATUS);

    LOG("NRF", "RX_PostRead", "INFO",
        "STATUS=0x%02X FIFO=0x%02X (RX_EMPTY=%u) payload[0..3]=%02X %02X %02X %02X",
        post_status, fifo, fifo & 0x01,
        payload[0], payload[1], payload[2], payload[3]);

    /* Drain remaining packets for throughput check */
    uint32_t extra_count                =   0;
    while (NRF_DataAvailable(hnrf))
    {
        uint8_t discard[NRF_PAYLOAD_SIZE];
        if (NRF_ReadPayload(hnrf, discard) == NRF_OK)
        {
            extra_count++;
        }
        else
        {
            break;
        }
    }

    uint32_t elapsed                    =   HAL_GetTick() - start;
    LOG("NRF", "RX_Summary", "INFO",
        "received 1+%lu packets in %lu ms",
        (unsigned long)extra_count, (unsigned long)elapsed);

#endif  /* NRF_TEST_RX_TIMEOUT_MS > 0 */

    return pass ? TEST_PASS : TEST_FAIL;
}

#endif  /* NRF_ROLE_RX */

/*============= MASTER TEST RUNNER =============================================================*/

TestResult NRF_Test_All(NRF_Handle *hnrf, const uint8_t addr[NRF_ADDR_WIDTH])
{
    LOG("NRF", "Test_All", "RUN", "=== NRF24L01+ SELF-TEST START ===");

    uint32_t    pass                    =   0;
    uint32_t    fail                    =   0;
    TestResult  r;

    /* Test 1: SPI communication */
    r                                   =   NRF_Test_SPI(hnrf);
    if (r == TEST_PASS) { pass++; } else { fail++; }

    /* Test 2: Register configuration */
    r                                   =   NRF_Test_Registers(hnrf);
    if (r == TEST_PASS) { pass++; } else { fail++; }

    /* Test 3: Address verification */
    r                                   =   NRF_Test_Address(hnrf, addr);
    if (r == TEST_PASS) { pass++; } else { fail++; }

    /* Test 4: FIFO exercise */
    r                                   =   NRF_Test_FIFO(hnrf);
    if (r == TEST_PASS) { pass++; } else { fail++; }

    /* Test 5: Power cycle */
    r                                   =   NRF_Test_PowerCycle(hnrf);
    if (r == TEST_PASS) { pass++; } else { fail++; }

    /* Test 6/7: Role-specific */
#if defined(NRF_ROLE_TX)
    r                                   =   NRF_Test_TX(hnrf);
    if (r == TEST_PASS) { pass++; } else { fail++; }
#endif

#if defined(NRF_ROLE_RX)
    r                                   =   NRF_Test_RX(hnrf);
    if (r == TEST_PASS) { pass++; } else { fail++; }
#endif

    bool all_pass                       =   (fail == 0);

    LOG("NRF", "Test_All", all_pass ? "PASS" : "FAIL",
        "=== NRF SELF-TEST: %lu PASS, %lu FAIL ===",
        (unsigned long)pass, (unsigned long)fail);

    return all_pass ? TEST_PASS : TEST_FAIL;
}

/*==============================================================================================*/
