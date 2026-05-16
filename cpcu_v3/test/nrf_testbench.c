/**
 *  @file   nrf_testbench.c
 *  @brief  NRF24L01+ Linux test — SPI connectivity, register read-back, packet RX.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include "nrf24l01_linux.h"

/* Config — must match cpcu_io.c */
#define SPI_DEVICE   "/dev/spidev0.0"
#define SPI_SPEED    8000000
#define GPIO_CE      25
#define RF_CHANNEL   108
#define RF_ADDR      {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}

/* Colors */
#define CG "\033[32m"
#define CR "\033[31m"
#define CY "\033[33m"
#define CC "\033[36m"
#define CD "\033[90m"
#define CB "\033[1m"
#define C0 "\033[0m"

/* Test counters */
static int g_pass = 0, g_fail = 0;

static bool check(const char *name, bool ok, const char *detail)
{
    fprintf(stderr, "  %s[%s]%s %-20s %s%s%s\n",
            ok ? CG : CR, ok ? "PASS" : "FAIL", C0,
            name, CD, detail, C0);
    ok ? g_pass++ : g_fail++;
    return ok;
}

static uint64_t now_ms(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;
}

static void ms_sleep(int ms)
{
    struct timespec ts = { .tv_sec = ms/1000, .tv_nsec = (ms%1000)*1000000L };
    nanosleep(&ts, NULL);
}

/*─── TB-PI-01: SPI Loopback ───*/
static void test_spi(NRF_Handle *h)
{
    uint8_t saved = NRF_ReadReg(h, NRF_REG_SETUP_AW);
    uint8_t tv = (saved == 0x01) ? 0x02 : 0x01;
    NRF_WriteReg(h, NRF_REG_SETUP_AW, tv);
    uint8_t rb = NRF_ReadReg(h, NRF_REG_SETUP_AW);
    char d[80]; snprintf(d, sizeof(d), "wrote=0x%02X read=0x%02X", tv, rb);
    check("SPI_Loopback", rb == tv, d);
    NRF_WriteReg(h, NRF_REG_SETUP_AW, saved);
    rb = NRF_ReadReg(h, NRF_REG_SETUP_AW);
    snprintf(d, sizeof(d), "restored=0x%02X expected=0x%02X", rb, saved);
    check("SPI_Restore", rb == saved, d);
}

/*─── TB-PI-02: Register Audit ───*/
static void test_regs(NRF_Handle *h)
{
    struct { uint8_t r, e, m; const char *n; } t[] = {
        {NRF_REG_SETUP_AW,   0x03, 0x03, "SETUP_AW"},
        {NRF_REG_RF_CH,      RF_CHANNEL, 0x7F, "RF_CH"},
        {NRF_REG_RF_SETUP,   0x0F, 0x0F, "RF_SETUP"},
        {NRF_REG_EN_AA,      0x01, 0x3F, "EN_AA"},
        {NRF_REG_EN_ADDR,    0x01, 0x3F, "EN_RXADDR"},
        {NRF_REG_RX_PW_P0,   NRF_PAYLOAD_SIZE, 0x3F, "RX_PW_P0"},
        {NRF_REG_SETUP_RETR, 0x1F, 0xFF, "SETUP_RETR"},
    };
    for(int i=0; i<(int)(sizeof(t)/sizeof(t[0])); i++)
    {
        uint8_t g = NRF_ReadReg(h, t[i].r);
        char d[80]; snprintf(d, sizeof(d), "got=0x%02X exp=0x%02X mask=0x%02X", g, t[i].e, t[i].m);
        check(t[i].n, (g & t[i].m) == (t[i].e & t[i].m), d);
    }
}

/*─── TB-PI-03: Address Verify ───*/
static void test_addr(NRF_Handle *h, const uint8_t exp[NRF_ADDR_WIDTH])
{
    uint8_t a[NRF_ADDR_WIDTH];
    NRF_ReadRegMulti(h, NRF_REG_RX_ADDR_P0, a, NRF_ADDR_WIDTH);
    char d[80]; snprintf(d, sizeof(d), "got=%02X:%02X:%02X:%02X:%02X",
                         a[0],a[1],a[2],a[3],a[4]);
    check("Addr_P0", memcmp(a, exp, NRF_ADDR_WIDTH)==0, d);
}

/*─── TB-PI-04: FIFO Exercise ───*/
static void test_fifo(NRF_Handle *h)
{
    NRF_CE_Low(h); ms_sleep(1);
    NRF_FlushRX(h); NRF_FlushTX(h);
    uint8_t f = NRF_ReadReg(h, NRF_REG_FIFO_STATUS);
    char d[80]; snprintf(d, sizeof(d), "FIFO=0x%02X (RX_EMPTY=%d TX_EMPTY=%d)",
                         f, !!(f&0x01), !!(f&0x10));
    check("FIFO_Empty", (f&0x01) && (f&0x10), d);
    NRF_FlushRX(h);
    f = NRF_ReadReg(h, NRF_REG_FIFO_STATUS);
    check("FIFO_Flushed", (f & 0x01), "RX empty after flush");
    NRF_CE_High(h);
}

/*─── TB-PI-05: Power Cycle ───*/
static void test_power(NRF_Handle *h)
{
    NRF_PowerDown(h); ms_sleep(2);
    uint8_t c = NRF_ReadReg(h, NRF_REG_CONFIG);
    char d[80]; snprintf(d, sizeof(d), "CONFIG=0x%02X PWR_UP=%d", c, !!(c & NRF_CONFIG_PWR_UP));
    check("PwrDown", !(c & NRF_CONFIG_PWR_UP), d);
    NRF_WriteReg(h, NRF_REG_CONFIG, c | NRF_CONFIG_PWR_UP); ms_sleep(2);
    c = NRF_ReadReg(h, NRF_REG_CONFIG);
    snprintf(d, sizeof(d), "CONFIG=0x%02X PWR_UP=%d", c, !!(c & NRF_CONFIG_PWR_UP));
    check("PwrUp", (c & NRF_CONFIG_PWR_UP), d);
    check("PwrRestore", (c & NRF_CONFIG_PRIM_RX), "PRIM_RX preserved");
    NRF_CE_High(h);
}

/*─── TB-PI-06: RX Ready ───*/
static void test_rx(NRF_Handle *h)
{
    uint8_t c = NRF_ReadReg(h, NRF_REG_CONFIG);
    char d[80]; snprintf(d, sizeof(d), "CONFIG=0x%02X PRIM_RX=%d PWR_UP=%d",
                         c, !!(c&NRF_CONFIG_PRIM_RX), !!(c&NRF_CONFIG_PWR_UP));
    check("RX_Ready", (c&NRF_CONFIG_PRIM_RX) && (c&NRF_CONFIG_PWR_UP), d);
}

/*─── Register Dump ───*/
static void dump_regs(NRF_Handle *h)
{
    struct { uint8_t r; const char *n; } rs[] = {
        {0x00,"CONFIG"},{0x01,"EN_AA"},{0x02,"EN_RXADDR"},{0x03,"SETUP_AW"},
        {0x04,"SETUP_RETR"},{0x05,"RF_CH"},{0x06,"RF_SETUP"},{0x07,"STATUS"},
        {0x08,"OBSERVE_TX"},{0x09,"RPD"},{0x11,"RX_PW_P0"},{0x17,"FIFO_STATUS"},
    };
    fprintf(stderr, "\n%s[REGS]%s\n", CC, C0);
    for(int i=0; i<(int)(sizeof(rs)/sizeof(rs[0])); i++)
    {
        uint8_t v = NRF_ReadReg(h, rs[i].r);
        fprintf(stderr, "  0x%02X %-11s = 0x%02X  ", rs[i].r, rs[i].n, v);
        for(int b=7;b>=0;b--) fprintf(stderr,"%c",(v&(1<<b))?'1':'0');
        fprintf(stderr, "\n");
    }
    uint8_t a[5]; NRF_ReadRegMulti(h, NRF_REG_RX_ADDR_P0, a, 5);
    fprintf(stderr, "  RX_ADDR_P0 = %02X:%02X:%02X:%02X:%02X\n",a[0],a[1],a[2],a[3],a[4]);
}

/*─── Live Reception ───*/
static int run_live(NRF_Handle *h, int tmo_ms)
{
    fprintf(stderr, "%s[LIVE]%s Waiting %ds for packets...\n", CY, C0, tmo_ms/1000);
    uint64_t t0 = now_ms(); int n = 0; uint8_t p[NRF_PAYLOAD_SIZE];
    while((now_ms()-t0) < (uint64_t)tmo_ms)
    {
        if(NRF_DataAvailable(h) && NRF_ReadPayload(h, p)==NRF_OK)
        {
            n++;
            fprintf(stderr, "%s[RX]%s #%d @%llums seq=%u flags=0x%02X\n",
                    CG, C0, n, (unsigned long long)(now_ms()-t0), p[0], p[1]);
            if(n==1 && (int)(tmo_ms-(now_ms()-t0))>500)
                tmo_ms = (int)(now_ms()-t0) + 500;
        }
        else usleep(1000);
    }
    fprintf(stderr, n ? "%s[LIVE]%s %d packets\n":"%s[LIVE]%s none\n", n?CG:CR, C0, n);
    return n ? 0 : 1;
}

/*─── Stress Reception ───*/
static int run_stress(NRF_Handle *h, int want)
{
    fprintf(stderr, "%s[STRESS]%s %d packets...\n", CY, C0, want);
    uint64_t t0=now_ms(); int n=0,errs=0,gaps=0; uint8_t ls=0; bool first=true;
    uint8_t p[NRF_PAYLOAD_SIZE];
    while(n<want && (now_ms()-t0)<30000)
    {
        if(NRF_DataAvailable(h))
        {
            if(NRF_ReadPayload(h,p)==NRF_OK)
            {
                n++;
                if(!first && p[0]!=(uint8_t)(ls+1)) gaps++;
                ls=p[0]; first=false;
                if(n%10==0) fprintf(stderr,"  [%d/%d] seq=%u gaps=%d\n",n,want,ls,gaps);
            } else errs++;
        } else usleep(500);
    }
    uint64_t el=now_ms()-t0;
    float pps=el>0?(float)n/((float)el/1000.0f):0;
    fprintf(stderr,"\n  pkts=%d/%d errs=%d gaps=%d %.1fpkt/s %llums\n",
            n,want,errs,gaps,pps,(unsigned long long)el);
    bool ok=n>=want && errs==0;
    fprintf(stderr,"  %s%s%s\n", ok?CG:CR, ok?"PASS":"FAIL", C0);
    return ok?0:1;
}

/*─── MAIN ───*/
int main(int argc, char *argv[])
{
    bool live=false, stress=false, dump=false;
    int tmo=5000, cnt=100;
    for(int i=1;i<argc;i++)
    {
        if(!strcmp(argv[i],"--live")) live=true;
        else if(!strcmp(argv[i],"--stress")&&i+1<argc) {stress=true;cnt=atoi(argv[++i]);}
        else if(!strcmp(argv[i],"--timeout")&&i+1<argc) tmo=atoi(argv[++i])*1000;
        else if(!strcmp(argv[i],"--dump")) dump=true;
        else if(!strcmp(argv[i],"-h")||!strcmp(argv[i],"--help"))
        {
            fprintf(stderr,"NRF24L01+ Testbench (Pi)\n\n"
                "  --live           wait for BSAU packets\n"
                "  --timeout <sec>  wait timeout (default 5)\n"
                "  --stress <N>     receive N packets\n"
                "  --dump           register dump\n"
                "  -h               this help\n\n"
                "SPI=%s CE=GPIO%d CH=%d\n", SPI_DEVICE, GPIO_CE, RF_CHANNEL);
            return 0;
        }
        else { fprintf(stderr,"Unknown: %s\n",argv[i]); return 1; }
    }

    fprintf(stderr, "\n%s%s═══ NRF24L01+ TESTBENCH ═══%s\n\n", CB, CC, C0);

    int gfd = open("/dev/gpiochip4", O_RDWR);
    if(gfd<0) gfd = open("/dev/gpiochip0", O_RDWR);
    if(gfd<0) { fprintf(stderr,"%s[FATAL]%s gpiochip: %s (sudo?)\n",CR,C0,strerror(errno)); return 1; }

    NRF_Handle nrf;
    const uint8_t addr[NRF_ADDR_WIDTH] = RF_ADDR;
    NRF_Status ret = NRF_ERR_NOT_DETECTED;
    for(int a=0;a<3;a++)
    {
        if(a) { fprintf(stderr,"[INIT] retry %d...\n",a+1); ms_sleep(200); }
        ret = NRF_Init(&nrf, SPI_DEVICE, SPI_SPEED, gfd, GPIO_CE, RF_CHANNEL, addr);
        if(ret==NRF_OK) break;
    }
    if(ret!=NRF_OK) { fprintf(stderr,"%s[FATAL]%s NRF_Init=%d\n",CR,C0,ret); close(gfd); return 1; }
    fprintf(stderr, "%s[INIT]%s OK\n\n", CG, C0);

    if(dump) dump_regs(&nrf);

    fprintf(stderr, "%s%s─── SELF-TEST ───%s\n\n", CB, CC, C0);
    test_spi(&nrf);
    test_regs(&nrf);
    test_addr(&nrf, addr);
    test_fifo(&nrf);
    test_power(&nrf);
    test_rx(&nrf);

    fprintf(stderr, "\n  %s%s=== %d PASS, %d FAIL ===%s\n",
            CB, g_fail?CR:CG, g_pass, g_fail, C0);

    int xr = 0;
    if(live && !g_fail)    { fprintf(stderr,"\n%s%s─── LIVE ───%s\n",CB,CC,C0); xr|=run_live(&nrf,tmo); }
    if(stress && !g_fail)  { fprintf(stderr,"\n%s%s─── STRESS ───%s\n",CB,CC,C0); xr|=run_stress(&nrf,cnt); }
    if(dump) { fprintf(stderr,"\n--- after ---"); dump_regs(&nrf); }

    NRF_Close(&nrf); close(gfd);
    return g_fail ? 1 : xr;
}

