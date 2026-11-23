/*
 * main.c - bare-metal (Vitis standalone) test application for the
 * moving-object-detection overlay on any Zynq-7000 board.
 *
 * A synthetic scene (textured background, sensor noise, moving
 * rectangles) is generated in DDR, processed frame by frame by the PL
 * core, and the detected bounding boxes plus per-frame timing are
 * printed on the UART (115200 8N1).
 */
#include <stdio.h>
#include <string.h>
#include "xparameters.h"
#include "xaxidma.h"
#include "xil_cache.h"
#include "xil_io.h"
#include "xtime_l.h"
#include "mod_hw.h"

#define W        640
#define H        480
#define NPIX     (W * H)
#define NFRAMES  200

/* base addresses: names follow the block design cell names */
#ifndef XPAR_MOD_TOP_0_BASEADDR
#define XPAR_MOD_TOP_0_BASEADDR 0x43C00000u   /* adjust to the address editor */
#endif
#define CORE XPAR_MOD_TOP_0_BASEADDR

static uint32_t pix[NPIX]            __attribute__((aligned(64)));
static uint8_t  mask[NPIX]           __attribute__((aligned(64)));
static uint64_t par[2][NPIX * 2]     __attribute__((aligned(64)));

static XAxiDma dma_par, dma_pix;

static inline void wr(uint32_t off, uint32_t v) { Xil_Out32(CORE + off, v); }
static inline uint32_t rd(uint32_t off)         { return Xil_In32(CORE + off); }

static int dma_init(XAxiDma *d, UINTPTR base_or_id)
{
    XAxiDma_Config *cfg;
#ifdef SDT
    cfg = XAxiDma_LookupConfig(base_or_id);
#else
    cfg = XAxiDma_LookupConfig((u32)base_or_id);
#endif
    if (!cfg || XAxiDma_CfgInitialize(d, cfg) != XST_SUCCESS) return -1;
    XAxiDma_IntrDisable(d, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
    XAxiDma_IntrDisable(d, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
    return 0;
}

/* ------------------------------------------------------------------ scene */
static uint32_t lfsr = 0x12345678u;
static inline uint32_t rnd(void) { lfsr ^= lfsr << 13; lfsr ^= lfsr >> 17; lfsr ^= lfsr << 5; return lfsr; }
static inline int noise(void) { return (int)(rnd() & 7) - 3 + (int)(rnd() & 3) - 1; }

typedef struct { int x, y, w, h, vx, vy; uint8_t r, g, b; } box_t;
static box_t boxes[3] = {
    { 40, 300, 60, 120,  3,  1, 220,  40,  40},
    {400, 250, 80,  60, -4,  2,  30, 200,  60},
    {250, 380, 40,  80,  2, -1,  40,  60, 230},
};

static void make_frame(int t)
{
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int base = 60 + (x * 100) / W + ((x / 8 + y / 8) & 1) * 20;
            int r = base + noise(), g = base - 10 + noise(), b = base - 20 + noise();
            for (int k = 0; k < 3; k++) {
                box_t *o = &boxes[k];
                if (t >= 10 && x >= o->x && x < o->x + o->w && y >= o->y && y < o->y + o->h) {
                    r = o->r + noise(); g = o->g + noise(); b = o->b + noise();
                }
            }
            r = r < 0 ? 0 : r > 255 ? 255 : r;
            g = g < 0 ? 0 : g > 255 ? 255 : g;
            b = b < 0 ? 0 : b > 255 ? 255 : b;
            pix[y * W + x] = (uint32_t)r | ((uint32_t)g << 8) | ((uint32_t)b << 16);
        }
    if (t >= 10)
        for (int k = 0; k < 3; k++) {
            box_t *o = &boxes[k];
            o->x += o->vx; o->y += o->vy;
            if (o->x < 0 || o->x + o->w >= W) { o->vx = -o->vx; o->x += 2 * o->vx; }
            if (o->y < 100 || o->y + o->h >= H) { o->vy = -o->vy; o->y += 2 * o->vy; }
        }
}

/* place a field of up to 64 bits at bit offset 'off' of a 128-bit word */
static inline void put128(uint64_t *lo, uint64_t *hi, uint64_t v, unsigned off)
{
    if (off < 64) {
        *lo |= v << off;
        if (off > 0) *hi |= v >> (64 - off);
    } else {
        *hi |= v << (off - 64);
    }
}

/* Bootstrap: G0 = (w=1, mu=x, s2=var_init, matchsum=1), G1 = G2 = (0, 0, var_init, 0) */
static void init_model(uint64_t *p, uint32_t fmt, uint32_t var_init)
{
    unsigned WB = fmt & 0xF, FM = (fmt >> 4) & 0xF, VB = ((fmt >> 8) & 0xF) + 8, SB = (fmt >> 12) & 0xF;
    unsigned MB = 8 + FM, VS = 14 - VB, GW = WB + MB + VB + SB;
    uint64_t vinit = var_init >> VS;
    if (vinit > ((1ull << VB) - 1)) vinit = (1ull << VB) - 1;
    for (int i = 0; i < NPIX; i++) {
        uint32_t c = pix[i];
        uint64_t gray = (77u * (c & 0xFF) + 150u * ((c >> 8) & 0xFF) + 29u * ((c >> 16) & 0xFF)) >> 8;
        uint64_t vf = vinit << (WB + MB);
        uint64_t g0 = ((1ull << WB) - 1) | ((gray << FM) << WB) | vf | (1ull << (WB + MB + VB));
        uint64_t lo = 0, hi = 0;
        put128(&lo, &hi, g0, 0);
        put128(&lo, &hi, vf, GW);
        put128(&lo, &hi, vf, 2 * GW);
        p[2 * i] = lo;
        p[2 * i + 1] = hi;
    }
}

/* ------------------------------------------------------------------ main */
int main(void)
{
    printf("\r\nGMM + CCL moving object detector, %dx%d\r\n", W, H);
#ifdef SDT
    if (dma_init(&dma_par, XPAR_AXI_DMA_PAR_BASEADDR) || dma_init(&dma_pix, XPAR_AXI_DMA_PIX_BASEADDR)) {
#else
    if (dma_init(&dma_par, XPAR_AXI_DMA_PAR_DEVICE_ID) || dma_init(&dma_pix, XPAR_AXI_DMA_PIX_DEVICE_ID)) {
#endif
        printf("DMA init failed\r\n");
        return -1;
    }
    if (rd(MOD_REG_ID) != MOD_CORE_ID) {
        printf("core not found (ID 0x%08lx)\r\n", (unsigned long)rd(MOD_REG_ID));
        return -1;
    }

    wr(MOD_REG_CTRL, 1);
    wr(MOD_REG_WIDTH, W);           wr(MOD_REG_HEIGHT, H);
    wr(MOD_REG_SHIFTS, 7u);                       /* alpha_w = 2^-7 */
    wr(MOD_REG_LAMBDA2, 100);       wr(MOD_REG_T_BG, 2867);
    wr(MOD_REG_VAR_INIT, 900);      wr(MOD_REG_VAR_MIN, 16);
    wr(MOD_REG_VAR_MAX, 16383);
    wr(MOD_REG_MIN_AREA, 64);
    wr(MOD_REG_CTRL, MOD_MORPH_CLOSE_OPEN << 2);

    int cur = 0;
    XTime t0, t1;
    double acc_ms = 0.0;
    for (int f = 0; f < NFRAMES; f++) {
        make_frame(f);
        if (f == 0) init_model(par[0], rd(MOD_REG_GMM_FMT), 900);
        Xil_DCacheFlushRange((UINTPTR)pix, sizeof(pix));
        Xil_DCacheFlushRange((UINTPTR)par[cur], sizeof(par[cur]));
        Xil_DCacheInvalidateRange((UINTPTR)par[cur ^ 1], sizeof(par[cur ^ 1]));
        Xil_DCacheInvalidateRange((UINTPTR)mask, sizeof(mask));

        /* warm-up schedule: alpha_w = 2^-min(7, ceil(log2(f+1))) */
        uint32_t a_t = 0;
        while (a_t < 7u && (1u << a_t) < (uint32_t)(f + 1)) a_t++;
        wr(MOD_REG_SHIFTS, a_t);
        uint32_t fc0 = rd(MOD_REG_STATUS) >> 16;
        XTime_GetTime(&t0);
        XAxiDma_SimpleTransfer(&dma_par, (UINTPTR)par[cur ^ 1], NPIX * 16, XAXIDMA_DEVICE_TO_DMA);
        XAxiDma_SimpleTransfer(&dma_pix, (UINTPTR)mask,         NPIX,      XAXIDMA_DEVICE_TO_DMA);
        XAxiDma_SimpleTransfer(&dma_par, (UINTPTR)par[cur],     NPIX * 16, XAXIDMA_DMA_TO_DEVICE);
        XAxiDma_SimpleTransfer(&dma_pix, (UINTPTR)pix,          NPIX * 4,  XAXIDMA_DMA_TO_DEVICE);
        while (XAxiDma_Busy(&dma_par, XAXIDMA_DMA_TO_DEVICE) || XAxiDma_Busy(&dma_pix, XAXIDMA_DMA_TO_DEVICE) ||
               XAxiDma_Busy(&dma_par, XAXIDMA_DEVICE_TO_DMA) || XAxiDma_Busy(&dma_pix, XAXIDMA_DEVICE_TO_DMA))
            ;
        while ((rd(MOD_REG_STATUS) >> 16) == fc0)
            ;
        XTime_GetTime(&t1);
        Xil_DCacheInvalidateRange((UINTPTR)mask, sizeof(mask));
        cur ^= 1;

        double ms = 1000.0 * (double)(t1 - t0) / COUNTS_PER_SECOND;
        if (f >= 5) acc_ms += ms;
        uint32_t n = rd(MOD_REG_NUM_OBJ);
        if (f % 20 == 0 || f == NFRAMES - 1) {
            printf("frame %3d: %lu objects, %lu fg px, %.3f ms\r\n", f, (unsigned long)n,
                   (unsigned long)rd(MOD_REG_FG_COUNT), ms);
            for (uint32_t i = 0; i < n; i++) {
                uint32_t a = rd(MOD_REG_OBJ(i)), b = rd(MOD_REG_OBJ(i) + 4), c = rd(MOD_REG_OBJ(i) + 8);
                printf("   [%lu] x %lu..%lu  y %lu..%lu  area %lu\r\n", (unsigned long)i,
                       (unsigned long)(a & 0x3FF), (unsigned long)((a >> 16) & 0x3FF),
                       (unsigned long)(b & 0x3FF), (unsigned long)((b >> 16) & 0x3FF), (unsigned long)c);
            }
        }
    }
    double avg = acc_ms / (NFRAMES - 5);
    printf("average PL+DMA time: %.3f ms/frame  (%.1f fps)\r\n", avg, 1000.0 / avg);
    return 0;
}
