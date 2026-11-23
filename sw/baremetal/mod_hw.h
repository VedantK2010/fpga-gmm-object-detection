/*
 * mod_hw.h - register map of the moving-object-detection core
 */
#ifndef MOD_HW_H
#define MOD_HW_H

#include <stdint.h>

#define MOD_REG_CTRL      0x000u  /* [0] soft reset [1] gray input [3:2] morphology mode */
#define MOD_MORPH_NONE 0u
#define MOD_MORPH_OPEN 1u
#define MOD_MORPH_CLOSE 2u
#define MOD_MORPH_CLOSE_OPEN 3u
#define MOD_REG_STATUS    0x004u  /* [0] label overflow  [31:16] frame counter      */
#define MOD_REG_WIDTH     0x008u
#define MOD_REG_HEIGHT    0x00Cu
#define MOD_REG_SHIFTS    0x010u  /* [3:0] a: alpha_w = 2^-a                         */
#define MOD_REG_LAMBDA2   0x014u  /* Q4.4   */
#define MOD_REG_T_BG      0x018u  /* Q0.12  */
#define MOD_REG_VAR_INIT  0x020u  /* integer sigma^2 */
#define MOD_REG_VAR_MIN   0x024u
#define MOD_REG_VAR_MAX   0x028u
#define MOD_REG_MIN_AREA  0x02Cu
#define MOD_REG_NUM_OBJ   0x030u
#define MOD_REG_FG_COUNT  0x034u
#define MOD_REG_ID        0x038u
#define MOD_REG_CAPS      0x03Cu
#define MOD_REG_GMM_FMT   0x040u  /* [3:0] WB [7:4] FM [11:8] VB-8 [15:12] SB */
#define MOD_REG_OBJ(i)    (0x200u + 16u * (i))   /* +0 {maxx,minx} +4 {maxy,miny} +8 area */

#define MOD_CORE_ID       0x4D4F4401u

typedef struct {
    uint16_t x0, x1, y0, y1;
    uint32_t area;
} mod_obj_t;

#endif
