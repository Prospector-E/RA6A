/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : RA6A Robot Arm - AccelStepper Coordinated Motion
  ******************************************************************************
  * RX topic 101: position[6] float32 degrees = 24 bytes (plan-execute)
  * RX topic 107: position[6] + velocity[6] float32 deg(/s) = 48 bytes (teleop)
  * TX topic 102: position[6] float32 degrees = 24 bytes (20 Hz)
  * TX topic 103: 1 byte (homing done)
  * TX topic 104: 1 byte (motion done)
  *
  * Hold CONFIRM (PC10) at boot to skip homing.
  * Blue LED (PB7) ON after homing. Toggles on each command received.
  ******************************************************************************
  */

#include "main.h"
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

/* ---- defines ---- */
#define NUM_JOINTS      6
#define FLASH_ADDR      0x080E0000UL
#define FLASH_MAGIC     0xA6A00001UL
#define DIR_NEGATIVE    GPIO_PIN_RESET
#define DIR_POSITIVE    GPIO_PIN_SET
#define TIMER_CLOCK_HZ  1000000UL
#define MIN_ARR         50UL
#define MAX_ARR         200000UL
#define TOPIC_CMD       101
#define TOPIC_STATE     102
#define TOPIC_HOMED     103
#define TOPIC_DONE      104
#define TOPIC_GRIPPER   105
#define TOPIC_STREAM    106
#define TOPIC_CMD_V     107
#define SYNC1           0xFF
#define SYNC2           0xFE
#define RX_BUF_SZ       256
#define JOG_STEPS       3
#define JOG_FRAC        0.08f
#define PULSE_US        5
#define MIN_SPEED       10.0f
#define LED_PORT        GPIOB
#define LED_PIN         GPIO_PIN_7
#define SERVO_PORT      GPIOB
#define SERVO_PIN       GPIO_PIN_8    /* D5 on Nucleo-144, TIM10_CH1 (AF3) */
#define SERVO_OPEN_DEG  26
#define SERVO_CLOSE_DEG 55

/* ---- types ---- */
typedef struct {
    GPIO_TypeDef* STEP_PORT;  uint16_t STEP_PIN;
    GPIO_TypeDef* DIR_PORT;   uint16_t DIR_PIN;
    GPIO_TypeDef* ENA_PORT;   uint16_t ENA_PIN;
    TIM_TypeDef*  TIM;
    volatile int32_t pos;
    volatile int32_t tgt;
    float max_spd;
    float accel;
    volatile float spd;
    volatile int8_t dir;
    volatile uint8_t run;
    volatile uint8_t vmode;
    uint32_t home_off;
} Motor;

typedef struct {
    uint32_t magic;
    uint8_t  cal;
    uint32_t off[NUM_JOINTS];
} Flash;

typedef enum {
    W_S1, W_S2, W_LL, W_LH, W_C1, W_TL, W_TH, W_PL, W_C2
} Parse;

/* ---- constants ---- */
static const uint32_t SPR[6] = { 5235, 15166, 7333, 4000, 3688, 1000 };
static const int8_t   INV[6] = { 1, 0, 1, 0, 1, 1 };
/*
 * Speed at 0.75 rad/s (50% of max). Accel = 1.5x speed (gentle ramp).
 * Original was 1.5 rad/s with 3x accel — too much inertia.
 */
static const float    MSPD[6] = { 625.0f, 1810.0f, 875.0f, 478.0f, 440.0f, 120.0f };
static const float    MACC[6] = { 940.0f, 2715.0f, 1315.0f, 716.0f, 660.0f, 180.0f };

/*
 * HARD JOINT LIMITS in degrees (from URDF).
 * STM32 will NEVER move past these regardless of commands received.
 */
static const float JMIN[6] = { -177.0f, -75.0f, -25.0f, -177.0f, -90.0f, -177.0f };
static const float JMAX[6] = {  177.0f,  97.0f, 205.0f,  177.0f,  90.0f,  177.0f };

/* ---- motors ---- */
static Motor M[6] = {
    { GPIOB, GPIO_PIN_5,  GPIOB, GPIO_PIN_3,  GPIOA, GPIO_PIN_4,  TIM1, 0,0, 0,0,0, 1,0,0, 0 },
    { GPIOB, GPIO_PIN_4,  GPIOB, GPIO_PIN_6,  GPIOB, GPIO_PIN_2,  TIM2, 0,0, 0,0,0, 1,0,0, 0 },
    { GPIOD, GPIO_PIN_13, GPIOD, GPIO_PIN_12, GPIOD, GPIO_PIN_11, TIM3, 0,0, 0,0,0, 1,0,0, 0 },
    { GPIOE, GPIO_PIN_2,  GPIOA, GPIO_PIN_0,  GPIOB, GPIO_PIN_0,  TIM4, 0,0, 0,0,0, 1,0,0, 0 },
    { GPIOE, GPIO_PIN_0,  GPIOE, GPIO_PIN_14, GPIOE, GPIO_PIN_12, TIM5, 0,0, 0,0,0, 1,0,0, 0 },
    { GPIOE, GPIO_PIN_10, GPIOE, GPIO_PIN_7,  GPIOE, GPIO_PIN_8,  TIM6, 0,0, 0,0,0, 1,0,0, 0 },
};

/* ---- buttons ---- */
#define BL_P GPIOC
#define BL_N GPIO_PIN_8
#define BR_P GPIOC
#define BR_N GPIO_PIN_9
#define BC_P GPIOC
#define BC_N GPIO_PIN_10

/* ---- rx state ---- */
static uint8_t rxbuf[RX_BUF_SZ];
static volatile uint16_t rxhead = 0;
static DMA_HandleTypeDef hdma;
static Parse    pstate = W_S1;
static uint16_t plen = 0, ptopic = 0, pidx = 0;
static uint8_t  ppay[64], pchk1 = 0, pchk2 = 0;

/* ---- command ---- */
static volatile uint8_t got_cmd = 0;
static volatile uint8_t got_grip = 0;
static volatile uint8_t grip_val = 0;
static volatile uint8_t got_stream = 0;
static float cmd_deg[6] = {0};
static float stream_deg[6] = {0};
static volatile uint8_t got_cmd_v = 0;
static float cmdv_deg[6] = {0};
static float cmdv_vel[6] = {0};

/* ---- prototypes ---- */
void SystemClock_Config(void);
static void init_gpio(void);
static void init_dma(void);
static void init_uart(void);
static void init_tim(TIM_TypeDef *t, IRQn_Type irq);
static void delay_us(uint32_t us);
static void dwt_init(void);
static int32_t d2s(float deg, int j);
static float s2d(int32_t s, int j);
static void enable_all(void);
static void init_servo(void);
static void servo_write(uint8_t angle);
static void gripper_open(void);
static void gripper_close(void);
static void tx_bytes(const uint8_t *d, uint16_t n);
static void rs_tx(uint16_t topic, const uint8_t *pay, uint16_t len);
static void tx_state(void);
static void tx_homed(void);
static void tx_done(void);
static void parse_byte(uint8_t b);
static void poll_rx(void);
static float ttm(float a, float v, int32_t d);
static void move_coord(const float deg[]);
static void update_targets(const float deg[]);
static void move_velocity(const float deg[], const float vel[]);
static void do_homing(void);
static void flash_save(void);

/* ============================================================
 *  MAIN
 * ============================================================ */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    init_gpio();
    init_dma();
    init_uart();
    init_tim(TIM1, TIM1_UP_TIM10_IRQn);
    init_tim(TIM2, TIM2_IRQn);
    init_tim(TIM3, TIM3_IRQn);
    init_tim(TIM4, TIM4_IRQn);
    init_tim(TIM5, TIM5_IRQn);
    init_tim(TIM6, TIM6_DAC_IRQn);
    dwt_init();
    init_servo();
    enable_all();

    /* Hold CONFIRM at boot = skip homing */
    HAL_Delay(200);
    if (HAL_GPIO_ReadPin(BC_P, BC_N) != GPIO_PIN_SET) {
        do_homing();
    }

    /* Blue LED ON = ready */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    gripper_open();  /* Start with gripper open */
    tx_homed();
    tx_state();

    uint32_t last_tx = HAL_GetTick();
    uint8_t was_moving = 0;

    while (1)
    {
        poll_rx();

        if (got_cmd) {
            got_cmd = 0;
            float p[6];
            __disable_irq();
            memcpy(p, (void *)cmd_deg, sizeof(p));
            __enable_irq();
            move_coord(p);
        }

        if (got_stream) {
            got_stream = 0;
            float p[6];
            __disable_irq();
            memcpy(p, (void *)stream_deg, sizeof(p));
            __enable_irq();
            update_targets(p);
        }

        if (got_cmd_v) {
            got_cmd_v = 0;
            float p[6], v[6];
            __disable_irq();
            memcpy(p, (void *)cmdv_deg, sizeof(p));
            memcpy(v, (void *)cmdv_vel, sizeof(v));
            __enable_irq();
            move_velocity(p, v);
        }

        if (got_grip) {
            got_grip = 0;
            servo_write(grip_val);  /* angle 0-180 directly */
        }

        /* Detect motion complete */
        uint8_t any = 0;
        for (int j = 0; j < 6; j++) if (M[j].run) { any = 1; break; }
        if (was_moving && !any) tx_done();
        was_moving = any;

        /* 20 Hz state feedback */
        uint32_t now = HAL_GetTick();
        if (now - last_tx >= 50) {
            tx_state();
            last_tx = now;
        }
    }
}

/* ============================================================
 *  CLOCK 180 MHz
 * ============================================================ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef c = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    o.HSEState = RCC_HSE_BYPASS;
    o.PLL.PLLState = RCC_PLL_ON;
    o.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    o.PLL.PLLM = 4; o.PLL.PLLN = 180;
    o.PLL.PLLP = RCC_PLLP_DIV2; o.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();
    c.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV4;
    c.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/* ============================================================
 *  GPIO
 * ============================================================ */

static void init_gpio(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;

    g.Pin = GPIO_PIN_0 | GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &g);

    g.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
    HAL_GPIO_Init(GPIOD, &g);

    g.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14;
    HAL_GPIO_Init(GPIOE, &g);

    g.Pin = BL_N|BR_N|BC_N;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &g);

    /* Servo on PB8 — TIM10 CH1 hardware PWM (AF3) */
    g.Pin = SERVO_PIN;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = 0x03;  /* AF3 = TIM8/9/10/11 family */
    HAL_GPIO_Init(SERVO_PORT, &g);

    for (int i = 0; i < 6; i++) {
        HAL_GPIO_WritePin(M[i].STEP_PORT, M[i].STEP_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M[i].DIR_PORT,  M[i].DIR_PIN,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M[i].ENA_PORT,  M[i].ENA_PIN,  GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

/* ============================================================
 *  DMA + UART
 * ============================================================ */

static void init_dma(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

static void init_uart(void)
{
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_8|GPIO_PIN_9;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &g);

    hdma.Instance = DMA1_Stream1;
    hdma.Init.Channel = DMA_CHANNEL_4;
    hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma.Init.MemInc = DMA_MINC_ENABLE;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma.Init.Mode = DMA_CIRCULAR;
    hdma.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma);

    USART3->BRR = HAL_RCC_GetPCLK1Freq() / 115200;
    USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    USART3->CR3 = USART_CR3_DMAR;
    HAL_DMA_Start(&hdma, (uint32_t)&USART3->DR, (uint32_t)rxbuf, RX_BUF_SZ);
}

/* ============================================================
 *  TIMERS
 * ============================================================ */

static void init_tim(TIM_TypeDef *t, IRQn_Type irq)
{
    if      (t==TIM1) __HAL_RCC_TIM1_CLK_ENABLE();
    else if (t==TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
    else if (t==TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
    else if (t==TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
    else if (t==TIM5) __HAL_RCC_TIM5_CLK_ENABLE();
    else if (t==TIM6) __HAL_RCC_TIM6_CLK_ENABLE();
    t->PSC = (t == TIM1) ? 179 : 89;
    t->ARR = 65535;
    t->DIER = TIM_DIER_UIE;
    t->CR1 = TIM_CR1_ARPE;
    HAL_NVIC_SetPriority(irq, 1, 0);
    HAL_NVIC_EnableIRQ(irq);
}

/* ============================================================
 *  UTILITIES
 * ============================================================ */

static void delay_us(uint32_t us)
{
    uint32_t c = us * (SystemCoreClock / 1000000UL);
    uint32_t s = DWT->CYCCNT;
    while ((DWT->CYCCNT - s) < c);
}

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static int32_t d2s(float deg, int j)
{
    return (int32_t)roundf(deg * (float)SPR[j] / 360.0f);
}

static float s2d(int32_t s, int j)
{
    return (float)s * 360.0f / (float)SPR[j];
}

static void enable_all(void)
{
    for (int j = 0; j < 6; j++)
        HAL_GPIO_WritePin(M[j].ENA_PORT, M[j].ENA_PIN, GPIO_PIN_RESET);
}

/*
 * Clamp joint angles to hard limits. This is the LAST LINE OF DEFENSE.
 * No matter what command is received, joints will never exceed these limits.
 */
static void clamp_joints(float deg[])
{
    for (int j = 0; j < 6; j++) {
        if (deg[j] < JMIN[j]) deg[j] = JMIN[j];
        if (deg[j] > JMAX[j]) deg[j] = JMAX[j];
    }
}

/*
 * Hardware PWM servo on PB8 — TIM10 CH1.
 * 50Hz PWM, continuous. servo_write() just sets duty cycle (instant, non-blocking).
 * No more 300ms blocking loop.
 */
static void init_servo(void)
{
    __HAL_RCC_TIM10_CLK_ENABLE();
    /* TIM10 on APB2: 180MHz timer clock (APB2 div=2 → 90MHz, timers get 2x) */
    TIM10->PSC  = 179;        /* 180MHz / 180 = 1MHz tick */
    TIM10->ARR  = 19999;      /* 1MHz / 20000 = 50Hz period */
    TIM10->CCMR1 = (6 << 4)   /* OC1M = PWM mode 1 */
                 | TIM_CCMR1_OC1PE;  /* preload enable */
    TIM10->CCER = TIM_CCER_CC1E;     /* enable CH1 output */
    TIM10->CCR1 = 1500;       /* 90° default = 1500µs */
    TIM10->EGR  = TIM_EGR_UG; /* load shadow registers */
    /* NO DIER — no interrupts needed. Hardware does PWM autonomously. */
    TIM10->CR1  = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void servo_write(uint8_t angle)
{
    if (angle > 180) angle = 180;
    /* Pulse: 500µs (0°) to 2500µs (180°). CCR in µs at 1MHz tick. */
    TIM10->CCR1 = 500 + ((uint32_t)angle * 2000) / 180;
}

static void gripper_open(void)  { servo_write(SERVO_OPEN_DEG); }
static void gripper_close(void) { servo_write(SERVO_CLOSE_DEG); }

/* ============================================================
 *  UART TX + ROSSERIAL TX
 * ============================================================ */

static void tx_bytes(const uint8_t *d, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        while (!(USART3->SR & USART_SR_TXE));
        USART3->DR = d[i];
    }
    while (!(USART3->SR & USART_SR_TC));
}

static void rs_tx(uint16_t topic, const uint8_t *pay, uint16_t len)
{
    uint8_t h[7];
    uint8_t ll = len & 0xFF, lh = (len >> 8) & 0xFF;
    h[0] = SYNC1; h[1] = SYNC2;
    h[2] = ll; h[3] = lh;
    h[4] = (255 - ((ll + lh) % 256)) & 0xFF;
    h[5] = topic & 0xFF;
    h[6] = (topic >> 8) & 0xFF;

    uint32_t cs = h[5] + h[6];
    for (uint16_t i = 0; i < len; i++) cs += pay[i];
    uint8_t c2 = (255 - (cs % 256)) & 0xFF;

    tx_bytes(h, 7);
    tx_bytes(pay, len);
    tx_bytes(&c2, 1);
}

static void tx_state(void)
{
    uint8_t buf[24];
    for (int j = 0; j < 6; j++) {
        float d = s2d(M[j].pos, j);
        memcpy(&buf[j*4], &d, 4);
    }
    rs_tx(TOPIC_STATE, buf, 24);
}

static void tx_homed(void) { uint8_t b = 1; rs_tx(TOPIC_HOMED, &b, 1); }
static void tx_done(void)  { uint8_t b = 1; rs_tx(TOPIC_DONE, &b, 1); }

/* ============================================================
 *  ROSSERIAL RX PARSER
 * ============================================================ */

static void parse_byte(uint8_t b)
{
    switch (pstate) {
    case W_S1: if (b == SYNC1) pstate = W_S2; break;
    case W_S2: pstate = (b == SYNC2) ? W_LL : W_S1; break;
    case W_LL: plen = b; pchk1 = b; pstate = W_LH; break;
    case W_LH: plen |= ((uint16_t)b << 8); pchk1 += b; pstate = W_C1; break;
    case W_C1:
        if (b == (uint8_t)((255 - (pchk1 % 256)) & 0xFF)) {
            pstate = W_TL; pchk2 = 0;
        } else {
            pstate = W_S1;
        }
        break;
    case W_TL: ptopic = b; pchk2 = b; pstate = W_TH; break;
    case W_TH:
        ptopic |= ((uint16_t)b << 8); pchk2 += b; pidx = 0;
        pstate = (plen > 0) ? W_PL : W_C2;
        break;
    case W_PL:
        if (pidx < sizeof(ppay)) ppay[pidx] = b;
        pchk2 += b; pidx++;
        if (pidx >= plen) pstate = W_C2;
        break;
    case W_C2:
        pstate = W_S1;
        if (b != (uint8_t)((255 - (pchk2 % 256)) & 0xFF)) break;

        if (ptopic == TOPIC_CMD && pidx == 24) {
            for (int j = 0; j < 6; j++)
                memcpy(&cmd_deg[j], &ppay[j * 4], 4);
            got_cmd = 1;
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        }
        else if (ptopic == TOPIC_GRIPPER && pidx == 1) {
            grip_val = ppay[0];
            got_grip = 1;
        }
        else if (ptopic == TOPIC_STREAM && pidx == 24) {
            for (int j = 0; j < 6; j++)
                memcpy(&stream_deg[j], &ppay[j * 4], 4);
            got_stream = 1;
        }
        else if (ptopic == TOPIC_CMD_V && pidx == 48) {
            for (int j = 0; j < 6; j++) {
                memcpy(&cmdv_deg[j], &ppay[j * 4], 4);
                memcpy(&cmdv_vel[j], &ppay[24 + j * 4], 4);
            }
            got_cmd_v = 1;
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        }
        break;
    default:
        pstate = W_S1;
    }
}

static void poll_rx(void)
{
    uint16_t dh = RX_BUF_SZ - DMA1_Stream1->NDTR;
    while (rxhead != dh) {
        parse_byte(rxbuf[rxhead]);
        rxhead = (rxhead + 1) % RX_BUF_SZ;
    }
}

/* ============================================================
 *  COORDINATED MOTION
 * ============================================================ */

static float ttm(float a, float v, int32_t dist)
{
    if (dist == 0) return 0.0f;
    float d = fabsf((float)dist);
    float ad = (v * v) / (2.0f * a);
    if (ad > d / 2.0f) ad = d / 2.0f;
    float vl = sqrtf(2.0f * a * ad);
    float at = vl / a;
    float vt = (d - 2.0f * ad) / vl;
    return 2.0f * at + vt;
}

static void move_coord(const float deg_in[])
{
    /* Clamp to hard limits */
    float deg[6];
    for (int j = 0; j < 6; j++) deg[j] = deg_in[j];
    clamp_joints(deg);

    int32_t delta[6];
    float times[6];

    for (int j = 0; j < 6; j++) {
        int32_t t = d2s(deg[j], j);
        delta[j] = t - M[j].pos;
        times[j] = ttm(MACC[j], MSPD[j], delta[j]);
    }

    float mt = 0.0f;
    for (int j = 0; j < 6; j++)
        if (times[j] > mt) mt = times[j];

    if (mt < 0.001f) return;

    for (int j = 0; j < 6; j++) {
        /* Stop timer before modifying state (prevent race with ISR) */
        M[j].TIM->CR1 &= ~TIM_CR1_CEN;
        M[j].TIM->SR   = ~TIM_SR_UIF;
        M[j].vmode = 0;

        if (delta[j] == 0) {
            M[j].tgt = M[j].pos;
            M[j].run = 0;
            M[j].spd = 0;
            continue;
        }

        float r = times[j] / mt;
        float sv = MSPD[j] * r;
        float sa = MACC[j] * r * r;
        if (sv > MSPD[j]) sv = MSPD[j];
        if (sa > MACC[j]) sa = MACC[j];
        if (sv < MIN_SPEED) sv = MIN_SPEED;
        if (sa < MIN_SPEED) sa = MIN_SPEED;

        M[j].max_spd = sv;
        M[j].accel = sa;
        M[j].tgt = d2s(deg[j], j);
        M[j].spd = MIN_SPEED;

        /* Direction */
        M[j].dir = (delta[j] > 0) ? 1 : -1;
        GPIO_PinState dp = (delta[j] > 0) ? DIR_POSITIVE : DIR_NEGATIVE;
        if (INV[j]) dp = (dp == DIR_POSITIVE) ? DIR_NEGATIVE : DIR_POSITIVE;
        HAL_GPIO_WritePin(M[j].DIR_PORT, M[j].DIR_PIN, dp);

        /* Start timer */
        uint32_t arr = (uint32_t)(TIMER_CLOCK_HZ / MIN_SPEED);
        if (arr > MAX_ARR) arr = MAX_ARR;
        M[j].TIM->ARR = arr;
        M[j].TIM->CNT = 0;
        M[j].TIM->EGR = TIM_EGR_UG;
        M[j].TIM->SR  = ~TIM_SR_UIF;
        M[j].TIM->CR1 |= TIM_CR1_CEN;
        M[j].run = 1;
    }
}

/* ============================================================
 *  STREAMING: update targets without resetting speed
 *  - If motor already running in same direction: just update target
 *  - If direction changes: update direction, keep speed
 *  - If motor stopped: start it from MIN_SPEED
 * ============================================================ */

static void update_targets(const float deg_in[])
{
    /* Clamp to hard limits */
    float deg[6];
    for (int j = 0; j < 6; j++) deg[j] = deg_in[j];
    clamp_joints(deg);

    for (int j = 0; j < 6; j++) {
        int32_t new_tgt = d2s(deg[j], j);

        /*
         * CRITICAL: Stop this joint's timer before modifying state.
         * This prevents the ISR from reading half-updated dir/tgt/pos
         * which causes position drift between pos counter and reality.
         */
        M[j].TIM->CR1 &= ~TIM_CR1_CEN;    /* stop timer */
        M[j].TIM->SR   = ~TIM_SR_UIF;      /* clear pending IRQ */
        M[j].vmode = 0;

        int32_t delta = new_tgt - M[j].pos;
        M[j].tgt = new_tgt;
        M[j].max_spd = MSPD[j];
        M[j].accel = MACC[j];

        if (delta == 0) {
            M[j].run = 0;
            M[j].spd = 0;
            continue;  /* leave timer stopped */
        }

        /* Direction — ALWAYS set GPIO and dir */
        int8_t new_dir = (delta > 0) ? 1 : -1;
        GPIO_PinState dp = (delta > 0) ? DIR_POSITIVE : DIR_NEGATIVE;
        if (INV[j]) dp = (dp == DIR_POSITIVE) ? DIR_NEGATIVE : DIR_POSITIVE;
        HAL_GPIO_WritePin(M[j].DIR_PORT, M[j].DIR_PIN, dp);

        if (new_dir != M[j].dir) {
            M[j].spd = MIN_SPEED;  /* slow down on reversal */
        }
        M[j].dir = new_dir;  /* always update */

        /* Restart timer */
        if (!M[j].run || M[j].spd < MIN_SPEED) {
            M[j].spd = MIN_SPEED;
        }
        uint32_t arr = (uint32_t)(TIMER_CLOCK_HZ / M[j].spd);
        if (arr < MIN_ARR) arr = MIN_ARR;
        if (arr > MAX_ARR) arr = MAX_ARR;
        M[j].TIM->ARR = arr;
        M[j].TIM->CNT = 0;
        M[j].TIM->EGR = TIM_EGR_UG;
        M[j].TIM->SR  = ~TIM_SR_UIF;
        M[j].TIM->CR1 |= TIM_CR1_CEN;
        M[j].run = 1;
    }
}

/* ============================================================
 *  VELOCITY MOVE — AR4 style (topic 107)
 *  Sets timer ARR directly to commanded speed.
 *  ISR just steps at that rate — no trapezoidal math in vmode.
 * ============================================================ */

static void move_velocity(const float deg_in[], const float vel_in[])
{
    float deg[6];
    for (int j = 0; j < 6; j++) deg[j] = deg_in[j];
    clamp_joints(deg);

    for (int j = 0; j < 6; j++) {
        int32_t new_tgt = d2s(deg[j], j);
        int32_t delta = new_tgt - M[j].pos;

        /* Convert commanded velocity: deg/s → steps/s */
        float vel_steps = fabsf(vel_in[j]) * (float)SPR[j] / 360.0f;

        /* Clamp to physical limits — no 40% floor */
        if (vel_steps < MIN_SPEED) vel_steps = MIN_SPEED;
        if (vel_steps > MSPD[j])   vel_steps = MSPD[j];

        /* Direction GPIO */
        GPIO_PinState dp = (delta >= 0) ? DIR_POSITIVE : DIR_NEGATIVE;
        if (INV[j]) dp = (dp == DIR_POSITIVE) ? DIR_NEGATIVE : DIR_POSITIVE;
        HAL_GPIO_WritePin(M[j].DIR_PORT, M[j].DIR_PIN, dp);

        /* Compute ARR for commanded speed */
        uint32_t arr = (uint32_t)(TIMER_CLOCK_HZ / vel_steps);
        if (arr < MIN_ARR) arr = MIN_ARR;
        if (arr > MAX_ARR) arr = MAX_ARR;

        /* Atomic update */
        __disable_irq();
        M[j].tgt     = new_tgt;
        M[j].dir     = (delta >= 0) ? 1 : -1;
        M[j].spd     = vel_steps;
        M[j].max_spd = vel_steps;
        M[j].vmode   = 1;
        M[j].TIM->ARR = arr;
        __enable_irq();

        /* Start timer if not running */
        if (!(M[j].TIM->CR1 & TIM_CR1_CEN)) {
            M[j].TIM->CNT = 0;
            M[j].TIM->EGR = TIM_EGR_UG;
            M[j].TIM->SR  = ~TIM_SR_UIF;
            M[j].TIM->CR1 |= TIM_CR1_CEN;
            M[j].run = 1;
        }
    }
}

/* ============================================================
 *  TIMER ISR — split path
 *  vmode=1: constant speed, skip at target (keep timer alive)
 *  vmode=0: original trapezoidal (unchanged)
 * ============================================================ */

static void step_isr(TIM_TypeDef *t)
{
    if (!(t->SR & TIM_SR_UIF)) return;
    t->SR = ~TIM_SR_UIF;

    for (int j = 0; j < 6; j++) {
        Motor *m = &M[j];
        if (t != m->TIM || !m->run) continue;

        int32_t dist = m->tgt - m->pos;

        /* ---- VELOCITY MODE: constant speed, like AR4 runSpeed ---- */
        if (m->vmode) {
            if (dist == 0) break;
            if ((dist > 0 && m->dir < 0) || (dist < 0 && m->dir > 0)) break;
            HAL_GPIO_WritePin(m->STEP_PORT, m->STEP_PIN, GPIO_PIN_SET);
            delay_us(PULSE_US);
            HAL_GPIO_WritePin(m->STEP_PORT, m->STEP_PIN, GPIO_PIN_RESET);
            m->pos += m->dir;
            break;
        }

        /* ---- NORMAL MODE: original trapezoidal ---- */
        if (dist == 0) {
            m->TIM->CR1 &= ~TIM_CR1_CEN;
            m->run = 0;
            m->spd = 0;
            break;
        }

        /* SAFETY: If moving AWAY from target, stop immediately.
         * This happens when target changes to opposite side mid-motion.
         * Without this check, motor runs forever past physical limits. */
        if ((dist > 0 && m->dir < 0) || (dist < 0 && m->dir > 0)) {
            m->TIM->CR1 &= ~TIM_CR1_CEN;
            m->run = 0;
            m->spd = 0;
            break;
        }

        /* Step pulse */
        HAL_GPIO_WritePin(m->STEP_PORT, m->STEP_PIN, GPIO_PIN_SET);
        delay_us(PULSE_US);
        HAL_GPIO_WritePin(m->STEP_PORT, m->STEP_PIN, GPIO_PIN_RESET);

        m->pos += m->dir;
        dist = m->tgt - m->pos;

        if (dist == 0) {
            m->TIM->CR1 &= ~TIM_CR1_CEN;
            m->run = 0;
            m->spd = 0;
            break;
        }

        /* Trapezoidal speed update */
        float spd = m->spd;
        float a = m->accel;
        float mx = m->max_spd;
        float ad = fabsf((float)dist);
        float dd = (spd * spd) / (2.0f * a);

        if (ad <= dd + 1.0f) {
            /* Decelerate */
            spd -= a / spd;
            if (spd < MIN_SPEED) spd = MIN_SPEED;
        } else if (spd < mx) {
            /* Accelerate */
            spd += a / spd;
            if (spd > mx) spd = mx;
        }

        /* Hard safety clamp */
        if (spd > MSPD[j]) spd = MSPD[j];
        m->spd = spd;

        uint32_t arr = (uint32_t)(TIMER_CLOCK_HZ / spd);
        if (arr < MIN_ARR) arr = MIN_ARR;
        if (arr > MAX_ARR) arr = MAX_ARR;
        m->TIM->ARR = arr;

        break;
    }
}

void TIM1_UP_TIM10_IRQHandler(void) { step_isr(TIM1); }
void TIM2_IRQHandler(void)          { step_isr(TIM2); }
void TIM3_IRQHandler(void)          { step_isr(TIM3); }
void TIM4_IRQHandler(void)          { step_isr(TIM4); }
void TIM5_IRQHandler(void)          { step_isr(TIM5); }
void TIM6_DAC_IRQHandler(void)      { step_isr(TIM6); }

/* ============================================================
 *  MANUAL HOMING
 * ============================================================ */

static void do_homing(void)
{
    for (int j = 0; j < 6; j++) {
        Motor *m = &M[j];
        float jv = 1.5f * JOG_FRAC;
        float js = jv / (2.0f * (float)M_PI) * (float)SPR[j];
        uint32_t dly = (uint32_t)(1000.0f / js);
        if (dly < 5) dly = 5;

        int32_t off = 0;
        while (1) {
            if (HAL_GPIO_ReadPin(BL_P, BL_N) == GPIO_PIN_SET) {
                HAL_GPIO_WritePin(m->DIR_PORT, m->DIR_PIN, INV[j] ? DIR_POSITIVE : DIR_NEGATIVE);
                for (int s = 0; s < JOG_STEPS; s++) {
                    HAL_GPIO_WritePin(m->STEP_PORT, m->STEP_PIN, GPIO_PIN_SET);
                    delay_us(PULSE_US);
                    HAL_GPIO_WritePin(m->STEP_PORT, m->STEP_PIN, GPIO_PIN_RESET);
                    HAL_Delay(dly);
                }
                off -= JOG_STEPS;
            }
            else if (HAL_GPIO_ReadPin(BR_P, BR_N) == GPIO_PIN_SET) {
                HAL_GPIO_WritePin(m->DIR_PORT, m->DIR_PIN, INV[j] ? DIR_NEGATIVE : DIR_POSITIVE);
                for (int s = 0; s < JOG_STEPS; s++) {
                    HAL_GPIO_WritePin(m->STEP_PORT, m->STEP_PIN, GPIO_PIN_SET);
                    delay_us(PULSE_US);
                    HAL_GPIO_WritePin(m->STEP_PORT, m->STEP_PIN, GPIO_PIN_RESET);
                    HAL_Delay(dly);
                }
                off += JOG_STEPS;
            }
            else if (HAL_GPIO_ReadPin(BC_P, BC_N) == GPIO_PIN_SET) {
                m->home_off = (uint32_t)abs(off);
                m->pos = 0;
                m->tgt = 0;
                HAL_Delay(500);
                break;
            }
        }
    }
    flash_save();
}

/* ============================================================
 *  FLASH
 * ============================================================ */

static void flash_save(void)
{
    Flash d;
    d.magic = FLASH_MAGIC;
    d.cal = 1;
    for (int i = 0; i < 6; i++) d.off[i] = M[i].home_off;

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef e = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = FLASH_SECTOR_11,
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    uint32_t se = 0;
    HAL_FLASHEx_Erase(&e, &se);
    uint32_t *p = (uint32_t *)&d;
    for (uint32_t i = 0; i < sizeof(d)/4; i++)
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR + i*4, p[i]);
    HAL_FLASH_Lock();
}

void Error_Handler(void) { __disable_irq(); while(1){} }

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *f, uint32_t l) {}
#endif