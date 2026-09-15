/**
 * PEARL Firmware — STM32 F401RE
 * Physical Environment Aware Reasoning Layer
 *
 * Streams structured JSON telemetry to Raspberry Pi over UART1.
 *
 * Pin Map:
 *   PA0  — ACS712 current sensor (ADC1 CH0, via voltage divider)
 *   PA5  — Onboard LED
 *   PA8  — Motor PWM (TIM1 CH1)
 *   PA9  — UART1 TX → Pi RX
 *   PA10 — UART1 RX ← Pi TX
 *   PB5  — L293D IN1 (motor direction)
 *   PB6  — L293D IN2 (motor direction)
 *   PB8  — I2C1 SCL (ADS1115 + MPU6050)
 *   PB9  — I2C1 SDA (ADS1115 + MPU6050)
 *   PC0  — Stepper IN1 (ULN2003)
 *   PC1  — Stepper IN2
 *   PC2  — Stepper IN3
 *   PC3  — Stepper IN4
 *
 * Clock: HSI 16MHz → PLL → 84MHz SYSCLK
 * UART1: 115200 baud
 * ADC1:  12-bit, single conversion on PA0
 * I2C1:  100kHz standard mode
 * TIM1:  20kHz PWM on CH1
 */

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ── Handles ─────────────────────────────────────────── */
UART_HandleTypeDef huart1;
ADC_HandleTypeDef  hadc1;
I2C_HandleTypeDef  hi2c1;
TIM_HandleTypeDef  htim1;

/* ── I2C Addresses ───────────────────────────────────── */
#define ADS1115_ADDR   (0x48 << 1)
#define MPU6050_ADDR   (0x68 << 1)
#define AK8963_ADDR    (0x0C << 1)   /* magnetometer inside MPU9250 */

/* ── Stepper State ───────────────────────────────────── */
static int32_t stepper_position = 0;   /* cumulative half-steps */
static const uint8_t halfstep_seq[8] = {
    0x01, 0x03, 0x02, 0x06,
    0x04, 0x0C, 0x08, 0x09
};

/* ── Telemetry State ─────────────────────────────────── */
static uint16_t adc_raw   = 0;
static int16_t  ads_ch[4] = {0};
static int16_t  mpu_acc[3] = {0};
static int16_t  mag[3]    = {0};   /* AK8963 magnetometer mx, my, mz */
static uint8_t  imu_ok    = 0;     /* 1 = MPU9250 I2C responding */
static uint8_t  pwm_duty  = 0;

/* ── Forward Declarations ────────────────────────────── */
void SystemClock_Config(void);
static void Init_GPIO(void);
static void Init_UART1(void);
static void Init_ADC1(void);
static void Init_I2C1(void);
static void Init_TIM1_PWM(void);

static uint16_t Read_ADC(void);
static int16_t  ADS1115_ReadChannel(uint8_t ch);
static void     MPU6050_Init(void);
static void     MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az);
static void     AK8963_Init(void);
static uint8_t  AK8963_ReadMag(int16_t *mx, int16_t *my, int16_t *mz);
static uint8_t  IMU_CheckWhoAmI(void);
static void     I2C1_BusRecover(void);

static void Stepper_HalfStep(int8_t dir);
static void Motor_SetPWM(uint8_t duty_pct);
static void Motor_SetDir(uint8_t forward);

static void UART_SendString(const char *s);
static void Send_Telemetry(void);

static void Check_Commands(void) {
    uint8_t rx;
    static char cmd_buf[64];
    static uint8_t cmd_idx = 0;

    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        HAL_UART_Receive(&huart1, &rx, 1, 1);
        if (rx == '\n') {
            cmd_buf[cmd_idx] = '\0';
            /* "L1" = LED on, "L0" = LED off, "P50" = PWM 50% */
            if (cmd_buf[0] == 'L') {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
                    cmd_buf[1] == '1' ? GPIO_PIN_SET : GPIO_PIN_RESET);
            } else if (cmd_buf[0] == 'S') {
                Motor_SetPWM(atoi(&cmd_buf[1]));
            }
            cmd_idx = 0;
        } else if (cmd_idx < sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_idx++] = rx;
        }
    }
}

/* ═══════════════════════════════════════════════════════
 *  MAIN
 * ═══════════════════════════════════════════════════════ */
int main(void) {
    HAL_Init();
    SystemClock_Config();

    Init_GPIO();
    Init_UART1();
    Init_ADC1();
    Init_I2C1();
    Init_TIM1_PWM();

    MPU6050_Init();
    AK8963_Init();

    /* Blink LED twice on boot to signal life */
    for (int i = 0; i < 4; i++) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(150);
    }

    UART_SendString("{\"event\":\"boot\",\"fw\":\"pearl-0.1\"}\n");

    /* Default: motor off, forward direction */
    Motor_SetDir(1);
    Motor_SetPWM(0);

    uint32_t tick_fast   = 0;
    uint32_t tick_medium = 0;
    uint32_t tick_slow   = 0;

    while (1) {
        Check_Commands();
        uint32_t now = HAL_GetTick();

        /* ── Fast: 10ms — ADC current + GPIO/PWM state ── */
        if (now - tick_fast >= 10) {
            tick_fast = now;
            adc_raw = Read_ADC();
        }

        /* ── Medium: 100ms — ADS1115 channels ─────────── */
        if (now - tick_medium >= 100) {
            tick_medium = now;
            for (uint8_t ch = 0; ch < 4; ch++) {
                ads_ch[ch] = ADS1115_ReadChannel(ch);
            }
        }

        /* ── Slow: 200ms — MPU9250 + send frame ───────── */
        if (now - tick_slow >= 200) {
            tick_slow = now;

            if (imu_ok) {
                /* Confirm the IMU is still on the bus each frame. */
                imu_ok = (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 2, 10) == HAL_OK) ? 1 : 0;
            } else {
                /* Bus dropped — attempt a full re-init every ~2s
                   (every 10th slow tick) rather than hammering a
                   disconnected bus every frame and stalling telemetry. */
                static uint8_t reinit_ctr = 0;
                if (++reinit_ctr >= 10) {
                    reinit_ctr = 0;
                    I2C1_BusRecover();  /* free a slave stuck holding SDA low */
                    MPU6050_Init();   /* wake + accel config */
                    AK8963_Init();    /* bypass, power down, continuous mode */
                    if (IMU_CheckWhoAmI()) {
                        imu_ok = 1;
                    }
                }
            }

            MPU6050_ReadAccel(&mpu_acc[0], &mpu_acc[1], &mpu_acc[2]);
            AK8963_ReadMag(&mag[0], &mag[1], &mag[2]);
            Send_Telemetry();
        }
    }
}


/* ═══════════════════════════════════════════════════════
 *  CLOCK — 84 MHz from HSI
 * ═══════════════════════════════════════════════════════ */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = 8;
    osc.PLL.PLLN            = 168;
    osc.PLL.PLLP            = RCC_PLLP_DIV4;
    osc.PLL.PLLQ            = 4;
    HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                          RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}


/* ═══════════════════════════════════════════════════════
 *  GPIO INIT
 * ═══════════════════════════════════════════════════════ */
static void Init_GPIO(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* PA5 — LED */
    gpio.Pin   = GPIO_PIN_5;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PB5, PB6 — L293D IN1, IN2 */
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* PC0..PC3 — Stepper */
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOC, &gpio);
}


/* ═══════════════════════════════════════════════════════
 *  UART1 — 115200, PA9 TX / PA10 RX
 * ═══════════════════════════════════════════════════════ */
static void Init_UART1(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart1.Instance        = USART1;
    huart1.Init.BaudRate   = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits   = UART_STOPBITS_1;
    huart1.Init.Parity     = UART_PARITY_NONE;
    huart1.Init.Mode       = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    HAL_UART_Init(&huart1);
}


/* ═══════════════════════════════════════════════════════
 *  ADC1 — PA0, 12-bit, single conversion
 * ═══════════════════════════════════════════════════════ */
static void Init_ADC1(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    hadc1.Instance                   = ADC1;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = ADC_CHANNEL_0;
    ch.Rank         = 1;
    ch.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &ch);
}

static uint16_t Read_ADC(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}


/* ═══════════════════════════════════════════════════════
 *  I2C1 — PB8 SCL, PB9 SDA, 100kHz
 * ═══════════════════════════════════════════════════════ */
static void Init_I2C1(void) {
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

/*
 * I2C bus recovery: if a slave was interrupted mid-transaction it can
 * hold SDA low, wedging the bus. Manually clocking SCL forces the slave
 * to finish shifting out its byte and release SDA. We tear down the
 * peripheral, bit-bang 16 SCL pulses on PB8, then restore the pins to
 * their I2C alternate function and re-init.
 */
static void I2C1_BusRecover(void) {
    HAL_I2C_DeInit(&hi2c1);

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB8 (SCL) as push-pull output for manual clocking */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_8;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 16 clock pulses, ~5us each phase (~100kHz), to flush a stuck byte */
    for (int i = 0; i < 16; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        for (volatile int d = 0; d < 420; d++) { __NOP(); }   /* ~5us @ 84MHz */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 420; d++) { __NOP(); }
    }

    /* Restore PB8/PB9 to I2C alternate function open-drain */
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_I2C_Init(&hi2c1);
}


/* ═══════════════════════════════════════════════════════
 *  ADS1115 — 16-bit ADC, I2C, 4 channels
 * ═══════════════════════════════════════════════════════ */
static int16_t ADS1115_ReadChannel(uint8_t ch) {
    if (ch > 3) return 0;

    /*
     * Config register:
     *  [15]    OS=1 (start single)
     *  [14:12] MUX: AINx vs GND (0x04+ch)
     *  [11:9]  PGA=001 (±4.096V)
     *  [8]     MODE=1 (single-shot)
     *  [7:5]   DR=100 (128 SPS)
     *  [4:0]   defaults
     */
    uint16_t mux = (0x04 + ch) & 0x07;
    uint16_t config = (1 << 15) | (mux << 12) | (1 << 9) | (1 << 8) | (4 << 5) | 0x03;

    uint8_t cfg_buf[3];
    cfg_buf[0] = 0x01;             /* config register pointer */
    cfg_buf[1] = config >> 8;
    cfg_buf[2] = config & 0xFF;

    if (HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDR, cfg_buf, 3, 50) != HAL_OK)
        return -1;

    HAL_Delay(10);  /* wait for conversion at 128 SPS */

    uint8_t ptr = 0x00;  /* conversion register */
    HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDR, &ptr, 1, 50);

    uint8_t data[2] = {0};
    HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDR, data, 2, 50);

    return (int16_t)((data[0] << 8) | data[1]);
}


/* ═══════════════════════════════════════════════════════
 *  MPU6050 — Accelerometer
 * ═══════════════════════════════════════════════════════ */
static void MPU6050_Init(void) {
    /* Wake up: write 0x00 to PWR_MGMT_1 (0x6B) */
    uint8_t buf[2] = {0x6B, 0x00};
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buf, 2, 50);

    /* Accel range ±2g: write 0x00 to ACCEL_CONFIG (0x1C) */
    buf[0] = 0x1C;
    buf[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buf, 2, 50);
}

static void MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t reg = 0x3B;  /* ACCEL_XOUT_H */
    uint8_t data[6] = {0};

    if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 50) != HAL_OK) {
        *ax = *ay = *az = 0;
        return;
    }
    if (HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, data, 6, 50) != HAL_OK) {
        *ax = *ay = *az = 0;
        return;
    }

    *ax = (int16_t)((data[0] << 8) | data[1]);
    *ay = (int16_t)((data[2] << 8) | data[3]);
    *az = (int16_t)((data[4] << 8) | data[5]);
}


/* Read WHO_AM_I (0x75); MPU9250 reports 0x71. Returns 1 on match. */
static uint8_t IMU_CheckWhoAmI(void) {
    uint8_t reg = 0x75;
    uint8_t id  = 0;
    if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 50) != HAL_OK)
        return 0;
    if (HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, &id, 1, 50) != HAL_OK)
        return 0;
    return (id == 0x71) ? 1 : 0;
}


/* ═══════════════════════════════════════════════════════
 *  AK8963 — Magnetometer (inside MPU9250)
 * ═══════════════════════════════════════════════════════ */
static void AK8963_Init(void) {
    /* Enable I2C bypass so the AK8963 appears on the main bus:
       write 0x02 to MPU INT_PIN_CFG (0x37) */
    uint8_t buf[2] = {0x37, 0x02};
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buf, 2, 50);
    HAL_Delay(10);

    /* Power down: write 0x00 to CNTL1 (0x0A) */
    buf[0] = 0x0A;
    buf[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDR, buf, 2, 50);
    HAL_Delay(10);

    /* 16-bit output, continuous measurement mode 2 (100Hz):
       write 0x16 to CNTL1 (0x0A) */
    buf[0] = 0x0A;
    buf[1] = 0x16;
    HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDR, buf, 2, 50);
    HAL_Delay(10);
}

static uint8_t AK8963_ReadMag(int16_t *mx, int16_t *my, int16_t *mz) {
    /* Check ST1 (0x02) bit 0 for data ready */
    uint8_t reg = 0x02;
    uint8_t st1 = 0;
    if (HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDR, &reg, 1, 50) != HAL_OK)
        return 0;
    if (HAL_I2C_Master_Receive(&hi2c1, AK8963_ADDR, &st1, 1, 50) != HAL_OK)
        return 0;
    if (!(st1 & 0x01))
        return 0;  /* no new data — keep previous values */

    /* Read 7 bytes from HXL (0x03): mx, my, mz (little-endian) + ST2 */
    reg = 0x03;
    uint8_t buf[7] = {0};
    if (HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDR, &reg, 1, 50) != HAL_OK)
        return 0;
    if (HAL_I2C_Master_Receive(&hi2c1, AK8963_ADDR, buf, 7, 50) != HAL_OK)
        return 0;

    /* buf[6] is ST2 — bit 3 (HOFL) flags magnetic overflow.
       Reading ST2 completes the measurement cycle. */
    if (buf[6] & 0x08)
        return 0;  /* overflow — discard this sample */

    *mx = (int16_t)((buf[1] << 8) | buf[0]);
    *my = (int16_t)((buf[3] << 8) | buf[2]);
    *mz = (int16_t)((buf[5] << 8) | buf[4]);
    return 1;
}


/* ═══════════════════════════════════════════════════════
 *  TIM1 PWM — PA8, 20kHz
 * ═══════════════════════════════════════════════════════ */
static void Init_TIM1_PWM(void) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_8;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /*
     * TIM1 is on APB2 = 84 MHz
     * 20 kHz PWM: prescaler=0, period = 84000000/20000 - 1 = 4199
     */
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 83;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 19999;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim1);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;  /* start at 0% duty */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);

    /* TIM1 is an advanced timer — needs MOE bit */
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

static void Motor_SetPWM(uint8_t duty_pct) {
    /* Servo: 1ms (0°) to 2ms (180°) at 50Hz
       1ms = 1000 ticks, 2ms = 2000 ticks */
    if (duty_pct > 180) duty_pct = 180;
    pwm_duty = duty_pct;
    uint32_t pulse = 1000 + ((uint32_t)duty_pct * 1000 / 180);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

static void Motor_SetDir(uint8_t forward) {
    if (forward) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    }
}


/* ═══════════════════════════════════════════════════════
 *  STEPPER — half-step drive via ULN2003
 * ═══════════════════════════════════════════════════════ */
static void Stepper_HalfStep(int8_t dir) {
    stepper_position += dir;
    uint8_t idx = ((stepper_position % 8) + 8) % 8;
    uint8_t bits = halfstep_seq[idx];

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, (bits & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (bits & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, (bits & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, (bits & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


/* ═══════════════════════════════════════════════════════
 *  TELEMETRY — JSON frame over UART
 * ═══════════════════════════════════════════════════════ */
static void UART_SendString(const char *s) {
    HAL_UART_Transmit(&huart1, (uint8_t *)s, strlen(s), 100);
}

static void Send_Telemetry(void) {
    char buf[256];

    /* Read GPIO states for diagnostic context */
    uint8_t pa8  = (GPIOA->ODR & GPIO_PIN_8)  ? 1 : 0;
    uint8_t pb5  = (GPIOB->ODR & GPIO_PIN_5)  ? 1 : 0;
    uint8_t pb6  = (GPIOB->ODR & GPIO_PIN_6)  ? 1 : 0;
    uint8_t led  = (GPIOA->ODR & GPIO_PIN_5)  ? 1 : 0;

    /*
     * Convert ACS712 raw ADC to millivolts:
     *   V = raw * 3300 / 4095
     * The voltage divider and ACS712 offset are handled
     * on the Pi side for flexibility.
     */
    uint16_t acs_mv = (uint32_t)adc_raw * 3300 / 4095;

    int len = snprintf(buf, sizeof(buf),
        "{\"t\":%lu,"
        "\"acs_mv\":%u,"
        "\"ads\":[%d,%d,%d,%d],"
        "\"mpu\":[%d,%d,%d],"
        "\"mag\":[%d,%d,%d],"
        "\"imu\":%u,"
        "\"pwm\":%u,"
        "\"gpio\":{\"PA8\":%u,\"PB5\":%u,\"PB6\":%u,\"LED\":%u},"
        "\"step\":%ld}\n",
        HAL_GetTick(),
        acs_mv,
        ads_ch[0], ads_ch[1], ads_ch[2], ads_ch[3],
        mpu_acc[0], mpu_acc[1], mpu_acc[2],
        mag[0], mag[1], mag[2],
        (unsigned)imu_ok,
        (unsigned)pwm_duty,
        (unsigned)pa8, (unsigned)pb5, (unsigned)pb6, (unsigned)led,
        (long)stepper_position
    );

    if (len > 0 && len < (int)sizeof(buf)) {
        UART_SendString(buf);
    }
}


/* ═══════════════════════════════════════════════════════
 *  ISR — SysTick for HAL timebase
 * ═══════════════════════════════════════════════════════ */
void SysTick_Handler(void) {
    HAL_IncTick();
}
