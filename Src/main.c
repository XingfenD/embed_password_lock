/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "gpio.h"
#include "zlg7290.h"
#include "usart.h"

/* Private variables ---------------------------------------------------------*/
#define ZLG_READ_ADDRESS1 0x01  /* key value register */
#define ZLG_READ_ADDRESS2 0x10
#define ZLG_WRITE_ADDRESS1 0x10
#define ZLG_WRITE_ADDRESS2 0x11
#define I2C_WRITE_ADDR 0x70
#define I2C_READ_ADDR 0x71
#define BUFFER_SIZE2 sizeof(Rx2_Buffer)

uint8_t flag;      // Key-specific flag value
uint8_t flag1 = 0; // Interrupt flag
uint8_t Rx2_Buffer[8] = {0};
uint8_t Tx1_Buffer[8] = {0};
uint8_t Rx1_Buffer[1] = {0};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void swtich_key(void);
void switch_flag(void);

/* Main function -------------------------------------------------------------*/
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    printf("\n\r");
    printf("\n\r Embeded Password Lock\n\r");

    while (1) {
        if (flag1 == 1){
            flag1 = 0;
            /* read key value from key value register into R1_Buffer through i2c */
            I2C_ZLG7290_Read(&hi2c1, I2C_READ_ADDR, ZLG_READ_ADDRESS1, Rx1_Buffer, 1);
            printf("\n\r key value = %x\n\r", Rx1_Buffer[0]);
            /* process key value and set flag */
            swtich_key();
            /* read 8-digit display into Rx2_Buffer through i2c */
            I2C_ZLG7290_Read(&hi2c1, I2C_READ_ADDR, ZLG_READ_ADDRESS2, Rx2_Buffer, 8);
            /* update display based on flag */
            switch_flag();
        }
    }
}

/* System Clock Configuration ------------------------------------------------*/
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    __PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* User functions ------------------------------------------------------------*/
void swtich_key(void) {
    switch (Rx1_Buffer[0]) {
        case 0x03: flag = 0; break;    /* 0 */
        case 0x1C: flag = 1; break;     /* 1 */
        case 0x1B: flag = 2; break;     /* 2 */
        case 0x1A: flag = 3; break;     /* 3 */
        case 0x14: flag = 4; break;     /* 4 */
        case 0x13: flag = 5; break;     /* 5 */
        case 0x12: flag = 6; break;     /* 6 */
        case 0x0C: flag = 7; break;     /* 7 */
        case 0x0B: flag = 8; break;     /* 8 */
        case 0x0A: flag = 9; break;     /* 9 */
        case 0x19: flag = 10; break;    /* A */
        case 0x11: flag = 11; break;    /* B */
        case 0x09: flag = 12; break;    /* C */
        case 0x01: flag = 13; break;    /* D */
        case 0x02: flag = 14; break;    /* # */
        case 0x04:                      /* * */
        default:
            break;
    }
}

void switch_flag(void) {
    uint8_t value;
    switch (flag) {
        case 0: value = 0xFC; break;
        case 1: value = 0x0C; break;
        case 2: value = 0xDA; break;
        case 3: value = 0xF2; break;
        case 4: value = 0x66; break;
        case 5: value = 0xB6; break;
        case 6: value = 0xBE; break;
        case 7: value = 0xE0; break;
        case 8: value = 0xFE; break;
        case 9: value = 0xE6; break;
        case 10: value = 0xEE; break;   /* A */
        case 11: value = 0x3E; break;   /* B */
        case 12: value = 0x9C; break;   /* C */
        case 13: value = 0x7A; break;   /* D */
        case 14:                        /* # */
            Tx1_Buffer[0] = 0x00;
            I2C_ZLG7290_Write(&hi2c1, I2C_WRITE_ADDR, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8);
            return;
        case 15:
        default: return;
    }

    Tx1_Buffer[0] = value;

    if (Rx2_Buffer[0] == 0) {
        I2C_ZLG7290_Write(&hi2c1, I2C_WRITE_ADDR, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 1);
    } else {
        I2C_ZLG7290_Write(&hi2c1, I2C_WRITE_ADDR, ZLG_WRITE_ADDRESS2, Rx2_Buffer, BUFFER_SIZE2);
        I2C_ZLG7290_Write(&hi2c1, I2C_WRITE_ADDR, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    flag1 = 1;
}

int fputc(int ch, FILE *f) {
    uint8_t tmp[1]={0};
    tmp[0] = (uint8_t)ch;
    HAL_UART_Transmit(&huart1,tmp,1,10);
    return ch;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
