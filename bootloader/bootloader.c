/* USER CODE BEGIN Header */
/**
  * Smart OTA Bootloader for STM32F407
  * - Auto-initializes Metadata to Bank A if invalid.
  * - Red LED: Bootloader running / Error
  * - Green LED: Jumping to Bank A
  * - Blue LED: Jumping to Bank B
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define APP_BANK_A_ADDR     0x08020000
#define APP_BANK_B_ADDR     0x08060000
#define METADATA_ADDR       0x080e0000
#define METADATA_MAGIC      0xDEADBEEF

/* LED Pins */
#define LED_GREEN_Pin       GPIO_PIN_12
#define LED_ORANGE_Pin      GPIO_PIN_13
#define LED_RED_Pin         GPIO_PIN_14
#define LED_BLUE_Pin        GPIO_PIN_15
#define LED_GPIO_Port       GPIOD

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint32_t magic;
    uint8_t  active_bank;      /* 0 = Bank A, 1 = Bank B */
    uint8_t  update_pending;
    uint8_t  boot_attempts;
    uint8_t  reserved;
} BootMetadata_t;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void JumpToApplication(uint32_t appAddr);
static int IsValidApplication(uint32_t appAddr);
static void SetDefaultMetadata(void); // NEW FUNCTION

/* Private user code ---------------------------------------------------------*/
int main(void)
{
    BootMetadata_t *meta = (BootMetadata_t *)METADATA_ADDR;

    /* Initialize */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    /* Red LED ON = Bootloader running */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    HAL_Delay(300); // Visual indication of reset

    /* -----------------------------------------------------------
     * 1. SELF-HEALING: Check for Invalid Metadata (e.g. Fresh Chip)
     * ----------------------------------------------------------- */
    if (meta->magic != METADATA_MAGIC)
    {
        /* Flash is likely erased or corrupted.
         * Force initialize to BANK A (0). */
        SetDefaultMetadata();

        /* Reset to reload the new values cleanly */
        NVIC_SystemReset();
    }

    /* -----------------------------------------------------------
     * 2. Select Bank based on Metadata
     * ----------------------------------------------------------- */

    // Priority: Check Bank B if Metadata says so
    if (meta->active_bank == 1)
    {
        if (IsValidApplication(APP_BANK_B_ADDR))
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
            HAL_Delay(200);
            JumpToApplication(APP_BANK_B_ADDR);
        }
        // If B is invalid, Fallthrough to A
    }

    // Default / Fallback: Check Bank A
    if (IsValidApplication(APP_BANK_A_ADDR))
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        HAL_Delay(200);
        JumpToApplication(APP_BANK_A_ADDR);
    }

    /* -----------------------------------------------------------
     * 3. Error Trap: No valid application found anywhere
     * ----------------------------------------------------------- */
    while (1)
    {
        // Fast Red Blink = ERROR
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_RED_Pin);
        HAL_Delay(100);
    }
}

/**
  * @brief Writes DEADBEEF and Bank 0 to Metadata Sector
  */
static void SetDefaultMetadata(void)
{
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t sectorError;

    // Unlock Flash
    HAL_FLASH_Unlock();

    // Clear Flags
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    // Erase Sector 11
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.Sector = FLASH_SECTOR_11;
    eraseInit.NbSectors = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) == HAL_OK)
    {
        // Write Magic
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, METADATA_ADDR, METADATA_MAGIC);
        // Write Bank 0 (Slot A)
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, METADATA_ADDR + 4, 0);
    }

    HAL_FLASH_Lock();
}

/**
  * @brief Check if application at address is valid
  */
static int IsValidApplication(uint32_t appAddr)
{
    uint32_t stackPtr = *(__IO uint32_t*)appAddr;

    /* Stack pointer must be in RAM (0x20000000 - 0x20020000) */
    if ((stackPtr >= 0x20000000) && (stackPtr <= 0x20020000))
    {
        return 1;
    }
    return 0;
}

/**
  * @brief Jump to application
  */
static void JumpToApplication(uint32_t appAddr)
{
    typedef void (*pFunction)(void);
    pFunction appEntry;

    uint32_t appStack = *(__IO uint32_t*)appAddr;
    uint32_t appEntry_addr = *(__IO uint32_t*)(appAddr + 4);

    /* Disable interrupts */
    __disable_irq();

    /* Set vector table */
    SCB->VTOR = appAddr;

    /* Set stack pointer */
    __set_MSP(appStack);

    /* Jump */
    appEntry = (pFunction)appEntry_addr;
    appEntry();

    /* Should never reach here */
    while(1);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* Configure LED pins */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

