/* * UNIFIED OTA APPLICATION (Ping-Pong Version)
 * - Supports running in Bank A or Bank B automatically.
 * - Non-blocking: Blinks LED while downloading data.
 */

#include "main.h"
#include <string.h>

/* =================================================================================
 * DEFINITIONS
 * =================================================================================*/

/* Memory Map (STM32F407) */
#define FLASH_BASE_ADDR      0x08000000
#define BANK_A_START_ADDR    0x08020000  // Sector 5
#define BANK_B_START_ADDR    0x08060000  // Sector 7
#define METADATA_ADDR        0x080e0000  // Sector 11
#define METADATA_MAGIC       0xDEADBEEF

/* LED Pins (From your snippet) */
#define LED_GREEN_Pin        GPIO_PIN_12
#define LED_ORANGE_Pin       GPIO_PIN_13
#define LED_RED_Pin          GPIO_PIN_14
#define LED_BLUE_Pin         GPIO_PIN_15
#define LED_GPIO_Port        GPIOD

/* SPI Commands */
#define CMD_PING             0x01
#define CMD_START_OTA        0x10
#define CMD_END_OTA          0x30
#define CMD_GET_VERSION      0x40
#define CMD_DATA_CHUNK       0x20
#define CMD_REBOOT           0x50
#define CMD_GET_BANK_ID      0x60 // New Command

/* SPI Responses */
#define RSP_PONG             0x02
#define RSP_OK               0xAA
#define RSP_ERROR            0xFF

typedef struct {
	uint32_t magic;
	uint8_t active_bank;
	uint8_t update_pending;
	uint8_t boot_attempts;
	uint8_t reserved;
} BootMetadata_t;

/* OTA States for Non-Blocking Logic */
typedef enum {
	OTA_IDLE, OTA_WAIT_LEN, OTA_WAIT_DATA
} OTA_State_t;

typedef enum {
	BANK_A = 0, BANK_B = 1
} Bank_ID_t;

/* =================================================================================
 * GLOBAL VARIABLES
 * =================================================================================*/
SPI_HandleTypeDef hspi1;

/* Application Variables */
uint32_t firmware_version = 1;  // Change this for your next update!

/* OTA State Machine Variables */
OTA_State_t current_state = OTA_IDLE;
uint8_t ota_active = 0;
uint32_t ota_write_addr = 0;
uint32_t ota_bytes_received = 0;
uint8_t chunk_len_expected = 0;
uint8_t chunk_bytes_processed = 0;
uint8_t next_bank_id = 0;
static volatile uint8_t spi_ready = 0;

/* =================================================================================
 * FUNCTION PROTOTYPES
 * =================================================================================*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void OTA_Task(void);
static uint8_t OTA_StateMachine(uint8_t rx_byte);
static void EraseBank(uint32_t Sector);
static void WriteByteToFlash(uint32_t addr, uint8_t data);
static void UpdateMetadata(uint8_t new_active_bank);
static Bank_ID_t GetCurrentBank(void);
static uint8_t ReadActiveBankFromMetadata(void);
void User_Application(void);

/* =================================================================================
 * MAIN
 * =================================================================================*/
int main(void) {
	/* 1. CRITICAL: Fix Interrupt Vector Table */
	/* SystemInit() reset this to 0, so we must set it back to our location */
	if (GetCurrentBank() == BANK_B) {
		SCB->VTOR = BANK_B_START_ADDR;
	} else {
		SCB->VTOR = BANK_A_START_ADDR;
	}

	/* 2. Enable Interrupts (just in case bootloader disabled them) */
	__enable_irq();

	/* 3. Standard Init */
	HAL_Init(); // This initializes SysTick!
	SystemClock_Config();
	MX_GPIO_Init();
	MX_SPI1_Init();

	uint8_t meta_bank = ReadActiveBankFromMetadata();
	Bank_ID_t actual_bank = GetCurrentBank(); // PC-based, internal only

	if (meta_bank != actual_bank) {
		// Metadata mismatch — fix it once
		UpdateMetadata(actual_bank);
	}

	spi_ready = 1;
	/* 4. Debug: Turn on GREEN LED immediately to prove we are alive */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

	/* 5. Infinite Loop */
	while (1) {
		OTA_Task();
		User_Application();
	}
}

/* =================================================================================
 * USER APPLICATION
 * =================================================================================*/
void User_Application(void) {
	static uint32_t last_blink = 0;

	// Blink Green LED every 500ms
	// This will continue running even during OTA download!
	if (HAL_GetTick() - last_blink > 500) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_GREEN_Pin);
		last_blink = HAL_GetTick();
	}
}

/* =================================================================================
 * OTA LOGIC
 * =================================================================================*/

static uint8_t ReadActiveBankFromMetadata(void) {
	BootMetadata_t *meta = (BootMetadata_t*) METADATA_ADDR;

	if (meta->magic != METADATA_MAGIC)
		return 0; // Safe fallback → Bank A

	if (meta->active_bank > 1)
		return 0;

	return meta->active_bank;
}

/* Helper: Detect where we are running */
/* CORRECTED: robust detection using memory address */
/* Helper: Detect where we are running using the ACTUAL Program Counter */
static Bank_ID_t GetCurrentBank(void) {
	uint32_t pc;

	/* READ THE REAL CPU REGISTER (PC) */
	/* This gets the address of the instruction currently executing */
	__asm volatile ("mov %0, pc" : "=r" (pc));

	/* Strict check against the Memory Map */
	if (pc >= BANK_B_START_ADDR && pc < (BANK_B_START_ADDR + 0x20000)) {
		return BANK_B; // We are executing inside Bank B
	} else if (pc >= BANK_A_START_ADDR && pc < BANK_B_START_ADDR) {
		return BANK_A; // We are executing inside Bank A
	}

	// Fallback: If we are in RAM or unknown, default to A to allow recovery
	return BANK_A;
}

/* Task: Checks SPI and feeds the State Machine */
static void OTA_Task(void) {
	if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE)) {
		uint8_t rx_byte = *(__IO uint8_t*) &hspi1.Instance->DR;
		uint8_t tx_resp = OTA_StateMachine(rx_byte);

		// If the state machine generated a response, send it back
		if (tx_resp != 0) {
			while (!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE))
				;
			*(__IO uint8_t*) &hspi1.Instance->DR = tx_resp;
		}
	}
}

/* The Brain: Non-Blocking State Machine */
static uint8_t OTA_StateMachine(uint8_t cmd) {

	if (!spi_ready) {
		return 0;   // Ignore everything until ready
	}

	switch (current_state) {
	/* ---------------- STATE: IDLE ---------------- */
	case OTA_IDLE:
		switch (cmd) {
		case CMD_PING:
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_BLUE_Pin);
			return RSP_PONG;

		case CMD_GET_VERSION:
			return (uint8_t) firmware_version;

		case CMD_START_OTA:
			//HAL_GPIO_WritePin(LED_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);

			/* DYNAMIC TARGET SELECTION */
			uint32_t target_sector;

			if (GetCurrentBank() == BANK_A) {
				// In A, write to B
				ota_write_addr = BANK_B_START_ADDR;
				target_sector = FLASH_SECTOR_7;
				next_bank_id = 1; // 1 = Bank B
			} else {
				// In B, write to A
				ota_write_addr = BANK_A_START_ADDR;
				target_sector = FLASH_SECTOR_5;
				next_bank_id = 0; // 0 = Bank A
			}

			/* ERASE (Blocking! LED will freeze for ~1s here) */
			EraseBank(target_sector);

			ota_active = 1;
			ota_bytes_received = 0;
			return RSP_OK;

		case CMD_DATA_CHUNK:
			if (ota_active) {
				current_state = OTA_WAIT_LEN; // Switch state
				return 0; // Wait for length byte
			}
			return RSP_ERROR;

		case CMD_END_OTA:
			if (ota_active) {
				ota_active = 0;
				UpdateMetadata(next_bank_id);
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
				return RSP_OK;
			}
			return RSP_ERROR;

		case CMD_REBOOT:
			NVIC_SystemReset();
			return RSP_OK;

		case CMD_GET_BANK_ID:
			return ReadActiveBankFromMetadata();

		default:
			return 0; // Ignore noise
		}
		break;

		/* ---------------- STATE: WAIT LEN ---------------- */
	case OTA_WAIT_LEN:
		chunk_len_expected = cmd;
		chunk_bytes_processed = 0;
		current_state = OTA_WAIT_DATA;
		return 0; // Ready for data

		/* ---------------- STATE: WAIT DATA ---------------- */
	case OTA_WAIT_DATA:
		/* Write to Flash */
		WriteByteToFlash(ota_write_addr, cmd);
		ota_write_addr++;
		ota_bytes_received++;
		chunk_bytes_processed++;

		/* Visual Feedback (Toggle Orange LED every 1KB) */
		if ((ota_bytes_received % 1024) == 0) {
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_BLUE_Pin);
		}

		/* Chunk Complete? */
		if (chunk_bytes_processed >= chunk_len_expected) {
			current_state = OTA_IDLE; // Go back to listening for commands
			return RSP_OK; // Ack the chunk
		}
		return 0; // Stay silent, waiting for more bytes
	}
	return 0;
}

/* =================================================================================
 * FLASH HELPERS
 * =================================================================================*/
static void EraseBank(uint32_t Sector) {
	FLASH_EraseInitTypeDef eraseInit;
	uint32_t sectorError;

	HAL_FLASH_Unlock();
	eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	eraseInit.Sector = Sector;
	eraseInit.NbSectors = 1;
	eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	HAL_FLASHEx_Erase(&eraseInit, &sectorError);
	HAL_FLASH_Lock();
}

static void WriteByteToFlash(uint32_t addr, uint8_t data) {
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr, data);
	HAL_FLASH_Lock();
}

static void UpdateMetadata(uint8_t new_active_bank) {
	FLASH_EraseInitTypeDef eraseInit;
	uint32_t sectorError;

	// 1. Unlock Flash
	HAL_FLASH_Unlock();

	// 2. Clear ALL Error Flags (Critical for F407)
	__HAL_FLASH_CLEAR_FLAG(
			FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	// 3. ALWAYS Erase Sector 11 (Don't check, just do it)
	eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	eraseInit.Sector = FLASH_SECTOR_11;
	eraseInit.NbSectors = 1;
	eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) == HAL_OK) {
		// 4. Program the Magic Word
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, METADATA_ADDR,
		METADATA_MAGIC);

		// 5. Program the Bank ID (0 or 1)
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, METADATA_ADDR + 4,
				(uint32_t) new_active_bank);
	}

	// 6. Lock Flash
	HAL_FLASH_Lock();
}

/* =================================================================================
 * HARDWARE INIT
 * =================================================================================*/
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	HAL_GPIO_WritePin(LED_GPIO_Port,
	LED_GREEN_Pin | LED_ORANGE_Pin | LED_RED_Pin | LED_BLUE_Pin,
			GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = LED_GREEN_Pin | LED_ORANGE_Pin | LED_RED_Pin
			| LED_BLUE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}

static void MX_SPI1_Init(void) {
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* SPI1: PA5=SCK, PA6=MISO, PA7=MOSI */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* PA4 = CS input */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_SLAVE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	HAL_SPI_Init(&hspi1);

	__HAL_SPI_ENABLE(&hspi1);
}

