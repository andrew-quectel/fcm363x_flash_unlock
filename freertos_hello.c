/**
 * @file freertos_hello.c
 * @brief XMC Flash unlock project for FCM363X module based on freertos_hello example
 * @version 0.1
 * @date 2025-10-30
 * 
 * Copyright (c) 2025, Quectel Wireless Solutions Co., Ltd. All rights reserved.
 * Quectel Wireless Solutions Proprietary and Confidential.
 * 
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_flexspi.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "board.h"
#include "app.h"

#include "mflash_drv.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)

/* Flash command definitions */
#define FLASH_PORT kFLEXSPI_PortA1
#define NOR_CMD_LUT_SEQ_IDX_READID  8  /* JEDEC ID read */
#define NOR_CMD_LUT_SEQ_IDX_RDSR1   9  /* Read Status Register 1 */
#define NOR_CMD_LUT_SEQ_IDX_RDSR2   10 /* Read Status Register 2 */
#define NOR_CMD_LUT_SEQ_IDX_RDSR3   11 /* Read Status Register 3 */
#define NOR_CMD_LUT_SEQ_IDX_WREN    12 /* Write Enable */
#define NOR_CMD_LUT_SEQ_IDX_WRENV   13 /* Write Enable for Volatile */
#define NOR_CMD_LUT_SEQ_IDX_WRSR1   14 /* Write Status Register 1 */
#define NOR_CMD_LUT_SEQ_IDX_WRSR2   15 /* Write Status Register 2 */
#define NOR_CMD_LUT_SEQ_IDX_WRSR3   16 /* Write Status Register 3 */

/* Flash unlock target values */
#define UNLOCK_SR1_VALUE    0x00  /* Clear all protection bits */
#define UNLOCK_SR2_VALUE    0x02  /* Keep QE=1, clear CMP and other protection bits */
#define UNLOCK_SR3_VALUE    0x20  /* Keep current driver settings */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void hello_task(void *pvParameters);
static status_t read_flash_jedec_id(uint32_t *jedec_id);
static status_t read_flash_status_register(uint8_t reg_num, uint8_t *status);
static void print_flash_info(uint32_t jedec_id);
static void print_status_registers(void);
static status_t flash_write_enable(bool volatile_mode);
static status_t flash_write_status_register(uint8_t reg_num, uint8_t value, bool volatile_mode);
static status_t flash_unlock_all_protection(void);
static bool is_flash_locked(void);
static status_t try_hardware_unlock_sequence(void);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitHardware();
    if (xTaskCreate(hello_task, "Hello_task", configMINIMAL_STACK_SIZE + 100, NULL, hello_task_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for Flash ID and status reading.
 */
static void hello_task(void *pvParameters)
{
    uint8_t vendor_id = 0;
    uint32_t jedec_id = 0;
    status_t status;

    PRINTF("Starting Flash JEDEC ID and Status Register reading...\r\n");
    
    /* Initialize mflash to ensure proper Flash configuration */
    int32_t mflash_ret = mflash_drv_init();
    if (mflash_ret != kStatus_Success)
    {
        PRINTF("mflash initialization failed: %d\r\n", mflash_ret);
        vTaskDelay(pdMS_TO_TICKS(1000));
        /* Continue anyway - might still work */
    }
    
    /* mflash_drv_init() has already configured FlexSPI properly */
    /* Add a small delay to ensure Flash is stable */
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Read Flash JEDEC ID */
    status = read_flash_jedec_id(&jedec_id);
    if (status == kStatus_Success)
    {
        PRINTF("Flash JEDEC ID: 0x%06X\r\n", jedec_id & 0xFFFFFF);
        print_flash_info(jedec_id);
    }
    else
    {
        PRINTF("Failed to read Flash JEDEC ID: 0x%x\r\n", status);
    }

    /* We only support XMC for now */
    vendor_id = (jedec_id >> 16) & 0xFF;
    if (vendor_id != 0x20)
    {
        PRINTF("Unsupported Flash vendor: 0x%02X\r\n", vendor_id);
        vTaskSuspend(NULL);
    }
    else
    {
        PRINTF("XMC Flash detected - proceeding with status register check.\r\n");
    }

    /* Read Status Registers */
    print_status_registers();
    
    /* Check if Flash is locked and unlock if necessary */
    if (is_flash_locked())
    {
        PRINTF("Flash protection detected. Attempting to unlock...\r\n");
        status_t unlock_status = flash_unlock_all_protection();
        if (unlock_status == kStatus_Success)
        {
            PRINTF("Flash unlock successful!\r\n");
            /* Read status registers again to verify */
            PRINTF("\r\n--- Status After Unlock ---\r\n");
            print_status_registers();
        }
        else
        {
            PRINTF("Flash unlock failed: 0x%x\r\n", unlock_status);
        }
    }
    else
    {
        PRINTF("Flash is not locked - no unlock needed.\r\n");
    }
    
    vTaskSuspend(NULL);
}

/*!
 * @brief Read Flash JEDEC ID using FlexSPI
 */
static status_t read_flash_jedec_id(uint32_t *jedec_id)
{
    flexspi_transfer_t flashXfer;
    status_t status;
    uint8_t jedec_data[3];
    
    if (jedec_id == NULL)
    {
        return kStatus_InvalidArgument;
    }
    
    /* Setup LUT for JEDEC ID read command (0x9F) */
    uint32_t readIdLUT[4] = {
        /* JEDEC ID read: CMD + READ 3 bytes */
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x03),
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
        0, 0
    };
    
    /* Update LUT table */
    FLEXSPI_UpdateLUT(FLEXSPI, NOR_CMD_LUT_SEQ_IDX_READID * 4, readIdLUT, 4);
    
    /* Setup FlexSPI transfer */
    flashXfer.deviceAddress = 0;
    flashXfer.port          = FLASH_PORT;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READID;
    flashXfer.data          = (uint32_t *)jedec_data;
    flashXfer.dataSize      = 3;
    
    status = FLEXSPI_TransferBlocking(FLEXSPI, &flashXfer);
    
    if (status == kStatus_Success)
    {
        /* Correct byte order: vendor_id, memory_type, capacity_id */
        *jedec_id = (jedec_data[0] << 16) | (jedec_data[1] << 8) | jedec_data[2];
    }
    
    return status;
}

/*!
 * @brief Read Flash Status Register
 */
static status_t read_flash_status_register(uint8_t reg_num, uint8_t *status)
{
    flexspi_transfer_t flashXfer;
    status_t ret;
    uint8_t cmd = 0x05; /* Default to Status Register 1 */
    uint8_t lut_idx = NOR_CMD_LUT_SEQ_IDX_RDSR1;
    
    if (status == NULL || reg_num < 1 || reg_num > 3)
    {
        return kStatus_InvalidArgument;
    }
    
    /* Select command and LUT index based on register number */
    switch (reg_num)
    {
        case 1: cmd = 0x05; lut_idx = NOR_CMD_LUT_SEQ_IDX_RDSR1; break;
        case 2: cmd = 0x35; lut_idx = NOR_CMD_LUT_SEQ_IDX_RDSR2; break;
        case 3: cmd = 0x15; lut_idx = NOR_CMD_LUT_SEQ_IDX_RDSR3; break;
    }
    
    /* Setup LUT for Status Register read */
    uint32_t readSrLUT[4] = {
        /* Status Register read: CMD + READ 1 byte */
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, cmd, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x01),
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
        0, 0
    };
    
    /* Update LUT table */
    FLEXSPI_UpdateLUT(FLEXSPI, lut_idx * 4, readSrLUT, 4);
    
    /* Setup FlexSPI transfer */
    flashXfer.deviceAddress = 0;
    flashXfer.port          = FLASH_PORT;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = lut_idx;
    flashXfer.data          = (uint32_t *)status;
    flashXfer.dataSize      = 1;
    
    ret = FLEXSPI_TransferBlocking(FLEXSPI, &flashXfer);
    
    return ret;
}

/*!
 * @brief Print Flash information based on JEDEC ID
 */
static void print_flash_info(uint32_t jedec_id)
{
    uint8_t vendor_id = (jedec_id >> 16) & 0xFF;
    uint8_t memory_type = (jedec_id >> 8) & 0xFF;
    uint8_t capacity_id = jedec_id & 0xFF;
    uint32_t flash_size = 0;
    
    PRINTF("\r\n--- Flash Information ---\r\n");
    PRINTF("Vendor ID: 0x%02X ", vendor_id);
    
    /* Decode vendor ID */
    switch (vendor_id)
    {
        case 0x20:
            PRINTF("(XMC, Supported)\r\n");
            break;
        case 0xEF:
            PRINTF("(Winbond, Supported)\r\n");
            break;
        case 0xC2:
            PRINTF("(Macronix, Not Supported)\r\n");
            break;
        default:
            PRINTF("(Unknown)\r\n");
            break;
    }
    
    PRINTF("Memory Type: 0x%02X\r\n", memory_type);
    PRINTF("Capacity ID: 0x%02X ", capacity_id);
    
    /* Decode capacity */
    if (capacity_id >= 0x10 && capacity_id <= 0x20)
    {
        flash_size = 1UL << capacity_id;
        if (flash_size >= 1024 * 1024)
        {
            PRINTF("(%u MB)\r\n", (unsigned int)(flash_size / (1024 * 1024)));
        }
        else if (flash_size >= 1024)
        {
            PRINTF("(%u KB)\r\n", (unsigned int)(flash_size / 1024));
        }
        else
        {
            PRINTF("(%u Bytes)\r\n", (unsigned int)flash_size);
        }
    }
    else
    {
        PRINTF("(Unknown capacity)\r\n");
    }
    
    PRINTF("-------------------------\r\n");
}

/*!
 * @brief Print Status Registers information
 */
static void print_status_registers(void)
{
    uint8_t sr1, sr2, sr3;
    status_t status;
    
    PRINTF("\r\n--- Status Registers ---\r\n");
    
    /* Read Status Register 1 */
    status = read_flash_status_register(1, &sr1);
    if (status == kStatus_Success)
    {
        PRINTF("Status Register 1 (0x05): 0x%02X\r\n", sr1);
        PRINTF("  BUSY: %d (Erase/Write in Progress)\r\n", (sr1 >> 0) & 1);
        PRINTF("  WEL:  %d (Write Enable Latch)\r\n", (sr1 >> 1) & 1);
        PRINTF("  BP0:  %d (Block Protect Bit 0)\r\n", (sr1 >> 2) & 1);
        PRINTF("  BP1:  %d (Block Protect Bit 1)\r\n", (sr1 >> 3) & 1);
        PRINTF("  BP2:  %d (Block Protect Bit 2)\r\n", (sr1 >> 4) & 1);
        PRINTF("  TB:   %d (Top/Bottom Protect)\r\n", (sr1 >> 5) & 1);
        PRINTF("  SEC:  %d (Sector Protect)\r\n", (sr1 >> 6) & 1);
        PRINTF("  SRP0: %d (Status Register Protect 0)\r\n", (sr1 >> 7) & 1);
    }
    else
    {
        PRINTF("Failed to read Status Register 1: 0x%x\r\n", status);
    }
    
    /* Read Status Register 2 */
    status = read_flash_status_register(2, &sr2);
    if (status == kStatus_Success)
    {
        PRINTF("Status Register 2 (0x35): 0x%02X\r\n", sr2);
        PRINTF("  SRP1: %d (Status Register Protect 1)\r\n", (sr2 >> 0) & 1);
        PRINTF("  QE:   %d (Quad Enable)\r\n", (sr2 >> 1) & 1);
        PRINTF("  LB1:  %d (Security Register Lock Bit 1)\r\n", (sr2 >> 3) & 1);
        PRINTF("  LB2:  %d (Security Register Lock Bit 2)\r\n", (sr2 >> 4) & 1);
        PRINTF("  LB3:  %d (Security Register Lock Bit 3)\r\n", (sr2 >> 5) & 1);
        PRINTF("  CMP:  %d (Complement Protect)\r\n", (sr2 >> 6) & 1);
        PRINTF("  SUS:  %d (Suspend Status)\r\n", (sr2 >> 7) & 1);
    }
    else
    {
        PRINTF("Failed to read Status Register 2: 0x%x\r\n", status);
    }
    
    /* Read Status Register 3 */
    status = read_flash_status_register(3, &sr3);
    if (status == kStatus_Success)
    {
        PRINTF("Status Register 3 (0x15): 0x%02X\r\n", sr3);
        PRINTF("  DC0:  %d (Dummy Control Bit 0)\r\n", (sr3 >> 0) & 1);
        PRINTF("  DC1:  %d (Dummy Control Bit 1)\r\n", (sr3 >> 1) & 1);
        PRINTF("  DRV0: %d (Output Driver Strength 0)\r\n", (sr3 >> 5) & 1);
        PRINTF("  DRV1: %d (Output Driver Strength 1)\r\n", (sr3 >> 6) & 1);
        PRINTF("  HOLD: %d (/HOLD or /RESET Function)\r\n", (sr3 >> 7) & 1);
    }
    else
    {
        PRINTF("Failed to read Status Register 3: 0x%x\r\n", status);
    }
    
    PRINTF("------------------------\r\n\r\n");
}

/*!
 * @brief Send Write Enable command to Flash
 */
static status_t flash_write_enable(bool volatile_mode)
{
    flexspi_transfer_t flashXfer;
    status_t status;
    uint8_t cmd = volatile_mode ? 0x50 : 0x06; /* 50h for volatile, 06h for non-volatile */
    uint8_t lut_idx = volatile_mode ? NOR_CMD_LUT_SEQ_IDX_WRENV : NOR_CMD_LUT_SEQ_IDX_WREN;
    
    /* Setup LUT for Write Enable command */
    uint32_t writeEnableLUT[4] = {
        /* Write Enable: CMD only */
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, cmd, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
        0, 0
    };
    
    /* Update LUT table */
    FLEXSPI_UpdateLUT(FLEXSPI, lut_idx * 4, writeEnableLUT, 4);
    
    /* Setup FlexSPI transfer */
    flashXfer.deviceAddress = 0;
    flashXfer.port          = FLASH_PORT;
    flashXfer.cmdType       = kFLEXSPI_Command;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = lut_idx;
    flashXfer.data          = NULL;
    flashXfer.dataSize      = 0;
    
    status = FLEXSPI_TransferBlocking(FLEXSPI, &flashXfer);
    
    if (status == kStatus_Success)
    {
        PRINTF("Write Enable (%s) command sent successfully\r\n", volatile_mode ? "volatile" : "non-volatile");
    }
    else
    {
        PRINTF("Write Enable command failed: 0x%x\r\n", status);
    }
    
    return status;
}

/*!
 * @brief Write Flash Status Register
 */
static status_t flash_write_status_register(uint8_t reg_num, uint8_t value, bool volatile_mode)
{
    flexspi_transfer_t flashXfer;
    status_t status;
    uint8_t cmd = 0x01; /* Default to Status Register 1 */
    uint8_t lut_idx = NOR_CMD_LUT_SEQ_IDX_WRSR1;
    
    if (reg_num < 1 || reg_num > 3)
    {
        return kStatus_InvalidArgument;
    }
    
    /* Select command and LUT index based on register number */
    switch (reg_num)
    {
        case 1: cmd = 0x01; lut_idx = NOR_CMD_LUT_SEQ_IDX_WRSR1; break;
        case 2: cmd = 0x31; lut_idx = NOR_CMD_LUT_SEQ_IDX_WRSR2; break;
        case 3: cmd = 0x11; lut_idx = NOR_CMD_LUT_SEQ_IDX_WRSR3; break;
    }
    
    /* First send Write Enable command */
    status = flash_write_enable(volatile_mode);
    if (status != kStatus_Success)
    {
        return status;
    }
    
    /* Small delay to ensure Write Enable is processed */
    vTaskDelay(pdMS_TO_TICKS(1));
    
    /* Setup LUT for Write Status Register */
    uint32_t writeSrLUT[4] = {
        /* Write Status Register: CMD + WRITE 1 byte */
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, cmd, kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x01),
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
        0, 0
    };
    
    /* Update LUT table */
    FLEXSPI_UpdateLUT(FLEXSPI, lut_idx * 4, writeSrLUT, 4);
    
    /* Setup FlexSPI transfer */
    flashXfer.deviceAddress = 0;
    flashXfer.port          = FLASH_PORT;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = lut_idx;
    flashXfer.data          = (uint32_t *)&value;
    flashXfer.dataSize      = 1;
    
    status = FLEXSPI_TransferBlocking(FLEXSPI, &flashXfer);
    
    if (status == kStatus_Success)
    {
        PRINTF("Status Register %d written with value 0x%02X (%s)\r\n", 
               reg_num, value, volatile_mode ? "volatile" : "non-volatile");
        
        /* For non-volatile writes, wait for completion */
        if (!volatile_mode)
        {
            /* Wait for write completion by checking BUSY bit */
            uint8_t sr1;
            int timeout = 1000; /* 1 second timeout */
            do {
                vTaskDelay(pdMS_TO_TICKS(1));
                read_flash_status_register(1, &sr1);
                timeout--;
            } while ((sr1 & 0x01) && timeout > 0); /* Wait while BUSY bit is set */
            
            if (timeout <= 0)
            {
                PRINTF("Warning: Write Status Register timeout\r\n");
                return kStatus_Timeout;
            }
        }
        else
        {
            /* For volatile writes, small delay for refresh */
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    else
    {
        PRINTF("Write Status Register %d failed: 0x%x\r\n", reg_num, status);
    }
    
    return status;
}

/*!
 * @brief Check if Flash has any protection enabled
 */
static bool is_flash_locked(void)
{
    uint8_t sr1, sr2, sr3;
    status_t status;
    
    /* Read all status registers */
    status = read_flash_status_register(1, &sr1);
    if (status != kStatus_Success) return false;
    
    status = read_flash_status_register(2, &sr2);
    if (status != kStatus_Success) return false;
    
    status = read_flash_status_register(3, &sr3);
    if (status != kStatus_Success) return false;
    
    /* Check for any protection bits set */
    /* SR1: BP[2:0], TB, SEC, SRP0 */
    bool sr1_locked = (sr1 & 0xFC) != 0; /* Bits 7,6,5,4,3,2 */
    
    /* SR2: CMP, LB[3:1], SRP1 */
    bool sr2_locked = (sr2 & 0x79) != 0; /* Bits 6,5,4,3,0 (exclude QE bit 1) */
    
    /* SR3: Generally doesn't affect locking, but check HOLD bit */
    bool sr3_locked = false; /* Usually not locking-related */
    
    if (sr1_locked)
    {
        PRINTF("SR1 protection detected: 0x%02X", sr1);
        if (sr1 & 0x40) PRINTF(" [SEC=1]");
        if (sr1 & 0x1C) PRINTF(" [BP=%d]", (sr1 >> 2) & 0x07);
        if (sr1 & 0x20) PRINTF(" [TB=1]");
        if (sr1 & 0x80) PRINTF(" [SRP0=1]");
        PRINTF("\r\n");
    }
    if (sr2_locked)
    {
        PRINTF("SR2 protection detected: 0x%02X", sr2);
        if (sr2 & 0x01) PRINTF(" [SRP1=1-HW_PROTECTED]");
        if (sr2 & 0x40) PRINTF(" [CMP=1]");
        if (sr2 & 0x38) PRINTF(" [LB=%d]", (sr2 >> 3) & 0x07);
        PRINTF("\r\n");
    }
    
    return sr1_locked || sr2_locked || sr3_locked;
}

/*!
 * @brief Unlock all Flash protection
 */
static status_t flash_unlock_all_protection(void)
{
    status_t status;
    uint8_t sr1, sr2, sr3;
    
    PRINTF("Starting Flash unlock procedure...\r\n");
    
    /* Read current status registers */
    read_flash_status_register(1, &sr1);
    read_flash_status_register(2, &sr2);
    read_flash_status_register(3, &sr3);
    
    PRINTF("Current values: SR1=0x%02X, SR2=0x%02X, SR3=0x%02X\r\n", sr1, sr2, sr3);
    
    /* Check for hardware write protection (SRP1=1) */
    if (sr2 & 0x01)
    {
        PRINTF("*** CRITICAL: Hardware Write Protection Detected! ***\r\n");
        PRINTF("SRP1=1 means Status Registers are hardware protected.\r\n");
        PRINTF("This typically requires:\r\n");
        PRINTF("1. WP# pin must be driven HIGH, or\r\n");
        PRINTF("2. Hardware design modification, or\r\n");
        PRINTF("3. Special unlock sequence\r\n");
        PRINTF("Attempting software unlock anyway...\r\n\r\n");
    }
    
    /* Try volatile unlock first (safer, reversible on power cycle) */
    PRINTF("Attempting volatile unlock...\r\n");
    
    /* Clear SR1 protection bits (keep everything else as 0) */
    if (sr1 != UNLOCK_SR1_VALUE)
    {
        PRINTF("Attempting to clear SR1 SEC bit and other protection...\r\n");
        status = flash_write_status_register(1, UNLOCK_SR1_VALUE, true);
        if (status != kStatus_Success)
        {
            PRINTF("Volatile SR1 unlock failed\r\n");
        }
    }
    
    /* Clear SR2 protection bits but keep QE=1 for proper operation */
    uint8_t target_sr2 = UNLOCK_SR2_VALUE; /* QE=1, all protection bits=0 */
    if (sr2 != target_sr2)
    {
        PRINTF("Attempting to clear SR2 CMP and SRP1 bits...\r\n");
        status = flash_write_status_register(2, target_sr2, true);
        if (status != kStatus_Success)
        {
            PRINTF("Volatile SR2 unlock failed\r\n");
        }
    }
    
    /* Verify volatile unlock worked */
    vTaskDelay(pdMS_TO_TICKS(20)); /* Longer delay for hardware protected devices */
    
    read_flash_status_register(1, &sr1);
    read_flash_status_register(2, &sr2);
    
    PRINTF("After volatile unlock: SR1=0x%02X, SR2=0x%02X\r\n", sr1, sr2);
    
    if (!is_flash_locked())
    {
        PRINTF("Volatile unlock successful!\r\n");
        return kStatus_Success;
    }
    
    /* If volatile unlock didn't work, try non-volatile (permanent) */
    PRINTF("Volatile unlock insufficient, attempting non-volatile unlock...\r\n");
    
    /* Try multiple approaches for non-volatile unlock */
    for (int attempt = 1; attempt <= 3; attempt++)
    {
        PRINTF("Non-volatile unlock attempt %d/3...\r\n", attempt);
        
        /* Non-volatile unlock - this is permanent until explicitly changed */
        status = flash_write_status_register(1, UNLOCK_SR1_VALUE, false);
        if (status == kStatus_Success)
        {
            PRINTF("SR1 non-volatile write succeeded\r\n");
        }
        
        status = flash_write_status_register(2, target_sr2, false);
        if (status == kStatus_Success)
        {
            PRINTF("SR2 non-volatile write succeeded\r\n");
        }
        
        /* Wait longer for non-volatile writes */
        vTaskDelay(pdMS_TO_TICKS(50));
        
        /* Re-read status registers */
        read_flash_status_register(1, &sr1);
        read_flash_status_register(2, &sr2);
        
        PRINTF("After attempt %d: SR1=0x%02X, SR2=0x%02X\r\n", attempt, sr1, sr2);
        
        if (!is_flash_locked())
        {
            PRINTF("Non-volatile unlock successful on attempt %d!\r\n", attempt);
            return kStatus_Success;
        }
        
        /* Small delay between attempts */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    /* Final check and detailed diagnosis */
    read_flash_status_register(1, &sr1);
    read_flash_status_register(2, &sr2);
    
    PRINTF("\r\n*** Flash Unlock Failed - Detailed Analysis ***\r\n");
    PRINTF("Final SR1: 0x%02X, Final SR2: 0x%02X\r\n", sr1, sr2);
    
    if (sr2 & 0x01)
    {
        PRINTF("DIAGNOSIS: SRP1=1 indicates hardware write protection is active.\r\n");
        PRINTF("SOLUTION: Trying hardware unlock sequences...\r\n");
        
        /* Try hardware unlock sequences */
        try_hardware_unlock_sequence();
        
        /* Check if hardware sequences worked */
        vTaskDelay(pdMS_TO_TICKS(20));
        read_flash_status_register(1, &sr1);
        read_flash_status_register(2, &sr2);
        
        PRINTF("After hardware unlock: SR1=0x%02X, SR2=0x%02X\r\n", sr1, sr2);
        
        if (!is_flash_locked())
        {
            PRINTF("Hardware unlock sequences successful!\r\n");
            return kStatus_Success;
        }
    }
    if (sr1 & 0x40)
    {
        PRINTF("DIAGNOSIS: SEC=1 indicates sector protection is still active.\r\n");
    }
    if (sr2 & 0x40)
    {
        PRINTF("DIAGNOSIS: CMP=1 indicates complement protection is still active.\r\n");
    }
    
    PRINTF("This Flash requires hardware intervention to unlock.\r\n");
    return kStatus_Fail;
}

/*!
 * @brief Try special hardware unlock sequences for XMC Flash
 */
static status_t try_hardware_unlock_sequence(void)
{
    PRINTF("\r\n*** Attempting Hardware Unlock Sequences ***\r\n");
    
    /* Method 1: Try Global Block Unlock command (98h) */
    PRINTF("Method 1: Global Block Unlock command (0x98)...\r\n");
    
    flexspi_transfer_t flashXfer;
    status_t status;
    
    /* Setup LUT for Global Block Unlock command */
    uint32_t globalUnlockLUT[4] = {
        /* Global Block Unlock: CMD only */
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x98, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
        0, 0
    };
    
    /* Update LUT table */
    FLEXSPI_UpdateLUT(FLEXSPI, NOR_CMD_LUT_SEQ_IDX_WREN * 4, globalUnlockLUT, 4);
    
    /* Setup FlexSPI transfer */
    flashXfer.deviceAddress = 0;
    flashXfer.port          = FLASH_PORT;
    flashXfer.cmdType       = kFLEXSPI_Command;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_WREN;
    flashXfer.data          = NULL;
    flashXfer.dataSize      = 0;
    
    status = FLEXSPI_TransferBlocking(FLEXSPI, &flashXfer);
    if (status == kStatus_Success)
    {
        PRINTF("Global Block Unlock command sent\r\n");
        vTaskDelay(pdMS_TO_TICKS(50)); /* Wait for unlock */
    }
    
    /* Method 2: Try Hardware Reset sequence */
    PRINTF("Method 2: Hardware Reset sequence...\r\n");
    
    /* Send Reset Enable (66h) + Reset (99h) */
    uint32_t resetEnableLUT[4] = {
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x66, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
        0, 0
    };
    
    FLEXSPI_UpdateLUT(FLEXSPI, NOR_CMD_LUT_SEQ_IDX_WRENV * 4, resetEnableLUT, 4);
    
    flashXfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_WRENV;
    status = FLEXSPI_TransferBlocking(FLEXSPI, &flashXfer);
    
    if (status == kStatus_Success)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
        
        /* Send Reset command */
        uint32_t resetLUT[4] = {
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x99, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
            0, 0
        };
        
        FLEXSPI_UpdateLUT(FLEXSPI, NOR_CMD_LUT_SEQ_IDX_WRSR1 * 4, resetLUT, 4);
        
        flashXfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_WRSR1;
        status = FLEXSPI_TransferBlocking(FLEXSPI, &flashXfer);
        
        if (status == kStatus_Success)
        {
            PRINTF("Reset sequence sent\r\n");
            vTaskDelay(pdMS_TO_TICKS(100)); /* Wait for reset to complete */
            
            /* Re-initialize mflash after reset */
            mflash_drv_init();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
    /* Method 3: Try power cycle simulation */
    PRINTF("Method 3: Power cycle simulation...\r\n");
    
    /* Send Deep Power Down (B9h) */
    uint32_t powerDownLUT[4] = {
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xB9, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
        0, 0
    };
    
    FLEXSPI_UpdateLUT(FLEXSPI, NOR_CMD_LUT_SEQ_IDX_WRSR2 * 4, powerDownLUT, 4);
    
    flashXfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_WRSR2;
    status = FLEXSPI_TransferBlocking(FLEXSPI, &flashXfer);
    
    if (status == kStatus_Success)
    {
        PRINTF("Deep Power Down sent\r\n");
        vTaskDelay(pdMS_TO_TICKS(50));
        
        /* Send Release from Deep Power Down (ABh) */
        uint32_t powerUpLUT[4] = {
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xAB, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
            0, 0
        };
        
        FLEXSPI_UpdateLUT(FLEXSPI, NOR_CMD_LUT_SEQ_IDX_WRSR3 * 4, powerUpLUT, 4);
        
        flashXfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_WRSR3;
        status = FLEXSPI_TransferBlocking(FLEXSPI, &flashXfer);
        
        if (status == kStatus_Success)
        {
            PRINTF("Release from Deep Power Down sent\r\n");
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    
    PRINTF("Hardware unlock sequences completed.\r\n");
    return kStatus_Success;
}
