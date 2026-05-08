// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Unicmicro UM324xF (UM3241F) embedded flash controller driver.
 *
 * Earlier bring-up tried to run the vendor-published FLM PrgCode directly on
 * the target. Its Init() prologue calls clock_init, which writes RCM unlock
 * keys at 0x40B021F0 and 0x40B02130. On tested UM3241F silicon those writes
 * read back as zero, and the FLM then faults while touching the same clock
 * block. The flash controller itself is not the failing part: direct SWD
 * writes of EFC_TIME, EFC_CTRL, EFC_SEC and a flash trigger store reliably
 * erase and program the device.
 *
 * The FLM disassembly shows ProgramPage/EraseSector/EraseChip do not call
 * clock_init. ProgramPage is just a word loop around the same EFC sequence
 * verified on hardware. This driver keeps the proven host-side erase path
 * because a full device erase is only 64 sectors (or one chip erase command),
 * while programming 512 KiB one word at a time over 1 MHz SWD costs tens of
 * seconds. The programming hot path below therefore runs a small Thumb-2
 * worker from target RAM with target_run_flash_async_algorithm(). OpenOCD
 * streams words into a circular RAM FIFO; the target consumes each word and
 * performs CTRL.PROG -> SEC unlock -> flash store -> DONE poll -> CTRL clear
 * locally.
 *
 * At 1 MHz SWD, the remaining host traffic is roughly one bulk RAM download
 * of the image plus FIFO pointer polling instead of five or more AP transfers
 * per programmed word. A 512 KiB image is expected to program in about
 * 5-10 seconds on the same adapter where the direct-register implementation
 * took about 30-60 seconds. Every flash operation path writes EFC_CTRL = 0
 * before returning, including host aborts and target algorithm failures.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <inttypes.h>
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>
#include <target/target.h>

/* ----- EFC register map (verified on UM3241F silicon) ----- */

#define UM324XF_EFC_BASE            0x40000000
#define UM324XF_EFC_CTRL            (UM324XF_EFC_BASE + 0x00)
#define UM324XF_EFC_TIME            (UM324XF_EFC_BASE + 0x04)
#define UM324XF_EFC_SEC             (UM324XF_EFC_BASE + 0x08)
#define UM324XF_EFC_STATUS          (UM324XF_EFC_BASE + 0x0C)

#define UM324XF_EFC_CTRL_PROG       (1u << 0)
#define UM324XF_EFC_CTRL_PERASE     (1u << 2)
#define UM324XF_EFC_CTRL_CERASE     (1u << 3)

#define UM324XF_EFC_STATUS_DONE     (1u << 0)

#define UM324XF_EFC_TIME_INIT       0xA500002F  /* write-key 0xA500 + value */
#define UM324XF_EFC_SEC_UNLOCK      0x55AAAA55  /* per-op unlock magic */

/* ----- Geometry (from the FLM FlashDevice descriptor) ----- */

#define UM324XF_FLASH_BASE          0x08000000
#define UM324XF_FLASH_SIZE          0x00080000  /* 512 KiB */
#define UM324XF_SECTOR_SIZE         0x00002000  /* 8 KiB; 64 uniform sectors */
#define UM324XF_NUM_SECTORS         (UM324XF_FLASH_SIZE / UM324XF_SECTOR_SIZE)

/* Per-op timeouts (ms). */

#define UM324XF_TMO_PROG_MS         50    /* per word, host fallback only */
#define UM324XF_TMO_ERASE_SECT_MS   8000  /* per 8 KiB sector */
#define UM324XF_TMO_ERASE_CHIP_MS   60000 /* whole 512 KiB */

#define UM324XF_WRITE_MIN_WA        256
#define UM324XF_WRITE_MIN_FIFO      16    /* two 32-bit FIFO slots plus header */

/*
 * Thumb-2 async write worker assembled for Cortex-M4.
 *
 * Register interface follows target_run_flash_async_algorithm():
 *   r0 = circular FIFO start, returns status
 *   r1 = circular FIFO end
 *   r2 = flash destination address
 *   r3 = number of 32-bit words to program
 *
 * The first two words at r0 are write/read pointers; FIFO data starts at r0+8.
 * rp = 0 reports a target-side abort to OpenOCD. The worker clears EFC_CTRL
 * after every programmed word and again immediately before the final BKPT.
 */
static const uint8_t um324xf_flash_write_code[] = {
    0xd0, 0xf8, 0x00, 0x80, 0xb8, 0xf1, 0x00, 0x0f,
    0x1a, 0xd0, 0x47, 0x68, 0x47, 0x45, 0xf7, 0xd0,
    0x4f, 0xf0, 0x80, 0x44, 0x01, 0x25, 0x25, 0x60,
    0x0f, 0x4d, 0xa5, 0x60, 0x57, 0xf8, 0x04, 0x6b,
    0x42, 0xf8, 0x04, 0x6b, 0xe5, 0x68, 0x15, 0xf0,
    0x01, 0x0f, 0xfb, 0xd0, 0x00, 0x25, 0x25, 0x60,
    0x8f, 0x42, 0x28, 0xbf, 0x00, 0xf1, 0x08, 0x07,
    0x47, 0x60, 0x01, 0x3b, 0x23, 0xb1, 0xdf, 0xe7,
    0x01, 0x26, 0x00, 0x25, 0x45, 0x60, 0x00, 0xe0,
    0x00, 0x26, 0x4f, 0xf0, 0x80, 0x44, 0x00, 0x25,
    0x25, 0x60, 0x30, 0x46, 0x00, 0xbe, 0x00, 0x00,
    0x55, 0xaa, 0xaa, 0x55,
};

struct um324xf_flash_bank {
    bool probed;
    bool time_initialized;
};

static int um324xf_clear_ctrl(struct target *target)
{
    return target_write_u32(target, UM324XF_EFC_CTRL, 0);
}

static int um324xf_init_time(struct flash_bank *bank)
{
    struct um324xf_flash_bank *priv = bank->driver_priv;
    struct target *target = bank->target;
    int retval;

    if (priv->time_initialized)
        return ERROR_OK;

    retval = target_write_u32(target, UM324XF_EFC_TIME, UM324XF_EFC_TIME_INIT);
    if (retval != ERROR_OK) {
        LOG_ERROR("UM324xF: failed to write EFC_TIME");
        return retval;
    }

    priv->time_initialized = true;
    LOG_DEBUG("UM324xF: EFC_TIME initialised (write-key 0xA500 + value 0x2F)");
    return ERROR_OK;
}

static int um324xf_poll_done(struct target *target, int timeout_ms)
{
    int64_t deadline = timeval_ms() + timeout_ms;
    uint32_t status = 0;
    int retval;

    do {
        retval = target_read_u32(target, UM324XF_EFC_STATUS, &status);
        if (retval != ERROR_OK)
            return retval;
        if (status & UM324XF_EFC_STATUS_DONE)
            return ERROR_OK;
    } while (timeval_ms() < deadline);

    LOG_ERROR("UM324xF: timeout waiting for EFC DONE (status=0x%08" PRIx32
        ", waited %d ms)", status, timeout_ms);
    return ERROR_FLASH_OPERATION_FAILED;
}

static int um324xf_run_op(struct target *target, uint32_t op_bit,
    uint32_t trigger_addr, uint32_t trigger_data, int timeout_ms)
{
    int retval;

    retval = target_write_u32(target, UM324XF_EFC_CTRL, op_bit);
    if (retval != ERROR_OK) {
        (void)um324xf_clear_ctrl(target);
        return retval;
    }

    retval = target_write_u32(target, UM324XF_EFC_SEC, UM324XF_EFC_SEC_UNLOCK);
    if (retval != ERROR_OK)
        goto clear_ctrl;

    retval = target_write_u32(target, trigger_addr, trigger_data);
    if (retval != ERROR_OK)
        goto clear_ctrl;

    retval = um324xf_poll_done(target, timeout_ms);

clear_ctrl:
    (void)um324xf_clear_ctrl(target);
    return retval;
}

FLASH_BANK_COMMAND_HANDLER(um324xf_flash_bank_command)
{
    struct um324xf_flash_bank *priv;

    if (CMD_ARGC < 6)
        return ERROR_COMMAND_SYNTAX_ERROR;

    priv = calloc(1, sizeof(*priv));
    if (!priv)
        return ERROR_FAIL;

    bank->driver_priv = priv;
    return ERROR_OK;
}

static int um324xf_probe(struct flash_bank *bank)
{
    struct um324xf_flash_bank *priv = bank->driver_priv;

    free(bank->sectors);
    bank->size = UM324XF_FLASH_SIZE;
    bank->num_sectors = UM324XF_NUM_SECTORS;
    bank->sectors = alloc_block_array(0, UM324XF_SECTOR_SIZE, UM324XF_NUM_SECTORS);
    if (!bank->sectors) {
        bank->num_sectors = 0;
        return ERROR_FAIL;
    }

    bank->write_start_alignment = 4;
    bank->write_end_alignment = 4;

    priv->probed = true;
    priv->time_initialized = false;
    LOG_INFO("UM324xF: %u KiB flash, %u sectors x %u KiB, base " TARGET_ADDR_FMT,
        (unsigned)(UM324XF_FLASH_SIZE / 1024),
        (unsigned)UM324XF_NUM_SECTORS,
        (unsigned)(UM324XF_SECTOR_SIZE / 1024),
        bank->base);
    return ERROR_OK;
}

static int um324xf_auto_probe(struct flash_bank *bank)
{
    struct um324xf_flash_bank *priv = bank->driver_priv;

    if (priv->probed)
        return ERROR_OK;
    return um324xf_probe(bank);
}

static int um324xf_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
    struct target *target = bank->target;
    int retval;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }
    if (last >= bank->num_sectors) {
        LOG_ERROR("UM324xF: sector %u out of range (have %u)",
            last, bank->num_sectors);
        return ERROR_FLASH_SECTOR_INVALID;
    }

    retval = um324xf_init_time(bank);
    if (retval != ERROR_OK) {
        (void)um324xf_clear_ctrl(target);
        return retval;
    }

    const bool whole_chip = (first == 0 && last == bank->num_sectors - 1);
    if (whole_chip) {
        LOG_INFO("UM324xF: chip erase");
        retval = um324xf_run_op(target, UM324XF_EFC_CTRL_CERASE,
            UM324XF_FLASH_BASE, 0, UM324XF_TMO_ERASE_CHIP_MS);
        if (retval != ERROR_OK)
            LOG_ERROR("UM324xF: chip erase failed");
        return retval;
    }

    for (unsigned int i = first; i <= last; i++) {
        uint32_t addr = bank->base + i * UM324XF_SECTOR_SIZE;

        retval = um324xf_run_op(target, UM324XF_EFC_CTRL_PERASE,
            addr, 0, UM324XF_TMO_ERASE_SECT_MS);
        if (retval != ERROR_OK) {
            LOG_ERROR("UM324xF: sector %u (0x%08" PRIx32 ") erase failed", i, addr);
            return retval;
        }
        LOG_DEBUG("UM324xF: erased sector %u (0x%08" PRIx32 ")", i, addr);
    }

    return ERROR_OK;
}

static int um324xf_write_block_async(struct flash_bank *bank, const uint8_t *buffer,
    uint32_t address, uint32_t words_count)
{
    struct target *target = bank->target;
    struct working_area *write_algorithm = NULL;
    struct working_area *source = NULL;
    struct armv7m_algorithm armv7m_info;
    struct reg_param reg_params[4];
    uint32_t buffer_size;
    int retval;

    retval = target_alloc_working_area(target, sizeof(um324xf_flash_write_code),
        &write_algorithm);
    if (retval != ERROR_OK) {
        LOG_WARNING("UM324xF: no working area available for flash write code");
        return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
    }

    retval = target_write_buffer(target, write_algorithm->address,
        sizeof(um324xf_flash_write_code), um324xf_flash_write_code);
    if (retval != ERROR_OK)
        goto free_code;

    buffer_size = target_get_working_area_avail(target);
    if (buffer_size > words_count * 4 + 12)
        buffer_size = words_count * 4 + 12;
    buffer_size &= ~3u;
    if (buffer_size < UM324XF_WRITE_MIN_FIFO)
        buffer_size = UM324XF_WRITE_MIN_FIFO;

    while (buffer_size >= UM324XF_WRITE_MIN_FIFO) {
        retval = target_alloc_working_area_try(target, buffer_size, &source);
        if (retval == ERROR_OK)
            break;
        buffer_size >>= 1;
        buffer_size &= ~3u;
    }

    if (!source) {
        LOG_WARNING("UM324xF: no working area available for flash write FIFO");
        retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
        goto free_code;
    }

    init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* FIFO start, status out */
    init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);    /* FIFO end */
    init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);    /* flash address */
    init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);    /* word count */

    buf_set_u32(reg_params[0].value, 0, 32, source->address);
    buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
    buf_set_u32(reg_params[2].value, 0, 32, address);
    buf_set_u32(reg_params[3].value, 0, 32, words_count);

    armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
    armv7m_info.core_mode = ARM_MODE_THREAD;

    retval = target_run_flash_async_algorithm(target, buffer, words_count, 4,
        0, NULL,
        ARRAY_SIZE(reg_params), reg_params,
        source->address, source->size,
        write_algorithm->address, 0,
        &armv7m_info);

    (void)um324xf_clear_ctrl(target);

    if (retval == ERROR_OK) {
        uint32_t status = buf_get_u32(reg_params[0].value, 0, 32);

        if (status != 0) {
            LOG_ERROR("UM324xF: target write algorithm returned status 0x%08" PRIx32,
                status);
            retval = ERROR_FLASH_OPERATION_FAILED;
        }
    } else {
        LOG_ERROR("UM324xF: target write algorithm failed");
    }

    for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
        destroy_reg_param(&reg_params[i]);

    target_free_working_area(target, source);

free_code:
    target_free_working_area(target, write_algorithm);
    return retval;
}

static int um324xf_write(struct flash_bank *bank, const uint8_t *buffer,
    uint32_t offset, uint32_t count)
{
    struct target *target = bank->target;
    uint32_t flash_addr = bank->base + offset;
    int retval;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }
    if ((offset & 3) != 0 || (count & 3) != 0) {
        LOG_ERROR("UM324xF: write must be word-aligned (offset=%" PRIu32
            ", count=%" PRIu32 ")", offset, count);
        return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
    }
    if (offset + count > bank->size) {
        LOG_ERROR("UM324xF: write past end of bank");
        return ERROR_FLASH_DST_OUT_OF_BANK;
    }
    if (count == 0)
        return ERROR_OK;

    retval = um324xf_init_time(bank);
    if (retval != ERROR_OK) {
        (void)um324xf_clear_ctrl(target);
        return retval;
    }

    retval = um324xf_write_block_async(bank, buffer, flash_addr, count / 4);
    if (retval != ERROR_OK)
        return retval;

    LOG_INFO("UM324xF: programmed %u words (%u bytes) at 0x%08" PRIx32,
        (unsigned)(count / 4), (unsigned)count, flash_addr);
    return ERROR_OK;
}

static int um324xf_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
    command_print_sameline(cmd,
        "Unicmicro UM324xF (UM3241F), %u KiB flash, %u sectors x %u KiB",
        (unsigned)(UM324XF_FLASH_SIZE / 1024),
        (unsigned)UM324XF_NUM_SECTORS,
        (unsigned)(UM324XF_SECTOR_SIZE / 1024));
    return ERROR_OK;
}

static void um324xf_free_driver_priv(struct flash_bank *bank)
{
    free(bank->driver_priv);
    bank->driver_priv = NULL;
}

const struct flash_driver unicmicro_efc_flash = {
    .name = "unicmicro_efc",
    .flash_bank_command = um324xf_flash_bank_command,
    .erase = um324xf_erase,
    .write = um324xf_write,
    .read = default_flash_read,
    .probe = um324xf_probe,
    .auto_probe = um324xf_auto_probe,
    .erase_check = default_flash_blank_check,
    .info = um324xf_get_info,
    .free_driver_priv = um324xf_free_driver_priv,
};
