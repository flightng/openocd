// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Unicmicro UM324xF (UM3241F) embedded flash controller driver for OpenOCD.
 *
 * IMPLEMENTATION NOTE
 * -------------------
 * The first revision of this driver embedded the vendor-published
 * UM324xF.FLM PrgCode and ran it as a target-side flash algorithm. On the
 * actual UM3241F reference board, the FLM's Init() prologue runs a
 * `clock_init` helper that writes RCM unlock keys at offset 0x11F0/0x1130
 * (absolute 0x40B021F0 / 0x40B02130). On real silicon those writes do not
 * stick and the subsequent reads/writes inside the algorithm escalate to
 * a HardFault -> double-fault -> lockup, which OpenOCD reports as
 * "Init() algorithm failed".
 *
 * Bench experiment on real silicon: with the chip running ANY firmware
 * chip running ANY firmware (so its RCM has already been brought up by
 * vendor SystemInit), a manual sequence over SWD AHB-AP works perfectly:
 *
 *   1. mww EFC_TIME 0xA500002F     ; the 0xA500 high half is the write key,
 *                                  ; only the low byte 0x2F lands in the
 *                                  ; register (verified by readback)
 *   2. mww EFC_CTRL  <op-bit>      ; bit0=PROG bit2=PERASE bit3=CERASE
 *   3. mww EFC_SEC   0x55AAAA55    ; per-op unlock
 *   4. mww <trigger-addr> <data>   ; the actual flash store
 *   5. poll EFC_STATUS bit0 == 1   ; DONE
 *   6. mww EFC_CTRL  0             ; clear op bit
 *
 * Because every step is a single AHB-AP transaction, the whole sequence
 * runs from the OpenOCD host, no on-target algorithm needed, no work-area
 * allocation, no Init/UnInit/lockup recovery surface. Programming is
 * adapter-clock bound (~1 MHz SWD here), but for the 512 KB UM324xF flash
 * that is still well under a minute end-to-end and a one-time cost during
 * bring-up; the driver can be rev'd to a target-side block-write algorithm
 * once we understand why the vendor FLM's clock_init misbehaves.
 *
 * The previous "embedded PrgCode blob" approach is left as the upstream
 * git history of this file.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/target.h>

/* ----- EFC register map (verified on real UM3241F silicon) ----- */

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

/* ----- Geometry (from FlashDevice descriptor in UM324xF.FLM) ----- */

#define UM324XF_FLASH_BASE          0x08000000
#define UM324XF_FLASH_SIZE          0x00080000  /* 512 KB */
#define UM324XF_SECTOR_SIZE         0x00002000  /* 8 KB; 64 uniform sectors */
#define UM324XF_NUM_SECTORS         (UM324XF_FLASH_SIZE / UM324XF_SECTOR_SIZE)

/* Per-op timeouts (ms). Worst-case silicon spec doubled for safety margin. */

#define UM324XF_TMO_PROG_MS         50    /* per word */
#define UM324XF_TMO_ERASE_SECT_MS   8000  /* per 8 KB sector */
#define UM324XF_TMO_ERASE_CHIP_MS   60000 /* whole 512 KB */

/* ----- Driver private state ----- */

struct um324xf_flash_bank {
	bool probed;
	bool time_initialized; /* once-per-session EFC_TIME write tracking */
};

/* ----- Helpers ----- */

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
	uint32_t status;
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

	/* set mode bit */
	retval = target_write_u32(target, UM324XF_EFC_CTRL, op_bit);
	if (retval != ERROR_OK)
		return retval;

	/* per-op unlock */
	retval = target_write_u32(target, UM324XF_EFC_SEC, UM324XF_EFC_SEC_UNLOCK);
	if (retval != ERROR_OK)
		goto clear_ctrl;

	/* trigger - the actual store the EFC turns into a flash op */
	retval = target_write_u32(target, trigger_addr, trigger_data);
	if (retval != ERROR_OK)
		goto clear_ctrl;

	/* poll DONE */
	retval = um324xf_poll_done(target, timeout_ms);

clear_ctrl:
	/* always clear the op bit, even on failure, so the EFC isn't left armed */
	(void)target_write_u32(target, UM324XF_EFC_CTRL, 0);
	return retval;
}

/* ----- OpenOCD driver hooks ----- */

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
	LOG_INFO("UM324xF: %u KB flash, %u sectors x %u KB, base " TARGET_ADDR_FMT,
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
	if (retval != ERROR_OK)
		return retval;

	const bool whole_chip = (first == 0 && last == bank->num_sectors - 1);
	if (whole_chip) {
		LOG_INFO("UM324xF: chip erase");
		retval = um324xf_run_op(target, UM324XF_EFC_CTRL_CERASE,
			UM324XF_FLASH_BASE, 0, UM324XF_TMO_ERASE_CHIP_MS);
		if (retval != ERROR_OK)
			LOG_ERROR("UM324xF: chip-erase failed");
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

static int um324xf_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
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

	retval = um324xf_init_time(bank);
	if (retval != ERROR_OK)
		return retval;

	uint32_t flash_addr = bank->base + offset;
	uint32_t words = count / 4;
	uint32_t progress_step = words / 20; /* ~5% progress increments */
	if (progress_step == 0)
		progress_step = 1;

	for (uint32_t i = 0; i < words; i++) {
		uint32_t data = buf_get_u32(buffer + i * 4, 0, 32);
		uint32_t word_addr = flash_addr + i * 4;

		retval = um324xf_run_op(target, UM324XF_EFC_CTRL_PROG,
			word_addr, data, UM324XF_TMO_PROG_MS);
		if (retval != ERROR_OK) {
			LOG_ERROR("UM324xF: word program failed at 0x%08" PRIx32, word_addr);
			return retval;
		}

		if ((i % progress_step) == 0) {
			LOG_DEBUG("UM324xF: programmed %u/%u words (%u%%)",
				(unsigned)i, (unsigned)words, (unsigned)(100 * i / words));
		}
	}
	LOG_INFO("UM324xF: programmed %u words (%u bytes) at 0x%08" PRIx32,
		(unsigned)words, (unsigned)count, flash_addr);
	return ERROR_OK;
}

static int um324xf_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	command_print_sameline(cmd,
		"Unicmicro UM324xF (UM3241F), %u KB flash, %u sectors x %u KB",
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
	.erase              = um324xf_erase,
	.write              = um324xf_write,
	.read               = default_flash_read,
	.probe              = um324xf_probe,
	.auto_probe         = um324xf_auto_probe,
	.erase_check        = default_flash_blank_check,
	.info               = um324xf_get_info,
	.free_driver_priv   = um324xf_free_driver_priv,
};
