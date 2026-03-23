// SPDX-License-Identifier: GPL-2.0
/*
 * CXL PCI state save/restore: DVSEC device registers and HDM decoders across
 * reset / link disable-enable.  Requires CONFIG_CXL_BUS=y (built-in CXL core).
 */
#if !IS_BUILTIN(CONFIG_CXL_BUS)
#error "drivers/pci/cxl.c is only built when CONFIG_CXL_BUS=y (CXL as module cannot hook PCI core)"
#endif

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <cxl/cxl.h>
#include <cxl/pci.h>

#include "pci.h"

MODULE_IMPORT_NS("CXL");

/* CXL DVSEC Device register offsets (CXL r3.1 §8.1.5) */
#define PCI_DVSEC_CXL_CTRL2_OFF		0x10
#define PCI_DVSEC_CXL_LOCK_OFF		0x14

#define CXL_HDM_MAX_DECODERS		32

struct pci_cmd_saved {
	struct pci_dev *pdev;
	u16 cmd;
};

DEFINE_FREE(restore_pci_cmd, struct pci_cmd_saved,
	    if (!(_T.cmd & PCI_COMMAND_MEMORY))
		    pci_write_config_word(_T.pdev, PCI_COMMAND, _T.cmd))

#define CXL_HDM_DECODER_CAP_OFF		0x0
#define   CXL_HDM_DECODER_COUNT_MASK	GENMASK(3, 0)
#define CXL_HDM_DECODER_CTRL_OFF	0x4
#define CXL_HDM_DECODER0_BASE_LOW(i)	(0x20 * (i) + 0x10)
#define CXL_HDM_DECODER0_BASE_HIGH(i)	(0x20 * (i) + 0x14)
#define CXL_HDM_DECODER0_SIZE_LOW(i)	(0x20 * (i) + 0x18)
#define CXL_HDM_DECODER0_SIZE_HIGH(i)	(0x20 * (i) + 0x1c)
#define CXL_HDM_DECODER0_CTRL(i)	(0x20 * (i) + 0x20)
#define   CXL_HDM_DECODER0_CTRL_LOCK	BIT(8)
#define   CXL_HDM_DECODER0_CTRL_COMMIT	BIT(9)
#define   CXL_HDM_DECODER0_CTRL_COMMITTED	BIT(10)
#define   CXL_HDM_DECODER0_CTRL_COMMIT_ERROR	BIT(11)
#define CXL_HDM_DECODER0_TL_LOW(i)	(0x20 * (i) + 0x24)
#define CXL_HDM_DECODER0_TL_HIGH(i)	(0x20 * (i) + 0x28)


struct cxl_hdm_decoder_snapshot {
	u32 base_lo, base_hi, size_lo, size_hi, ctrl, tl_lo, tl_hi;
};

struct cxl_pci_state {
	u16 dvsec;
	u16 dvsec_ctrl, dvsec_ctrl2;
	u32 range_base_hi[CXL_DVSEC_RANGE_MAX];
	u32 range_base_lo[CXL_DVSEC_RANGE_MAX];
	u16 dvsec_lock;
	bool dvsec_valid;

	int hdm_bar;
	unsigned long hdm_bar_offset, hdm_map_size;
	u32 hdm_global_ctrl;
	int hdm_count;
	struct cxl_hdm_decoder_snapshot decoders[CXL_HDM_MAX_DECODERS];
	bool hdm_valid;
};

static void __iomem *cxl_hdm_map(struct pci_dev *pdev, int *bar_out,
				 unsigned long *offset_out,
				 unsigned long *size_out)
{
	struct cxl_register_map map = { .host = &pdev->dev };
	struct cxl_component_reg_map *comp;
	resource_size_t hdm_phys;
	int bar, rc;

	rc = cxl_find_regblock(pdev, CXL_REGLOC_RBI_COMPONENT, &map);
	if (rc)
		return NULL;
	rc = cxl_setup_regs(&map);
	if (rc)
		return NULL;

	comp = &map.component_map;
	if (!comp->hdm_decoder.valid)
		return NULL;

	hdm_phys = map.resource + comp->hdm_decoder.offset;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		resource_size_t start = pci_resource_start(pdev, bar);
		resource_size_t len = pci_resource_len(pdev, bar);
		unsigned long offset;
		void __iomem *hdm;

		if (!start || !len || hdm_phys < start ||
		    hdm_phys + comp->hdm_decoder.size > start + len)
			continue;

		offset = hdm_phys - start;
		hdm = pci_iomap_range(pdev, bar, offset, comp->hdm_decoder.size);
		if (!hdm)
			return NULL;

		*bar_out = bar;
		*offset_out = offset;
		*size_out = comp->hdm_decoder.size;
		return hdm;
	}
	return NULL;
}

static void cxl_save_hdm(void __iomem *hdm, struct cxl_pci_state *state,
			 int count)
{
	int i;

	state->hdm_count = min_t(int, count, CXL_HDM_MAX_DECODERS);
	state->hdm_global_ctrl = readl(hdm + CXL_HDM_DECODER_CTRL_OFF);

	for (i = 0; i < state->hdm_count; i++) {
		struct cxl_hdm_decoder_snapshot *d = &state->decoders[i];

		d->base_lo = readl(hdm + CXL_HDM_DECODER0_BASE_LOW(i));
		d->base_hi = readl(hdm + CXL_HDM_DECODER0_BASE_HIGH(i));
		d->size_lo = readl(hdm + CXL_HDM_DECODER0_SIZE_LOW(i));
		d->size_hi = readl(hdm + CXL_HDM_DECODER0_SIZE_HIGH(i));
		d->ctrl = readl(hdm + CXL_HDM_DECODER0_CTRL(i));
		d->tl_lo = readl(hdm + CXL_HDM_DECODER0_TL_LOW(i));
		d->tl_hi = readl(hdm + CXL_HDM_DECODER0_TL_HIGH(i));
	}
}

static void cxl_restore_hdm(void __iomem *hdm, struct pci_dev *pdev,
			    const struct cxl_pci_state *state)
{
	int i;

	writel(state->hdm_global_ctrl, hdm + CXL_HDM_DECODER_CTRL_OFF);

	for (i = 0; i < state->hdm_count; i++) {
		const struct cxl_hdm_decoder_snapshot *d = &state->decoders[i];
		unsigned long timeout;
		u32 ctrl;

		if (!(d->ctrl & CXL_HDM_DECODER0_CTRL_COMMITTED))
			continue;

		ctrl = readl(hdm + CXL_HDM_DECODER0_CTRL(i));
		if ((ctrl & CXL_HDM_DECODER0_CTRL_LOCK) &&
		    (ctrl & CXL_HDM_DECODER0_CTRL_COMMITTED))
			continue;

		if (ctrl & CXL_HDM_DECODER0_CTRL_COMMITTED) {
			ctrl &= ~CXL_HDM_DECODER0_CTRL_COMMIT;
			writel(ctrl, hdm + CXL_HDM_DECODER0_CTRL(i));
		}

		writel(d->base_lo, hdm + CXL_HDM_DECODER0_BASE_LOW(i));
		writel(d->base_hi, hdm + CXL_HDM_DECODER0_BASE_HIGH(i));
		writel(d->size_lo, hdm + CXL_HDM_DECODER0_SIZE_LOW(i));
		writel(d->size_hi, hdm + CXL_HDM_DECODER0_SIZE_HIGH(i));
		writel(d->tl_lo, hdm + CXL_HDM_DECODER0_TL_LOW(i));
		writel(d->tl_hi, hdm + CXL_HDM_DECODER0_TL_HIGH(i));

		wmb();

		ctrl = d->ctrl & ~(CXL_HDM_DECODER0_CTRL_COMMITTED |
				   CXL_HDM_DECODER0_CTRL_COMMIT_ERROR);
		ctrl |= CXL_HDM_DECODER0_CTRL_COMMIT;
		writel(ctrl, hdm + CXL_HDM_DECODER0_CTRL(i));

		timeout = jiffies + msecs_to_jiffies(10);
		for (;;) {
			ctrl = readl(hdm + CXL_HDM_DECODER0_CTRL(i));
			if (ctrl & CXL_HDM_DECODER0_CTRL_COMMITTED)
				break;
			if (ctrl & CXL_HDM_DECODER0_CTRL_COMMIT_ERROR) {
				pci_warn(pdev,
					 "HDM decoder %d commit error on restore\n", i);
				break;
			}
			if (time_after(jiffies, timeout)) {
				pci_warn(pdev,
					 "HDM decoder %d commit timeout on restore\n", i);
				break;
			}
			cpu_relax();
		}
	}
}

static void cxl_save_dvsec(struct pci_dev *pdev, struct cxl_pci_state *state)
{
	u16 dvsec;
	int i, rc_ctrl, rc_ctrl2;

	dvsec = pci_find_dvsec_capability(pdev, PCI_VENDOR_ID_CXL,
					  PCI_DVSEC_CXL_DEVICE);
	if (!dvsec)
		return;

	state->dvsec = dvsec;
	rc_ctrl = pci_read_config_word(pdev, dvsec + PCI_DVSEC_CXL_CTRL,
				     &state->dvsec_ctrl);
	rc_ctrl2 = pci_read_config_word(pdev, dvsec + PCI_DVSEC_CXL_CTRL2_OFF,
				      &state->dvsec_ctrl2);
	if (rc_ctrl || rc_ctrl2) {
		pci_warn(pdev,
			 "CXL: DVSEC read failed (ctrl rc=%d, ctrl2 rc=%d)\n",
			 rc_ctrl, rc_ctrl2);
		return;
	}

	for (i = 0; i < CXL_DVSEC_RANGE_MAX; i++) {
		pci_read_config_dword(pdev,
			dvsec + PCI_DVSEC_CXL_RANGE_BASE_HIGH(i),
			&state->range_base_hi[i]);
		pci_read_config_dword(pdev,
			dvsec + PCI_DVSEC_CXL_RANGE_BASE_LOW(i),
			&state->range_base_lo[i]);
	}

	pci_read_config_word(pdev, dvsec + PCI_DVSEC_CXL_LOCK_OFF,
			     &state->dvsec_lock);

	state->dvsec_valid = true;
}

static u32 cxl_merge_rwl(u32 saved, u32 current_hw, u32 rwl_mask)
{
	return (current_hw & rwl_mask) | (saved & ~rwl_mask);
}

static void cxl_restore_dvsec(struct pci_dev *pdev,
			      const struct cxl_pci_state *state)
{
	u16 lock_reg = 0;
	int i;

	if (!state->dvsec_valid)
		return;

	pci_read_config_word(pdev, state->dvsec + PCI_DVSEC_CXL_LOCK_OFF,
			     &lock_reg);

	if (lock_reg & PCI_DVSEC_CXL_LOCK_CONFIG) {
		u16 hw_ctrl;
		u32 hw_range_hi, hw_range_lo;

		pci_read_config_word(pdev,
				state->dvsec + PCI_DVSEC_CXL_CTRL, &hw_ctrl);
		pci_write_config_word(pdev,
			state->dvsec + PCI_DVSEC_CXL_CTRL,
			(u16)cxl_merge_rwl(state->dvsec_ctrl, hw_ctrl,
					   PCI_DVSEC_CXL_CTRL_RWL));

		pci_write_config_word(pdev,
			state->dvsec + PCI_DVSEC_CXL_CTRL2_OFF,
			state->dvsec_ctrl2);

		for (i = 0; i < CXL_DVSEC_RANGE_MAX; i++) {
			pci_read_config_dword(pdev,
				state->dvsec + PCI_DVSEC_CXL_RANGE_BASE_HIGH(i),
				&hw_range_hi);
			pci_write_config_dword(pdev,
				state->dvsec + PCI_DVSEC_CXL_RANGE_BASE_HIGH(i),
				cxl_merge_rwl(state->range_base_hi[i],
					hw_range_hi,
					PCI_DVSEC_CXL_RANGE_BASE_HI_RWL));

			pci_read_config_dword(pdev,
				state->dvsec + PCI_DVSEC_CXL_RANGE_BASE_LOW(i),
				&hw_range_lo);
			pci_write_config_dword(pdev,
				state->dvsec + PCI_DVSEC_CXL_RANGE_BASE_LOW(i),
				cxl_merge_rwl(state->range_base_lo[i],
					hw_range_lo,
					PCI_DVSEC_CXL_RANGE_BASE_LO_RWL));
		}
	} else {
		pci_write_config_word(pdev,
				state->dvsec + PCI_DVSEC_CXL_CTRL,
				state->dvsec_ctrl);
		pci_write_config_word(pdev,
				state->dvsec + PCI_DVSEC_CXL_CTRL2_OFF,
				state->dvsec_ctrl2);
		for (i = 0; i < CXL_DVSEC_RANGE_MAX; i++) {
			pci_write_config_dword(pdev,
				state->dvsec + PCI_DVSEC_CXL_RANGE_BASE_HIGH(i),
				state->range_base_hi[i]);
			pci_write_config_dword(pdev,
				state->dvsec + PCI_DVSEC_CXL_RANGE_BASE_LOW(i),
				state->range_base_lo[i]);
		}

		pci_write_config_word(pdev,
			state->dvsec + PCI_DVSEC_CXL_LOCK_OFF,
			state->dvsec_lock);
	}
}

static void cxl_save_hdm_decoders(struct pci_dev *pdev,
				  struct cxl_pci_state *state)
{
	int hdm_bar;
	unsigned long hdm_bar_offset, hdm_map_size;
	void __iomem *hdm;
	u16 cmd;
	u32 cap;
	struct pci_cmd_saved saved __free(restore_pci_cmd) = {
		.pdev = pdev, .cmd = PCI_COMMAND_MEMORY,
	};

	pci_read_config_word(pdev, PCI_COMMAND, &cmd);
	saved.cmd = cmd;
	if (!(cmd & PCI_COMMAND_MEMORY))
		pci_write_config_word(pdev, PCI_COMMAND,
				      cmd | PCI_COMMAND_MEMORY);

	hdm = cxl_hdm_map(pdev, &hdm_bar, &hdm_bar_offset, &hdm_map_size);
	if (!hdm)
		return;

	cap = readl(hdm + CXL_HDM_DECODER_CAP_OFF);
	cxl_save_hdm(hdm, state, cxl_hdm_decoder_count(cap));
	state->hdm_bar = hdm_bar;
	state->hdm_bar_offset = hdm_bar_offset;
	state->hdm_map_size = hdm_map_size;
	state->hdm_valid = true;
	pci_iounmap(pdev, hdm);
}

static void cxl_restore_hdm_decoders(struct pci_dev *pdev,
				     const struct cxl_pci_state *state)
{
	void __iomem *hdm;
	u16 cmd;
	struct pci_cmd_saved saved __free(restore_pci_cmd) = {
		.pdev = pdev, .cmd = PCI_COMMAND_MEMORY,
	};

	if (!state->hdm_valid)
		return;

	pci_read_config_word(pdev, PCI_COMMAND, &cmd);
	saved.cmd = cmd;
	if (!(cmd & PCI_COMMAND_MEMORY))
		pci_write_config_word(pdev, PCI_COMMAND,
				      cmd | PCI_COMMAND_MEMORY);

	hdm = pci_iomap_range(pdev, state->hdm_bar, state->hdm_bar_offset,
			      state->hdm_map_size);
	if (!hdm) {
		pci_warn(pdev, "CXL: failed to map HDM for restore\n");
		return;
	}

	cxl_restore_hdm(hdm, pdev, state);
	pci_iounmap(pdev, hdm);
}

static bool pci_has_cxl_device_dvsec(struct pci_dev *dev)
{
	return pci_find_dvsec_capability(dev, PCI_VENDOR_ID_CXL,
					 PCI_DVSEC_CXL_DEVICE) != 0;
}

void pci_allocate_cxl_save_buffer(struct pci_dev *dev)
{
	if (!pci_has_cxl_device_dvsec(dev))
		return;

	if (pci_add_virtual_ext_cap_save_buffer(dev,
			PCI_EXT_CAP_ID_CXL_DVSEC_VIRTUAL,
			sizeof(struct cxl_pci_state)))
		pci_err(dev, "unable to allocate CXL save buffer\n");
}

void pci_save_cxl_state(struct pci_dev *pdev)
{
	struct pci_cap_saved_state *save_state;
	struct cxl_pci_state *state;

	save_state = pci_find_saved_ext_cap(pdev,
					    PCI_EXT_CAP_ID_CXL_DVSEC_VIRTUAL);
	if (!save_state)
		return;

	state = (struct cxl_pci_state *)save_state->cap.data;
	state->dvsec_valid = false;
	state->hdm_valid = false;

	cxl_save_dvsec(pdev, state);
	cxl_save_hdm_decoders(pdev, state);
}

void pci_restore_cxl_state(struct pci_dev *pdev)
{
	struct pci_cap_saved_state *save_state;
	struct cxl_pci_state *state;

	save_state = pci_find_saved_ext_cap(pdev,
					    PCI_EXT_CAP_ID_CXL_DVSEC_VIRTUAL);
	if (!save_state)
		return;

	state = (struct cxl_pci_state *)save_state->cap.data;
	if (!state->dvsec_valid && !state->hdm_valid)
		return;

	cxl_restore_dvsec(pdev, state);
	cxl_restore_hdm_decoders(pdev, state);
}
