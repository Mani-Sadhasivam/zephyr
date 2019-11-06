/*
 * Copyright (c) 2019 Broadcom
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <device.h>
#include <irq_nextlevel.h>
#include <dt-bindings/interrupt-controller/arm-gic.h>
#include <drivers/interrupt_controller/itr_common.h>

#define GIC_DIST_BASE		DT_INST_0_ARM_GIC_BASE_ADDRESS_0
#define GIC_RDIST_BASE		DT_INST_0_ARM_GIC_BASE_ADDRESS_1

#define GICR_SGIBASE_OFFSET	0x10000

/* Offsets from GICD base or GICR sgi_base */
#define	GIC_CTRL		0x0
#define	GIC_TYPER		0x4
#define	GIC_IIDR		0x8
#define	GIC_IGROUPR		0x80
#define	GIC_ISENABLER		0x100
#define	GIC_ICENABLER		0x180
#define	GIC_ISPENDR		0x200
#define	GIC_ICPENDR		0x280
#define	GIC_ISACTIVER		0x300
#define	GIC_ICACTIVER		0x380
#define	GIC_IPRIORITYR		0x400
#define	GIC_ITARGETSR		0x800
#define	GIC_ICFGR		0xc00
#define GIC_IGROUPMODR		0xd00
#define	GIC_SGIR		0xf00

#define	GICD_CTRL		GIC_DIST_BASE
#define	GICR_CTRL		GIC_RDIST_BASE

#define GICR_WAKER		0x14
#define GIC_WAKER_PS		0x1
#define GIC_WAKER_CA		0x2

#define IGROUPR(base, n)	(base + GIC_IGROUPR + (n) * 4)
#define ISENABLER(base, n)	(base + GIC_ISENABLER + (n) * 4)
#define ICENABLER(base, n)	(base + GIC_ICENABLER + (n) * 4)
#define ISPENDR(base, n)	(base + GIC_ISPENDR + (n) * 4)
#define ICPENDR(base, n)	(base + GIC_ICPENDR + (n) * 4)
#define IPRIORITYR(base, n)	(base + GIC_IPRIORITYR + n)
#define ITARGETSR(base, n)	(base + GIC_ITARGETSR + (n) * 8)
#define ICFGR(base, n)		(base + GIC_ICFGR + (n) * 8)
#define IGROUPMODR(base, n)	(base + GIC_IGROUPMODR + (n) * 4)

#define NUM_INTR_PER_REG	0x1F
/* system register interface to GICv3 */
#define ICC_IGRPEN1_EL1         S3_0_C12_C12_7
#define ICC_SGI1R               S3_0_C12_C11_5
#define ICC_SRE_EL1             S3_0_C12_C12_5
#define ICC_SRE_EL2             S3_4_C12_C9_5
#define ICC_SRE_EL3             S3_6_C12_C12_5
#define ICC_CTLR_EL1            S3_0_C12_C12_4
#define ICC_CTLR_EL3            S3_6_C12_C12_4
#define ICC_PMR_EL1             S3_0_C4_C6_0
#define ICC_RPR_EL1             S3_0_C12_C11_3
#define ICC_IGRPEN1_EL3         S3_6_c12_c12_7
#define ICC_IGRPEN0_EL1         S3_0_c12_c12_6
#define ICC_HPPIR0_EL1          S3_0_c12_c8_2
#define ICC_HPPIR1_EL1          S3_0_c12_c12_2
#define ICC_IAR0_EL1            S3_0_c12_c8_0
#define ICC_IAR1_EL1            S3_0_c12_c12_0
#define ICC_EOIR0_EL1           S3_0_c12_c8_1
#define ICC_EOIR1_EL1           S3_0_c12_c12_1
#define ICC_SGI0R_EL1           S3_0_c12_c11_7

#define	GIC_SPI_BASE		32

#define GIC_INTID_PENDING_NONE	1023
/* allow all priorities accessible by current state */
#define GIC_DEFAULT_PRIO	0xff

/* Interrupt group definitions */
#define ENABLE_G0		0
#define ENABLE_G1NS		1
#define ENABLE_G1S		2
#define RWP			31
/* ICC_SRE_ELx SRE bit */
#define SRE			0
#define DIF			1
#define DIB			2

#define GIC_INIT_PRIORITY_DEFAULT 0

/*
 * wait for read write progress
 * TODO: add timed wait
 */
static int gic_wait_rwp(uint32_t intid)
{
	mem_addr_t base;

	base = (intid < GIC_SPI_BASE) ? GICR_CTRL : GICD_CTRL;

	while (sys_read32(base) & BIT(RWP))
		;

	return 0;
}

/* set configurations for an interrupt */
void gic_intr_config(u32_t intid, uint32_t prio, uint32_t flags)
{
	u32_t bit = intid & NUM_INTR_PER_REG;
	u32_t mask = BIT(bit);
	u32_t idx = intid / 32;
	mem_addr_t base;

	base = (intid < GIC_SPI_BASE) ?
	       (GIC_RDIST_BASE + GICR_SGIBASE_OFFSET)
	       : GIC_DIST_BASE;

	/* Disable the interrupt */
	sys_write32(mask, ICENABLER(base, idx));

	gic_wait_rwp(intid);

#ifndef ZEPHYR_NS_STATE
	/* Assign it to group0 */
	sys_clear_bit(IGROUPR(base, idx), bit);

	/* Assign it to group1S */
	sys_set_bit(IGROUPMODR(base, idx), bit);
#endif
	/* PRIORITYR registers provide byte access */
	sys_write8(prio, IPRIORITYR(base, intid));
	/* TODO: use flags to set more configurations */

	printk("Config INTID %d\n", intid);
}

static void gic_intr_enable(struct device *dev, uint32_t intid)
{
	u32_t mask = BIT(intid & NUM_INTR_PER_REG);
	u32_t idx = intid / 32;
	mem_addr_t base;

	base = (intid < GIC_SPI_BASE) ?
	       (GIC_RDIST_BASE + GICR_SGIBASE_OFFSET)
	       : GIC_DIST_BASE;

	sys_write32(mask, ISENABLER(base, idx));

	printk("Enable INTID %d\n", intid);
}

static void gic_intr_disable(struct device *dev, uint32_t intid)
{
	u32_t mask = BIT(intid & NUM_INTR_PER_REG);
	u32_t idx = intid / 32;
	mem_addr_t base;

	base = (intid < GIC_SPI_BASE) ?
	       (GIC_RDIST_BASE + GICR_SGIBASE_OFFSET)
	       : GIC_DIST_BASE;

	sys_write32(~mask, ICENABLER(base, idx));
	/* poll to ensure write is complete */
	gic_wait_rwp(intid);
}

#ifndef ZEPHYR_NS_STATE
/*
 * Wake up GIC redistributor.
 * clear ProcessorSleep and wait till ChildAsleep is cleared.
 * ProcessSleep to be cleared only when ChildAsleep is set
 * Check if redistributor is not powered already.
 */
static void gic_rdist_enable(mem_addr_t rdist)
{
	if (!(sys_read32(rdist + GICR_WAKER) & BIT(GIC_WAKER_CA)))
		return;

	sys_clear_bit(rdist + GICR_WAKER, GIC_WAKER_PS);
	while (sys_read32(rdist + GICR_WAKER) & BIT(GIC_WAKER_CA))
		;
}
#endif

/* Initialize the cpu interface */
static void gic_cpuif_init(void)
{
	u32_t icc_sre;

	/* disable all sgi ppi */
	sys_write32(~0, ICENABLER((GIC_RDIST_BASE + GICR_SGIBASE_OFFSET), 0));
	/* any sgi/ppi intid ie. 0-31 will select GICR_CTRL */
	gic_wait_rwp(0);

	/* check if system interface can be enabled */
	icc_sre = read_sysreg(ICC_SRE_EL1);
	if (!(icc_sre & BIT(SRE))) {
		write_sysreg((icc_sre | BIT(SRE) | BIT(DIB) | BIT(DIF)),
			     ICC_SRE_EL1);
		icc_sre = read_sysreg(ICC_SRE_EL1);
		assert(icc_sre & BIT(SRE));
	}

	write_sysreg(GIC_DEFAULT_PRIO, ICC_PMR_EL1);

	write_sysreg(1, ICC_IGRPEN1_EL1);
}

/*
 * TODO: program the SPI default configurations in case of standlone Zephyr
 * ie: if the previous boot stages did not do the settings
 */
static void gic_dist_init(void)
{
#ifndef ZEPHYR_NS_STATE
	/* enable Group 1 secure interrupts */
	sys_set_bit(GICD_CTRL, ENABLE_G1S);
#else
	/* enable Group 1 non secure interrupts */
	/* TODO: handle legacy mode based on ARE_NS */
	sys_set_bit(GICD_CTRL, ENABLE_G1NS);
#endif
}

/* first level interrupt handler */
void gic_intr_handler(void)
{
	int intid = read_sysreg(ICC_IAR1_EL1);

	while (intid != GIC_INTID_PENDING_NONE) {
		if (generic_handle_intr(intid) == INTR_UNHANDLED) {
			printk("Unhandled interrupt %d\n", intid);
			irq_disable(intid);
		}
		/* eoi will move the state to inactive or active pending */
		write_sysreg(intid, ICC_EOIR1_EL1);
		/* check if more interrupts are pending */
		intid = read_sysreg(ICC_IAR1_EL1);
	}
}

/*
 * currently use already available irq_next_level_api. This needs to change to
 * more suitable ops structure.
 */
static const struct irq_next_level_api gic_apis = {
	.intr_enable = gic_intr_enable,
	.intr_disable = gic_intr_disable,
	.intr_get_state = NULL,
	.intr_set_priority = NULL,
};

static int gic_init(struct device *unused);
DEVICE_AND_API_INIT(arm_gicv3, DT_INST_0_ARM_GIC_LABEL, gic_init, NULL, NULL,
		    PRE_KERNEL_1, GIC_INIT_PRIORITY_DEFAULT,
		    &gic_apis);

static int gic_init(struct device *unused)
{
	set_intr_handler(gic_intr_handler);
	register_intctlr(DEVICE_GET(arm_gicv3));

	gic_dist_init();
#ifndef ZEPHYR_NS_STATE
	gic_rdist_enable(GIC_RDIST_BASE);
#endif
	gic_cpuif_init();

	return 0;
}
