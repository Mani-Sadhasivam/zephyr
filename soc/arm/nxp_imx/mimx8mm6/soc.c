/*
 * Copyright (c) 2019, Manivannan Sadhasivam <mani@kernel.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <kernel.h>
#include <soc.h>
#include <device.h>
#include <fsl_clock.h>
#include <fsl_common.h>
#include <fsl_rdc.h>
#include <dt-bindings/rdc/imx_rdc.h>

/* OSC/PLL is already initialized by ROM and Cortex-A7 (u-boot) */
void SOC_RdcInit(void)
{
	/* Move M4 core to specific RDC domain 1 */
	rdc_domain_assignment_t assignment = {0};

	assignment.domainId = M4_DOMAIN_ID;
	RDC_SetMasterDomainAssignment(RDC, kRDC_Master_M4, &assignment);

	/*
	 * The M4 core is running at domain 1, enable clock gate for
	 * Iomux to run at domain 1.
	 */
	CLOCK_EnableClock(kCLOCK_Iomux0);
	CLOCK_EnableClock(kCLOCK_Iomux1);
	CLOCK_EnableClock(kCLOCK_Iomux2);
	CLOCK_EnableClock(kCLOCK_Iomux3);
	CLOCK_EnableClock(kCLOCK_Iomux4);

	CLOCK_EnableClock(kCLOCK_Qspi);

	/*
	 * The M4 core is running at domain 1, enable the PLL clock sources
	 * to domain 1.
	 */
	/* Enable SysPLL1 to Domain 1 */
	CLOCK_ControlGate(kCLOCK_SysPll1Gate, kCLOCK_ClockNeededAll);
	/* Enable SysPLL2 to Domain 1 */
	CLOCK_ControlGate(kCLOCK_SysPll2Gate, kCLOCK_ClockNeededAll);
	/* Enable SysPLL3 to Domain 1 */
	CLOCK_ControlGate(kCLOCK_SysPll3Gate, kCLOCK_ClockNeededAll);
	/* Enable AudioPLL1 to Domain 1 */
	CLOCK_ControlGate(kCLOCK_AudioPll1Gate, kCLOCK_ClockNeededAll);
	/* Enable AudioPLL2 to Domain 1 */
	CLOCK_ControlGate(kCLOCK_AudioPll2Gate, kCLOCK_ClockNeededAll);
	/* Enable VideoPLL1 to Domain 1 */
	CLOCK_ControlGate(kCLOCK_VideoPll1Gate, kCLOCK_ClockNeededAll);
}

/* AUDIO PLL1 configuration */
const ccm_analog_frac_pll_config_t g_audioPll1Config = {
    .refSel  = kANALOG_PllRefOsc24M, /* PLL reference OSC24M */
    .mainDiv = 655U,
    .dsm     = 23593U,
    .preDiv  = 5U,
    .postDiv = 2U, /* AUDIO PLL1 frequency  = 786432000HZ */
};

/* AUDIO PLL2 configuration */
const ccm_analog_frac_pll_config_t g_audioPll2Config = {
    .refSel  = kANALOG_PllRefOsc24M, /*!< PLL reference OSC24M */
    .mainDiv = 301U,
    .dsm     = 3670U,
    .preDiv  = 5U,
    .postDiv = 1U, /* AUDIO PLL2 frequency  = 722534399HZ */
};

void SOC_ClockInit(void)
{
	/* Switch AHB NOC root to 24M first in order to configure the SYSTEM PLL1 */
	CLOCK_SetRootMux(kCLOCK_RootAhb, kCLOCK_AhbRootmuxOsc24M);
	/* Switch AXI M4 root to 24M first in order to configure the SYSTEM PLL2 */
	CLOCK_SetRootMux(kCLOCK_RootM4, kCLOCK_M4RootmuxOsc24M);

	/* Init AUDIO PLL1 to run at 786432000HZ */
	CLOCK_InitAudioPll1(&g_audioPll1Config);
	/* Init AUDIO PLL2 to run at 722534399HZ */
	CLOCK_InitAudioPll2(&g_audioPll2Config);

	CLOCK_SetRootDivider(kCLOCK_RootM4, 1U, 2U);
	/* Switch cortex-m4 to SYSTEM PLL1 */
	CLOCK_SetRootMux(kCLOCK_RootM4, kCLOCK_M4RootmuxSysPll1);

	CLOCK_SetRootDivider(kCLOCK_RootAhb, 1U, 1U);
	/* Switch AHB to SYSTEM PLL1 DIV6 = 133MHZ */
	CLOCK_SetRootMux(kCLOCK_RootAhb, kCLOCK_AhbRootmuxSysPll1Div6);

	/* Set root clock to 800MHZ/ 2= 400MHZ */
	CLOCK_SetRootDivider(kCLOCK_RootAudioAhb, 1U, 2U);
	/* switch AUDIO AHB to SYSTEM PLL1 */
	CLOCK_SetRootMux(kCLOCK_RootAudioAhb, kCLOCK_AudioAhbRootmuxSysPll1);

	/* Set UART source to SysPLL1 Div10 80MHZ */
	CLOCK_SetRootMux(kCLOCK_RootUart4, kCLOCK_UartRootmuxSysPll1Div10);
	/* Set root clock to 80MHZ/ 1= 80MHZ */
	CLOCK_SetRootDivider(kCLOCK_RootUart4, 1U, 1U);

	/* Enable RDC clock */
	CLOCK_EnableClock(kCLOCK_Rdc);

	/*
	 * The purpose to enable the following modules clock is to make
	 * sure the M4 core could work normally when A53 core
	 * enters the low power state
	 */
	CLOCK_EnableClock(kCLOCK_Sim_display);
	CLOCK_EnableClock(kCLOCK_Sim_m);
	CLOCK_EnableClock(kCLOCK_Sim_main);
	CLOCK_EnableClock(kCLOCK_Sim_s);
	CLOCK_EnableClock(kCLOCK_Sim_wakeup);
	CLOCK_EnableClock(kCLOCK_Debug);
	CLOCK_EnableClock(kCLOCK_Dram);
	CLOCK_EnableClock(kCLOCK_Sec_Debug);

	CLOCK_EnableClock(kCLOCK_Uart4);

	/* Update core clock */
	SystemCoreClockUpdate();
}

/* Initialize MPU, configure non-cacheable memory */
void BOARD_InitMemory(void)
{
    extern uint32_t _nocache_ram_start[];
    extern uint32_t _nocache_ram_size[];
    uint32_t cacheStart = (uint32_t)_nocache_ram_start;
    uint32_t size       = (uint32_t)_nocache_ram_size;
    uint32_t i = 0;
    /* Make sure outstanding transfers are done. */
    __DMB();
    /* Disable the MPU. */
    MPU->CTRL = 0;

    /*
     *  The ARMv7-M default address map define the address space 0x20000000 to 0x3FFFFFFF as SRAM with Normal type, but
     *  there the address space 0x28000000 ~ 0x3FFFFFFF has been physically mapped to smart subsystems, so there need
     *  change the default memory attributes.
     *  Since the base address of MPU region should be multiples of region size, to make it simple, the MPU region 0 set
     *  the all 512M of SRAM space with device attributes, then disable subregion 0 and 1 (address space 0x20000000 ~
     *  0x27FFFFFF) to use the
     *  background memory attributes.
     */

    /* Select Region 0 and set its base address to the M4 code bus start address. */
    MPU->RBAR = (0x20000000U & MPU_RBAR_ADDR_Msk) | MPU_RBAR_VALID_Msk | (0 << MPU_RBAR_REGION_Pos);

    /* Region 0 setting:
     * 1) Disable Instruction Access;
     * 2) AP = 011b, full access;
     * 3) Non-shared device;
     * 4) Region Not Shared;
     * 5) Sub-Region 0,1 Disabled;
     * 6) MPU Protection Region size = 512M byte;
     * 7) Enable Region 0.
     */
    MPU->RASR = (0x1 << MPU_RASR_XN_Pos) | (0x3 << MPU_RASR_AP_Pos) | (0x2 << MPU_RASR_TEX_Pos) |
                (0x3 << MPU_RASR_SRD_Pos) | (28 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

    /*
     *  Non-cacheable area is provided in DDR memory, the DDR region 2MB - 128MB totally 126MB is revserved for CM4
     *  cores. You can put global or static uninitialized variables in NonCacheable section(initialized variables in
     *  NonCacheable.init section) to make them uncacheable. Since the base address of MPU region should be multiples of
     * region size,
     *  to make it simple, the MPU region 1 & 2 set all DDR address space 0x40000000 ~ 0xBFFFFFFF to be non-cacheable).
     *  Then MPU region 3 set the text and data section to be cacheable if the program running on DDR.
     *  The cacheable area base address should be multiples of its size in linker file, they can be modified per your
     * needs.
     */

    /* Select Region 1 and set its base address to the DDR start address. */
    MPU->RBAR = (0x40000000U & MPU_RBAR_ADDR_Msk) | MPU_RBAR_VALID_Msk | (1 << MPU_RBAR_REGION_Pos);

    /* Region 1 setting:
     * 1) Enable Instruction Access;
     * 2) AP = 011b, full access;
     * 3) Shared Device;
     * 4) MPU Protection Region size = 1024M byte;
     * 5) Enable Region 1.
     */
    MPU->RASR = (0x3 << MPU_RASR_AP_Pos) | (0x1 << MPU_RASR_B_Pos) | (29 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

    /* Select Region 2 and set its base address to the DDR start address. */
    MPU->RBAR = (0x80000000U & MPU_RBAR_ADDR_Msk) | MPU_RBAR_VALID_Msk | (2 << MPU_RBAR_REGION_Pos);

    /* Region 2 setting:
     * 1) Enable Instruction Access;
     * 2) AP = 011b, full access;
     * 3) Shared Device;
     * 4) MPU Protection Region size = 1024M byte;
     * 5) Enable Region 2.
     */
    MPU->RASR = (0x3 << MPU_RASR_AP_Pos) | (0x1 << MPU_RASR_B_Pos) | (29 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

    while ((size >> i) > 0x1U)
    {
        i++;
    }

    /* If run on DDR, configure text and data section to be cacheable */
    if (i != 0)
    {
        /* The MPU region size should be 2^N, 5<=N<=32, region base should be multiples of size. */
        assert((size & (size - 1)) == 0);
        assert(!(cacheStart % size));
        assert(size == (uint32_t)(1 << i));
        assert(i >= 5);

        /* Select Region 3 and set its base address to the cache able region start address. */
        MPU->RBAR = (cacheStart & MPU_RBAR_ADDR_Msk) | MPU_RBAR_VALID_Msk | (3 << MPU_RBAR_REGION_Pos);

        /* Region 3 setting:
         * 1) Enable Instruction Access;
         * 2) AP = 011b, full access;
         * 3) Outer and inner Cacheable, write and read allocate;
         * 4) Region Not Shared;
         * 5) All Sub-Region Enabled;
         * 6) MPU Protection Region size get from linker file;
         * 7) Enable Region 3.
         */
        MPU->RASR = (0x3 << MPU_RASR_AP_Pos) | (0x1 << MPU_RASR_TEX_Pos) | (0x1 << MPU_RASR_C_Pos) |
                    (0x1 << MPU_RASR_B_Pos) | ((i - 1) << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;
    }

    /* Enable Privileged default memory map and the MPU. */
    MPU->CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk;
    /* Memory barriers to ensure subsequence data & instruction
     * transfers using updated MPU settings.
     */
    __DSB();
    __ISB();
}

static int nxp_mimx8mm6_init(struct device *arg)
{
	ARG_UNUSED(arg);

	/* SoC specific RDC settings */
	SOC_RdcInit();

	/* SoC specific Clock settings */
	SOC_ClockInit();

	BOARD_InitMemory();
#ifdef CONFIG_I2C_3
	CLOCK_SetRootMux(kCLOCK_RootI2c3, kCLOCK_I2cRootmuxSysPll1Div5);
	CLOCK_SetRootDivider(kCLOCK_RootI2c3, 1U, 10U);
#endif

	return 0;
}

SYS_INIT(nxp_mimx8mm6_init, PRE_KERNEL_1, 0);
