/*
 * Copyright (c) 2019, Manivannan Sadhasivam <mani@kernel.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <fsl_iomuxc.h>

static int mimx8mm6_evk_pinmux_init(struct device *dev)
{
	ARG_UNUSED(dev);

#ifdef CONFIG_I2C_3
    IOMUXC_SetPinMux(IOMUXC_I2C3_SCL_I2C3_SCL, 1U);
    IOMUXC_SetPinConfig(IOMUXC_I2C3_SCL_I2C3_SCL,
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_FSEL(2U) |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
    IOMUXC_SetPinMux(IOMUXC_I2C3_SDA_I2C3_SDA, 1U);
    IOMUXC_SetPinConfig(IOMUXC_I2C3_SDA_I2C3_SDA,
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_FSEL(2U) |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
#endif

#ifdef CONFIG_I2S_3
    IOMUXC_SetPinMux(IOMUXC_I2C4_SDA_GPIO5_IO21, 1U);
    IOMUXC_SetPinMux(IOMUXC_SAI3_MCLK_SAI3_MCLK, 0U);
    IOMUXC_SetPinConfig(IOMUXC_SAI3_MCLK_SAI3_MCLK,
                        IOMUXC_SW_PAD_CTL_PAD_DSE(2U) |
                        IOMUXC_SW_PAD_CTL_PAD_FSEL(2U) |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
    IOMUXC_SetPinMux(IOMUXC_SAI3_TXC_SAI3_TX_BCLK, 0U);
    IOMUXC_SetPinConfig(IOMUXC_SAI3_TXC_SAI3_TX_BCLK,
                        IOMUXC_SW_PAD_CTL_PAD_DSE(2U) |
                        IOMUXC_SW_PAD_CTL_PAD_FSEL(2U) |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
    IOMUXC_SetPinMux(IOMUXC_SAI3_TXD_SAI3_TX_DATA0, 0U);
    IOMUXC_SetPinConfig(IOMUXC_SAI3_TXD_SAI3_TX_DATA0,
                        IOMUXC_SW_PAD_CTL_PAD_DSE(2U) |
                        IOMUXC_SW_PAD_CTL_PAD_FSEL(2U) |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
    IOMUXC_SetPinMux(IOMUXC_SAI3_TXFS_SAI3_TX_SYNC, 0U);
    IOMUXC_SetPinConfig(IOMUXC_SAI3_TXFS_SAI3_TX_SYNC,
                        IOMUXC_SW_PAD_CTL_PAD_DSE(2U) |
                        IOMUXC_SW_PAD_CTL_PAD_FSEL(2U) |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
#endif

#ifdef CONFIG_UART_MCUX_IMX_4
    IOMUXC_SetPinMux(IOMUXC_UART4_RXD_UART4_RX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_UART4_RXD_UART4_RX,
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_FSEL(2U));
    IOMUXC_SetPinMux(IOMUXC_UART4_TXD_UART4_TX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_UART4_TXD_UART4_TX,
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_FSEL(2U));
#endif  /* CONFIG_UART_MCUX_IMX_4 */

	return 0;

}

SYS_INIT(mimx8mm6_evk_pinmux_init, PRE_KERNEL_1, 0);
