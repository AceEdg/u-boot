// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023-2024, Linaro Limited
 *
 * Based on the Linux phy-qcom-snps-eusb2.c driver
 */

#include <clk.h>
#include <clk-uclass.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/devres.h>
#include <dm/ofnode.h>
#include <dm/read.h>
#include <generic-phy.h>
#include <i2c.h>
#include <malloc.h>
#include <power/regulator.h>
#include <reset.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>

#define USB_PHY_UTMI_CTRL0		(0x3c)
#define SLEEPM				BIT(0)
#define OPMODE_MASK			GENMASK(4, 3)
#define OPMODE_NONDRIVING		BIT(3)

#define USB_PHY_UTMI_CTRL5		(0x50)
#define POR				BIT(1)

#define USB_PHY_HS_PHY_CTRL_COMMON0	(0x54)
#define PHY_ENABLE			BIT(0)
#define SIDDQ_SEL			BIT(1)
#define SIDDQ				BIT(2)
#define RETENABLEN			BIT(3)
#define FSEL_MASK			GENMASK(6, 4)
#define FSEL_19_2_MHZ_VAL		(0x0)
#define FSEL_38_4_MHZ_VAL		(0x4)

#define USB_PHY_CFG_CTRL_1		(0x58)
#define PHY_CFG_PLL_CPBIAS_CNTRL_MASK	GENMASK(7, 1)

#define USB_PHY_CFG_CTRL_2		(0x5c)
#define PHY_CFG_PLL_FB_DIV_7_0_MASK	GENMASK(7, 0)
#define DIV_7_0_19_2_MHZ_VAL		(0x90)
#define DIV_7_0_38_4_MHZ_VAL		(0xc8)

#define USB_PHY_CFG_CTRL_3		(0x60)
#define PHY_CFG_PLL_FB_DIV_11_8_MASK	GENMASK(3, 0)
#define DIV_11_8_19_2_MHZ_VAL		(0x1)
#define DIV_11_8_38_4_MHZ_VAL		(0x0)

#define PHY_CFG_PLL_REF_DIV		GENMASK(7, 4)
#define PLL_REF_DIV_VAL			(0x0)

#define USB_PHY_HS_PHY_CTRL2		(0x64)
#define VBUSVLDEXT0			BIT(0)
#define USB2_SUSPEND_N			BIT(2)
#define USB2_SUSPEND_N_SEL		BIT(3)
#define VBUS_DET_EXT_SEL		BIT(4)

#define USB_PHY_CFG_CTRL_4		(0x68)
#define PHY_CFG_PLL_GMP_CNTRL_MASK	GENMASK(1, 0)
#define PHY_CFG_PLL_INT_CNTRL_MASK	GENMASK(7, 2)

#define USB_PHY_CFG_CTRL_5		(0x6c)
#define PHY_CFG_PLL_PROP_CNTRL_MASK	GENMASK(4, 0)
#define PHY_CFG_PLL_VREF_TUNE_MASK	GENMASK(7, 6)

#define USB_PHY_CFG_CTRL_6		(0x70)
#define PHY_CFG_PLL_VCO_CNTRL_MASK	GENMASK(2, 0)

#define USB_PHY_CFG_CTRL_7		(0x74)

#define USB_PHY_CFG_CTRL_8		(0x78)
#define PHY_CFG_TX_FSLS_VREF_TUNE_MASK	GENMASK(1, 0)
#define PHY_CFG_TX_FSLS_VREG_BYPASS	BIT(2)
#define PHY_CFG_TX_HS_VREF_TUNE_MASK	GENMASK(5, 3)
#define PHY_CFG_TX_HS_XV_TUNE_MASK	GENMASK(7, 6)

#define USB_PHY_CFG_CTRL_9		(0x7c)
#define PHY_CFG_TX_PREEMP_TUNE_MASK	GENMASK(2, 0)
#define PHY_CFG_TX_RES_TUNE_MASK	GENMASK(4, 3)
#define PHY_CFG_TX_RISE_TUNE_MASK	GENMASK(6, 5)
#define PHY_CFG_RCAL_BYPASS		BIT(7)

#define USB_PHY_CFG_CTRL_10		(0x80)

#define USB_PHY_CFG0			(0x94)
#define DATAPATH_CTRL_OVERRIDE_EN	BIT(0)
#define CMN_CTRL_OVERRIDE_EN		BIT(1)

#define UTMI_PHY_CMN_CTRL0		(0x98)
#define TESTBURNIN			BIT(6)

#define USB_PHY_FSEL_SEL		(0xb8)
#define FSEL_SEL			BIT(0)

#define USB_PHY_APB_ACCESS_CMD		(0x130)
#define RW_ACCESS			BIT(0)
#define APB_START_CMD			BIT(1)
#define APB_LOGIC_RESET			BIT(2)

#define USB_PHY_APB_ACCESS_STATUS	(0x134)
#define ACCESS_DONE			BIT(0)
#define TIMED_OUT			BIT(1)
#define ACCESS_ERROR			BIT(2)
#define ACCESS_IN_PROGRESS		BIT(3)

#define USB_PHY_APB_ADDRESS		(0x138)
#define APB_REG_ADDR_MASK		GENMASK(7, 0)

#define USB_PHY_APB_WRDATA_LSB		(0x13c)
#define APB_REG_WRDATA_7_0_MASK		GENMASK(3, 0)

#define USB_PHY_APB_WRDATA_MSB		(0x140)
#define APB_REG_WRDATA_15_8_MASK	GENMASK(7, 4)

#define USB_PHY_APB_RDDATA_LSB		(0x144)
#define APB_REG_RDDATA_7_0_MASK		GENMASK(3, 0)

#define USB_PHY_APB_RDDATA_MSB		(0x148)
#define APB_REG_RDDATA_15_8_MASK	GENMASK(7, 4)

struct qcom_snps_eusb2_phy_priv {
	void __iomem *base;
	struct clk *ref_clk;
	struct reset_ctl_bulk resets;
};

static void qcom_snps_eusb2_hsphy_write_mask(void __iomem *base, u32 offset,
					     u32 mask, u32 val)
{
	u32 reg;

	reg = readl_relaxed(base + offset);
	reg &= ~mask;
	reg |= val & mask;
	writel_relaxed(reg, base + offset);

	/* Ensure above write is completed */
	readl_relaxed(base + offset);
}

static void qcom_eusb2_default_parameters(struct qcom_snps_eusb2_phy_priv *qcom_snps_eusb2)
{
	/* default parameters: tx pre-emphasis */
	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_9,
					 PHY_CFG_TX_PREEMP_TUNE_MASK,
					 FIELD_PREP(PHY_CFG_TX_PREEMP_TUNE_MASK, 0));

	/* tx rise/fall time */
	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_9,
					 PHY_CFG_TX_RISE_TUNE_MASK,
					 FIELD_PREP(PHY_CFG_TX_RISE_TUNE_MASK, 0x2));

	/* source impedance adjustment */
	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_9,
					 PHY_CFG_TX_RES_TUNE_MASK,
					 FIELD_PREP(PHY_CFG_TX_RES_TUNE_MASK, 0x1));

	/* dc voltage level adjustement */
	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_8,
					 PHY_CFG_TX_HS_VREF_TUNE_MASK,
					 FIELD_PREP(PHY_CFG_TX_HS_VREF_TUNE_MASK, 0x3));

	/* transmitter HS crossover adjustement */
	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_8,
					 PHY_CFG_TX_HS_XV_TUNE_MASK,
					 FIELD_PREP(PHY_CFG_TX_HS_XV_TUNE_MASK, 0x0));
}

static int qcom_eusb2_ref_clk_init(struct qcom_snps_eusb2_phy_priv *qcom_snps_eusb2)
{
	unsigned long ref_clk_freq = clk_get_rate(qcom_snps_eusb2->ref_clk);

	/*
	 * The RPMh clock controller is stubbed on a number of Qualcomm platforms
	 * (see drivers/clk/clk-stub.c) and therefore reports a rate of 0, since
	 * nothing ever programs it.  The XO feeding this PHY is 19.2 MHz there,
	 * so fall back to that instead of bailing out with -EINVAL.
	 */
	if (!ref_clk_freq)
		ref_clk_freq = 19200000;

	switch (ref_clk_freq) {
	case 19200000:
		qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_HS_PHY_CTRL_COMMON0,
						 FSEL_MASK,
						 FIELD_PREP(FSEL_MASK, FSEL_19_2_MHZ_VAL));

		qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_2,
						 PHY_CFG_PLL_FB_DIV_7_0_MASK,
						 DIV_7_0_19_2_MHZ_VAL);

		qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_3,
						 PHY_CFG_PLL_FB_DIV_11_8_MASK,
						 DIV_11_8_19_2_MHZ_VAL);
		break;

	case 38400000:
		qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_HS_PHY_CTRL_COMMON0,
						 FSEL_MASK,
						 FIELD_PREP(FSEL_MASK, FSEL_38_4_MHZ_VAL));

		qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_2,
						 PHY_CFG_PLL_FB_DIV_7_0_MASK,
						 DIV_7_0_38_4_MHZ_VAL);

		qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_3,
						 PHY_CFG_PLL_FB_DIV_11_8_MASK,
						 DIV_11_8_38_4_MHZ_VAL);
		break;

	default:
		printf("%s: unsupported ref_clk_freq:%lu\n", __func__, ref_clk_freq);
		return -EINVAL;
	}

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_3,
					 PHY_CFG_PLL_REF_DIV, PLL_REF_DIV_VAL);

	return 0;
}

static int qcom_snps_eusb2_usb_init(struct phy *phy)
{
	struct qcom_snps_eusb2_phy_priv *qcom_snps_eusb2 = dev_get_priv(phy->dev);
	int ret;

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG0,
					 CMN_CTRL_OVERRIDE_EN, CMN_CTRL_OVERRIDE_EN);

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_UTMI_CTRL5, POR, POR);

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_HS_PHY_CTRL_COMMON0,
					 PHY_ENABLE | RETENABLEN, PHY_ENABLE | RETENABLEN);

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_APB_ACCESS_CMD,
					 APB_LOGIC_RESET, APB_LOGIC_RESET);

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, UTMI_PHY_CMN_CTRL0, TESTBURNIN, 0);

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_FSEL_SEL,
					 FSEL_SEL, FSEL_SEL);

	/* update ref_clk related registers */
	ret = qcom_eusb2_ref_clk_init(qcom_snps_eusb2);
	if (ret)
		return ret;

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_1,
					 PHY_CFG_PLL_CPBIAS_CNTRL_MASK,
					 FIELD_PREP(PHY_CFG_PLL_CPBIAS_CNTRL_MASK, 0x1));

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_4,
					 PHY_CFG_PLL_INT_CNTRL_MASK,
					 FIELD_PREP(PHY_CFG_PLL_INT_CNTRL_MASK, 0x8));

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_4,
					 PHY_CFG_PLL_GMP_CNTRL_MASK,
					 FIELD_PREP(PHY_CFG_PLL_GMP_CNTRL_MASK, 0x1));

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_5,
					 PHY_CFG_PLL_PROP_CNTRL_MASK,
					 FIELD_PREP(PHY_CFG_PLL_PROP_CNTRL_MASK, 0x10));

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_6,
					 PHY_CFG_PLL_VCO_CNTRL_MASK,
					 FIELD_PREP(PHY_CFG_PLL_VCO_CNTRL_MASK, 0x0));

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_CFG_CTRL_5,
					 PHY_CFG_PLL_VREF_TUNE_MASK,
					 FIELD_PREP(PHY_CFG_PLL_VREF_TUNE_MASK, 0x1));

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_HS_PHY_CTRL2,
					 VBUS_DET_EXT_SEL, VBUS_DET_EXT_SEL);

	/* set default parameters */
	qcom_eusb2_default_parameters(qcom_snps_eusb2);

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_HS_PHY_CTRL2,
					 USB2_SUSPEND_N_SEL | USB2_SUSPEND_N,
					 USB2_SUSPEND_N_SEL | USB2_SUSPEND_N);

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_UTMI_CTRL0, SLEEPM, SLEEPM);

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_HS_PHY_CTRL_COMMON0,
					 SIDDQ_SEL, SIDDQ_SEL);

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_HS_PHY_CTRL_COMMON0,
					 SIDDQ, 0);

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_UTMI_CTRL5, POR, 0);

	qcom_snps_eusb2_hsphy_write_mask(qcom_snps_eusb2->base, USB_PHY_HS_PHY_CTRL2,
					 USB2_SUSPEND_N_SEL, 0);

	return 0;
}

/*
 * Many boards route the eUSB2 bus through an external eUSB2-to-USB2 repeater
 * (for example the NXP PTN3222), referenced from the "phys" property of this
 * PHY node.  It has to be powered and taken out of reset before the SoC side
 * PHY can drive the bus.
 *
 * The optional tuning tables live in "qcom,param-override-seq" as a list of
 * (value, register) pairs, matching the downstream binding.  Tuning is best
 * effort - the repeater works with its default values, these only improve
 * signal quality.
 */
static void qcom_eusb2_repeater_tune(ofnode node)
{
	struct udevice *bus, *chip;
	u32 seq[16], addr;
	ofnode bus_node;
	int n, i, ret;

	if (ofnode_read_u32(node, "reg", &addr))
		return;

	n = ofnode_read_size(node, "qcom,param-override-seq") / sizeof(u32);
	if (n <= 0 || n > ARRAY_SIZE(seq) || (n % 2))
		return;

	if (ofnode_read_u32_array(node, "qcom,param-override-seq", seq, n))
		return;

	bus_node = ofnode_get_parent(node);
	ret = uclass_get_device_by_ofnode(UCLASS_I2C, bus_node, &bus);
	if (ret) {
		log_debug("%s: no i2c bus for repeater (%d)\n", __func__, ret);
		return;
	}

	/* This also sanity-checks that the repeater answers on the bus */
	ret = dm_i2c_probe(bus, addr, 0, &chip);
	if (ret) {
		log_debug("%s: repeater not answering at 0x%02x (%d)\n",
			  __func__, addr, ret);
		return;
	}

	for (i = 0; i + 1 < n; i += 2) {
		ret = dm_i2c_reg_write(chip, seq[i + 1], seq[i]);
		if (ret) {
			log_debug("%s: tune write reg 0x%02x failed (%d)\n",
				  __func__, seq[i + 1], ret);
			return;
		}
	}

	log_debug("%s: applied %d tuning writes\n", __func__, n / 2);
}

static void qcom_eusb2_repeater_enable(struct udevice *dev)
{
	struct ofnode_phandle_args args, rargs;
	struct udevice *reg;
	struct gpio_desc reset;
	int ret;

	ret = dev_read_phandle_with_args(dev, "phys", NULL, 0, 0, &args);
	if (ret) {
		log_debug("%s: no repeater phandle (%d)\n", __func__, ret);
		return;
	}

	/*
	 * vdd3 is an RPMh LDO which U-Boot does control.  vdd18 is a PMIC SMPS
	 * that the U-Boot RPMh regulator driver deliberately does not model
	 * (its SMPS entries are compiled out); it is already enabled by the
	 * previous bootloader, so treat it as best effort here.
	 */
	ret = ofnode_parse_phandle_with_args(args.node, "vdd3-supply",
					     NULL, 0, 0, &rargs);
	if (!ret) {
		ret = uclass_get_device_by_ofnode(UCLASS_REGULATOR, rargs.node,
						  &reg);
		if (!ret) {
			regulator_set_enable(reg, true);
			log_debug("%s: vdd3 enabled\n", __func__);
		} else {
			log_err("%s: vdd3 unavailable (%d)\n", __func__, ret);
		}
	}

	ret = ofnode_parse_phandle_with_args(args.node, "vdd18-supply",
					     NULL, 0, 0, &rargs);
	if (!ret) {
		ret = uclass_get_device_by_ofnode(UCLASS_REGULATOR, rargs.node,
						  &reg);
		if (!ret) {
			regulator_set_enable(reg, true);
			log_debug("%s: vdd18 enabled\n", __func__);
		} else {
			log_debug("%s: vdd18 not modelled (%d), assuming on\n",
				  __func__, ret);
		}
	}

	/* Release the repeater reset (DT describes it as active low) */
	ret = gpio_request_by_name_nodev(args.node, "reset-gpios", 0, &reset,
					 GPIOD_IS_OUT);
	if (ret) {
		log_err("%s: no repeater reset gpio (%d)\n", __func__, ret);
		return;
	}

	dm_gpio_set_value(&reset, 0);
	mdelay(1);

	/* Reset has been released, now program the board tuning tables */
	qcom_eusb2_repeater_tune(args.node);

	log_debug("%s: eUSB2 repeater enabled\n", __func__);
}

static int qcom_snps_eusb2_phy_power_on(struct phy *phy)
{
	struct qcom_snps_eusb2_phy_priv *qcom_snps_eusb2 = dev_get_priv(phy->dev);
	int ret;

	/* Bring up the external eUSB2 repeater first, when the board has one */
	qcom_eusb2_repeater_enable(phy->dev);

	clk_prepare_enable(qcom_snps_eusb2->ref_clk);

	ret = reset_deassert_bulk(&qcom_snps_eusb2->resets);
	if (ret)
		return ret;

	ret = qcom_snps_eusb2_usb_init(phy);
	if (ret)
		return ret;

	return 0;
}

static int qcom_snps_eusb2_phy_power_off(struct phy *phy)
{
	struct qcom_snps_eusb2_phy_priv *qcom_snps_eusb2 = dev_get_priv(phy->dev);

	reset_assert_bulk(&qcom_snps_eusb2->resets);
	clk_disable_unprepare(qcom_snps_eusb2->ref_clk);

	return 0;
}

static int qcom_snps_eusb2_phy_probe(struct udevice *dev)
{
	struct qcom_snps_eusb2_phy_priv *qcom_snps_eusb2 = dev_get_priv(dev);
	int ret;

	qcom_snps_eusb2->base = (void __iomem *)dev_read_addr(dev);
	if (IS_ERR(qcom_snps_eusb2->base))
		return PTR_ERR(qcom_snps_eusb2->base);

	qcom_snps_eusb2->ref_clk = devm_clk_get(dev, "ref");
	if (IS_ERR(qcom_snps_eusb2->ref_clk)) {
		ret = PTR_ERR(qcom_snps_eusb2->ref_clk);
		printf("%s: failed to get ref clk %d\n", __func__, ret);
		return ret;
	}

	ret = reset_get_bulk(dev, &qcom_snps_eusb2->resets);
	if (ret < 0) {
		printf("failed to get resets, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static struct phy_ops qcom_snps_eusb2_phy_ops = {
	.power_on = qcom_snps_eusb2_phy_power_on,
	.power_off = qcom_snps_eusb2_phy_power_off,
};

static const struct udevice_id qcom_snps_eusb2_phy_ids[] = {
	{
		.compatible = "qcom,sm8550-snps-eusb2-phy",
	},
	{}
};

U_BOOT_DRIVER(qcom_usb_qcom_snps_eusb2) = {
	.name = "qcom-snps-eusb2-hsphy",
	.id = UCLASS_PHY,
	.of_match = qcom_snps_eusb2_phy_ids,
	.ops = &qcom_snps_eusb2_phy_ops,
	.probe = qcom_snps_eusb2_phy_probe,
	.priv_auto = sizeof(struct qcom_snps_eusb2_phy_priv),
};
