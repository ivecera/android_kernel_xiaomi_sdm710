
#ifndef __TAS2562_
#define __TAS2562_

#define TAS2562_DRIVER_ID           "1.0.1"

#define TAS2562_REG(book, page, reg)	(((book) << 15) | ((page) << 7) | reg)

/* Page Control Register */
#define TAS2562_PAGE_CTRL		TAS2562_REG(0x00, 0x00, 0x00)

/* Book Control Register (available in page0 of each book) */
#define TAS2562_BOOK_CTRL		TAS2562_REG(0x00, 0x00, 0x7f)

#define TAS2562_BOOK_ID(reg)			(reg / (256 * 128))
#define TAS2562_PAGE_ID(reg)			((reg % (256 * 128)) / 128)
#define TAS2562_BOOK_REG(reg)			(reg % (256 * 128))
#define TAS2562_PAGE_REG(reg)			((reg % (256 * 128)) % 128)

/* Software Reset */
#define TAS2562_SW_RESET	TAS2562_REG(0x0, 0x0, 0x01)
#define TAS2562_RESET		BIT(0)

/* Power Control */
#define TAS2562_PWR_CTL		TAS2562_REG(0x0, 0x0, 0x02)
#define TAS2562_ISNS_PD		BIT(3)
#define TAS2562_VSNS_PD		BIT(2)
#define TAS2562_MODE_MASK	GENMASK(1, 0)
#define TAS2562_MODE_ACTIVE	0x00
#define TAS2562_MODE_MUTE	0x01
#define TAS2562_MODE_SHUTDOWN	0x02

/* Playback Configuration 1 */
#define TAS2562_PB_CFG1		TAS2562_REG(0x0, 0x0, 0x03)

/* Misc Configuration 1 */
#define TAS2562_MISC_CFG1	TAS2562_REG(0x0, 0x0, 0x04)
#define TAS2562_IRQZ_PU		BIT(3)

/* TDM Configuration 0 */
#define TAS2562_TDM_CFG0	TAS2562_REG(0x0, 0x0, 0x06)
#define TAS2562_SAMP_RATE_MASK	(BIT(5) | GENMASK(3, 1))
#define TAS2562_SAMP_RATE_48	(0x4 << 1)
#define TAS2562_SAMP_RATE_96	(0x5 << 1)
#define TAS2562_SAMP_RATE_192	(0x6 << 1)
#define TAS2562_SAMP_RATE_44_1	(BIT(5) | (0x4 << 1))
#define TAS2562_SAMP_RATE_88_2	(BIT(5) | (0x5 << 1))
#define TAS2562_SAMP_RATE_176_4	(BIT(5) | (0x6 << 1))

/* TDM Configuration 1 */
#define TAS2562_TDM_CFG1	TAS2562_REG(0x0, 0x0, 0x07)
#define TAS2562_RX_OFFSET_MASK	GENMASK(5, 1)
#define TAS2562_RX_OFFSET(X)	((X) << 1)
#define TAS2562_RX_EDGE_MASK	BIT(0)
#define TAS2562_RX_EDGE_RISING	0
#define TAS2562_RX_EDGE_FALLING	1

/* TDM Configuration 2 */
#define TAS2562_TDM_CFG2	TAS2562_REG(0x0, 0x0, 0x08)
#define TAS2562_RX_WLEN_MASK	GENMASK(3, 2)
#define TAS2562_RX_WLEN_16	(0x0 << 2)
#define TAS2562_RX_WLEN_20	(0x1 << 2)
#define TAS2562_RX_WLEN_24	(0x2 << 2)
#define TAS2562_RX_WLEN_32	(0x3 << 2)
#define TAS2562_RX_SLEN_MASK	GENMASK(1, 0)
#define TAS2562_RX_SLEN_16	(0x0 << 0)
#define TAS2562_RX_SLEN_24	(0x1 << 0)
#define TAS2562_RX_SLEN_32	(0x2 << 0)

/* TDM Configuration 4 */
#define TAS2562_TDM_CFG4	TAS2562_REG(0x0, 0x0, 0x0A)
#define TAS2562_TX_FILL_MASK	BIT(4)
#define TAS2562_TX_OFFSET_MASK	GENMASK(3, 1)
#define TAS2562_TX_OFFSET(X)	((X) << 1)
#define TAS2562_TX_EDGE_MASK	BIT(0)
#define TAS2562_TX_EDGE_FALLING	1

/* TDM Configuration 5 */
#define TAS2562_TDM_CFG5	TAS2562_REG(0x0, 0x0, 0x0B)
#define TAS2562_VSNS_TX_EN	BIT(6)
#define TAS2562_VSNS_SLOT_MASK	GENMASK(5, 0)
#define TAS2562_VSNS_SLOT(X)	(X)

/* TDM Configuration 6 */
#define TAS2562_TDM_CFG6	TAS2562_REG(0x0, 0x0, 0x0C)
#define TAS2562_ISNS_TX_EN	BIT(6)
#define TAS2562_ISNS_SLOT_MASK	GENMASK(5, 0)
#define TAS2562_ISNS_SLOT(X)	(X)

/* Limiter Configuration 0 */
#define TAS2562_LIM_CFG0	TAS2562_REG(0x0, 0x0, 0x12)

/* Interrupt Mask 0 */
#define TAS2562_INT_MASK0	TAS2562_REG(0x0, 0x0, 0x1A)
#define TAS2562_INT_TCE		BIT(2) /* TDM clock error */
#define TAS2562_INT_OCE		BIT(1) /* Over current error */
#define TAS2562_INT_OTE		BIT(0) /* Over temp error */
#define TAS2562_INT_ALL		0xFF

/* Interrupt Mask 1 */
#define TAS2562_INT_MASK1	TAS2562_REG(0x0, 0x0, 0x1B)
#define TAS2562_INT_SPK_SL	BIT(4) /* Speaker short load detection */
#define TAS2562_INT_BDPD	BIT(2) /* Brownout device power down start */
#define TAS2562_INT_BPA		BIT(1) /* Brownout protection active */
#define TAS2562_INT_VBAT_BD	BIT(0) /* VBAT brownout detected */

/* Latched Interrupt Readback 0 */
#define TAS2562_INT_LTCH0	TAS2562_REG(0x0, 0x0, 0x24)
#define TAS2562_INTS_TCE	BIT(2) /* TDM clock error */
#define TAS2562_INTS_OCE	BIT(1) /* Over current error */
#define TAS2562_INTS_OTE	BIT(0) /* Over temp error */

/* Latched-Interrupt Readback 1 */
#define TAS2562_INT_LTCH1	TAS2562_REG(0x0, 0x0, 0x25)
#define TAS2562_INTS_SPK_OVOL	BIT(3) /* Speaker over voltage */
#define TAS2562_INTS_SPK_UVOL	BIT(2) /* Speaker under voltage */
#define TAS2562_INTS_BOF	BIT(1) /* Brown-out flag */

/* Latched-Interrupt Readback 2 - 4 */
#define TAS2562_INT_LTCH2	TAS2562_REG(0x0, 0x0, 0x28)
#define TAS2562_INT_LTCH3	TAS2562_REG(0x0, 0x0, 0x26)
#define TAS2562_INT_LTCH4	TAS2562_REG(0x0, 0x0, 0x27)

/* VBAT MSB */
#define TAS2562_VBATMSB		TAS2562_REG(0x0, 0x0, 0x2A)
#define TAS2562_VBAT_CNVH_MASK	GENMASK(7, 0)

/* VBAT LSB */
#define TAS2562_VBATLSB		TAS2562_REG(0x0, 0x0, 0x2B)
#define TAS2562_VBAT_CNVL_MASK	GENMASK(7, 6)

#define TAS2562_VBAT_CNV(H, L)	((((H) & TAS2562_VBAT_CNVH_MASK) << 2) | \
				 (((L) & TAS2562_VBAT_CNVL_MASK) >> 6))

/* TEMP */
#define TAS2562_TEMP		TAS2562_REG(0x0, 0x0, 0x2C)
#define TAS2562_TEMP_CNV_MASK	GENMASK(7, 0)


/* Interrupt Configuration */
#define TAS2562_INT_CLK		TAS2562_REG(0x0, 0x0, 0x30)
#define TAS2562_INT_CLR_LTCH	BIT(2)

/* Boost Configure 3 */
#define TAS2562_BOOST_CFG3	TAS2562_REG(0x0, 0x0, 0x35)

/* Clock Configuration */
#define TAS2562_CLK_CFG		TAS2562_REG(0x0, 0x0, 0x38)

#define TAS2562_VBAT_FLT	TAS2562_REG(0x0, 0x0, 0x3b)
#define TAS2562_CLS_H_RTMR	TAS2562_REG(0x0, 0x0, 0x3c)

#define TAS2562_IDC_CFG0	TAS2562_REG(0x0, 0x2, 0x64)
#define TAS2562_IDC_CFG1	TAS2562_REG(0x0, 0x2, 0x65)
#define TAS2562_IDC_CFG2	TAS2562_REG(0x0, 0x2, 0x66)
#define TAS2562_IDC_CFG3	TAS2562_REG(0x0, 0x2, 0x67)

#define TAS2562_TEST_PG_CFG	TAS2562_REG(0x0, 0xfd, 0xd)
#define TAS2562_CLS_D_CFG1	TAS2562_REG(0x0, 0xfd, 0x19)
#define TAS2562_CLS_D_CFG2	TAS2562_REG(0x0, 0xfd, 0x32)
#define TAS2562_CLS_D_CFG3	TAS2562_REG(0x0, 0xfd, 0x33)
#define TAS2562_CLS_D_CFG4	TAS2562_REG(0x0, 0xfd, 0x3f)
#define TAS2562_EFFC_CFG	TAS2562_REG(0x0, 0xfd, 0x5f)

#define TAS2562_DEF_BOOK_ID	0x00
#define TAS2562_CLS_H_BOOK_ID	0x64

#define TAS2562_CLS_H_HDRM_CFG0	TAS2562_REG(0x00, 0x7, 0x48)
#define TAS2562_CLS_H_HDRM_CFG1	TAS2562_REG(0x00, 0x7, 0x49)
#define TAS2562_CLS_H_HDRM_CFG2	TAS2562_REG(0x00, 0x7, 0x4a)
#define TAS2562_CLS_H_HDRM_CFG3	TAS2562_REG(0x00, 0x7, 0x4b)
#define TAS2562_CLS_H_HYST_CFG0	TAS2562_REG(0x00, 0x7, 0x4c)
#define TAS2562_CLS_H_HYST_CFG1	TAS2562_REG(0x00, 0x7, 0x4d)
#define TAS2562_CLS_H_HYST_CFG2	TAS2562_REG(0x00, 0x7, 0x4e)
#define TAS2562_CLS_H_HYST_CFG3	TAS2562_REG(0x00, 0x7, 0x4f)
#define TAS2562_CLS_H_MTCT_CFG0	TAS2562_REG(0x00, 0x5, 0x4c)
#define TAS2562_CLS_H_MTCT_CFG1	TAS2562_REG(0x00, 0x5, 0x4d)
#define TAS2562_CLS_H_MTCT_CFG2	TAS2562_REG(0x00, 0x5, 0x4e)
#define TAS2562_CLS_H_MTCT_CFG3	TAS2562_REG(0x00, 0x5, 0x4f)

#define TAS2562_POWER_ACTIVE 0
#define TAS2562_POWER_MUTE 1
#define TAS2562_POWER_SHUTDOWN 2

#define ERROR_NONE		0x0000000
#define ERROR_PLL_ABSENT	0x0000000
#define ERROR_PRAM_CRCCHK	0x0000000
#define ERROR_DTMCLK_ERROR	0x0000001
#define ERROR_OVER_CURRENT	0x0000002
#define ERROR_DIE_OVERTEMP	0x0000004
#define ERROR_OVER_VOLTAGE	0x0000008
#define ERROR_UNDER_VOLTAGE	0x0000010
#define ERROR_BROWNOUT		0x0000020
#define ERROR_CLASSD_PWR	0x0000040
#define ERROR_DEVA_I2C_COMM	0x0000080
#define ERROR_FAILSAFE		0x4000000

#define CHECK_PERIOD	5000	/* 5 second */

#define TAS2562_I2C_RETRY_COUNT     8
#define ERROR_I2C_SUSPEND           -1
#define ERROR_I2C_FAILED            -2

struct tas2562_priv {
	struct device *dev;
	struct regmap *regmap;
	struct mutex dev_lock;
	struct delayed_work irq_work;
	struct hrtimer mtimer;
	struct snd_soc_codec *codec;
	bool power_up;
	int power_state;
	int cur_book;
	int cur_page;
	int asi_format;
	int reset_gpio;
	int irq_gpio;
	int irq;
	bool irq_enabled;
	int sample_rate;
	int channel_width;
	int slot_width;
	int pcm_format;
	bool muted;
	bool i2c_suspend;
	/* device is working, but system is suspended */
	bool runtime_suspended;

	unsigned int err_code;
	struct mutex codec_lock;
};

int tas2562_read(struct tas2562_priv *tas_priv, unsigned int reg,
		 unsigned int *pvalue);
int tas2562_write(struct tas2562_priv *tas_priv, unsigned int reg,
		  unsigned int value);
int tas2562_bulk_write(struct tas2562_priv *tas_priv, unsigned int reg,
		       const unsigned char *buf, unsigned int len);
int tas2562_update_bits(struct tas2562_priv *tas_priv, unsigned int reg,
			unsigned int mask, unsigned int value);
int tas2562_change_book(struct tas2562_priv *tas_priv, u8 book);

void tas2562_hw_reset(struct tas2562_priv *tas_priv);
void tas2562_enable_irq(struct tas2562_priv *tas_priv, bool enable);

int tas2562_runtime_suspend(struct tas2562_priv *tas_priv);
int tas2562_runtime_resume(struct tas2562_priv *tas_priv);

void tas2562_smartamp_add_controls(struct tas2562_priv *tas_priv);

#endif /* __TAS2562_ */
