
#ifndef __TAS2562_
#define __TAS2562_

#define TAS2562_DRIVER_ID           "1.0.1"

/* Page Control Register */
#define TAS2562_PAGECTL_REG			0

/* Book Control Register (available in page0 of each book) */
#define TAS2562_BOOKCTL_PAGE		0
#define TAS2562_BOOKCTL_REG			127

#define TAS2562_REG(book, page, reg)	(((book * 256 * 128) + \
											(page * 128)) + reg)


#define TAS2562_BOOK_ID(reg)			(reg / (256 * 128))

#define TAS2562_PAGE_ID(reg)			((reg % (256 * 128)) / 128)

#define TAS2562_BOOK_REG(reg)			(reg % (256 * 128))

#define TAS2562_PAGE_REG(reg)			((reg % (256 * 128)) % 128)


    /* Software Reset */
#define TAS2562_SoftwareReset  TAS2562_REG(0x0, 0x0, 0x01)
#define TAS2562_SoftwareReset_SoftwareReset_Mask  (0x1 << 0),
#define TAS2562_SoftwareReset_SoftwareReset_DontReset  (0x0 << 0)
#define TAS2562_SoftwareReset_SoftwareReset_Reset  (0x1 << 0)

    /* Power Control */
#define TAS2562_PowerControl  TAS2562_REG(0x0, 0x0, 0x02)
#define TAS2562_PowerControl_ISNSPower_Mask  (0x1 << 3)
#define TAS2562_PowerControl_ISNSPower_Active  (0x0 << 3)
#define TAS2562_PowerControl_ISNSPower_PoweredDown  (0x1 << 3)
#define TAS2562_PowerControl_VSNSPower_Mask  (0x1 << 2)
#define TAS2562_PowerControl_VSNSPower_Active  (0x0 << 2)
#define TAS2562_PowerControl_VSNSPower_PoweredDown  (0x1 << 2)
#define TAS2562_PowerControl_OperationalMode10_Mask  (0x3 << 0)
#define TAS2562_PowerControl_OperationalMode10_Active  (0x0 << 0)
#define TAS2562_PowerControl_OperationalMode10_Mute  (0x1 << 0)
#define TAS2562_PowerControl_OperationalMode10_Shutdown  (0x2 << 0)

    /* Playback Configuration Reg0 */
#define TAS2562_PlaybackConfigurationReg0  TAS2562_REG(0x0, 0x0, 0x03)

    /* Misc Configuration Reg0 */
#define TAS2562_MiscConfigurationReg0  TAS2562_REG(0x0, 0x0, 0x04)
#define TAS2562_MiscConfigurationReg0_IRQZPull_Mask  (0x1 << 3)
#define TAS2562_MiscConfigurationReg0_IRQZPull_Disabled  (0x0 << 3)
#define TAS2562_MiscConfigurationReg0_IRQZPull_Enabled  (0x1 << 3)

    /* TDM Configuration Reg0 */
#define TAS2562_TDMConfigurationReg0  TAS2562_REG(0x0, 0x0, 0x06)
#define TAS2562_TDMConfigurationReg0_SAMPRATERAMP_Mask  (0x1 << 5)
#define TAS2562_TDMConfigurationReg0_SAMPRATERAMP_48KHz  (0x0 << 5)
#define TAS2562_TDMConfigurationReg0_SAMPRATERAMP_44_1KHz  (0x1 << 5)
#define TAS2562_TDMConfigurationReg0_SAMPRATE31_Mask  (0x7 << 1)
#define TAS2562_TDMConfigurationReg0_SAMPRATE31_14_7_16kHz  (0x1 << 1)
#define TAS2562_TDMConfigurationReg0_SAMPRATE31_44_1_48kHz  (0x4 << 1)
#define TAS2562_TDMConfigurationReg0_SAMPRATE31_88_2_96kHz  (0x5 << 1)
#define TAS2562_TDMConfigurationReg0_SAMPRATE31_176_4_192kHz  (0x6 << 1)

    /* TDM Configuration Reg1 */
#define TAS2562_TDMConfigurationReg1  TAS2562_REG(0x0, 0x0, 0x07)
#define TAS2562_TDMConfigurationReg1_RXOFFSET51_Mask  (0x1f << 1)
#define TAS2562_TDMConfigurationReg1_RXOFFSET51_Shift (1)
#define TAS2562_TDMConfigurationReg1_RXEDGE_Mask  (0x1 << 0)
#define TAS2562_TDMConfigurationReg1_RXEDGE_Rising  (0x0 << 0)
#define TAS2562_TDMConfigurationReg1_RXEDGE_Falling  (0x1 << 0)

    /* TDM Configuration Reg2 */
#define TAS2562_TDMConfigurationReg2  TAS2562_REG(0x0, 0x0, 0x08)
#define TAS2562_TDMConfigurationReg2_RXWLEN32_Mask  (0x3 << 2)
#define TAS2562_TDMConfigurationReg2_RXWLEN32_16Bits  (0x0 << 2)
#define TAS2562_TDMConfigurationReg2_RXWLEN32_20Bits  (0x1 << 2)
#define TAS2562_TDMConfigurationReg2_RXWLEN32_24Bits  (0x2 << 2)
#define TAS2562_TDMConfigurationReg2_RXWLEN32_32Bits  (0x3 << 2)
#define TAS2562_TDMConfigurationReg2_RXSLEN10_Mask  (0x3 << 0)
#define TAS2562_TDMConfigurationReg2_RXSLEN10_16Bits  (0x0 << 0)
#define TAS2562_TDMConfigurationReg2_RXSLEN10_24Bits  (0x1 << 0)
#define TAS2562_TDMConfigurationReg2_RXSLEN10_32Bits  (0x2 << 0)

    /* TDM Configuration Reg4 */
#define TAS2562_TDMConfigurationReg4  TAS2562_REG(0x0, 0x0, 0x0A)

    /* TDM Configuration Reg5 */
#define TAS2562_TDMConfigurationReg5  TAS2562_REG(0x0, 0x0, 0x0B)

    /* TDM Configuration Reg6 */
#define TAS2562_TDMConfigurationReg6  TAS2562_REG(0x0, 0x0, 0x0C)

    /* Limiter Configuration Reg0 */
#define TAS2562_LimiterConfigurationReg0  TAS2562_REG(0x0, 0x0, 0x12)

    /* Interrupt Mask Reg0 */
#define TAS2562_InterruptMaskReg0  TAS2562_REG(0x0, 0x0, 0x1A)
#define TAS2562_InterruptMaskReg0_Disable 0xff

    /* Interrupt Mask Reg1 */
#define TAS2562_InterruptMaskReg1  TAS2562_REG(0x0, 0x0, 0x1B)
#define TAS2562_InterruptMaskReg1_Disable 0xff

    /* Latched-Interrupt Reg0 */
#define TAS2562_LatchedInterruptReg0  TAS2562_REG(0x0, 0x0, 0x24)
#define TAS2562_LatchedInterruptReg0_TDMClockErrorSticky_Mask  (0x1 << 2)
#define TAS2562_LatchedInterruptReg0_TDMClockErrorSticky_NoInterrupt  (0x0 << 2)
#define TAS2562_LatchedInterruptReg0_TDMClockErrorSticky_Interrupt  (0x1 << 2)
#define TAS2562_LatchedInterruptReg0_OCEFlagSticky_Mask  (0x1 << 1)
#define TAS2562_LatchedInterruptReg0_OCEFlagSticky_NoInterrupt  (0x0 << 1)
#define TAS2562_LatchedInterruptReg0_OCEFlagSticky_Interrupt  (0x1 << 1)
#define TAS2562_LatchedInterruptReg0_OTEFlagSticky_Mask  (0x1 << 0)
#define TAS2562_LatchedInterruptReg0_OTEFlagSticky_NoInterrupt  (0x0 << 0)
#define TAS2562_LatchedInterruptReg0_OTEFlagSticky_Interrupt  (0x1 << 0)

    /* Latched-Interrupt Reg1 */
#define TAS2562_LatchedInterruptReg1  TAS2562_REG(0x0, 0x0, 0x25)
#define TAS2562_LatchedInterruptReg1_VBATOVLOSticky_Mask  (0x1 << 3)
#define TAS2562_LatchedInterruptReg1_VBATOVLOSticky_NoInterrupt  (0x0 << 3)
#define TAS2562_LatchedInterruptReg1_VBATOVLOSticky_Interrupt  (0x1 << 3)
#define TAS2562_LatchedInterruptReg1_VBATUVLOSticky_Mask  (0x1 << 2)
#define TAS2562_LatchedInterruptReg1_VBATUVLOSticky_NoInterrupt  (0x0 << 2)
#define TAS2562_LatchedInterruptReg1_VBATUVLOSticky_Interrupt  (0x1 << 2)
#define TAS2562_LatchedInterruptReg1_BrownOutFlagSticky_Mask  (0x1 << 1)
#define TAS2562_LatchedInterruptReg1_BrownOutFlagSticky_NoInterrupt  (0x0 << 1)
#define TAS2562_LatchedInterruptReg1_BrownOutFlagSticky_Interrupt  (0x1 << 1)

#define TAS2562_LatchedInterruptReg2  TAS2562_REG(0x0, 0x0, 0x28)
#define TAS2562_LatchedInterruptReg3  TAS2562_REG(0x0, 0x0, 0x26)
#define TAS2562_LatchedInterruptReg4  TAS2562_REG(0x0, 0x0, 0x27)

    /* VBAT MSB */
#define TAS2562_VBATMSB  TAS2562_REG(0x0, 0x0, 0x2A)
#define TAS2562_VBATMSB_VBATMSB70_Mask  (0xff << 0)

    /* VBAT LSB */
#define TAS2562_VBATLSB  TAS2562_REG(0x0, 0x0, 0x2B)
#define TAS2562_VBATLSB_VBATLSB74_Mask  (0xf << 4)

    /* TEMP */
#define TAS2562_TEMP  TAS2562_REG(0x0, 0x0, 0x2C)
#define TAS2562_TEMP_TEMPMSB70_Mask  (0xff << 0)


    /* Interrupt Configuration */
#define TAS2562_InterruptConfiguration  TAS2562_REG(0x0, 0x0, 0x30)
#define TAS2562_InterruptConfiguration_LTCHINTClear_Mask (0x1 << 2)
#define TAS2562_InterruptConfiguration_LTCHINTClear (0x1 << 2)

#define TAS2562_BoostSlope TAS2562_REG(0x0, 0x0, 0x35)

    /* Clock Configuration */
#define TAS2562_ClockConfiguration  TAS2562_REG(0x0, 0x0, 0x38)

#define TAS2562_VBatFilter TAS2562_REG(0x0, 0x0, 0x3b)
#define TAS2562_ClassHReleaseTimer TAS2562_REG(0x0, 0x0, 0x3c)

#define TAS2562_ICN_REG TAS2562_REG(0x0, 0x2, 0x64)

#define TAS2562_TestPageConfiguration TAS2562_REG(0x0, 0xfd, 0xd)
#define TAS2562_ClassDConfiguration1	TAS2562_REG(0x0, 0xfd, 0x19)
#define TAS2562_ClassDConfiguration2	TAS2562_REG(0x0, 0xfd, 0x32)
#define TAS2562_ClassDConfiguration3	TAS2562_REG(0x0, 0xfd, 0x33)
#define TAS2562_ClassDConfiguration4	TAS2562_REG(0x0, 0xfd, 0x3f)
#define TAS2562_EfficiencyConfiguration	TAS2562_REG(0x0, 0xfd, 0x5f)

#define TAS2562_ClassHHeadroom TAS2562_REG(0x64, 0x7, 0x48)
#define TAS2562_ClassHHysteresis TAS2562_REG(0x64, 0x7, 0x4c)
#define TAS2562_ClassHMtct TAS2562_REG(0x64, 0x5, 0x4c)

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
#define ERROR_FAILSAFE      0x4000000

#define CHECK_PERIOD	5000	/* 5 second */

#define TAS2562_I2C_RETRY_COUNT     8
#define ERROR_I2C_SUSPEND           -1
#define ERROR_I2C_FAILED            -2

struct tas2562_priv {
	struct device *dev;
	struct regmap *regmap;
	struct mutex dev_lock;
	struct delayed_work irq_work;
	struct delayed_work init_work;
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
int tas2562_bulk_read(struct tas2562_priv *tas_priv, unsigned int reg,
		      unsigned char *buf, unsigned int len);
int tas2562_update_bits(struct tas2562_priv *tas_priv, unsigned int reg,
			unsigned int mask, unsigned int value);

void tas2562_hw_reset(struct tas2562_priv *tas_priv);
void tas2562_enable_irq(struct tas2562_priv *tas_priv, bool enable);

int tas2562_runtime_suspend(struct tas2562_priv *tas_priv);
int tas2562_runtime_resume(struct tas2562_priv *tas_priv);

void tas2562_smartamp_add_controls(struct tas2562_priv *tas_priv);

#endif /* __TAS2562_ */
