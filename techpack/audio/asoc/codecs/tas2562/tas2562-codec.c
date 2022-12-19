/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE.See the GNU General Public License for more details.
**
** File:
**     tas2562-codec.c
**
** Description:
**     ALSA SoC driver for Texas Instruments TAS2562 High Performance 4W Smart
**     Amplifier
**
** =============================================================================
*/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "tas2562.h"

#define TAS2562_MDELAY 0xFFFFFFFE
#define TAS2562_MSLEEP 0xFFFFFFFD

static char const *iv_enable_text[] = {"Off", "On"};
static int tas2562iv_enable = 1;
static int muted;
static const struct soc_enum tas2562_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(iv_enable_text), iv_enable_text),
};
static int tas2562_set_fmt(struct tas2562_priv *tas_priv, unsigned int fmt);

static int tas2562_mute_ctrl_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
static int tas2562_mute_ctrl_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
static int tas2562_load_init(struct tas2562_priv *tas_priv);

static unsigned int tas2562_codec_read(struct snd_soc_codec *codec,
		unsigned int reg)
{
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);
	int result = 0;
	unsigned int value = 0;

	result = tas2562_read(tas_priv, reg, &value);

	if (result < 0)
		dev_err(tas_priv->dev, "%s, ERROR, reg=0x%x, E=%d\n",
			__func__, reg, result);
	else
		dev_dbg(tas_priv->dev, "%s, reg: 0x%x, value: 0x%x\n",
				__func__, reg, value);

	if (result >= 0)
		return value;
	else
		return result;
}

static int tas2562_iv_enable(struct tas2562_priv *tas_priv, int enable)
{
	int result;

	if (enable) {
		pr_debug("%s: tas2562iv_enable \n", __func__);
		result = tas2562_update_bits(tas_priv, TAS2562_PWR_CTL,
					     TAS2562_ISNS_PD | TAS2562_VSNS_PD,
					     0);
	} else {
		pr_debug("%s: tas2562iv_disable \n", __func__);
		result = tas2562_update_bits(tas_priv, TAS2562_PWR_CTL,
					     TAS2562_ISNS_PD | TAS2562_VSNS_PD,
					     TAS2562_ISNS_PD | TAS2562_VSNS_PD);
	}
	tas2562iv_enable = enable;

	return result;
}

static int tas2562iv_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);
	int iv_enable = 0, result = 0;

    if (codec == NULL) {
		pr_err("%s: codec is NULL \n",  __func__);
		return 0;
    }

    iv_enable = ucontrol->value.integer.value[0];

	result = tas2562_iv_enable(tas_priv, iv_enable);

	pr_debug("%s: tas2562iv_enable = %d\n", __func__, tas2562iv_enable);

	return result;
}

static int tas2562iv_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0] = tas2562iv_enable;
   return 0;
}

static const struct snd_kcontrol_new tas2562_controls[] = {
SOC_ENUM_EXT("TAS2562 IVSENSE ENABLE", tas2562_enum[0],
		    tas2562iv_get, tas2562iv_put),
};

static int tas2562_codec_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	int result = 0;

	result = tas2562_write(tas_priv, reg, value);
	if (result < 0) {
		dev_err(tas_priv->dev, "%s, ERROR, reg=0x%x, E=%d\n",
			__func__, reg, result);
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(2));
	}
	else
		dev_dbg(tas_priv->dev, "%s, reg: 0x%x, 0x%x\n",
			__func__, reg, value);

	return result;

}

#define TAS2562_INIT_REG(REG, ...)				\
do {								\
	const u8 values[] = { __VA_ARGS__ };			\
	dev_info(tas_priv->dev, "Init register %s\n", #REG);	\
	if (ARRAY_SIZE(values) == 1)				\
		ret = tas2562_write(tas_priv, REG, values[0]);	\
	else							\
		ret = tas2562_bulk_write(tas_priv, REG, values,	\
					 ARRAY_SIZE(values));	\
	if (ret < 0)						\
		goto err;					\
} while (0)

static int tas2562_load_init_cfg(struct tas2562_priv *tas_priv)
{
	int ret;

	TAS2562_INIT_REG(TAS2562_CLS_H_HDRM_CFG0, 0x09, 0x99, 0x99, 0x9A);
	TAS2562_INIT_REG(TAS2562_CLS_H_HYST_CFG0, 0x00, 0x00, 0x00, 0x00);
	TAS2562_INIT_REG(TAS2562_CLS_H_MTCT_CFG0, 0x0B, 0x00, 0x00, 0x00);
	TAS2562_INIT_REG(TAS2562_VBAT_FLT, 0x38);
	TAS2562_INIT_REG(TAS2562_CLS_H_RTMR, 0x3C);
	TAS2562_INIT_REG(TAS2562_BOOST_CFG3, 0x78);
	TAS2562_INIT_REG(TAS2562_TEST_PG_CFG, 0x0D);
	TAS2562_INIT_REG(TAS2562_CLS_D_CFG3, 0x8E);
	TAS2562_INIT_REG(TAS2562_CLS_D_CFG2, 0x49);
	TAS2562_INIT_REG(TAS2562_CLS_D_CFG4, 0x21);
	TAS2562_INIT_REG(TAS2562_CLS_D_CFG1, 0x80);
	TAS2562_INIT_REG(TAS2562_EFFC_CFG, 0xC1);
err:
	dev_info(tas_priv->dev, "Loading initial configuration %s\n",
		 ret < 0 ? "failed" : "succeeded");

	if (ret < 0)
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(2));

	return ret;
}

static int tas2562_codec_suspend(struct snd_soc_codec *codec)
{
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	mutex_lock(&tas_priv->codec_lock);

	dev_dbg(tas_priv->dev, "%s\n", __func__);
	tas2562_runtime_suspend(tas_priv);

	mutex_unlock(&tas_priv->codec_lock);
	return ret;
}

static int tas2562_codec_resume(struct snd_soc_codec *codec)
{
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	mutex_lock(&tas_priv->codec_lock);

	dev_dbg(tas_priv->dev, "%s\n", __func__);
	tas2562_runtime_resume(tas_priv);

	mutex_unlock(&tas_priv->codec_lock);
	return ret;
}

static const struct snd_kcontrol_new tas2562_asi_controls[] = {
	SOC_DAPM_SINGLE("Left", TAS2562_TDM_CFG2, 4, 1, 0),
	SOC_DAPM_SINGLE("Right", TAS2562_TDM_CFG2, 4, 2, 0),
	SOC_DAPM_SINGLE("LeftRightDiv2", TAS2562_TDM_CFG2, 4, 3, 0),
};

static int tas2562_set_power_state(struct tas2562_priv *tas_priv, int state)
{
	int result = 0;
	/*unsigned int nValue;*/
	int irq_reg;

	if ((tas_priv->muted) && (state == TAS2562_POWER_ACTIVE))
		state = TAS2562_POWER_MUTE;
	dev_err(tas_priv->dev, "set power state: %d\n", state);

	switch (state) {
	case TAS2562_POWER_ACTIVE:
		result = tas2562_load_init(tas_priv);
		if (result < 0)
			return result;
        //if set format was not called by asoc, then set it default
		if(tas_priv->asi_format == 0)
			tas_priv->asi_format = SND_SOC_DAIFMT_CBS_CFS
				| SND_SOC_DAIFMT_IB_NF
				| SND_SOC_DAIFMT_I2S;

		result = tas2562_set_fmt(tas_priv, tas_priv->asi_format);
		if (result < 0)
			goto activer_end;

//Clear latched IRQ before power on

		result = tas2562_update_bits(tas_priv, TAS2562_INT_CLK,
					     TAS2562_INT_CLR_LTCH,
					     TAS2562_INT_CLR_LTCH);
		if (result < 0)
			goto activer_end;

		result = tas2562_read(tas_priv, TAS2562_INT_LTCH0, &irq_reg);
		if (result < 0)
			goto activer_end;
		dev_info(tas_priv->dev, "IRQ reg is: %s %d, %d\n", __func__, irq_reg, __LINE__);

activer_end:
		tas_priv->power_up = true;
		tas_priv->power_state = TAS2562_POWER_ACTIVE;
/* irq routine will handle the error, and power on */
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(10));
		break;

	case TAS2562_POWER_MUTE:
		result = tas2562_update_bits(tas_priv, TAS2562_PWR_CTL,
					     TAS2562_ISNS_PD |
					     TAS2562_VSNS_PD |
					     TAS2562_MODE_MASK,
					     TAS2562_MODE_MUTE);
		tas_priv->power_up = true;
		tas_priv->power_state = TAS2562_POWER_MUTE;
		break;

	case TAS2562_POWER_SHUTDOWN:
		result = tas2562_update_bits(tas_priv, TAS2562_PWR_CTL,
					     TAS2562_MODE_MASK,
					     TAS2562_MODE_SHUTDOWN);
		tas_priv->power_up = false;
		tas_priv->power_state = TAS2562_POWER_SHUTDOWN;
		msleep(20);

		break;

	default:
		dev_err(tas_priv->dev, "wrong power state setting %d\n", state);

	}

	if(result < 0)
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(10));
	return result;
}

static const struct snd_soc_dapm_widget tas2562_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("Voltage Sense", "ASI1 Capture", 1,
			     TAS2562_PWR_CTL, 2, 1),
	SND_SOC_DAPM_AIF_OUT("Current Sense", "ASI1 Capture", 0,
			     TAS2562_PWR_CTL, 3, 1),
	SND_SOC_DAPM_MIXER("ASI1 Sel", TAS2562_TDM_CFG2, 4, 0,
			   &tas2562_asi_controls[0],
			   ARRAY_SIZE(tas2562_asi_controls)),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_SIGGEN("VMON"),
	SND_SOC_DAPM_SIGGEN("IMON")
};

static const struct snd_soc_dapm_route tas2562_audio_map[] = {
	{"ASI1 Sel", "Left", "ASI1"},
	{"ASI1 Sel", "Right", "ASI1"},
	{"ASI1 Sel", "LeftRightDiv2", "ASI1"},
	{"DAC", NULL, "ASI1 Sel"},
	{"OUT", NULL, "DAC"},
	/*{"VMON", NULL, "Voltage Sense"},
	{"IMON", NULL, "Current Sense"},*/
	{"Voltage Sense", NULL, "VMON"},
	{"Current Sense", NULL, "IMON"},
};


static int tas2562_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	dev_dbg(tas_priv->dev, "%s, %d \n", __func__, mute);

	mutex_lock(&tas_priv->codec_lock);
	if (mute) {
		tas2562_set_power_state(tas_priv, TAS2562_POWER_SHUTDOWN);
	} else {
		tas2562_set_power_state(tas_priv, TAS2562_POWER_ACTIVE);
	}
	mutex_unlock(&tas_priv->codec_lock);
	return 0;
}

static int tas2562_slot_config(struct snd_soc_codec *codec, struct tas2562_priv *tas_priv, int blr_clk_ratio)
{
	int ret = 0;
	if(tas_priv->slot_width == 16)
		ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG5,
					  TAS2562_VSNS_TX_EN |
					  TAS2562_VSNS_SLOT_MASK,
					  TAS2562_VSNS_TX_EN |
					  TAS2562_VSNS_SLOT(2));
	else
		ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG5,
					  TAS2562_VSNS_TX_EN |
					  TAS2562_VSNS_SLOT_MASK,
					  TAS2562_VSNS_TX_EN |
					  TAS2562_VSNS_SLOT(4));
	if (ret < 0)
		goto end;

	ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG6,
				  TAS2562_ISNS_TX_EN | TAS2562_ISNS_SLOT_MASK,
				  TAS2562_ISNS_TX_EN | TAS2562_ISNS_SLOT(0));
end:
	return ret;
}

static int tas2562_set_slot(struct snd_soc_codec *codec, int slot_width)
{
	int ret = 0;
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	switch (slot_width) {
	case 16:
		ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG2,
					  TAS2562_RX_SLEN_MASK,
					  TAS2562_RX_SLEN_16);
		break;
	case 24:
		ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG2,
					  TAS2562_RX_SLEN_MASK,
					  TAS2562_RX_SLEN_24);
		break;
	case 32:
		ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG2,
					  TAS2562_RX_SLEN_MASK,
					  TAS2562_RX_SLEN_32);
		break;
	case 0:
		/* Do not change slot width */
		break;
	default:
		dev_err(tas_priv->dev, "slot width not supported");
		ret = -EINVAL;
	}

	if (ret >= 0)
		tas_priv->slot_width = slot_width;

	return ret;
}

static int tas2562_set_bitwidth(struct tas2562_priv *tas_priv, int bitwidth)
{
	int slot_width_tmp = 16;
	int ret = 0;
	dev_info(tas_priv->dev, "%s %d\n", __func__, bitwidth);

	switch (bitwidth) {
	case SNDRV_PCM_FORMAT_S16_LE:
		ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG2,
					  TAS2562_RX_WLEN_MASK,
					  TAS2562_RX_WLEN_16);
		tas_priv->channel_width = 16;
		slot_width_tmp = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG2,
					  TAS2562_RX_WLEN_MASK,
					  TAS2562_RX_WLEN_24);
		tas_priv->channel_width = 24;
		slot_width_tmp = 32;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG2,
					  TAS2562_RX_WLEN_MASK,
					  TAS2562_RX_WLEN_32);
		tas_priv->channel_width = 32;
		slot_width_tmp = 32;
		break;

	default:
		dev_info(tas_priv->dev, "Not supported params format\n");
	}

	/* If machine driver did not call set slot width */
	//if (tas_priv->slot_width == 0)
	if (ret < 0)
		goto end;
	ret = tas2562_set_slot(tas_priv->codec, slot_width_tmp);

end:
	dev_info(tas_priv->dev, "channel_width: %d,  slot_width_tmp: %d\n", tas_priv->channel_width, slot_width_tmp);
	tas_priv->pcm_format = bitwidth;

	return ret;
}

static int tas2562_set_samplerate(struct tas2562_priv *tas_priv, int samplerate)
{
	unsigned int value;
	int ret;

	switch (samplerate) {
	case 48000:
		value = TAS2562_SAMP_RATE_48;
		break;
	case 44100:
		value = TAS2562_SAMP_RATE_44_1;
		break;
	case 96000:
		value = TAS2562_SAMP_RATE_96;
		break;
	case 88200:
		value = TAS2562_SAMP_RATE_88_2;
		break;
	case 192000:
		value = TAS2562_SAMP_RATE_192;
		break;
	case 176400:
		value = TAS2562_SAMP_RATE_176_4;
		break;
	default:
		dev_err(tas_priv->dev, "%s, unsupported sample rate, %d\n",
			__func__, samplerate);

		return -EINVAL;
	}

	ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG0,
				  TAS2562_SAMP_RATE_MASK, value);
	if (ret < 0)
		return ret;

	tas_priv->sample_rate = samplerate;

	return 0;
}

static int tas2562_mute_ctrl_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = tas_priv->muted;
	dev_dbg(tas_priv->dev, "tas2562_mute_ctrl_get = %d\n",
		tas_priv->muted);

	return 0;
}

static int tas2562_mute_ctrl_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	muted = ucontrol->value.integer.value[0];

	dev_dbg(tas_priv->dev, "tas2562_mute_ctrl_put = %d\n", muted);

	tas_priv->muted = !!muted;

	return 0;
}

static int tas2562_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);
	int blr_clk_ratio;
	int ret = 0;

	dev_dbg(tas_priv->dev, "%s, format: %d\n", __func__,
		params_format(params));

	mutex_lock(&tas_priv->codec_lock);

	ret = tas2562_set_bitwidth(tas_priv, params_format(params));
	if(ret < 0)
	{
		dev_info(tas_priv->dev, "set bitwidth failed, %d\n", ret);
		goto end;
	}

	blr_clk_ratio = params_channels(params) * tas_priv->channel_width;
	dev_info(tas_priv->dev, "blr_clk_ratio: %d\n", blr_clk_ratio);
	if(blr_clk_ratio != 0) {
		ret = tas2562_slot_config(tas_priv->codec, tas_priv, blr_clk_ratio);
		if(ret < 0)
			goto end;
	}

	dev_info(tas_priv->dev, "%s, sample rate: %d\n", __func__,
		params_rate(params));

	ret = tas2562_set_samplerate(tas_priv, params_rate(params));

end:
	mutex_unlock(&tas_priv->codec_lock);
	if(tas_priv->err_code & ERROR_DEVA_I2C_COMM)
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(10));
	return 0;
}

static int tas2562_set_fmt(struct tas2562_priv *tas_priv, unsigned int fmt)
{
	u8 tdm_rx_start_slot = 0, asi_cfg_1 = 0;
	int ret = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		asi_cfg_1 = 0x00;
		break;
	default:
		dev_err(tas_priv->dev, "ASI format master is not found\n");
		ret = -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		dev_info(tas_priv->dev, "INV format: NBNF\n");
		asi_cfg_1 |= TAS2562_RX_EDGE_RISING;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		dev_info(tas_priv->dev, "INV format: IBNF\n");
		asi_cfg_1 |= TAS2562_RX_EDGE_FALLING;
		break;
	default:
		dev_err(tas_priv->dev, "ASI format Inverse is not found\n");
		ret = -EINVAL;
	}

	ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG1,
				  TAS2562_RX_EDGE_MASK, asi_cfg_1);
	if (ret < 0)
		goto end;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case (SND_SOC_DAIFMT_I2S):
		tdm_rx_start_slot = 1;
		break;
	case (SND_SOC_DAIFMT_DSP_A):
	case (SND_SOC_DAIFMT_DSP_B):
		tdm_rx_start_slot = 1;
		break;
	case (SND_SOC_DAIFMT_LEFT_J):
		tdm_rx_start_slot = 0;
		break;
	default:
		dev_err(tas_priv->dev, "DAI Format is not found, fmt=0x%x\n", fmt);
		ret = -EINVAL;
		break;
	}

	ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG1,
				  TAS2562_RX_OFFSET_MASK,
				  TAS2562_RX_OFFSET(tdm_rx_start_slot));
	if (ret < 0)
		goto end;

	ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG4,
				  TAS2562_TX_EDGE_MASK |
				  TAS2562_TX_OFFSET_MASK |
				  TAS2562_TX_FILL_MASK,
				  TAS2562_TX_EDGE_FALLING |
				  TAS2562_TX_OFFSET(0));
	if (ret < 0)
		goto end;

	tas_priv->asi_format = fmt;

end:
	return ret;
}

static int tas2562_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	dev_dbg(tas_priv->dev, "%s, format=0x%x\n", __func__, fmt);

	ret = tas2562_set_fmt(tas_priv, fmt);
	if(ret < 0)
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(10));
	return 0;
}

static int tas2562_set_dai_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	int ret = 0;
	struct snd_soc_codec *codec = dai->codec;
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	dev_dbg(tas_priv->dev, "%s, tx_mask:%d, rx_mask:%d, slots:%d, slot_width:%d",
			__func__, tx_mask, rx_mask, slots, slot_width);

	ret = tas2562_set_slot(codec, slot_width);
	if(ret < 0)
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(10));

	return 0;
}

static struct snd_soc_dai_ops tas2562_dai_ops = {
	.digital_mute = tas2562_mute,
	.hw_params  = tas2562_hw_params,
	.set_fmt    = tas2562_set_dai_fmt,
	.set_tdm_slot = tas2562_set_dai_tdm_slot,
};

#define TAS2562_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

#define TAS2562_RATES (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 \
						SNDRV_PCM_RATE_88200 |\
						SNDRV_PCM_RATE_96000 |\
						SNDRV_PCM_RATE_176400 |\
						SNDRV_PCM_RATE_192000\
						)

static struct snd_soc_dai_driver tas2562_dai_driver[] = {
	{
		.name = "tas2562 ASI1",
		.id = 0,
		.playback = {
			.stream_name    = "ASI1 Playback",
			.channels_min   = 2,
			.channels_max   = 2,
			.rates      = SNDRV_PCM_RATE_8000_192000,
			.formats    = TAS2562_FORMATS,
		},
		.capture = {
			.stream_name    = "ASI1 Capture",
			.channels_min   = 0,
			.channels_max   = 2,
			.rates          = SNDRV_PCM_RATE_8000_192000,
			.formats    = TAS2562_FORMATS,
		},
		.ops = &tas2562_dai_ops,
		.symmetric_rates = 1,
	},
};

static int tas2562_load_init(struct tas2562_priv *tas_priv)
{
	int ret;

	ret = tas2562_update_bits(tas_priv, TAS2562_MISC_CFG1, TAS2562_IRQZ_PU,
				  TAS2562_IRQZ_PU);
	if (ret < 0)
		return ret;

	ret = tas2562_update_bits(tas_priv, TAS2562_TDM_CFG4,
				  TAS2562_TX_EDGE_MASK |
				  TAS2562_TX_OFFSET_MASK |
				  TAS2562_TX_FILL_MASK,
				  TAS2562_TX_EDGE_FALLING |
				  TAS2562_TX_OFFSET(0));
	if (ret < 0)
		return ret;

	ret = tas2562_write(tas_priv, TAS2562_CLK_CFG, 0x0c);
	if (ret < 0)
		return ret;

	ret = tas2562_load_init_cfg(tas_priv);

	return ret;
}

static int tas2562_codec_probe(struct snd_soc_codec *codec)
{
	int ret;
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	ret = snd_soc_add_codec_controls(codec, tas2562_controls,
					 ARRAY_SIZE(tas2562_controls));
	if (ret < 0) {
		pr_err("%s: add_codec_controls failed, err %d\n",
			__func__, ret);
		return ret;
	}

	tas2562_load_init(tas_priv);
	tas2562_iv_enable(tas_priv, 1);
	tas_priv->codec = codec;

	tas2562_smartamp_add_controls(tas_priv);

	dev_err(tas_priv->dev, "%s\n", __func__);

	return 0;
}

static int tas2562_codec_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static DECLARE_TLV_DB_SCALE(tas2562_digital_tlv, 1100, 50, 0);

static const struct snd_kcontrol_new tas2562_snd_controls[] = {
	SOC_SINGLE_TLV("Amp Output Level", TAS2562_PB_CFG1, 0, 0x16, 0,
		       tas2562_digital_tlv),
	SOC_SINGLE_EXT("SmartPA Mute", SND_SOC_NOPM, 0, 0x0001, 0,
		       tas2562_mute_ctrl_get, tas2562_mute_ctrl_put),
};

static struct snd_soc_codec_driver soc_codec_driver_tas2562 = {
	.probe			= tas2562_codec_probe,
	.remove			= tas2562_codec_remove,
	.read			= tas2562_codec_read,
	.write			= tas2562_codec_write,
	.suspend		= tas2562_codec_suspend,
	.resume			= tas2562_codec_resume,
	.component_driver = {
	.controls		= tas2562_snd_controls,
	.num_controls		= ARRAY_SIZE(tas2562_snd_controls),
		.dapm_widgets		= tas2562_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(tas2562_dapm_widgets),
		.dapm_routes		= tas2562_audio_map,
		.num_dapm_routes	= ARRAY_SIZE(tas2562_audio_map),
	},
};

int tas2562_register_codec(struct tas2562_priv *tas_priv)
{
	int result = 0;

	dev_info(tas_priv->dev, "%s, enter\n", __func__);
	result = snd_soc_register_codec(tas_priv->dev,
		&soc_codec_driver_tas2562,
		tas2562_dai_driver, ARRAY_SIZE(tas2562_dai_driver));
	return result;
}

int tas2562_deregister_codec(struct tas2562_priv *tas_priv)
{
	snd_soc_unregister_codec(tas_priv->dev);

	return 0;
}

void tas2562_load_config(struct tas2562_priv *tas_priv)
{
	int ret = 0;

	tas2562_hw_reset(tas_priv);
	msleep(2);
	tas2562_write(tas_priv, TAS2562_SW_RESET, TAS2562_RESET);
	msleep(3);

	ret = tas2562_slot_config(tas_priv->codec, tas_priv, 1);
	if(ret < 0) {
		goto end;
	}

	tas2562_load_init(tas_priv);
	tas2562_iv_enable(tas_priv, 1);

	ret = tas2562_set_slot(tas_priv->codec, tas_priv->slot_width);
	if (ret < 0)
		goto end;

	ret = tas2562_set_fmt(tas_priv, tas_priv->asi_format);
	if (ret < 0)
		goto end;

	ret = tas2562_set_bitwidth(tas_priv, tas_priv->pcm_format);
	if (ret < 0)
		goto end;

	ret = tas2562_set_samplerate(tas_priv, tas_priv->sample_rate);
	if (ret < 0)
		goto end;

	ret = tas2562_set_power_state(tas_priv, tas_priv->power_state);
	if (ret < 0)
		goto end;

end:
/* power up failed, restart later */
	if (ret < 0)
		schedule_delayed_work(&tas_priv->irq_work,
				msecs_to_jiffies(1000));
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2562 ALSA SOC Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
