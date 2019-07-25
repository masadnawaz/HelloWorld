#define DEBUG

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/regulator/consumer.h>

#include <linux/mipi_csi2.h>
#include <media/v4l2-chip-ident.h>
#include "mxc_v4l2_capture.h"
#include <linux/debugfs.h>

#define DEBUG
#define USER_SUB_MAP		0
#define VDP_SUB_MAP		1
#define USER_SUB_MAP_2		2
#define VPP_SUB_MAP		3
#define CSI_SUB_MAP		4

#define REG_USM_INPUT_CONTROL 			0x0000
#define REG_USM_AUTODETECT_ENABLE 		0x0007
#define REG_DEFAULT_VALUE_Y 		    	0x000C
#define REG_USM_ADI_CONTROL_1 		    	0x000E
#define REG_USM_POWER_MANAGEMENT 		0x000F
#define REG_USM_STATUS_1 			0x0010
#define REG_USM_STATUS_3 			0x0013
#define REG_ANALOG_CLAMP_CONTROL 		0x0014
#define REG_USM_POLARITY 			0x0037
#define REG_USM_ADC_CONTROL 			0x003A
#define REG_USM_ST_NOISE_READBACK_1 		0x00DE
#define REG_USM_ST_NOISE_READBACK_2 		0x00DF
#define REG_VS_MODE_CONTROL 	        	0x00F9
#define REG_DRIVE_STRENGTH 	            	0x00F4

#define REG_VDP_RAW_STATUS_2 			0x2045
#define REG_VDP_RAW_STATUS_3 			0x2049

#define REG_CSI_CKSUM_EN 			0x000E
#define REG_CSI_VC_REF				0x000D
#define REG_CSI_ESC_MODE_CTL 			0x0026
#define REG_CSI_DPHY_PWDN_CTL 			0x00DE

struct adv7280_chipset {
	struct sensor_data sen;
	struct device *dev;
	struct i2c_client *client;
	struct i2c_client *client_csi_tx;
	struct i2c_client *client_vpp_tx;
	unsigned int page;
#ifdef CONFIG_ADV7280_DEBUGFS
	struct dentry	  *dbgfs_dir;
#endif
};

#ifdef CONFIG_ADV7280_DEBUGFS

static int adv7280_dbgfs_init(struct adv7280_chipset* adv7280);

ssize_t adv7280_dbgfs_read(struct file *file, char __user *buf, size_t count,
	loff_t *ppos);

static const struct file_operations adv7280_dbgfs_fops = {
	.read		= adv7280_dbgfs_read,
};

struct adv7280_reg {
	size_t		size;
	uint16_t	addr;
	u8		map_number;
};

static struct adv7280_reg user_sub_map[] = {
		{ .addr	= REG_USM_INPUT_CONTROL,				.size	= 1, .map_number = USER_SUB_MAP,},
		{ .addr	= REG_USM_AUTODETECT_ENABLE,				.size	= 1, .map_number = USER_SUB_MAP,},
		{ .addr	= REG_DEFAULT_VALUE_Y,			        	.size	= 1, .map_number = USER_SUB_MAP,},
		{ .addr	= REG_USM_STATUS_1,					.size	= 1, .map_number = USER_SUB_MAP,},
		{ .addr	= REG_USM_STATUS_3,					.size	= 1, .map_number = USER_SUB_MAP,},
		{ .addr	= REG_ANALOG_CLAMP_CONTROL,				.size	= 1, .map_number = USER_SUB_MAP,},
		{ .addr	= REG_USM_ADI_CONTROL_1,				.size	= 1, .map_number = USER_SUB_MAP,},
		{ .addr	= REG_USM_POWER_MANAGEMENT,				.size	= 1, .map_number = USER_SUB_MAP,},
		{ .addr	= REG_USM_POLARITY,					.size	= 1, .map_number = USER_SUB_MAP,},
		{ .addr	= REG_USM_ADC_CONTROL,					.size	= 1, .map_number = USER_SUB_MAP,},
		{ .addr	= REG_USM_ST_NOISE_READBACK_1,				.size	= 1, .map_number = USER_SUB_MAP,},
		{ .addr	= REG_USM_ST_NOISE_READBACK_2,				.size	= 1, .map_number = USER_SUB_MAP,},
		{ .addr	= REG_VS_MODE_CONTROL,					.size	= 1, .map_number = USER_SUB_MAP,},
		{ .addr	= REG_DRIVE_STRENGTH,					.size	= 1, .map_number = USER_SUB_MAP,},
};

static struct adv7280_reg vdp_sub_map[] = {
		{ .addr	= REG_VDP_RAW_STATUS_2,		.size	= 1, .map_number = VDP_SUB_MAP,},
		{ .addr	= REG_VDP_RAW_STATUS_3,		.size	= 1, .map_number = VDP_SUB_MAP,},
};

static struct adv7280_reg csi_map[] = {
		{ .addr	= REG_CSI_CKSUM_EN,		.size	= 1, .map_number = CSI_SUB_MAP,},
		{ .addr	= REG_CSI_ESC_MODE_CTL,		.size	= 1, .map_number = CSI_SUB_MAP,},
		{ .addr	= REG_CSI_DPHY_PWDN_CTL,	.size	= 1, .map_number = CSI_SUB_MAP,},
		{ .addr	= REG_CSI_VC_REF,		.size	= 1, .map_number = CSI_SUB_MAP,}
};
#endif // CONFIG_ADV7280_DEBUGFS

static short i2p_enabled = false;
module_param(i2p_enabled, short, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(i2p_enabled, "Whether to enable the interlaced-to-progressive convertor");
/*
static int interlace_mode = 4;
module_param(interlace_mode, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(interlace_mode, "enum v4l2_field interlace type the module will report");
*/
static int g_ain = 0;

static int adv7280_select_page(struct i2c_client *client, unsigned int page)
{
	int ret= i2c_smbus_write_byte_data(client, 0x0E,page);
	if (ret < 0) {
		printk("adv7280 : %s reg %x error  %d\n",__func__,0x0E,ret);
	}
	return 0;
}

static inline int adv7280_read_reg(struct i2c_client *client, u16 reg) {
	int ret;
	if((client->addr == 0x20) && (reg != 0x000E)){
		adv7280_select_page(client, reg>>8);
	}
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_dbg(&client->dev, "read reg error: ret = %d\n", ret);
	}
	return ret;
}

static inline int adv7280_write_reg(struct i2c_client *client, u16 reg, u8 val) {
	int ret;
	if((client->addr == 0x20) && (reg != 0x000E)){
		adv7280_select_page(client, reg>>8);
	}
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		printk("adv7280 : %s reg %x error  %d\n",__func__,reg,ret);
        //dev_dbg(&client->dev, "write reg error: ret = %d register value : %x \n ", ret,reg);
	}
	printk("adv7280 : %s reg %x returned  %d\n",__func__,reg,ret);
	return ret;
}

static void adv7280_setup(struct adv7280_chipset* adv7280) {
    struct i2c_client* i2c = adv7280->client;
    struct i2c_client* csi = adv7280->client_csi_tx;
    struct i2c_client* vpp = adv7280->client_vpp_tx;
	u8 v_chan = (adv7280->sen.virtual_channel << 6 );

dev_dbg( &i2c->dev, "%s: writing virtual_channel [%#hhx]\n", __func__, v_chan);

adv7280_write_reg(i2c,REG_USM_POWER_MANAGEMENT, 0x00); 				// Exit Power Down Mode
adv7280_write_reg(i2c,REG_USM_ADI_CONTROL_1, 0x00 ); 				// Enter User Sub Map
adv7280_write_reg(i2c,REG_USM_INPUT_CONTROL, g_ain); 				// INSEL = CVBS in on Ain 1 - Day Cam0
adv7280_write_reg(i2c,REG_USM_AUTODETECT_ENABLE, 0x00); 			// AUTODETECT PAL and NTSC

adv7280_write_reg(i2c,0x000E, 0x80); 						// ADI Required Write
adv7280_write_reg(i2c,0x009C, 0x00); 						// ADI Required Write
adv7280_write_reg(i2c,0x009C, 0xFF); 						// ADI Required Write

adv7280_write_reg(i2c, REG_DEFAULT_VALUE_Y, 0x37); 				// force free run
adv7280_write_reg(i2c, REG_VS_MODE_CONTROL, 0x07);   				// 576i
adv7280_write_reg(i2c, REG_ANALOG_CLAMP_CONTROL, 0x01); 			// color bar
adv7280_write_reg(i2c, REG_DRIVE_STRENGTH, 0x17); 				// high drive strength


adv7280_write_reg(i2c, 0x0003, 0x4E); 						// ADI Required Write
adv7280_write_reg(i2c, 0x0004, 0x37); 						// Power-up INTRQ pin
adv7280_write_reg(i2c, 0x0013, 0x00); 						// Enable INTRQ output driver
adv7280_write_reg(i2c,0x0017, 0x41); 						// select SH1
adv7280_write_reg(i2c,0x001D, 0xC0); 						// Tri-State LLC output driver
adv7280_write_reg(i2c,0x0052, 0xCD); 						// this register is not valid in adv7280-m

adv7280_write_reg(i2c,0x0080, 0x51 ); 						// ADI Required Write
adv7280_write_reg(i2c, 0x0081, 0x51); 						// ADI Required Write
adv7280_write_reg(i2c, 0x0082, 0x68); 						// ADI Required Write

if (i2p_enabled) {
	adv7280_write_reg(i2c, 0x00FD, 0x84); 					// Set VPP Map Address
	adv7280_write_reg(vpp, 0x00A3, 0x00); 					// ADI Required Write         //TODO:check cant see this register
	adv7280_write_reg(vpp, 0x005B, 0x00); 					// Advanced Timing Enable
	adv7280_write_reg(vpp, 0x0055, 0x80); 					// Enable I2P
}

/* set virtual_channel
 * on iMX6 this will control the destination CSI:
 * virtual_channel 0 : IPU1 CSI0
 * virtual_channel 1 : IPU1 CSI1
 * virtual_channel 2 : IPU2 CSI0
 * virtual_channel 3 : IPU2 CSI1
 */

adv7280_write_reg(i2c,0x00FE, 0x88); //  Set CSI Map Address
adv7280_write_reg(csi, 0x000D, v_chan );

//adv7280_write_reg(csi,0x02, 0x28);  //me
adv7280_write_reg(csi, 0x00DE, 0x02); 						// Power up MIPI D-PHY
adv7280_write_reg(csi, 0x00D2, 0xF7); 						// ADI Required Write
adv7280_write_reg(csi, 0x00D8, 0x65); 						// ADI Required Write
adv7280_write_reg(csi, 0x00E0, 0x09); 						// ADI Required Write
adv7280_write_reg(csi, 0x002C, 0x00); 						// ADI Required Write
adv7280_write_reg(csi, 0x001D, 0x80); 						// ADI Required Write
adv7280_write_reg(csi, 0x0000, 0x80); 						// Power Down MIPI CSI-2 Tx
adv7280_write_reg(csi, 0x0000, 0x00); 						// Power up MIPI CSI-2 Tx
}

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p) {
	printk("adv7280: %s type %d\n",__func__);
	struct adv7280_chipset *adv7280 = s->priv;
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}
	memset(p, 0, sizeof(*p));
	return 0;
}

static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	printk("adv7280: %s type %d\n",__func__,f->type);
	struct adv7280_chipset *adv7280 = s->priv;
	int res = 0xF;

	switch (f->type) {
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
			f->fmt.pix = adv7280->sen.pix;
			f->fmt.pix.field = i2p_enabled ? V4L2_FIELD_NONE : V4L2_FIELD_ALTERNATE;
			f->fmt.pix.field = V4L2_FIELD_INTERLACED;
			pr_debug("BUF_TYPE_VIDEO_CAPTURE adv7280->sen.pixformat is: %d\n\n", adv7280->sen.pix.pixelformat);
			break;
		case V4L2_BUF_TYPE_PRIVATE:
			res = (adv7280_read_reg(adv7280->client , 0x10) >> 4);
			if(res == 4){ // PAL
				f->fmt.pix.pixelformat = V4L2_STD_PAL;
			}
			else if(res == 0){ // NTSC
				f->fmt.pix.pixelformat = V4L2_STD_NTSC;
			}
			pr_debug("BUF_TYPE_PRIVATE pixel format is: %d\n\n", f->fmt.pix.pixelformat);
			break;
		case V4L2_BUF_TYPE_SENSOR:
			pr_debug("f->type == V4L2_BUF_TYPE_SENSOR [%#x]", f->type );
			f->fmt.spix.swidth = 720;
			f->fmt.spix.sheight = 288;
			f->fmt.spix.left = 0;
			f->fmt.spix.top = 0;
			break;
		default:
			f->fmt.pix = adv7280->sen.pix;
			break;
	}

	return 0;
}

static int ioctl_s_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	printk("adv7280: %s\n",__func__);
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		/* We only support UYVY */
		f->fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
		/* Set the field format according to whether the I2P is enabled */
		f->fmt.pix.field = i2p_enabled ? V4L2_FIELD_NONE : V4L2_FIELD_ALTERNATE;
		f->fmt.pix.field = V4L2_FIELD_INTERLACED;
		return 0;
	}
	return -EINVAL;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	printk("adv7280: %s vc id %x\n",__func__,vc->id);
	struct adv7280_chipset *adv7280 = s->priv;
 	int ret = 0;

        switch (vc->id) {
        case V4L2_CID_BRIGHTNESS:
                vc->value = adv7280->sen.brightness;
                break;
        case V4L2_CID_HUE:
                vc->value =adv7280->sen.hue;
                break;
        case V4L2_CID_CONTRAST:
                vc->value = adv7280->sen.contrast;
                break;
        case V4L2_CID_SATURATION:
                vc->value = adv7280->sen.saturation;
                break;
        case V4L2_CID_RED_BALANCE:
                vc->value = adv7280->sen.red;
                break;
        case V4L2_CID_BLUE_BALANCE:
                vc->value = adv7280->sen.blue;
                break;
        case V4L2_CID_EXPOSURE:
                vc->value = adv7280->sen.ae_mode;
                break;
        case V4L2_CID_SHARPNESS:
        	printk("%s V4L2_CID_SHARPNESS not assigned.\n",__func__);
                break;
        default:
                ret = -EINVAL;
        }

        return ret;

}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	printk("adv7280: %s\n",__func__);
        return 0;
}


/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	printk("adv7280: %s\n",__func__);
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per second */
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = 25;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =	(u32)a->parm.capture.capturemode;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_err("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_err("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	printk("adv7280: %s type %d\n",__func__,a->type);
	struct adv7280_chipset *adv7280 = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = adv7280->sen.streamcap.capability;
		cparm->timeperframe = adv7280->sen.streamcap.timeperframe;
		cparm->capturemode = adv7280->sen.streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_err("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}
	return ret;
}


static int ioctl_enum_framesizes(struct v4l2_int_device *s, struct v4l2_frmsizeenum *fsize) {
	printk( "%s index %d\n",__func__,fsize->index);

	if (fsize->index > 0){
		return -EINVAL;
	}
	struct adv7280_chipset *adv7280 = s->priv;
	fsize->pixel_format = adv7280->sen.pix.pixelformat;
	fsize->discrete.width = adv7280->sen.pix.width;
	fsize->discrete.height  = adv7280->sen.pix.height;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	return 0;
}

/*!
 *  * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *   *                             VIDIOC_ENUM_FRAMEINTERVALS ioctl
 *    * @s: pointer to standard V4L2 device structure
 *     * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *      *
 *       * Return 0 if successful, otherwise -EINVAL.
 *        */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s, struct v4l2_frmivalenum *fival)
{
	printk("%s index %d\n",__func__,fival->index);
	if (fival->index > 0){
		return -EINVAL;
	}
	struct adv7280_chipset *adv7280 = s->priv;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
        fival->discrete.numerator = 1;
	if (fival->pixel_format == adv7280->sen.pix.pixelformat && fival->width == adv7280->sen.pix.width && fival->height == adv7280->sen.pix.height) {
		fival->discrete.denominator = 25;
		return 0;
	};
        return -EINVAL;
}

static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	printk("adv7280: %s\n",__func__);
	((struct v4l2_dbg_chip_ident *)id)->match.type = V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "adv7280_decoder");
	((struct v4l2_dbg_chip_ident *)id)->ident = V4L2_IDENT_ADV7180;
	return 0;
}

static int ioctl_dev_init(struct v4l2_int_device *s)
{
    printk("adv7280: %s\n",__func__);
    void *mipi_csi2_info;
    struct adv7280_chipset *adv7280 = s->priv;

    // added: force reset
    adv7280_write_reg(adv7280->client,REG_USM_POWER_MANAGEMENT, 0x80); // Force reset
    msleep(10); // from datasheet

    mipi_csi2_info = mipi_csi2_get_info();

    if(mipi_csi2_get_status(mipi_csi2_info)) {
        mipi_csi2_disable(mipi_csi2_info);
        msleep(1);
    }
    
    mipi_csi2_enable(mipi_csi2_info);

    if (!mipi_csi2_get_status(mipi_csi2_info)) {
    		pr_err("Can not enable mipi csi2 driver!\n");
    		return -1;
    	}
    mipi_csi2_set_lanes(mipi_csi2_info, 1);

    mipi_csi2_reset(mipi_csi2_info);
    //mipi_csi2_reset_with_dphy_freq(mipi_csi2_info, 0x26); // 40 02 26 0c

	mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_YUV422);

	msleep(200);
	u32 mipi_reg = 0;
	if (mipi_csi2_info) {
		unsigned int i = 0;

		/* wait for mipi sensor ready */
		while (1) {
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			if (mipi_reg != 0x200)
				break;
			if (i++ >= 20) {
				printk("adv7280 : mipi csi2 can not receive sensor clk! %x\n", mipi_reg);
				//return -1;
				break;
			}
			msleep(10);
		}

		i = 0;
		/* wait for mipi stable */
		while (1) {
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
			if (!mipi_reg)
				break;
			if (i++ >= 20) {
				printk("adv7280: mipi csi2 can not receive data correctly 1!\n");
				break;
				//return -1;
			}
			msleep(10);
		}

		i = 0;
		/* wait for mipi stable */
		while (1) {
			mipi_reg = mipi_csi2_get_error2(mipi_csi2_info);
			if (!mipi_reg)
				break;
			if (i++ >= 20) {
				printk("adv7280: mipi csi2 can not receive data correctly 2!\n");
				break;
				//return -1;
			}
			msleep(10);
		}
	}
	printk("adv7280:- setup adv7280\n");
	adv7280_setup(adv7280);

	msleep(800);

    return 0;
}

static int ioctl_s_input_video(struct v4l2_int_device *s, int *ain)
{
	printk("adv7280: %s\n",__func__);
       struct adv7280_chipset *adv7280 = s->priv;
       g_ain = *ain;
       adv7280_write_reg(adv7280->client, 0x00, g_ain);
       ioctl_dev_init(s);
       return 0;
}

static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	printk("adv7280: %s\n",__func__);
	void *mipi_csi2_info;
	mipi_csi2_info = mipi_csi2_get_info();
    	mipi_csi2_disable(mipi_csi2_info);
	return 0;
}

static int ioctl_enum_fmt_cap(struct v4l2_int_device *s, struct v4l2_fmtdesc *fmt)
{
	printk("adv7280: %s\n",__func__);
	if (fmt->index > 0)
		return -EINVAL;

	fmt->pixelformat = ((struct adv7280_chipset*)s->priv)->sen.pix.pixelformat;

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc adv7280_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*) ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	//{vidioc_int_s_power_num, (v4l2_int_ioctl_func*) ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	//{vidioc_int_init_num, (v4l2_int_ioctl_func*) ioctl_init},
	{vidioc_int_enum_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
/*	{vidioc_int_try_fmt_cap_num,(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_s_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm}, 
	//{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, 
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,(v4l2_int_ioctl_func *) ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num, (v4l2_int_ioctl_func *) ioctl_g_chip_ident},
	{vidioc_int_s_input_video_num, (v4l2_int_ioctl_func *) ioctl_s_input_video},
};

static struct v4l2_int_slave adv7280_slave = {
	.ioctls = adv7280_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(adv7280_ioctl_desc),
};

static struct v4l2_int_device adv7280_int_device = {
	.module = THIS_MODULE,
	.name = "adv7280",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &adv7280_slave,
	},
};

static int adv7280_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    struct adv7280_chipset *adv7280;
	int retval;

    adv7280 = kzalloc(sizeof(struct adv7280_chipset), GFP_KERNEL);
    i2c_set_clientdata(client, adv7280);
    adv7280->dev = &client->dev;
    adv7280->client = client;
    adv7280->client_csi_tx = i2c_new_dummy(client->adapter, 0x44);
    adv7280->client_vpp_tx = i2c_new_dummy(client->adapter, 0x42);

    adv7280->sen.i2c_client = client;
    adv7280->sen.streamcap.timeperframe.denominator = 25;
    adv7280->sen.streamcap.timeperframe.numerator = 1;
    adv7280->sen.streamcap.capability = V4L2_MODE_HIGHQUALITY | V4L2_CAP_TIMEPERFRAME;
    adv7280->sen.streamcap.capturemode = 0;
    adv7280->sen.pix.width = 720;
    adv7280->sen.pix.height = 288;

//    adv7280->sen.brightness= 50;
//    adv7280->sen.hue = 50;
//    adv7280->sen.contrast = 50;
//    adv7280->sen.saturation = 50;
	
    adv7280->sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;// V4L2_PIX_FMT_YUV420;
    adv7280->sen.sensor_clk = devm_clk_get(&client->dev, "csi_mclk");
    if(IS_ERR(adv7280->sen.sensor_clk))
        pr_err("Could not get sensor clock\n");
    adv7280->sen.on = 1;
    adv7280->sen.mipi_camera = 1;



	retval = of_property_read_u32(client->dev.of_node, "ipu_id",
					&(adv7280->sen.ipu_id));
	if (retval) {
		dev_err( &client->dev, "ipu_id missing or invalid, reverting to default [0], ipu_id was [%d]\n", adv7280->sen.ipu_id);
		adv7280->sen.ipu_id = 0;
	}

	retval = of_property_read_u32(client->dev.of_node, "csi_id",
					&(adv7280->sen.csi));
	if (retval) {
		dev_err( &client->dev, "csi id missing or invalid, reverting to default [0], csi was [%d]\n", adv7280->sen.csi);
		adv7280->sen.csi = 0;
	}


	retval = of_property_read_u32(client->dev.of_node, "v_channel",
					&(adv7280->sen.virtual_channel));
	if (retval) {
		dev_err( &client->dev,  "v_channel missing or invalid, reverting to default [0], v_channel was [%d]\n", adv7280->sen.virtual_channel);
		adv7280->sen.virtual_channel = 0;
	}

	dev_dbg( &client->dev, "v_channel is [%d]\n", adv7280->sen.virtual_channel);

    clk_prepare_enable(adv7280->sen.sensor_clk);

    adv7280_int_device.priv = adv7280;
    return v4l2_int_device_register(&adv7280_int_device);

#ifdef CONFIG_ADV7280_DEBUGFS
    int r_val = adv7280_dbgfs_init(adv7280);
    if(0 != r_val)
    {
    	printk("error in debugfs");
    }
#endif // CONFIG_ADV7280_DEBUGFS
}


static int adv7280_i2c_remove(struct i2c_client *i2c_client)
{
    struct adv7280_chipset *adv7280 = i2c_get_clientdata(i2c_client);
#ifdef CONFIG_ADV7280_DEBUGFS
	printk("adv7280: removing debugfs content");
	debugfs_remove_recursive(adv7280->dbgfs_dir);
	adv7280->dbgfs_dir = NULL;
#endif /* CONFIG_ADV7280_DEBUGFS */
    v4l2_int_device_unregister(&adv7280_int_device);
    i2c_unregister_device(adv7280->client_csi_tx);
    i2c_unregister_device(adv7280->client_vpp_tx);
    kfree(adv7280);
    return 0;
}


static void adv7280_power_down(struct i2c_client *i2c)
{
	adv7280_write_reg(i2c,0x0F, 0x20); // Power Down Mode
}


#ifdef CONFIG_PM
static int adv7280_suspend(struct device *dev)
{
	adv7280_power_down(to_i2c_client(dev));
	return 0;
}

static int adv7280_resume(struct device *dev)
{
	adv7280_setup(i2c_get_clientdata(to_i2c_client(dev)));
	return 0;
}


static const struct dev_pm_ops adv7280_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(adv7280_suspend, adv7280_resume)
};

#define ADV7280_PM_OPS		(&adv7280_pm_ops)
#else /* CONFIG_PM */
#define ADV7280_PM_OPS		NULL
#endif /* CONFIG_PM */



static const struct i2c_device_id adv7280_i2c_id[] = {
    {"adv7280", 0},
    {},
};

MODULE_DEVICE_TABLE(i2c, adv7280_i2c_id);

static struct i2c_driver adv7280_i2c_driver = {
    .driver = {
           .owner = THIS_MODULE,
           .name = "adv7280",
           .pm = ADV7280_PM_OPS,
           },
    .probe = adv7280_i2c_probe,
    .remove = adv7280_i2c_remove,
    .id_table = adv7280_i2c_id,
};

static __init int adv7280_init(void) {
    int err = i2c_add_driver(&adv7280_i2c_driver);
    if (err < 0)
        pr_err("%s: driver registration failed, error=%d\n", __func__, err);
    return err;
}

static void __exit adv7280_clean(void) {
    i2c_del_driver(&adv7280_i2c_driver);
}

#ifdef CONFIG_ADV7280_DEBUGFS
static int adv7280_dbgfs_init(struct adv7280_chipset* adv7280)
{
	char dbgfs_fn[12];
	int rv;
	struct dentry *dbgfs_file;
	unsigned long i;

	printk("adv7280: enter\n");

	adv7280->dbgfs_dir = debugfs_create_dir("adv7280", NULL);
	if (IS_ERR_OR_NULL(adv7280->dbgfs_dir)) {
		rv = PTR_ERR(adv7280->dbgfs_dir);
		printk("adv7280: unable to create directory %s (%d)", "adv7280",rv);
		goto out;
	}
	adv7280->dbgfs_dir->d_inode->i_private = adv7280;
	for (i = 0; ARRAY_SIZE(user_sub_map) > i; i++) {
		snprintf(dbgfs_fn, ARRAY_SIZE(dbgfs_fn),"usm_" "%#06x",
				user_sub_map[i].addr);
		dbgfs_file = debugfs_create_file(dbgfs_fn, 0660, adv7280->dbgfs_dir,
			&user_sub_map[i], &adv7280_dbgfs_fops);
		if (IS_ERR_OR_NULL(dbgfs_file)) {
			rv = PTR_ERR(dbgfs_file);
			printk("adv7280: unable to create file %s (%d)", dbgfs_fn,rv);
			goto err;
		}
	}

	for (i = 0; ARRAY_SIZE(vdp_sub_map) > i; i++) {
		snprintf(dbgfs_fn, ARRAY_SIZE(dbgfs_fn),"vdp_" "%#06x",
				vdp_sub_map[i].addr);
		dbgfs_file = debugfs_create_file(dbgfs_fn, 0660, adv7280->dbgfs_dir,
			&vdp_sub_map[i], &adv7280_dbgfs_fops);
		if (IS_ERR_OR_NULL(dbgfs_file)) {
			rv = PTR_ERR(dbgfs_file);
			printk("adv7280: unable to create file %s (%d)", dbgfs_fn,rv);
			goto err;
		}
	}

	for (i = 0; ARRAY_SIZE(csi_map) > i; i++) {
			snprintf(dbgfs_fn, ARRAY_SIZE(dbgfs_fn),"csi_" "%#06x",
					csi_map[i].addr);
			dbgfs_file = debugfs_create_file(dbgfs_fn, 0660, adv7280->dbgfs_dir,
				&csi_map[i], &adv7280_dbgfs_fops);
			if (IS_ERR_OR_NULL(dbgfs_file)) {
				rv = PTR_ERR(dbgfs_file);
				printk("adv7280: unable to create file %s (%d)", dbgfs_fn,rv);
				goto err;
			}
	}

	rv = 0;
	goto out;

err:
	debugfs_remove_recursive(adv7280->dbgfs_dir);
	adv7280->dbgfs_dir = NULL;
out:
	return rv;
}

ssize_t adv7280_dbgfs_read(struct file *file, char __user *buf, size_t count,
	loff_t *ppos)
{
	char kbuf[8];
	int kcount;
	int rv;
	struct adv7280_chipset *adv7280;
	struct adv7280_reg *adv7280_reg;

	adv7280 = (struct adv7280_chipset *)file->f_path.dentry->d_parent->d_inode->i_private;

	//printk("adv7280: adv7280_dbgfs_read enter");

	adv7280_reg =  (struct adv7280_reg *)file->f_inode->i_private;
	if((adv7280_reg->map_number == USER_SUB_MAP) ||
			(adv7280_reg->map_number == VDP_SUB_MAP)||
			(adv7280_reg->map_number == USER_SUB_MAP_2)){
		rv = adv7280_read_reg(adv7280->client, adv7280_reg->addr);
	}else if(adv7280_reg->map_number == VPP_SUB_MAP){
		rv = adv7280_read_reg(adv7280->client_vpp_tx, adv7280_reg->addr);
	}else if(adv7280_reg->map_number == CSI_SUB_MAP){
		rv = adv7280_read_reg(adv7280->client_csi_tx, adv7280_reg->addr);
	}else{
		printk("adv7280: adv7280_dbgfs_read error\n");
		goto out;
	}
	//printk("adv7280: read value of reg %#x val %#x\n",adv7280_reg->addr,rv);
	if (0 > rv) {
		goto out;
	}
	kcount = snprintf(kbuf, ARRAY_SIZE(kbuf), "%#05x\n", rv);
	rv = simple_read_from_buffer(buf, count, ppos, kbuf, kcount);

out:
	return rv;
}
#endif

module_init(adv7280_init);
module_exit(adv7280_clean);

MODULE_AUTHOR("L4B");
MODULE_DESCRIPTION("adv7280 video decoder driver");
MODULE_LICENSE("GPL");
