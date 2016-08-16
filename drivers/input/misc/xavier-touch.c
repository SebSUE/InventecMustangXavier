/*
 * touch platform driver for xavier board
 *
 * copyright (c) 2016 eSoftThings
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/mfd/xavier-i2c.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/platform_device.h>

#define XAVIER_TOUCH_NAME "Xavier Touch panel"
#define XAVIER_TOUCH_DEVICE_PATH "Xavier-Touch/input0"
#define XAVIER_TOUCH_MAX_EVENT_SIZE 4
#define XAVIER_TOUCH_LOCATION_MAX_VALUE 127
#define XAVIER_TOUCH_VELOCITY_MAX_VALUE 127
#define XAVIER_LOCATION_INCREMENT 5

#define XAVIER_TOUCH_ID_EVENT_MASK 0xE0 /* 3 first MSB bits */
#define XAVIER_TOUCH_BTN0_EVENT_MASK 0x10 /* 1 bit after id event */
#define XAVIER_TOUCH_BTN1_EVENT_MASK 0x08 /* 1 bit after BTN_0 event */
#define XAVIER_TOUCH_LOCATION_EVENT_MSB_MASK 0x1F
#define XAVIER_TOUCH_LOCATION_EVENT_LSB_MASK 0xC0 /* 7 bits after id event */
#define XAVIER_TOUCH_ORIENTATION_EVENT_MASK 0x20 /* 1 bit after location */
#define XAVIER_TOUCH_VELOCITY_EVENT_MSB_MASK 0x1F
#define XAVIER_TOUCH_VELOCITY_EVENT_LSB_MASK 0xC0 /* 7 bits after id event */
#define XAVIER_TOUCH_ERROR_EVENT_MASK 0x18 /* 2 bits after id event */

#define XAVIER_TOUCH_EVENT_SHIFT 5
#define XAVIER_TOUCH_BTN0_EVENT_SHIFT 4
#define XAVIER_TOUCH_BTN1_EVENT_SHIFT 3
#define XAVIER_TOUCH_LOCATION_EVENT_MSB_SHIFT  2
#define XAVIER_TOUCH_LOCATION_EVENT_LSB_SHIFT  6
#define XAVIER_TOUCH_VELOCITY_EVENT_MSB_SHIFT  2
#define XAVIER_TOUCH_VELOCITY_EVENT_LSB_SHIFT  6
#define XAVIER_TOUCH_ORIENTATION_EVENT_SHIFT   5
#define XAVIER_TOUCH_ERROR_EVENT_SHIFT 3

#define BTN_TOUCH_EVENT 0x00
#define START_RING_EVENT 0x01
#define END_RING_EVENT 0x02
#define POSITION_CHANGE_EVENT 0x03
#define ERROR_EVENT 0x04

#define XAVIER_TOUCH_BTN0 BTN_1
#define XAVIER_TOUCH_BTN1 BTN_2
#define XAVIER_TOUCH_BTNWHEEL BTN_WHEEL
#define XAVIER_TOUCH_WHEEL ABS_WHEEL
#define XAVIER_TOUCH_ORIENTATION ABS_MT_ORIENTATION
#define XAVIER_TOUCH_VELOCITY ABS_MISC

#define BTN_PRESSED 1
#define BTN_RELEASED 0

struct xavier_touch {
	struct device *dev;
	struct input_dev *input_dev;
	struct xavier_dev *mfd;
	struct work_struct input_work;

	int btn0;
	int btn1;
	int btn_wheel;
	int abs_wheel;
	int direction;
	int velocity;
	char btn_0;
	char btn_1;
};

static int input_handler(struct platform_device *touch_dev,
			 char *data, int size)
{

	struct xavier_touch *xavier_touch;
	char event,btn0_value,btn1_value,location,orientation,velocity,error;
	int ret;

	ret = 0;

	xavier_touch = dev_get_drvdata(&touch_dev->dev);
	if (xavier_touch == NULL) {
		ret = -EFAULT;
		goto error;
	}

	/*check if MCU is booted and touch is enabled */
	if (xavier_touch->mfd->touch_ena == 0 || xavier_touch->mfd->boot == 0) {
		ret = 0;
		goto error;
	}

	/* get the type of event received */
	event = ((data[0] & XAVIER_TOUCH_ID_EVENT_MASK) >>
		 XAVIER_TOUCH_EVENT_SHIFT);

	switch (event) {
	case BTN_TOUCH_EVENT: /* btn_touch event */

		btn0_value = ((data[0] & XAVIER_TOUCH_BTN0_EVENT_MASK) >>
			      XAVIER_TOUCH_BTN0_EVENT_SHIFT);
		btn1_value = ((data[0] & XAVIER_TOUCH_BTN1_EVENT_MASK) >>
			      XAVIER_TOUCH_BTN1_EVENT_SHIFT);

		if (xavier_touch->btn_0 != btn0_value) {
			dev_dbg(xavier_touch->dev, "BTN_TOUCH_0 event\n");
			xavier_touch->btn_0 = btn0_value;
			input_report_key(xavier_touch->input_dev,
					 xavier_touch->btn0, btn0_value);
		}

		if (xavier_touch->btn_1 != btn1_value) {
			dev_dbg(xavier_touch->dev, "BTN_TOUCH_1 event\n");
			xavier_touch->btn_1 = btn1_value;
			input_report_key(xavier_touch->input_dev,
					 xavier_touch->btn1, btn1_value);
		}

		break;

	case START_RING_EVENT: /* start_ring event */

		dev_dbg(xavier_touch->dev, "START RING event\n");
		location = ((data[0] & XAVIER_TOUCH_LOCATION_EVENT_MSB_MASK) <<
			    XAVIER_TOUCH_LOCATION_EVENT_MSB_SHIFT);
		location += ((data[1] & XAVIER_TOUCH_LOCATION_EVENT_LSB_MASK) >>
			     XAVIER_TOUCH_LOCATION_EVENT_LSB_SHIFT);

		input_report_key(xavier_touch->input_dev,
				 xavier_touch->btn_wheel, BTN_PRESSED);

		input_report_abs(xavier_touch->input_dev,
				 xavier_touch->abs_wheel,
				 location * XAVIER_LOCATION_INCREMENT);

		break;

	case END_RING_EVENT: /* end_ring */
		dev_dbg(xavier_touch->dev, "END RING event\n");
		location = ((data[0] & XAVIER_TOUCH_LOCATION_EVENT_MSB_MASK) <<
			    XAVIER_TOUCH_LOCATION_EVENT_MSB_SHIFT);
		location += ((data[1] & XAVIER_TOUCH_LOCATION_EVENT_LSB_MASK) >>
			     XAVIER_TOUCH_LOCATION_EVENT_LSB_SHIFT);

		input_report_key(xavier_touch->input_dev,
				 xavier_touch->btn_wheel, BTN_RELEASED);

		input_report_abs(xavier_touch->input_dev,
				 xavier_touch->abs_wheel,
				 location * XAVIER_LOCATION_INCREMENT);
		break;

	case POSITION_CHANGE_EVENT: /* position_change_ring */
		dev_dbg(xavier_touch->dev, "POSITION event\n");
		location = ((data[0] & XAVIER_TOUCH_LOCATION_EVENT_MSB_MASK) <<
			    XAVIER_TOUCH_LOCATION_EVENT_MSB_SHIFT);
		location += ((data[1] & XAVIER_TOUCH_LOCATION_EVENT_LSB_MASK) >>
			     XAVIER_TOUCH_LOCATION_EVENT_LSB_SHIFT);

		orientation = ((data[1] & XAVIER_TOUCH_ORIENTATION_EVENT_MASK) >>
			       XAVIER_TOUCH_ORIENTATION_EVENT_SHIFT);

		velocity = ((data[1] & XAVIER_TOUCH_VELOCITY_EVENT_MSB_MASK) <<
			    XAVIER_TOUCH_VELOCITY_EVENT_MSB_SHIFT);
		velocity += ((data[2] & XAVIER_TOUCH_VELOCITY_EVENT_LSB_MASK) >>
			     XAVIER_TOUCH_VELOCITY_EVENT_LSB_SHIFT);

		input_report_abs(xavier_touch->input_dev,
				 xavier_touch->abs_wheel,
				 location * XAVIER_LOCATION_INCREMENT);
		input_report_abs(xavier_touch->input_dev,
				 xavier_touch->direction, orientation );
		input_report_abs(xavier_touch->input_dev,
				 xavier_touch->velocity, velocity);

		break;

	case ERROR_EVENT: /* error event */
		dev_dbg(xavier_touch->dev, "Error event\n");
		error = ((data[0] & XAVIER_TOUCH_ERROR_EVENT_MASK) >>
			 XAVIER_TOUCH_ERROR_EVENT_SHIFT);
		ret = kstrtoint(&error, 0, &xavier_touch->mfd->error_code);

		break;

	default:
		printk(KERN_ERR "Xavier : Error received unknown data : %s\n",
		       data);
		break;
	}

	//Sync the new report for this device
	input_sync(xavier_touch->input_dev);

error:
	return ret;
}

static ssize_t xavier_value_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{

	int ret = 0;
	struct xavier_dev *xavier_dev = dev_get_drvdata(dev);
	if (xavier_dev == NULL) {
		ret = -EFAULT;
		printk(KERN_ERR "Xavier : %s - NULL i2c device found \n",
		       __func__);
		goto exit;
	}

	ret = sprintf(buf, "%d\n", xavier_dev->touch_ena);

exit:
	return ret;
}

static ssize_t xavier_value_store(struct device *dev,
				  struct device_attribute *attr,
				  const  char *buf, size_t count)
{
	int temp;
	int ret = 0;
	char msg;
	struct xavier_dev *xavier_dev = dev_get_drvdata(dev);

	if (xavier_dev == NULL) {
		ret = -EFAULT;
		printk(KERN_ERR "Xavier : %s - NULL i2c device found \n",
		       __func__);
		goto error;
	}

	ret = kstrtoint(buf,0,&temp);
	if (ret) {
		printk(KERN_ERR "Xavier : %s - Failed to transform data \n",
		       __func__);
		goto error;
	}

	if (temp != 0 && temp != 1) {
		ret = -EINVAL;
		printk(KERN_ERR "Xavier : %s - Wrong value must be 1 or 0 : found %d\n",
		       __func__, temp);
		goto error;
	}
	else {

		msg = XAVIER_CONTROL_TOUCH << XAVIER_CONTROL_TYPE_SHIFT;
		msg += buf[0] << 4; /*3 bits for control idx 1bit for the value*/
		xavier_dev->touch_ena = temp;
		ret = xavier_dev->write_dev(xavier_dev, 1, &msg, 0);
		if (ret < 0) {
			ret = -EIO;
			printk(KERN_ERR "Xavier : %s - Failed to write to the device\n",
			       __func__);
			goto error;
		}
	}

	return count;
error:
	return ret;
}

static DEVICE_ATTR(touch_ena, S_IWUSR | S_IRUSR, xavier_value_show,
		   xavier_value_store);

static struct attribute *xavier_attrs[] = {
	&dev_attr_touch_ena.attr,
	NULL
};

static struct attribute_group xavier_attr_group = {
	.attrs = xavier_attrs,
};

static int xavier_touch_probe(struct platform_device *pdev)
{
	struct xavier_dev *xavier_dev;
	struct xavier_touch *xavier_touch;
	struct input_dev *input_dev;
	int ret;

	printk(KERN_INFO "Xavier : Xavier touch started\n");
	xavier_dev = dev_get_drvdata(pdev->dev.parent);

	xavier_touch = kzalloc(sizeof(struct xavier_touch), GFP_KERNEL);
	if (!xavier_touch) {
		ret = -ENOMEM;
		printk(KERN_ERR "Xavier : %s - Failed to allocate structure",
		       __func__);
		goto error;
	}

	xavier_touch->mfd = xavier_dev;
	xavier_touch->dev = &pdev->dev;
	xavier_touch->btn_0 = 0;
	xavier_touch->btn_1 = 0;

	xavier_dev->touch_dev = pdev;
	xavier_dev->input_handler = input_handler;

	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		printk(KERN_ERR "Xavier : %s - Failed to allocate input device",
		       __func__);
		goto error_free;
	}

	xavier_touch->input_dev = input_dev;

	if (of_property_read_u32(xavier_dev->dev->of_node, "btn0",
				 &xavier_touch->btn0)) {

		printk(KERN_ERR "Xavier : %s - Failed to found btn0 node\n",
		       __func__);
		ret = -EFAULT;
		goto error_free_device;
	}

	if (of_property_read_u32(xavier_dev->dev->of_node, "btn1",
				 &xavier_touch->btn1)) {

		printk(KERN_ERR "Xavier : %s - Failed to found btn1 node\n",
		       __func__);
		ret = -EFAULT;
		goto error_free_device;
	}

	if (of_property_read_u32(xavier_dev->dev->of_node, "btn_wheel",
				 &xavier_touch->btn_wheel)) {

		printk(KERN_ERR "Xavier : %s - Failed to found btn_wheel node\n",
		       __func__);
		ret = -EFAULT;
		goto error_free_device;
	}

	if (of_property_read_u32(xavier_dev->dev->of_node, "abs_wheel",
				 &xavier_touch->abs_wheel)) {

		printk(KERN_ERR "Xavier : %s - Failed to found abs_wheel node\n",
		       __func__);
		ret = -EFAULT;
		goto error_free_device;
	}

	if (of_property_read_u32(xavier_dev->dev->of_node, "direction",
				 &xavier_touch->direction)) {

		printk(KERN_ERR "Xavier : %s - Failed to found direction node\n",
		       __func__);
		ret = -EFAULT;
		goto error_free_device;
	}

	if (of_property_read_u32(xavier_dev->dev->of_node, "velocity",
				 &xavier_touch->velocity)) {

		printk(KERN_ERR "Xavier : %s - Failed to found velocity node\n",
		       __func__);
		ret = -EFAULT;
		goto error_free_device;
	}

	/* Set input device configuration */
	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(xavier_touch->btn0)] |= BIT_MASK(xavier_touch->btn0);
	input_dev->keybit[BIT_WORD(xavier_touch->btn1)] |= BIT_MASK(xavier_touch->btn1);

	input_dev->keybit[BIT_WORD(xavier_touch->btn_wheel)] |= BIT_MASK(xavier_touch->btn_wheel);

	input_set_abs_params(input_dev, xavier_touch->abs_wheel, 0,
			     XAVIER_TOUCH_LOCATION_MAX_VALUE, 0, 0);
	input_set_abs_params(input_dev, xavier_touch->direction, 0, 1, 0, 0);
	input_set_abs_params(input_dev, xavier_touch->velocity, 0,
			     XAVIER_TOUCH_VELOCITY_MAX_VALUE, 0, 0);

	input_dev->name = XAVIER_TOUCH_NAME;
	input_dev->phys = XAVIER_TOUCH_DEVICE_PATH;
	input_dev->dev.parent = xavier_touch->dev;
	input_dev->id.bustype = BUS_I2C;

	ret = input_register_device(input_dev);
	if (ret) {
		printk(KERN_ERR "Xavier : %s - Failed to register input device",
		       __func__);
		goto error_free_device;
	}
	ret = sysfs_create_group(&xavier_touch->mfd->dev->kobj,
				 &xavier_attr_group);
	if (ret) {
		printk(KERN_ERR "Xavier : %s - Failed to create sysfs entries\n",
		       __func__);
		goto error_free_device;
	}

	platform_set_drvdata(pdev, xavier_touch);
	return 0;

error_free_device:
	input_free_device(input_dev);
error_free:
	kfree(xavier_touch);
error:
	return ret;
}

static int xavier_touch_remove(struct platform_device *pdev)
{
	struct xavier_touch *xavier_touch = platform_get_drvdata(pdev);
	struct input_dev *input_dev = xavier_touch->input_dev;

	disable_irq(xavier_touch->mfd->irq);
	input_unregister_device(input_dev);
	input_free_device(input_dev);
	sysfs_remove_group(&xavier_touch->mfd->dev->kobj, &xavier_attr_group);
	kfree(xavier_touch);
	return 0;
}

static struct platform_driver xavier_touch_driver = {
	.driver = {
		.name = "Xavier-touch",
	},
	.probe = xavier_touch_probe,
	.remove = xavier_touch_remove,

};
module_platform_driver(xavier_touch_driver);

MODULE_DESCRIPTION("Xavier - Touch driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:Xavier-touch");
