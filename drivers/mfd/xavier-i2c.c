/*
 * mfd driver for xavier board
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/xavier-i2c.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>


#define XAVIER_DEVICE_ID 0
#define XAVIER_MESSAGE_SIZE_MASK 0x7C
#define XAVIER_MESSAGE_TYPE_MASK 0x03
#define XAVIER_MESSAGE_SIZE_SHIFT 2
#define IRQ_TRIGGER IRQF_TRIGGER_FALLING
#define XAVIER_FIRMWARE_MESSAGE_SIZE 32
#define XAVIER_CONTROL_TYPE_SHIFT 5
#define XAVIER_SLAVE_ADDRESS 0x12

#define XAVIER_NB_CHILD_INTERFACE 3
#define GPIO_MCU_BOOT 5
#define XAVIER_SLEEP_BOOT_TIMER 1
#define XAVIER_MCU_FIRMWARE_PATH "/etc/firmware/touch_firmware.bin"
#define XAVIER_FIRMWARE_VALIDATION 's'
#define XAVIER_FIRMWARE_SLAVE_ADDRESS 0x15
#define XAVIER_FIRMWARE_SIZE_SIZE 4 /* format of the size of the firmware */


#define CEILING(x,y) (((x) + (y) - 1) / (y))



static const struct mfd_cell xavier_devs[] = {
  { .name = "xavier-led" , },
  { .name = "xavier-touch" , },
};

static int xavier_i2c_read(struct xavier_dev *xavier, void *dest,
			   int size, int *child) {

  int ret;
  int nbbytesleft;
  char buf[XAVIER_I2C_NB_DATA_BYTES + 1];
  char *cur_dest;
  struct i2c_client *i2c;

  /* Checking arguments values */
  if (xavier == NULL || dest == NULL) {
    ret = -EFAULT;
    printk(KERN_ERR "Xavier : %s - NULL pointer argument\n", __func__);
    goto error;
  }

  if (size <= 0) {
    ret = -EINVAL;
    printk(KERN_ERR "Xavier : %s - size must be > 0 : found %d\n",
	   __func__,size);
    goto error;
  }

  i2c = xavier->i2c;
  if (i2c == NULL) {
    ret = -EFAULT;
    printk(KERN_ERR "Xavier : %s - NULL i2c device found \n",
	   __func__);
    goto error;
  }

  cur_dest = dest;
  mutex_lock(&xavier->lock);

  ret = i2c_master_recv(i2c, buf, XAVIER_I2C_NB_DATA_BYTES + 1);
  if (ret < 0) {
    printk(KERN_ERR "Xavier : %s - failed to read device : %d\n",
	   __func__,ret);
    goto error_unlock;
  }

  printk(KERN_ERR "Xavier : %s - data : \nXavier: %*ph\n",
	 __func__, XAVIER_I2C_NB_DATA_BYTES + 1, buf);

  *child = buf[0] & XAVIER_MESSAGE_TYPE_MASK;

  /* copy the data without the header */
  memcpy(cur_dest, &buf[1], ((size - 1) % XAVIER_I2C_NB_DATA_BYTES + 1));
  cur_dest += XAVIER_I2C_NB_DATA_BYTES;

  /* 5 bits give the size in 32 bits words */
  nbbytesleft = ((buf[0] & XAVIER_MESSAGE_SIZE_MASK) >>
		 XAVIER_MESSAGE_SIZE_SHIFT) * 4;

  /* Check if buffer is big enough for the data */
  if (nbbytesleft > size) {
    ret = -1;
    printk(KERN_ERR "Xavier : %s - buffer too small: found %d needed %d\n",
	   __func__, size, nbbytesleft);
    goto error_unlock;
  }

  /* Remove the bytes from the first read */
  nbbytesleft = nbbytesleft - XAVIER_I2C_NB_DATA_BYTES;

  /* Check  if more data is needed */
  while (nbbytesleft > 0) {

    ret = i2c_master_recv(i2c, buf, XAVIER_I2C_NB_DATA_BYTES + 1);
    printk(KERN_INFO "Xavier : %s - data : \nXavier:  %*ph\n",
	   __func__, XAVIER_I2C_NB_DATA_BYTES + 1, buf);

    if (ret < 0) {
      printk(KERN_ERR "Xavier : %s - failed to read device : %d\n",
	     __func__,ret);
      goto error_unlock;
    }

    /* Checking for padding to remove */
    if (nbbytesleft < XAVIER_I2C_NB_DATA_BYTES)
      memcpy(cur_dest, &buf[1], nbbytesleft);
    else
      memcpy(cur_dest, &buf[1], XAVIER_I2C_NB_DATA_BYTES);

    nbbytesleft = nbbytesleft - XAVIER_I2C_NB_DATA_BYTES;
    cur_dest += XAVIER_I2C_NB_DATA_BYTES;
  }

 error_unlock:
  mutex_unlock(&xavier->lock);
 error:
  return ret;
}

static int xavier_i2c_write(struct xavier_dev *xavier, int bytes,
			    const void *src, int child) {

  const char *cur_src;
  char msg[XAVIER_I2C_NB_DATA_BYTES + 1]; /* +1 for header */
  int bytesleft;
  int ret;
  int nb32Words;
  struct i2c_client *i2c;

  ret = 0;
  bytesleft = bytes;
  cur_src = src;

  /*Checking arguments values */
  if (xavier == NULL || src == NULL) {
    ret = -EFAULT;
    printk(KERN_ERR "Xavier : %s - NULL pointer argument\n", __func__);
    goto error;
  }

  i2c = xavier->i2c;
  if (i2c == NULL) {
    ret = -EFAULT;
    printk(KERN_ERR "Xavier : %s - NULL i2c device found \n", __func__);
    goto error;
  }

  if (bytes > XAVIER_I2C_MESSAGE_MAX_SIZE) {
    ret = -EINVAL;
    printk(KERN_ERR "Xavier : %s - Too much data to send : %d bytes (> %d)\n",
	   __func__, bytes, XAVIER_I2C_MESSAGE_MAX_SIZE);
    goto error;

  }

  if (child < 0 || child > XAVIER_NB_CHILD_INTERFACE) {
    ret = -EINVAL;
    printk(KERN_ERR "Xavier : %s - wrong child argument : %d \n",
	   __func__, child);
    goto error;

  }

  nb32Words = CEILING(bytesleft, 4);
  mutex_lock(&xavier->lock);

  while (bytesleft > XAVIER_I2C_NB_DATA_BYTES) {

    msg[0] = 0;
    msg[0] |= 0x00; /* set header frag bit */
    msg[0] |= (nb32Words << XAVIER_MESSAGE_SIZE_SHIFT); /* set header size bits */
    msg[0] |= child;    /* header type bits (control,led,touch)*/

    /* copy the data after the header*/
    memcpy(&msg[1], cur_src, XAVIER_I2C_NB_DATA_BYTES);
    ret += i2c_master_send(i2c, msg, XAVIER_I2C_NB_DATA_BYTES + 1);
    /*printk(KERN_INFO "Xavier : %s - data : \nXavier: %*ph\n",
      __func__, XAVIER_I2C_NB_DATA_BYTES + 1, msg);*/
    bytesleft = bytesleft - XAVIER_I2C_NB_DATA_BYTES;
    cur_src += XAVIER_I2C_NB_DATA_BYTES;
  }

  /*send the last frame with padding if necessary */
  msg[0] = 0;
  msg[0] |= 0x80; //header frag bit
  msg[0] |= (nb32Words << XAVIER_MESSAGE_SIZE_SHIFT); //header size bits
  msg[0] |= child;    //header type bits (control,led,touch)

  memcpy(&msg[1], cur_src, bytesleft); /* copy the last data segment after header*/

  /* add the padding if necessary */
  memset(&msg[bytesleft + 1], 0, XAVIER_I2C_NB_DATA_BYTES - bytesleft);

  ret += i2c_master_send(i2c, msg, XAVIER_I2C_NB_DATA_BYTES + 1);

  mutex_unlock(&xavier->lock);

 error:
  return ret;

}



static irqreturn_t xavier_interrupt_handler(int irq, void *dev) {

  struct xavier_dev *xavier_dev = dev_get_drvdata(dev);
  printk(KERN_INFO  "Xavier : Interruption detected \n");

  /*Workqueue needed as write and read can't be done in irq context */
  queue_work(xavier_dev->workqueue, &xavier_dev->interrupt_work);

  return IRQ_HANDLED;
}

static int xavier_mcu_reset(struct device *dev, bool flash) {

  int ret;
  struct xavier_dev *xavier_dev = dev_get_drvdata(dev);

  if (xavier_dev == NULL) {
    ret = -EFAULT;
    goto error;
  }

  xavier_dev->boot = 0;
  /* Retriver the gpio for the first reset */
  if (xavier_dev->reset_gpio == -1) {

    ret = of_get_named_gpio(dev->of_node, "reset-gpio", 0);
    if (ret < 0 ) {
      printk(KERN_ERR "Xavier : %s - No gpio node found \n",__func__);
      ret = -EINVAL;
      goto error;
    }

    xavier_dev->reset_gpio = ret;

    ret = gpio_request(xavier_dev->reset_gpio, "xavier_reset");
    if (ret) {
      printk(KERN_ERR "Xavier : %s - Failed to request gpio %d\n",
	     __func__, xavier_dev->reset_gpio);
      goto error;
    }

    ret = gpio_direction_output(xavier_dev->reset_gpio, 1);
    if (ret) {
      printk(KERN_ERR "Xavier : %s - Failed to set gpio direction\n",__func__);
      goto error;
    }

    /*export the gpio into sysfs */
    gpio_export(xavier_dev->reset_gpio,true);

  }

  /* Free irq to take control of the gpio */
  free_irq(xavier_dev->irq, xavier_dev->dev);

  if(flash) {
    /* Take control of the gpio */
    ret = gpio_direction_output(xavier_dev->slave_int, 1);
    if (ret) {
      printk(KERN_ERR "Xavier : %s - Failed to set gpio direction\n", __func__);
      goto error;
    }

    /* Set slave_int to one to not get stuck in bootloader mode */
    gpio_set_value(xavier_dev->slave_int, 1);
  }


  /*force hardware reset */
  gpio_set_value(xavier_dev->reset_gpio, 0);

  printk(KERN_INFO "Xavier : %s - gpio  value :%d \n",
	 __func__, gpio_get_value(xavier_dev->reset_gpio));


  gpio_set_value(xavier_dev->reset_gpio,1);
  printk(KERN_INFO "Xavier : %s - gpio  value :%d \n",
	 __func__, gpio_get_value(xavier_dev->reset_gpio));


  /* Flash need to retain control of the slave_int gpio until it's finish */
  if (!flash) {
    /*release the GPIO for the MCU */
    ret = gpio_direction_input(xavier_dev->slave_int);
    if (ret) {
      printk(KERN_ERR "Xavier : %s - Failed to set gpio direction\n",__func__);
      goto error;
    }

    /* Set back the IRQ */
    ret = request_irq(xavier_dev->irq, xavier_interrupt_handler,
		      IRQ_TRIGGER, "xavier", xavier_dev->dev);
    if (ret) {
      printk(KERN_INFO "Xavier : %s - Failed to request IRQ \n", __func__);
      goto error;
    }
  }

 error:
  return ret;
}


static int xavier_mcu_flash(struct device *dev) {

  int i;
  int ret;
  int size;
  int nb_data_chunk;
  loff_t pos;
  char buffer[XAVIER_FIRMWARE_MESSAGE_SIZE];
  struct file *firmfile;
  struct kstat ks;
  struct i2c_client *i2c;
  struct xavier_dev *xavier_dev = dev_get_drvdata(dev);
  mm_segment_t old_fs;

  /* necessary for vfs_read */
  old_fs = get_fs();

  if (xavier_dev == NULL) {
    ret = -EFAULT;
    printk(KERN_ERR "Xavier : %s - NULL xavier device found \n", __func__);
    goto error;
  }

  xavier_dev->led_ena = 0;
  i2c = xavier_dev->i2c;
  if (i2c == NULL) {
    ret = -EFAULT;
    printk(KERN_ERR "Xavier : %s - NULL i2c device found \n", __func__);
    goto error;
  }

  /* Reset the MCU */
  xavier_mcu_reset(dev, true);


  /* Open the firmware file */
  firmfile = filp_open(XAVIER_MCU_FIRMWARE_PATH, O_RDONLY, 0);
  if (IS_ERR(firmfile)) {
    printk(KERN_ERR "Xavier : %s - Failed to open %s \n",
	   __func__, XAVIER_MCU_FIRMWARE_PATH);
    ret = -EIO;
    goto error;
  }

  /* get the size of the file */
  vfs_getattr(&firmfile->f_path, &ks);
  size = ks.size;

  /* The size need to be send MSB first */
  for (i = 1; i <= XAVIER_FIRMWARE_SIZE_SIZE; i++) {
    buffer[i - 1] = size >> (XAVIER_FIRMWARE_SIZE_SIZE - i) * 8;
  }

  i2c->addr = XAVIER_FIRMWARE_SLAVE_ADDRESS;
  printk(KERN_INFO " Xavier : %s - size = %d (%x), buffer = %*ph", __func__, size, size, 4, buffer);

  /* First message is the size of the firmware */
  ret = i2c_master_send(i2c, buffer, XAVIER_FIRMWARE_SIZE_SIZE);
  /*printk(KERN_INFO "Xavier : %s - master_send ret = %d\n", __func__, ret);*/

  if (ret < 0) {
    printk(KERN_ERR "Xavier : %s - Failed to send to device ret = %d\n", __func__, ret);
    ret = -EIO;
    goto exit;
  }

  /* set the slave back to 1 */
  gpio_set_value(xavier_dev->slave_int, 1);


  nb_data_chunk = size / XAVIER_FIRMWARE_MESSAGE_SIZE;

  /* Send the firmware by chunk of 32 bytes */
  pos = 0;
  for (i = 0; i < nb_data_chunk; i++) {

    /*printk(KERN_INFO "\n\n Xavier: Data chunk number %d / %d", i, nb_data_chunk);*/

    set_fs(KERNEL_DS);
    ret = vfs_read(firmfile, (char __user *)buffer,
		   XAVIER_FIRMWARE_MESSAGE_SIZE, &pos);
    if (ret < 0) {
      printk(KERN_ERR "Xavier : %s - Failed to read the file - ret = %d\n",
	     __func__, ret);
      set_fs(old_fs);
      goto exit;
    }

    set_fs(old_fs);
    /*printk(KERN_INFO "Xavier : %s - Read from file : %*ph\n",
      __func__, XAVIER_FIRMWARE_MESSAGE_SIZE,buffer);*/

    ret = i2c_master_send(i2c, buffer, XAVIER_FIRMWARE_MESSAGE_SIZE);
    if (ret < 0) {
      printk(KERN_ERR "Xavier : %s - Failed to send to device ret = %d\n",
	     __func__, ret);
      goto exit;
    }

    /* The MCU send a validation message when it has copy the data into EEPROM */
    do {
      ret = i2c_master_recv(i2c, buffer, 1);
      if (ret != -ETIMEDOUT && ret < 0) {
	printk(KERN_ERR "Xavier : %s - failed to read device : %d\n",
	       __func__, ret);
	goto exit;
      }
    } while (ret == -ETIMEDOUT);


    if (buffer[0] != XAVIER_FIRMWARE_VALIDATION) {
      printk(KERN_ERR "Xavier : %s - Firmware flash failed - %s",
	     __func__, buffer);
    }
    /*printk(KERN_INFO "Xavier : ACK received : %c\n", buffer[0]);*/

  }

 exit:
  i2c->addr = XAVIER_SLAVE_ADDRESS;
  filp_close(firmfile, NULL);
  ret = gpio_direction_input(xavier_dev->slave_int);
  if (ret) {
    printk(KERN_ERR "Xavier : %s - Failed to set gpio direction\n", __func__);
    goto error;
  }

  /* Set back the IRQ */
  ret = request_irq(xavier_dev->irq, xavier_interrupt_handler,
		    IRQ_TRIGGER, "xavier", xavier_dev->dev);
  if (ret) {
    printk(KERN_INFO "Xavier : %s - Failed to request IRQ \n", __func__);
    goto error;
  }

  xavier_mcu_reset(dev, false);
  xavier_dev->led_ena = 1;
 error:
  return ret;
}


static ssize_t xavier_reset_store(struct device *dev,
				  struct device_attribute *attr,
				  const  char *buf, size_t count) {
  int ret;
  struct xavier_dev *xavier_dev = dev_get_drvdata(dev);

  if (xavier_dev == NULL) {
    ret = -EFAULT;
    goto exit;
  }

  ret = kstrtoint(buf, 0, &xavier_dev->reset);
  if (ret)
    goto exit;

  if (xavier_dev->reset == 1) {
    ret = xavier_mcu_reset(dev, false);
    if (ret)
      goto exit;
    else
      xavier_dev->reset = 0; /*sucessfull reset we go back to normal state */
  }

  return count;

 exit:
  return ret;
}

static ssize_t xavier_flash_store(struct device *dev,
				  struct device_attribute *attr,
				  const  char *buf, size_t count) {
  int ret;
  struct xavier_dev *xavier_dev = dev_get_drvdata(dev);

  if (xavier_dev == NULL) {
    ret = -EFAULT;
    goto exit;
  }

  ret = kstrtoint(buf, 0, &xavier_dev->flash);
  if (ret)
    goto exit;

  if (xavier_dev->flash == 1) {
    ret = xavier_mcu_flash(dev);
    if (ret)
      goto exit;
    else
      xavier_dev->reset = 0; /*sucessfull flash we go back to normal state */
  }

  return count;

 exit:
  return ret;
}

static ssize_t xavier_boot_show(struct device *dev,
				struct device_attribute *attr, char *buf) {

  int ret;
  struct xavier_dev *xavier_dev = dev_get_drvdata(dev);

  if (xavier_dev == NULL){
    ret = -EFAULT;
    printk(KERN_ERR  "Xavier : %s - NULL structure found\n", __func__);
    goto exit;
  }

  ret = sprintf(buf, "%d\n", xavier_dev->boot);

 exit:
  return ret;
}

static ssize_t xavier_error_code_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf) {

  int ret;
  struct xavier_dev *xavier_dev = dev_get_drvdata(dev);

  if (xavier_dev == NULL){
    ret = -EFAULT;
    printk(KERN_ERR  "Xavier : %s - NULL structure found\n", __func__);
    goto exit;
  }

  ret = sprintf(buf, "%d\n", xavier_dev->error_code);

 exit:
  return ret;
}

static ssize_t xavier_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf) {

  int ret;
  struct xavier_dev *xavier_dev = dev_get_drvdata(dev);

  if (xavier_dev == NULL){
    ret = -EFAULT;
    printk(KERN_ERR  "Xavier : %s, NULL structure found\n", __func__);
    goto exit;
  }

  ret = sprintf(buf, "%s\n", xavier_dev->version);

 exit:
  return ret;
}

static ssize_t xavier_flash_show(struct device *dev,
				 struct device_attribute *attr, char *buf) {

  int ret;
  struct xavier_dev *xavier_dev = dev_get_drvdata(dev);

  if (xavier_dev == NULL){
    ret = -EFAULT;
    printk(KERN_ERR  "Xavier : %s - NULL structure found\n", __func__);
    goto exit;
  }

  ret = sprintf(buf, "%d\n", xavier_dev->flash);

 exit:
  return ret;
}

static ssize_t xavier_reset_show(struct device *dev,
				 struct device_attribute *attr, char *buf) {

  int ret;
  struct xavier_dev *xavier_dev = dev_get_drvdata(dev);

  if (xavier_dev == NULL){
    ret = -EFAULT;
    printk(KERN_ERR  "Xavier : %s - NULL structure found\n", __func__);
    goto exit;
  }

  ret = sprintf(buf, "%d\n", xavier_dev->reset);

 exit:
  return ret;
}

static ssize_t xavier_reset_cause_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf) {

  int ret;
  struct xavier_dev *xavier_dev = dev_get_drvdata(dev);

  if (xavier_dev == NULL){
    ret = -EFAULT;
    printk(KERN_ERR  "Xavier : %s - NULL structure found\n", __func__);
    goto exit;
  }

  ret = sprintf(buf, "%d\n", xavier_dev->reset_cause);

 exit:
  return ret;
}



static DEVICE_ATTR(boot, S_IRUSR, xavier_boot_show, NULL);
static DEVICE_ATTR(error_code, S_IRUSR, xavier_error_code_show, NULL);
static DEVICE_ATTR(version, S_IRUSR, xavier_version_show, NULL);
static DEVICE_ATTR(flash, S_IWUSR | S_IRUSR, xavier_flash_show,
		   xavier_flash_store);
static DEVICE_ATTR(reset, S_IWUSR | S_IRUSR, xavier_reset_show,
		   xavier_reset_store);
static DEVICE_ATTR(reset_cause, S_IRUSR, xavier_reset_cause_show, NULL);


static struct attribute *xavier_attrs[] = {
  &dev_attr_boot.attr,
  &dev_attr_error_code.attr,
  &dev_attr_flash.attr,
  &dev_attr_reset.attr,
  &dev_attr_version.attr,
  &dev_attr_reset_cause.attr,
  NULL
};

static struct attribute_group xavier_attr_group = {
  .attrs = xavier_attrs,
};


static void xavier_interrupt_work(struct work_struct *work) {


  struct xavier_dev *xavier_dev;
  char buf[XAVIER_VERSION_SIZE];
  int ret;
  int child;

  xavier_dev = container_of(work, struct xavier_dev, interrupt_work);
  if (xavier_dev == NULL){
    printk(KERN_ERR  "Xavier : %s - NULL structure found\n", __func__);
    goto exit;
  }

  ret = xavier_i2c_read(xavier_dev, buf, XAVIER_VERSION_SIZE, &child);
  if (ret < 0){
    printk(KERN_ERR  "Xavier : %s - failed to read : %d\n", __func__, ret);
    goto exit;
  }

  printk(KERN_INFO  "Xavier : event detected \n");
  /*if it's a control header it must be an error_code or the version */
  if (child == XAVIER_HEADER_CONTROL_ID){

    /*type code are in MSB so we need to shift them */
    if ((buf[0] >> XAVIER_CONTROL_TYPE_SHIFT) == XAVIER_CONTROL_ERROR_CODE){


      switch(buf[0] & XAVIER_CONTROL_ERROR_DATA_MASK) {

      case NO_ERROR :

	printk(KERN_INFO  "Xavier : NO_ERROR received \n");

	/* no error mean the host rebooted after handshake */
	if (xavier_dev->boot == 0) {
	  printk(KERN_INFO  "Xavier : Boot handshake started \n");
	  buf[0] = XAVIER_CONTROL_VERSION << XAVIER_CONTROL_TYPE_SHIFT;
	  ret = xavier_i2c_write(xavier_dev, 1, buf, 0);
	  if (ret < 0) {
	    printk(KERN_ERR  "Xavier : %s - failed to write : %d\n",
		   __func__, ret);
	    goto exit;
	  }

	  ret = xavier_i2c_read(xavier_dev, buf, XAVIER_VERSION_SIZE, &child);
	  if (ret < 0 || child != XAVIER_HEADER_CONTROL_ID){
	    printk(KERN_ERR  "Xavier : %s - failed to read : %d\n",
		   __func__, ret);
	    goto exit;
	  }

	  if ((buf[0] >> XAVIER_CONTROL_TYPE_SHIFT) == XAVIER_CONTROL_VERSION) {
	    strncpy(xavier_dev->version, &buf[1], XAVIER_VERSION_SIZE);
	    xavier_dev->boot = 1;
	    xavier_dev->error_code = 0;
	    printk(KERN_INFO  "Xavier : Boot handshake done !\n");
	  }
	  else {
	    xavier_dev->error_code = buf[0] & XAVIER_CONTROL_ERROR_DATA_MASK;
	    printk(KERN_INFO  "Xavier : error code %d received\n",
		   xavier_dev->error_code);
	  }

	}
	else
	  xavier_dev->error_code = buf[0] & XAVIER_CONTROL_ERROR_DATA_MASK;

	break;

      case BOOT_NOT_DONE :

	printk(KERN_INFO  "Xavier : BOOT_NOT_DONE event received\n");
	xavier_dev->boot = 0;
	/*Ask for the version to validate the boot */
	buf[0] = XAVIER_CONTROL_VERSION << XAVIER_CONTROL_TYPE_SHIFT;
	ret = xavier_i2c_write(xavier_dev, 1, buf, 0);
	if (ret < 0) {
	  printk(KERN_ERR  "Xavier : %s - failed to write : %d\n",
		 __func__, ret);
	  goto exit;
	}

	break;

      case ERROR_EEPROM:

	printk(KERN_INFO  "Xavier : Error EEPROM received\n");
	ret = xavier_mcu_reset(xavier_dev->dev, false);
	if (ret)
	  goto exit;
	break;

      case ERROR_TOUCH:

	printk(KERN_INFO  "Xavier : Error touch received\n");
	ret = xavier_mcu_reset(xavier_dev->dev, false);
	if (ret)
	  goto exit;
	break;

      case ERROR_IPC :

	ret = xavier_mcu_reset(xavier_dev->dev, false);
	if (ret)
	  goto exit;
	printk(KERN_INFO  "Xavier : Error EEPROM received\n");
	break;

      case ERROR_WDT :

	ret = xavier_mcu_reset(xavier_dev->dev, false);
	if (ret)
	  goto exit;
	printk(KERN_INFO  "Xavier : Error LED length received\n");
	break;

      default :

	printk(KERN_INFO  "Xavier : Error code %d received\n",
	       buf[0] & XAVIER_CONTROL_ERROR_DATA_MASK);
	/*store error_code */
	xavier_dev->error_code = buf[0] & XAVIER_CONTROL_ERROR_DATA_MASK;
      }
    }
    else if ((buf[0] >> XAVIER_CONTROL_TYPE_SHIFT) == XAVIER_CONTROL_VERSION) {

      /*if it's not an error_code it is the version so we store it */
      strncpy(xavier_dev->version, &buf[1], XAVIER_VERSION_SIZE);
      xavier_dev->error_code = NO_ERROR; /* boot done we remove the error code */
      xavier_dev->boot = 1;
      printk(KERN_INFO "Xavier : Boot handshake done !\n");
      /* Ask for the reson of the reset if any */

      buf[0] = XAVIER_CONTROL_RESET_CAUSE << XAVIER_CONTROL_TYPE_SHIFT;
      ret = xavier_i2c_write(xavier_dev, 1, buf, 0);
      if (ret < 0) {
	printk(KERN_ERR  "Xavier : %s - failed to write : %d\n", __func__, ret);
	goto exit;
      }

      ret = xavier_i2c_read(xavier_dev, buf, XAVIER_VERSION_SIZE, &child);
      if (ret < 0){
	printk(KERN_ERR  "Xavier : %s - failed to read : %d\n", __func__, ret);
	goto exit;
      }

      if ((buf[0] >> XAVIER_CONTROL_TYPE_SHIFT) == XAVIER_CONTROL_RESET_CAUSE) {
	xavier_dev->reset_cause = buf[0] & XAVIER_CONTROL_ERROR_DATA_MASK;
      }


      /*reset data to default */
      xavier_dev->led_ena = 1;
      xavier_dev->touch_ena = 1;
      xavier_dev->ledctl = 0;

    }

  }
  else if (child == XAVIER_HEADER_TOUCH_ID) {

    printk(KERN_INFO  "Xavier : Touch event detected \n");
    /*  if touch driver has been init do the input handler */
    if (xavier_dev->input_handler != NULL && xavier_dev->touch_dev != NULL)
      xavier_dev->input_handler(xavier_dev->touch_dev, buf, ret);
  }


 exit: ;
}

static int xavier_probe(struct i2c_client * i2c, const struct i2c_device_id *id)
{
  struct xavier_dev *xavier;
  int gpio;
  int ret;

  printk(KERN_INFO "Xavier : Beginning xavier Probe\n");
  if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA
			       | I2C_FUNC_SMBUS_I2C_BLOCK)){
    ret = -EIO;
    printk(KERN_ERR "Xavier : %s - failed to create workqueue\n", __func__);
    goto error;
  }


  /* Init the Xavier mfd device */
  xavier = devm_kzalloc(&i2c->dev, sizeof(struct xavier_dev), GFP_KERNEL);
  if (!xavier){
    ret = -ENOMEM;
    printk(KERN_ERR "Xavier : %s - failed to allocate device structure \n",
	   __func__);
    goto error;
  }

  /* Search for device node in the dtb */
  if (!i2c->dev.of_node) {
    printk(KERN_ERR "Xavier : %s - No device node detected\n", __func__);
    ret = -EINVAL;
    goto error;
  }

  /* retrieve the gpio node from the dtb */
  ret = of_get_named_gpio(i2c->dev.of_node, "gpio", 0);
  if (ret < 0 ) {
    printk(KERN_ERR "Xavier : %s - No gpio node found \n", __func__);
    ret = -EINVAL;
    goto error;
  }
  gpio = ret;

  printk(KERN_INFO "xavier :GPIO node : %d\n GPIO value : %d ",
	 gpio, gpio_get_value(gpio));

  ret = gpio_request(gpio, "xavier_slave_int");
  if (ret){
    printk(KERN_ERR "Xavier : %s - Failed to request GPIO %d\n",
	   __func__, gpio);
    goto error;
  }

  ret = gpio_direction_input(gpio);
  if (ret){
    printk(KERN_ERR "Xavier : %s - Failed to set gpio direction\n",__func__);
    goto error;
  }

  xavier->slave_int = gpio;
  i2c->irq = gpio_to_irq(gpio);


  gpio_export(gpio, true);

  i2c->addr = XAVIER_SLAVE_ADDRESS;

  xavier->dev = &i2c->dev;
  xavier->i2c = i2c;
  xavier->irq = i2c->irq;

  /*request the gpio cause a reset so we wait the first reset to request it */
  xavier->reset_gpio = -1;
  xavier->read_dev = xavier_i2c_read;
  xavier->write_dev = xavier_i2c_write;


  /* Create control sysfs files */
  ret = sysfs_create_group(&xavier->dev->kobj, &xavier_attr_group);
  if (ret) {
    printk(KERN_ERR "Xavier : %s - Failed to create sysfs :%d\n",
	   __func__, ret);
    goto error;
  }

  /*Init workqueue */
  xavier->workqueue = create_workqueue("Xavier_queue");
  if (xavier->workqueue == NULL) {
    ret = -ENOMEM;
    printk(KERN_ERR "Xavier : %s - Failed to create workqueue\n", __func__);
    goto error;
  }

  INIT_WORK(&xavier->interrupt_work, xavier_interrupt_work);


  mutex_init(&xavier->lock);

  xavier->error_code = 0;
  xavier->flash = 0;
  xavier->reset = 0;
  xavier->touch_ena = 1;
  xavier->ledctl =  0;
  xavier->led_ena = 1;
  xavier->reset_cause = 0;
  strcpy(xavier->version," ");
  strcpy(xavier->cur_state, " ");

  i2c_set_clientdata(i2c, xavier);

  ret = request_irq(xavier->irq, xavier_interrupt_handler,
		    IRQ_TRIGGER, "xavier", xavier->dev);
  if (ret){
    printk(KERN_INFO "Xavier : %s - Failed to request IRQ \n", __func__);
    goto error;
  }

  /*To handle the case linux rebooted after handshake */
  queue_work(xavier->workqueue, &xavier->interrupt_work);

  return mfd_add_devices(xavier->dev, -1, xavier_devs,ARRAY_SIZE(xavier_devs),
			 NULL, 0, NULL);
 error:
  return ret;
}

static int xavier_remove(struct i2c_client *i2c)
{
  struct xavier_dev *xavier;
  printk(KERN_INFO "Xavier : Removing xavier device\n");
  xavier = i2c_get_clientdata(i2c);
  sysfs_remove_group(&xavier->dev->kobj, &xavier_attr_group);
  free_irq(xavier->irq, xavier->dev);
  flush_workqueue(xavier->workqueue);
  gpio_free(xavier->irq);
  destroy_workqueue(xavier->workqueue);
  mfd_remove_devices(xavier->dev);
  return 0;
}

static const struct i2c_device_id xavier_id[] = {
  {"xavier", XAVIER_DEVICE_ID },
  {}
};
MODULE_DEVICE_TABLE(i2c, xavier_id);

static struct of_device_id xavier_of_match[] = {
  { .compatible = "xavier"},
  { },
};
MODULE_DEVICE_TABLE(of,xavier_of_match);

static struct i2c_driver xavier_i2c_driver = {
  .driver = {
    .name	= "xavier",
    .of_match_table = of_match_ptr(xavier_of_match),
  },
  .probe		= xavier_probe,
  .remove		= xavier_remove,
  .id_table	= xavier_id,
};

static int __init xavier_init(void)
{
  printk(KERN_INFO "Xavier : Beginning xavier device initialisation\n");
  return i2c_add_driver(&xavier_i2c_driver);

}
module_init(xavier_init);

static void __exit xavier_exit(void)
{
  i2c_del_driver(&xavier_i2c_driver);
}
module_exit(xavier_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Xavier MFD core driver");
