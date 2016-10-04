/*
 * Led platform driver for xavier board
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
#include <linux/string.h>
#include <linux/stddef.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>

#define XAVIER_NB_LED 12
#define XAVIER_MCU_RAM_SIZE 10000

#define XAVIER_LED_ANIMATION_BUFFER_SIZE 1024
#define XAVIER_LED_IDX_MASK 0x7F
#define XAVIER_LED_FLASH_SHIFT_MASK 7
#define XAVIER_LED_NB_ANIMATION_MASK 0x7F

#define XAVIER_LED_FLASH_PATTERN_SIZE ((5 * 2 * XAVIER_NB_LED + 8) / 8)
#define XAVIER_LED_RAM_PATTERN_SIZE ((16 * 2 * XAVIER_NB_LED + 8) / 8)
#define XAVIER_LED_DELIMITERS " /n=,"
#define XAVIER_LED_PLAY "PLAY"
#define XAVIER_LED_PLAY_MONO "PLAY_MONO"
#define XAVIER_LED_STOP "STOP\n"
#define XAVIER_LED_QUEUE "QUEUE"
#define XAVIER_LED_QUEUE_MONO "QUEUE_MONO"
#define XAVIER_STATE_DATA_SIZE 24
#define XAVIER_STATE_LED_SHIFT 4
#define XAVIER_STATE_MIRROR_SHIFT 3
#define XAVIER_STATE_REVERSE_SHIFT 2
#define XAVIER_LED_PLAY_STATE_IDX 0x00
#define XAVIER_LED_PLAY_MONO_STATE_IDX 0x02
#define XAVIER_LED_STOP_STATE_IDX 0x04
#define XAVIER_LED_QUEUE_STATE_IDX 0x06
#define XAVIER_LED_QUEUE_STATE_MONO_IDX 0x08
#define XAVIER_LED_DURATION_MSG_LENGTH 4
#define XAVIER_LED_STATE_LIST_SIZE 25
#define XAVIER_DURATION_SIZE 4



#define CEILING(x, y) (((x) + (y) - 1) / (y))


struct animations {
	int nb_animation;
	int cur_animation;
	int nb_pattern;
	int cur_pattern;
	int nb_fragment;
	int data_fragment;
	int pattern_size;
	int data_size;
	char *buffer;
};

struct xavier_led {
	struct device *dev;
	struct xavier_dev *mfd;
	struct animations anims;
};

static unsigned int convert_str_hex_to_int(const char* mstr)
{
	uint8_t i = 0, start = 0;
	uint16_t res = 0;
	size_t length = strlen(mstr);

	if (mstr[0] == '0' && (mstr[1] == 'x' || mstr[1] == 'X'))
	{
		start = 2;
	}
	for (i = start ; i < length ; i++)
	{
		if (mstr[i] >= '0' && mstr[i] <= '9')
		{
			res *= 16;
			res += mstr[i] - '0';
		}
		else if (mstr[i] >= 'A' && mstr[i] <= 'F')
		{
			res *= 16;
			res += mstr[i] - 'A' + 10;
		}
		else if (mstr[i] >= 'a' && mstr[i] <= 'f')
		{
			res *= 16;
			res += mstr[i] - 'a' + 10;
		}
		else if (mstr[i] == '\0' || mstr[i] == '\n' || mstr[i] == '\r')
			return res;
		else
			return -1;
	}
	return res;
}

static ssize_t xavier_led_write(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t buf_size)
{

	int i;
	int j;
	int ret;
	int idx;
	int flash;
	int bytesleft;
	int offset;
	int memoffset;
	uint8_t nb_message;
	uint8_t nb_animation;
	const char *buf_it;
	struct xavier_dev *xavier_dev;
	struct xavier_led *xavier_led;
	struct animations *anims;

	xavier_dev = dev_get_drvdata(dev);

	if (xavier_dev == NULL) {
		ret = -EFAULT;
		dev_err(xavier_dev->dev, "%s - NULL i2c device found\n",
			__func__);
		goto exit;
	}

	xavier_led = platform_get_drvdata(xavier_dev->led_dev);
	if (xavier_led == NULL) {
		ret = -EFAULT;
		dev_err(xavier_dev->dev, "%s - NULL LED device found\n",
			__func__);
		goto exit;
	}

	anims = &xavier_led->anims;
	if (!xavier_dev->boot) {
		dev_err(xavier_dev->dev, "%s - can't send data, MCU boot is not done\n",
			__func__);
		ret = -EIO;
		goto error;
	}

	dev_dbg(xavier_dev->dev, "Sending led animations\n");
	/* ledctl need a simple twelve led data */
	if (xavier_dev->ledctl == 1) {

		if (buf_size != XAVIER_NB_LED * 2) {
			dev_err(xavier_dev->dev, "%s - Wrong data size\n",
				__func__);
			ret = -EINVAL;
			xavier_dev->error_code = ERROR_LED_MSG;
			goto error;
		}

		ret = xavier_dev->write_dev(xavier_dev, XAVIER_NB_LED * 2, buf,
					    XAVIER_HEADER_LED_ID);
		if (ret < 0) {
			ret = -EIO;
			dev_err(xavier_dev->dev, "%s - Failed to write to the device\n",
				__func__);
			goto error;
		}
	} else {
		/* Checking data structure */
		buf_it = buf;
		offset = 0;
		if (anims->data_fragment > 0) {
			offset += anims->data_fragment;
			buf_it += offset;
			anims->cur_pattern++;
		} else {

			/* First byte is the animation number */
			nb_animation = buf_it[0] & XAVIER_LED_NB_ANIMATION_MASK;
			dev_dbg(xavier_dev->dev, "%d animation in the file\n",
				nb_animation);
			flash = buf_it[0] >> XAVIER_LED_FLASH_SHIFT_MASK;

			if (flash)
				anims->pattern_size = XAVIER_LED_FLASH_PATTERN_SIZE;
			else
				anims->pattern_size = XAVIER_LED_RAM_PATTERN_SIZE;

			/* Initiate animation structure */

			anims->nb_animation = nb_animation;
			anims->cur_animation = 0;
			anims->nb_pattern = 0;
			anims->cur_pattern = 0;
			anims->nb_fragment = 0;
			anims->data_fragment = 0;
			anims->data_size = 0;
			anims->buffer = NULL;

			buf_it++;
			offset++;
		}
		/* Check every animation to be sure that the file
		 * correspond to a led animation file
		 */
		for (i = anims->cur_animation; i < anims->nb_animation; i++) {

			if (buf_it == NULL) {
				dev_err(xavier_dev->dev, "%s - Wrong binary data formating, NULL pointer found\n",
					__func__);
				ret = -EINVAL;
				xavier_dev->error_code = ERROR_LED_MSG;
				goto error;
			}

			if (anims->cur_pattern == 0) {
				/* if there is not enough space for the begining
				 * of an animation (idx + nb_pattern + first pattern)
				 * copy everything and leave the loop
				 */
				if (offset + (2 + anims->pattern_size) > PAGE_SIZE) {
					if (anims->buffer == NULL)
						anims->buffer = kzalloc(XAVIER_MCU_RAM_SIZE, GFP_KERNEL);
					memoffset = anims->nb_fragment * PAGE_SIZE;
					memcpy(&anims->buffer[memoffset], buf, PAGE_SIZE);
					/* compute the fragment of pattern at the end of the buffer */
					anims->data_fragment = anims->pattern_size - (offset - PAGE_SIZE);
					anims->cur_animation = i;
					anims->nb_pattern = buf_it[1];
					anims->cur_pattern = 0;
					anims->nb_fragment++;
					ret = buf_size;
					goto exit;
				}

				idx = buf_it[0] & XAVIER_LED_IDX_MASK;

				/*idx must be in the right order allowing easier structure checking */
				if (idx != i) {
					dev_err(xavier_dev->dev, "%s - Wrong binary data formating : found %d needed %d\n",
						__func__, idx, i);
					ret = -EINVAL;
					xavier_dev->error_code = ERROR_LED_MSG;
					goto error;
				}
				buf_it++;
				offset++;

				/*check for the number of pattern and skip them */
				anims->nb_pattern = buf_it[0];
				anims->data_size += 2 + (anims->nb_pattern * anims->pattern_size);
			}

			for (j = anims->cur_pattern; j < anims->nb_pattern; j++) {

				dev_dbg(xavier_dev->dev, "Pattern %d : %*ph\n", j,
					anims->pattern_size, buf_it + 1);

				/* if there is not enough space for a pattern
				 * copy everything and leave the loop
				 */
				if (offset + anims->pattern_size > PAGE_SIZE) {
					if (anims->buffer == NULL)
						anims->buffer = kzalloc(XAVIER_MCU_RAM_SIZE, GFP_KERNEL);
					memoffset = anims->nb_fragment * PAGE_SIZE;

					if (memoffset + PAGE_SIZE > XAVIER_MCU_RAM_SIZE) {
						dev_err(xavier_dev->dev, "%s - Data to big for MCU RAM %lu > %d\n",
							__func__, memoffset + PAGE_SIZE, XAVIER_MCU_RAM_SIZE);
						ret = -EINVAL;
						xavier_dev->error_code = ERROR_LED_MSG;
						goto error;
					}
					memcpy(&anims->buffer[memoffset], buf, PAGE_SIZE);
					/* compute the fragment of pattern at the end of the buffer */
					anims->data_fragment = anims->pattern_size - (PAGE_SIZE - offset);
					anims->cur_animation = i;
					anims->nb_pattern = anims->nb_pattern;
					anims->cur_pattern = j;
					anims->nb_fragment++;
					ret = buf_size;
					goto exit;
				}
				buf_it += anims->pattern_size;
				offset += anims->pattern_size;
			}
			buf_it++;
			offset++;

			if (anims->buffer == NULL)
				anims->buffer = kzalloc(XAVIER_MCU_RAM_SIZE, GFP_KERNEL);
			memoffset = anims->nb_fragment * PAGE_SIZE;
			if (memoffset + buf_size > XAVIER_MCU_RAM_SIZE) {
				dev_err(xavier_dev->dev, "%s - Data to big for MCU RAM %lu > %d\n",
					__func__, memoffset + PAGE_SIZE, XAVIER_MCU_RAM_SIZE);
				ret = -EINVAL;
				xavier_dev->error_code = ERROR_LED_MSG;
				goto error;
			}
			memcpy(&anims->buffer[memoffset], buf, buf_size);

			/* Set data for the next animation */
			anims->cur_animation = i + 1;
			anims->cur_pattern = 0;
		}

		/*compute the number of message necessary for the data */
		memoffset = anims->nb_fragment * PAGE_SIZE + buf_size - 1;
		if (anims->data_size !=  memoffset) {
			dev_err(xavier_dev->dev, "%s - File size mismatch : found %d should be %d\n", __func__, memoffset, anims->data_size);
			ret = -EINVAL;
			xavier_dev->error_code = ERROR_LED_MSG;
			goto error;
		}

		nb_message = CEILING(anims->nb_fragment * PAGE_SIZE + buf_size,
				     XAVIER_I2C_MESSAGE_MAX_SIZE);

		/* The first message is the number of message to be sent */
		ret = xavier_dev->write_dev(xavier_dev, sizeof(uint8_t), &nb_message,
					    XAVIER_HEADER_LED_ID);
		if (ret < 0) {
			ret = -EIO;
			dev_err(xavier_dev->dev, "%s - Failed to send to device\n", __func__);
			goto error;
		}

		bytesleft = anims->nb_fragment * PAGE_SIZE + buf_size;

		for (i = 0; i < nb_message; i++) {

			buf_it = anims->buffer + (i * XAVIER_I2C_MESSAGE_MAX_SIZE);
			if (bytesleft < XAVIER_I2C_MESSAGE_MAX_SIZE) {
				ret = xavier_dev->write_dev(xavier_dev, bytesleft,
							     buf_it, XAVIER_HEADER_LED_ID);
			} else {
				ret = xavier_dev->write_dev(xavier_dev, XAVIER_I2C_MESSAGE_MAX_SIZE,
							     buf_it, XAVIER_HEADER_LED_ID);
			}
			if (ret < 0) {
				ret = -EIO;
				dev_err(xavier_dev->dev, "%s - Failed to send to device\n", __func__);
				goto error;
			}
			bytesleft -= XAVIER_I2C_MESSAGE_MAX_SIZE;
		}
	}
	dev_info(xavier_dev->dev, "Animation sucessfully sent\n");
	ret = buf_size;

error:
	if (anims->buffer != NULL) {
		kfree(anims->buffer);
		anims->buffer = NULL;
	}
	anims->nb_animation = 0;
	anims->cur_animation = 0;
	anims->nb_pattern = 0;
	anims->cur_pattern = 0;
	anims->nb_fragment = 0;
	anims->data_fragment = 0;
	anims->pattern_size = 0;
exit:
	return ret;
}

static char *xavier_new_token_data(char *msg, unsigned int *data)
{

	char *token;
	int ret;

	token = strsep(&msg, XAVIER_LED_DELIMITERS);
	if (token == NULL) {
		pr_err("Xavier : %s - Null token found\n", __func__);
		goto error;
	}

	ret = kstrtoint(token, 0, data);
	if (ret) {
		pr_err("Xavier : %s - Failed to transform data\n", __func__);
		goto error;
	}
	return msg;

error:
	return NULL;
}

static ssize_t xavier_cur_state_store(struct device *dev,
				      struct device_attribute *attr,
				      const  char *buf, size_t count)
{

	struct xavier_dev *xavier_dev;
	char *token, *msg, *memmsg, data[XAVIER_STATE_DATA_SIZE], *temp;
	unsigned int sequence, loop, red = 0, green, blue, shift, mirror, reverse;
	int ret;

	ret = 0;
	xavier_dev = dev_get_drvdata(dev);
	if (xavier_dev == NULL) {
		ret = -EFAULT;
		dev_err(xavier_dev->dev, "%s - NULL i2c device found\n", __func__);
		goto exit;
	}

	/*Check if MCU is booted and Led are enabled */
	if (xavier_dev->led_ena && xavier_dev->boot) {

		memset(data, 0, XAVIER_STATE_DATA_SIZE);

		/* copy the buf to work with it
		 * msg is the working pointer
		 * memmsg is the memory pointer for the kfree
		 */
		msg = memmsg = kstrdup(buf, GFP_KERNEL);

		/*Parse the receive command */
		token = strsep(&msg, XAVIER_LED_DELIMITERS);
		if (token == NULL) {
			ret = -EINVAL;
			dev_err(xavier_dev->dev, "%s - NULL token found\n", __func__);
			goto error;
		}

		/*If the command is a play command parse the 2 needed argument */
		if (!strcmp(token, XAVIER_LED_PLAY) || !strcmp(token, XAVIER_LED_QUEUE)) {

			msg = xavier_new_token_data(msg, &sequence);
			if (msg == NULL) {
				dev_err(xavier_dev->dev, "%s - NULL token found (index)\n", __func__);
				ret = -EINVAL;
				goto error;
			}

			msg = xavier_new_token_data(msg, &loop);
			if (msg == NULL) {
				dev_err(xavier_dev->dev, "%s - NULL token found (loop count)\n", __func__);
				ret = -EINVAL;
				goto error;
			}

			msg = xavier_new_token_data(msg, &shift);
			if (msg == NULL) {
				dev_err(xavier_dev->dev, "%s - NULL token found (shift)\n", __func__);
				ret = -EINVAL;
				goto error;
			}

			msg = xavier_new_token_data(msg, &mirror);
			if (msg == NULL) {
				dev_err(xavier_dev->dev, "%s - NULL token found (mirror)\n", __func__);
				ret = -EINVAL;
				goto error;
			}

			msg = xavier_new_token_data(msg, &reverse);
			if (msg != NULL) { /* must be the last token */
				dev_err(xavier_dev->dev, "%s - NULL token found\n", __func__);
				ret = -EINVAL;
				goto error;
			}

			/*Send the formated command to the MCU */
			data[0] = XAVIER_CONTROL_CUR_STATE << XAVIER_CONTROL_TYPE_SHIFT; /* 0xE0(3bits) for idx */

			if (!strcmp(token, XAVIER_LED_PLAY))
				data[0] += XAVIER_LED_PLAY_STATE_IDX; /*  0x18 (2bits) for play*/
			else
				data[0] += XAVIER_LED_QUEUE_STATE_IDX; /*  0x18 (2bits) for play*/

			data[1] |= sequence;
			data[2] |= loop;
			data[3] |= shift << XAVIER_STATE_LED_SHIFT;
			data[3] |= mirror << XAVIER_STATE_MIRROR_SHIFT;
			data[3] |= reverse << XAVIER_STATE_REVERSE_SHIFT;

			ret = xavier_dev->write_dev(xavier_dev, 4,
						    data, XAVIER_HEADER_CONTROL_ID);
			if (ret < 0) {
				ret = -EIO;
				dev_err(xavier_dev->dev, "%s - Failed to write to the device\n",
					__func__);
				goto error;
			}
		}

		/*if Play_mono command parse the needed argument */
		else if (!strcmp(token, XAVIER_LED_PLAY_MONO) || !strcmp(token, XAVIER_LED_QUEUE_MONO)) {

			msg = xavier_new_token_data(msg, &sequence);
			if (msg == NULL) {
				ret = -EINVAL;
				dev_err(xavier_dev->dev, "%s - NULL token found (sequence)\n",
				       __func__);
				goto error;
			}

			temp = strsep(&msg, XAVIER_LED_DELIMITERS);
			if (token == NULL) {
				dev_err(xavier_dev->dev, "%s - Null token found\n", __func__);
				goto error;
			}

			if (temp[1] == 'x' || temp[1] == 'X') {
				red = convert_str_hex_to_int(temp);
				green = (red & 0x03E0) >> 5;
				blue = red & 0x1F;
				red = red >> 10;

			} else {

				ret = kstrtoint(temp, 0, &red);
				if (ret) {
					dev_err(xavier_dev->dev, "%s - NULL token found (red)\n", __func__);
					goto error;
				}

				msg = xavier_new_token_data(msg, &green);
				if (msg == NULL) {
					ret = -EINVAL;
					dev_err(xavier_dev->dev, "%s - NULL token found (green)\n", __func__);
					goto error;
				}

				msg = xavier_new_token_data(msg, &blue);
				if (msg == NULL) {
					ret = -EINVAL;
					dev_err(xavier_dev->dev, "%s - NULL token found (blue)\n", __func__);
					goto error;
				}
			}

			msg = xavier_new_token_data(msg, &loop);
			if (msg == NULL) {
				ret = -EINVAL;
				dev_err(xavier_dev->dev, "%s - NULL token found (loop)\n", __func__);
				goto error;
			}

			msg = xavier_new_token_data(msg, &shift);
			if (msg == NULL) {
				dev_err(xavier_dev->dev, "%s - NULL token found (shift)\n", __func__);
				ret = -EINVAL;
				goto error;
			}

			msg = xavier_new_token_data(msg, &mirror);
			if (msg == NULL) {
				dev_err(xavier_dev->dev, "%s - NULL token found (mirror)\n", __func__);
				ret = -EINVAL;
				goto error;
			}

			msg = xavier_new_token_data(msg, &reverse);
			if (msg != NULL) { /* must be the last token */
				dev_err(xavier_dev->dev, "%s - To many argument detected\n", __func__);
				ret = -EINVAL;
				goto error;
			}

			/*Send the formated command to the MCU */
			data[0] = XAVIER_CONTROL_CUR_STATE << XAVIER_CONTROL_TYPE_SHIFT; /* 0xE0(3bits) for idx */

			if (!strcmp(token, XAVIER_LED_PLAY_MONO))
				data[0] += XAVIER_LED_PLAY_MONO_STATE_IDX;
			else
				data[0] += XAVIER_LED_QUEUE_STATE_MONO_IDX;

			data[1] |= sequence;
			data[2] |= (red << 2);
			data[2] |= (green >> 3);
			data[3] |= (green << 5);
			data[3] |= blue;
			data[4] |= loop;
			data[5] |= shift << XAVIER_STATE_LED_SHIFT;
			data[5] |= mirror << XAVIER_STATE_MIRROR_SHIFT;
			data[5] |= reverse << XAVIER_STATE_REVERSE_SHIFT;

			ret = xavier_dev->write_dev(xavier_dev, 6, data,
						    XAVIER_HEADER_CONTROL_ID);
			if (ret < 0) {
				ret = -EIO;

				goto error;
			}
		} else if (!strcmp(token, XAVIER_LED_STOP)) {

			data[0] = XAVIER_CONTROL_CUR_STATE << XAVIER_CONTROL_TYPE_SHIFT; /* 0xE0(3bits) for idx */
			data[0] += XAVIER_LED_STOP_STATE_IDX; /* 0x18 (2bits) for stop */
			ret = xavier_dev->write_dev(xavier_dev, 1,
						    data, XAVIER_HEADER_CONTROL_ID);
			if (ret < 0) {
				ret = -EIO;
				dev_err(xavier_dev->dev, "%s - Failed to write to the device\n",
					__func__);
				goto error;
			}
		} else {
			dev_err(xavier_dev->dev, "%s - Wrong argument detected\n", __func__);
			ret = -EINVAL;
			goto error;
		}

		strcpy(xavier_dev->cur_state, buf);
		return count;

error:
		kfree(memmsg);

	}

	return count;
exit:
	return ret;
}

static ssize_t xavier_cur_state_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{

	int ret;
	struct xavier_dev *xavier_dev;

	xavier_dev = dev_get_drvdata(dev);
	if (xavier_dev == NULL) {
		ret = -EFAULT;
		dev_err(xavier_dev->dev, "%s - NULL i2c device found\n", __func__);
		goto exit;
	}

	ret = sprintf(buf, "%s\n", xavier_dev->cur_state);

exit:
	return ret;
}

static ssize_t xavier_ledctl_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{

	int ret;
	struct xavier_dev *xavier_dev;

	xavier_dev = dev_get_drvdata(dev);
	if (xavier_dev == NULL) {
		ret = -EFAULT;
		dev_err(xavier_dev->dev, "%s - NULL i2c device found\n",
			__func__);
		goto exit;
	}

	ret = sprintf(buf, "%d\n", xavier_dev->ledctl);

exit:
	return ret;
}

static ssize_t xavier_led_ena_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{

	int ret;
	struct xavier_dev *xavier_dev;

	xavier_dev = dev_get_drvdata(dev);
	if (xavier_dev == NULL) {
		ret = -EFAULT;
		dev_err(xavier_dev->dev, "%s - NULL i2c device found\n", __func__);
		goto exit;
	}

	ret = sprintf(buf, "%d\n", xavier_dev->led_ena);

exit:
	return ret;
}

static ssize_t xavier_available_state_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{

	int i, j, k, idx, ret, nb_idx, nb_message, child, buf_size;
	unsigned int duration;
	struct xavier_dev *xavier_dev  = dev_get_drvdata(dev);
	char data[XAVIER_I2C_NB_DATA_BYTES], *animation_buffer, *buf_it, temp[50];

	if (xavier_dev == NULL) {
		ret = -EFAULT;
		dev_err(xavier_dev->dev, "%s - NULL i2c device found\n", __func__);
		goto exit;
	}

	if (xavier_dev->boot == 0) {
		ret = -1;
		dev_err(xavier_dev->dev, "%s - available_state access impossible - boot not done\n",
			__func__);
		goto exit;
	}

	/*Ask the MCU for the Flash animation */
	data[0] = XAVIER_CONTROL_AVAILABLE_STATE << XAVIER_CONTROL_TYPE_SHIFT;
	ret = xavier_dev->write_dev(xavier_dev, 1, &data, 0);
	if (ret < 0) {
		ret = -EIO;
		dev_err(xavier_dev->dev, "%s - Failed to write to the device\n", __func__);
		goto exit;
	}

	/*first read give the number of index to be retrieve */
	ret = xavier_dev->read_dev(xavier_dev, &data,
				   XAVIER_I2C_NB_DATA_BYTES, &child);

	if (ret <= 0 || child != XAVIER_HEADER_CONTROL_ID) {
		ret = -EIO;
		dev_err(xavier_dev->dev, "%s - Failed to read the device\n", __func__);
		goto exit;
	}

	if ((data[0] & XAVIER_TYPE_MASK) == XAVIER_CONTROL_ERROR_CODE) {
		xavier_dev->error_code = (data[0] &
					  XAVIER_CONTROL_ERROR_DATA_MASK);
		ret = -EIO;
		dev_err(xavier_dev->dev, "%s - Error code received\n", __func__);
		goto exit;
	}

	nb_idx = data[0];
	nb_message = CEILING(5 * nb_idx, XAVIER_I2C_MESSAGE_MAX_SIZE);

	buf_size = nb_message * XAVIER_I2C_MESSAGE_MAX_SIZE;
	animation_buffer = kzalloc(buf_size, GFP_KERNEL);

	if (animation_buffer == NULL) {
		ret = -ENOMEM;
		dev_err(xavier_dev->dev, "%s - Failed to allocate animation buffer\n",
			__func__);
		goto exit;
	}

	for (j = 0; j < nb_message; j++) {

		ret = xavier_dev->read_dev(xavier_dev, animation_buffer, buf_size, &child);
		if (ret <= 0 || child != XAVIER_HEADER_CONTROL_ID) {
			ret = -EIO;
			dev_err(xavier_dev->dev, "%s - Failed to read the device\n", __func__);
			goto error;
		}

		if ((animation_buffer[0] & XAVIER_TYPE_MASK) == XAVIER_CONTROL_ERROR_CODE) {
			xavier_dev->error_code = (animation_buffer[0] &
						  XAVIER_CONTROL_ERROR_DATA_MASK);
			ret = -EIO;
			dev_err(xavier_dev->dev, "%s - Error code received\n", __func__);
			goto error;
		}

		buf_it = animation_buffer;
		sprintf(temp, "Current animation (duration in 10 ms steps) :\n");
		strcat(buf, temp);
		/*Parse and store the reply */
		for (i = 0; i < nb_idx; i++) {

			idx = buf_it[0];

			if (idx < nb_idx) {
				sprintf(temp, "idx = %d ", idx);
				strcat(buf, temp);
				duration = 0;

				/* the duration is in 32 bits word */
				for (k = 1; k <= XAVIER_DURATION_SIZE; k++)
					duration += buf_it[k] << (XAVIER_DURATION_SIZE - k) * 8;

				sprintf(temp, "duration = %d\n", duration);
				strcat(buf, temp);
				buf_it += XAVIER_LED_DURATION_MSG_LENGTH + 1;
			}
		}
	}

	kfree(animation_buffer);
	return strlen(buf);

error:
	kfree(animation_buffer);
exit:
	return ret;
}

static ssize_t xavier_led_ena_store(struct device *dev,
				    struct device_attribute *attr,
				    const  char *buf, size_t count)
{

	char data;
	int ret;
	int temp;
	struct xavier_dev *xavier_dev = dev_get_drvdata(dev);

	if (xavier_dev == NULL) {
		ret = -EFAULT;
		dev_err(xavier_dev->dev, "%s - NULL i2c device found\n", __func__);
		goto exit;
	}


	ret = kstrtoint(buf, 0, &temp);
	if (ret) {
		dev_err(xavier_dev->dev, "%s - Failed to transform data\n", __func__);
		goto exit;
	}

	data = XAVIER_CONTROL_LED_ENA << XAVIER_CONTROL_TYPE_SHIFT;
	data += temp << (XAVIER_CONTROL_TYPE_SHIFT - 1); /*3 bits for control idx (0) then 1 bit for the value */

	if (temp != 0 &&  temp != 1) {
		ret = -EINVAL;
		dev_err(xavier_dev->dev, "%s - Wrong value must be 1 or 0 : found %d\n",
			__func__, temp);
		goto exit;
	}

	xavier_dev->led_ena = temp;

	/*write the new value to MCU to make it ready */
	ret = xavier_dev->write_dev(xavier_dev, 1, &data, 0);
	if (ret < 0) {
		ret = -EIO;
		dev_err(xavier_dev->dev, "%s - Failed to write to the device\n", __func__);
		goto exit;
	}

	return count;

exit:
	return ret;
}

static ssize_t xavier_ledctl_store(struct device *dev,
				   struct device_attribute *attr,
				   const  char *buf, size_t count)
{

	char data;
	int ret;
	int temp;

	struct xavier_dev *xavier_dev = dev_get_drvdata(dev);

	if (xavier_dev == NULL) {
		ret = -EFAULT;
		dev_err(xavier_dev->dev, "%s - NULL i2c device found\n", __func__);
		goto exit;
	}

	ret = kstrtoint(buf, 0, &temp);
	if (ret) {
		dev_err(xavier_dev->dev, "%s - Failed to transform data\n", __func__);
		goto exit;
	}

	if (temp != 0 &&  temp != 1) {
		ret = -EINVAL;
		dev_err(xavier_dev->dev, "%s - Wrong value must be 1 or 0 : found %d\n",
		       __func__, temp);
		goto exit;
	}

	data = temp << (XAVIER_CONTROL_TYPE_SHIFT - 1);

	xavier_dev->ledctl = temp;
	xavier_dev->error_code = 0;

	/*write the new value to MCU to make it ready */
	ret = xavier_dev->write_dev(xavier_dev, 1, &data, 0);
	if (ret < 0) {
		ret = -EIO;
		dev_err(xavier_dev->dev, "%s - Failed to write to the device\n", __func__);
		goto exit;
	}

	return count;

exit:
	return ret;
}


static ssize_t xavier_bright_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{

	int ret;
	struct xavier_dev *xavier_dev;

	xavier_dev = dev_get_drvdata(dev);
	if (xavier_dev == NULL) {
		ret = -EFAULT;
		dev_err(xavier_dev->dev, "%s - NULL i2c device found\n", __func__);
		goto exit;
	}

	ret = sprintf(buf, "%d %d\n", xavier_dev->bright, xavier_dev->bright_dur);

exit:
	return ret;
}


static ssize_t xavier_bright_store(struct device *dev,
				   struct device_attribute *attr,
				   const  char *buf, size_t count)
{

	struct xavier_dev *xavier_dev;
	char *msg, *memmsg, data[XAVIER_STATE_DATA_SIZE];
	int ret, bright, duration;

	ret = 0;
	xavier_dev = dev_get_drvdata(dev);
	if (xavier_dev == NULL) {
		ret = -EFAULT;
		dev_err(xavier_dev->dev, "%s - NULL i2c device found\n", __func__);
		goto exit;
	}

	/*Check if MCU is booted and Led are enabled */
	if (xavier_dev->led_ena && xavier_dev->boot) {

		memset(data, 0, XAVIER_STATE_DATA_SIZE);

		/* copy the buf to work with it
		 * msg is the working pointer
		 * memmsg is the memory pointer for the kfree
		 */
		msg = memmsg = kstrdup(buf, GFP_KERNEL);

		msg = xavier_new_token_data(msg, &bright);
		if (msg == NULL) {
			ret = -EINVAL;
			dev_err(xavier_dev->dev, "%s - NULL token found (bright)\n", __func__);
			goto error;
		}

		msg = xavier_new_token_data(msg, &duration);

		/*Send the formated command to the MCU */
		data[0] = XAVIER_CONTROL_BRIGHT << XAVIER_CONTROL_TYPE_SHIFT; /* 0xE0(3bits) for idx */
		data[1] |= bright;
		data[2] |= duration;

		ret = xavier_dev->write_dev(xavier_dev, 3,
					    data, XAVIER_HEADER_CONTROL_ID);
		if (ret < 0) {
			ret = -EIO;
			dev_err(xavier_dev->dev, "%s - Failed to write to the device\n",
				__func__);
				goto error;
		}
	kfree(memmsg);
	}

	return count;
error:
	kfree(memmsg);
exit:
	return ret;
}

static DEVICE_ATTR(cur_state, S_IWUSR | S_IRUSR, xavier_cur_state_show,
		   xavier_cur_state_store);
static DEVICE_ATTR(ledctl, S_IWUSR | S_IRUSR, xavier_ledctl_show,
		   xavier_ledctl_store);
static DEVICE_ATTR(LED_ena, S_IWUSR | S_IRUSR, xavier_led_ena_show,
		   xavier_led_ena_store);
static DEVICE_ATTR(available_state, S_IRUSR, xavier_available_state_show,
		   NULL);
static DEVICE_ATTR(send_state, S_IWUSR | S_IRUSR, NULL,
		   xavier_led_write);
static DEVICE_ATTR(bright, S_IWUSR | S_IRUSR, xavier_bright_show,
		   xavier_bright_store);

static struct attribute *xavier_attrs[] = {
	&dev_attr_cur_state.attr,
	&dev_attr_ledctl.attr,
	&dev_attr_LED_ena.attr,
	&dev_attr_available_state.attr,
	&dev_attr_send_state.attr,
	&dev_attr_bright.attr,
	NULL
};

static struct attribute_group xavier_attr_group = {
	.attrs = xavier_attrs,
};

static int xavier_led_probe(struct platform_device *pdev)
{

	struct xavier_dev *xavier_dev;
	struct xavier_led *xavier_led;
	int ret;

	xavier_dev = dev_get_drvdata(pdev->dev.parent);
	dev_info(xavier_dev->dev, "Xavier LED started\n");
	xavier_led = kzalloc(sizeof(struct xavier_led), GFP_KERNEL);
	if (!xavier_led) {
		ret = -ENOMEM;
		dev_err(xavier_dev->dev, "%s - Failed to allocate animation buffer",
			__func__);
		goto error;
	}

	xavier_led->mfd = xavier_dev;
	xavier_led->dev = &pdev->dev;
	xavier_dev->led_dev = pdev;

	ret = sysfs_create_group(&xavier_led->mfd->dev->kobj, &xavier_attr_group);
	if (ret) {
		dev_err(xavier_dev->dev, "%s - Failed to create sysfs entries\n", __func__);
		goto exit;
	}

	platform_set_drvdata(pdev, xavier_led);

	return 0;

exit:
	kfree(xavier_led);
error:
	return ret;
}

static int xavier_led_remove(struct platform_device *pdev)
{

	struct xavier_led *xavier_led = platform_get_drvdata(pdev);

	sysfs_remove_group(&xavier_led->mfd->dev->kobj, &xavier_attr_group);
	kfree(xavier_led);
	return 0;
}

static struct platform_driver xavier_led_driver = {

	.driver = {
		.name = "Xavier-led",
	},
	.probe = xavier_led_probe,
	.remove = xavier_led_remove,

};
module_platform_driver(xavier_led_driver);

MODULE_DESCRIPTION("Xavier - LED driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:Xavier-led");
