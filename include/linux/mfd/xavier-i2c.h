/*
 * header of mfd driver for xavier board
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

#ifndef __LINUX_MFD_XAVIER_H
#define __LINUX_MFD_XAVIER_H

#define XAVIER_VERSION_SIZE 20
#define XAVIER_CUR_STATE_SIZE 35

#define XAVIER_I2C_MESSAGE_MAX_SIZE 120
#define XAVIER_I2C_NB_DATA_BYTES 20

#define XAVIER_HEADER_CONTROL_ID 0
#define XAVIER_HEADER_LED_ID 1
#define XAVIER_HEADER_TOUCH_ID 2


#define XAVIER_CONTROL_LEDCTL 0x00
#define XAVIER_CONTROL_TOUCH 0x01
#define XAVIER_CONTROL_CUR_STATE 0x02
#define XAVIER_CONTROL_VERSION 0x03
#define XAVIER_CONTROL_ERROR_CODE 0x04
#define XAVIER_CONTROL_AVAILABLE_STATE 0x05
#define XAVIER_CONTROL_RESET_CAUSE 0x06
#define XAVIER_CONTROL_LED_ENA 0x07
#define XAVIER_CONTROL_TYPE_SHIFT 5

#define XAVIER_TYPE_MASK 0xE0
#define XAVIER_CONTROL_ERROR_DATA_MASK 0x1F

#define NO_ERROR            0
#define BOOT_NOT_DONE       1
#define ERROR_EEPROM        2
#define ERROR_LED_MSG       3
#define ERROR_LED_CTRL      4
#define ERROR_TOUCH         5
#define ERROR_CTRL          6
#define ERROR_BAD_HEADER    7
#define ERROR_EEPROM_ERASED 8
#define ERROR_IPC           9
#define ERROR_LED_LENGTH    10
#define ERROR_WDT           11


struct xavier_dev {
  struct device *dev;
  struct platform_device *touch_dev;
  struct platform_device *led_dev;
  struct i2c_client *i2c;
  int (*read_dev)(struct xavier_dev *xavier, void *dest, int size, int *child);
  int (*write_dev)(struct xavier_dev *xavier, int size, const void *src, int child);
  int (*input_handler)(struct platform_device *touch_dev,char *data, int size);
  int irq;
  int slave_int;
  int reset_gpio;
  struct mutex lock;
  struct workqueue_struct *workqueue;
  struct work_struct interrupt_work;
  struct work_struct *input_work;

  /*sysfs variables */
  int boot;
  int error_code;
  int flash;
  int reset;
  int touch_ena;
  int led_ena;
  int ledctl;
  int reset_cause;
  char version[XAVIER_VERSION_SIZE];
  char cur_state[XAVIER_CUR_STATE_SIZE];
};


#endif
