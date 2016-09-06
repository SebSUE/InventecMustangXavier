/*
 *  BSD LICENSE
 *
 *  Copyright(c) 2014 Broadcom Corporation.  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *    * Neither the name of Broadcom Corporation nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _BCM_CYGNUS_M0_IPC_H_
#define _BCM_CYGNUS_M0_IPC_H_

#define M0_IPC_CMD_DONE_MASK    0x80000000
#define M0_IPC_CMD_REPLY_MASK   0x3fff0000
#define M0_IPC_CMD_REPLY_SHIFT  16

enum m0_pm_state {
	M0_IPC_PM_STATE_ACTIVE,
	M0_IPC_PM_STATE_SLEEP,
	M0_IPC_PM_STATE_DEEPSLEEP,
	M0_IPC_PM_STATE_OFF,
};

/**************************************
 * IPROC to M0 commands and parameters.
 *************************************/
enum cygnus_m0_cmd_param {
	/*
	 * Reset request
	 * Param - must be 0xffffffff
	 * Response - return code
	 * Note - defined as 0 and requires param 0xffffffff to be compatible
	 * with M0 bootrom code. This allows boot1 and kernel to issue the same
	 * M0 message for IPROC soft reset both with or without custom M0 patch
	 * installed.
	 *
	 * For now, this command is an alias of L3 reset (is identical to
	 * M0_IPC_CMD_L3_RESET).
	 */
	M0_IPC_M0_CMD_IPROC_RESET =                0,
	M0_IPC_M0_CMD_IPROC_RESET_PARAM =          0xffffffff,

	/*
	 * Initialization command to M0.
	 *  Needs to be sent from host after installing M0 patch.
	 *  Starts periodic activities, WDOG, etc.
	 * Param - none
	 * Response - return code
	 */
	M0_IPC_M0_CMD_INIT =                       0x1,

	/*
	 * No-op command
	 *  does nothing, returns 0 return code
	 * Param - none
	 * Response - return code
	 */
	M0_IPC_M0_CMD_NOP =                        0x2,

	/*
	 * Run SW iProc reset sequence (L3 reset)
	 * Param - none
	 * Response - return code
	 */
	M0_IPC_M0_CMD_L3_RESET =                   0x3,

	/*
	 * Issue chip L1 reset
	 * Param - none
	 * Response - none (M0 will reset as a result of this command)
	 */
	M0_IPC_M0_CMD_L1_RESET =                   0x4,

	/*
	 * Issue chip L0 reset
	 * Param - none
	 * Response - none (M0 will reset as a result of this command)
	 */
	M0_IPC_M0_CMD_L0_RESET =                   0x5,

	/*
	 * Enter OFF state
	 * Param - physical address to resume execution from upon power saving
	 *   mode exit.
	 * Response - none (M0 will power the system off as a result of this
	 *   command)
	 */
	M0_IPC_M0_CMD_ENTER_OFF =                  0xa,

	/*
	 * Enter DEEPSLEEP state
	 * Param - physical address to resume execution from upon power saving
	 *   mode exit.
	 * Response - return code
	 */
	M0_IPC_M0_CMD_ENTER_DEEPSLEEP =            0xb,

	/*
	 * Enter SLEEP state
	 * Param - physical address to resume execution from upon power saving
	 *   mode exit.
	 * Response - return code
	 */
	M0_IPC_M0_CMD_ENTER_SLEEP =                0xc,

	/*
	 * Issue chip Warm reset
	 * Param - none
	 * Response - none (A9 will reset as a result of this command)
	 */
	M0_IPC_M0_CMD_WARM_RESET =                 0xd,

	/*
	 * Enable/disable GPIO event forwarding from M0 to A9
	 * Param - 1 to enable, 0 to disable
	 * Response - return code
	 */
	M0_IPC_M0_CMD_AON_GPIO_FORWARDING_ENABLE = 0xe,

	/*
	 * Get M0 uptime
	 *
	 * Param - physical address of m0_ipc_m0_cmd_get_uptime struct,
	 *         where m0 will store uptime value
	 */
	M0_IPC_M0_CMD_GET_UPTIME =                 0xf,

	/*
	 * Set delay for timed wakeup from sleep, deepsleep and off states.
	 * Upon entering sleep, deepsleep or off state, M0 will setup a wakeup
	 * timer with the configured delay.
	 * Delay is given in msec; setting to 0 disables timed wakeup.
	 *
	 * Param - delay in msec
	 */
	M0_IPC_M0_CMD_SET_TIMER_WAKEUP_DELAY =     0x10,

	/*
	 * Get M0 version
	 *
	 * Param - physical address of m0_ipc_m0_cmd_get_version struct,
	 *         where m0 will store version string
	 */
	M0_IPC_M0_CMD_GET_VERSION =                0x11,

	/* Memory map config
	 * Param - physical address of m0_ipc_m0_cmd_memory_cfg struct
	 * Response - return code
	 */
	M0_IPC_M0_CMD_DRAM_MAP_CFG =               0x12,

	/*
	 * Configuration of AON GPIO that controls power rails voltage regulator
	 * Param - m0_ipc_m0_cmd_power_rail_regulator_cfg
	 * Response - return code
	 */
	M0_IPC_M0_CMD_POWER_RAIL_REGULATOR_CFG =   0x13,

	/*
	 * Configuration of AON GPIO wakeup parameters
	 * Param - physical address of m0_ipc_m0_cmd_aon_gpio_wakeup_cfg
	 *  struct
	 */
	M0_IPC_M0_CMD_AON_GPIO_WAKEUP_CFG =        0x14,

	/*
	 * Enable wakeup on a particular AON GPIO
	 * Param - GPIO number
	 */
	M0_IPC_M0_CMD_AON_GPIO_WAKEUP_GPIO_ENABLE_CFG =        0x15,

	/*
	 * Disable wakeup on a prticular AON GPIO
	 * Param - GPIO number
	 */
	M0_IPC_M0_CMD_AON_GPIO_WAKEUP_GPIO_DISABLE_CFG =        0x16,

#ifdef CONFIG_M0_REG_ACCESS_IPC
	/*
	 * Reg read request - request from M0 to read register (debug only)
	 *  M0 reads a reg location, and issues M0-to-IPROC M0_IPC_CMD_NOP
	 *  message with read value as param.
	 * Param - Addr to read
	 * Response - return code;
	 */
	M0_IPC_M0_CMD_REG_READ_SEC =               0xff,
	M0_IPC_M0_CMD_REG_READ_NS  =               0xfe,
#endif

	/*
	 * Profiling commands.
	 * Start/stop one of statically allocated profilers defined on M0 side.
	 * Upon start command, current time is recorded in m0 statistics
	 * structure. Upon stop command, difference between current time and
	 * the recorded time is saved in m0 statistics structure.
	 *  Param: profiler id
	 *  Response - return code
	 */
	M0_IPC_M0_CMD_PROFILER_START =             0xfd,
	M0_IPC_M0_CMD_PROFILER_STOP  =             0xfc,

	/***********************************************
	 * M0 to IPROC messages
	 *
	 * All messages are async events without reply
	 ***********************************************/

	/*
	 * No-op message
	 * Normally used to clear (zero out) mailbox regs
	 *
	 * Param - don't care
	 */
	M0_IPC_HOST_CMD_NOP =                      0x100,

	/*
	 * Standby wakeup event
	 * Triggered by M0 to wakeup A9 during STANDBY system state,
	 * following a trigger from one of standard M0 wakeup sources
	 *
	 * Param - none
	 */
	M0_IPC_HOST_CMD_WAKEUP =                   0x101,

	/*
	 * AON GPIO interrupt ("forwarded" to IPROC)
	 * Param - AON GPIO mask
	 */
	M0_IPC_HOST_CMD_AON_GPIO_EVENT =           0x102,

};

struct m0_ipc_m0_cmd_power_rail_regulator_cfg {
	/* AON GPIO number */
	uint32_t gpio;
	/*
	 * 1 if voltage regulator controlled by this GPIO is active high;
	 * 0 if it is active low
	 */
	uint32_t enable_active_high;
	/* Enable ramp up time in usec */
	uint32_t startup_delay_usec;
	/*
	 * Mask of PM states in which this regulator should be disabled.
	 * Bit M0_IPC_PM_STATE_xyz is 1 iff this regulator should be in disabled
	 *  state in PM state xyz
	 */
	uint32_t disabled_in_pm_states_mask;
};

struct m0_ipc_m0_cmd_dram_map_cfg {
	/* Physical address of secure DRAm area reserved for PM */
	uint32_t sec_dram_area_for_pm_base_addr;
	/* Size of secure DRAm area reserved for PM in bytes */
	uint32_t sec_dram_area_for_pm_size;
};

struct m0_ipc_m0_cmd_get_version {
	char version_string[256];
};

struct m0_ipc_m0_cmd_get_uptime {
	uint32_t sec;
	uint32_t nsec;
};

struct m0_ipc_m0_cmd_aon_gpio_wakeup_cfg {
	/* AON gpio mask of AON GPIOs
	 * Bit i is 1 iff AON GPIO i is configured to force
	 * immediate wakeup if GPIO value is HIGH on sleep entry
	 */
	uint32_t mask_skip_sleep_if_high;
	/* AON gpio mask of AON GPIOs
	 * Bit i is 1 iff AON GPIO i is configured to force
	 * immediate wakeup if GPIO value is LOW on sleep entry
	 */
	uint32_t mask_skip_sleep_if_low;
};

#endif
