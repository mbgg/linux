/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2012 - 2013 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <ilw@linux.intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2012 - 2013 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef __IWL_MVM_H__
#define __IWL_MVM_H__

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/leds.h>
#include <linux/in6.h>

#include "iwl-op-mode.h"
#include "iwl-trans.h"
#include "iwl-notif-wait.h"
#include "iwl-eeprom-parse.h"
#include "iwl-test.h"
#include "iwl-trans.h"
#include "sta.h"
#include "fw-api.h"

#define IWL_INVALID_MAC80211_QUEUE	0xff
#define IWL_MVM_MAX_ADDRESSES		5
/* RSSI offset for WkP */
#define IWL_RSSI_OFFSET 50

enum iwl_mvm_tx_fifo {
	IWL_MVM_TX_FIFO_BK = 0,
	IWL_MVM_TX_FIFO_BE,
	IWL_MVM_TX_FIFO_VI,
	IWL_MVM_TX_FIFO_VO,
};

/* Placeholder */
#define IWL_OFFCHANNEL_QUEUE 8
#define IWL_FIRST_AMPDU_QUEUE 11

extern struct ieee80211_ops iwl_mvm_hw_ops;
/**
 * struct iwl_mvm_mod_params - module parameters for iwlmvm
 * @init_dbg: if true, then the NIC won't be stopped if the INIT fw asserted.
 *	We will register to mac80211 to have testmode working. The NIC must not
 *	be up'ed after the INIT fw asserted. This is useful to be able to use
 *	proprietary tools over testmode to debug the INIT fw.
 * @power_scheme: CAM(Continuous Active Mode)-1, BPS(Balanced Power
 *	Save)-2(default), LP(Low Power)-3
 */
struct iwl_mvm_mod_params {
	bool init_dbg;
	int power_scheme;
};
extern struct iwl_mvm_mod_params iwlmvm_mod_params;

struct iwl_mvm_phy_ctxt {
	u16 id;
	u16 color;

	/*
	 * TODO: This should probably be removed. Currently here only for rate
	 * scaling algorithm
	 */
	struct ieee80211_channel *channel;
};

struct iwl_mvm_time_event_data {
	struct ieee80211_vif *vif;
	struct list_head list;
	unsigned long end_jiffies;
	u32 duration;
	bool running;
	u32 uid;

	/*
	 * The access to the 'id' field must be done when the
	 * mvm->time_event_lock is held, as it value is used to indicate
	 * if the te is in the time event list or not (when id == TE_MAX)
	 */
	u32 id;
};

 /* Power management */

/**
 * enum iwl_power_scheme
 * @IWL_POWER_LEVEL_CAM - Continuously Active Mode
 * @IWL_POWER_LEVEL_BPS - Balanced Power Save (default)
 * @IWL_POWER_LEVEL_LP  - Low Power
 */
enum iwl_power_scheme {
	IWL_POWER_SCHEME_CAM = 1,
	IWL_POWER_SCHEME_BPS,
	IWL_POWER_SCHEME_LP
};

#define IWL_CONN_MAX_LISTEN_INTERVAL	70

/**
 * struct iwl_mvm_vif - data per Virtual Interface, it is a MAC context
 * @id: between 0 and 3
 * @color: to solve races upon MAC addition and removal
 * @ap_sta_id: the sta_id of the AP - valid only if VIF type is STA
 * @uploaded: indicates the MAC context has been added to the device
 * @ap_active: indicates that ap context is configured, and that the interface
 *  should get quota etc.
 * @queue_params: QoS params for this MAC
 * @bcast_sta: station used for broadcast packets. Used by the following
 *  vifs: P2P_DEVICE, GO and AP.
 * @beacon_skb: the skb used to hold the AP/GO beacon template
 */
struct iwl_mvm_vif {
	u16 id;
	u16 color;
	u8 ap_sta_id;

	bool uploaded;
	bool ap_active;

	u32 ap_beacon_time;

	enum iwl_tsf_id tsf_id;

	/*
	 * QoS data from mac80211, need to store this here
	 * as mac80211 has a separate callback but we need
	 * to have the data for the MAC context
	 */
	struct ieee80211_tx_queue_params queue_params[IEEE80211_NUM_ACS];
	struct iwl_mvm_time_event_data time_event_data;

	struct iwl_mvm_int_sta bcast_sta;

	/*
	 * Assigned while mac80211 has the interface in a channel context,
	 * or, for P2P Device, while it exists.
	 */
	struct iwl_mvm_phy_ctxt *phy_ctxt;

#ifdef CONFIG_PM_SLEEP
	/* WoWLAN GTK rekey data */
	struct {
		u8 kck[NL80211_KCK_LEN], kek[NL80211_KEK_LEN];
		__le64 replay_ctr;
		bool valid;
	} rekey_data;

	int tx_key_idx;

#if IS_ENABLED(CONFIG_IPV6)
	/* IPv6 addresses for WoWLAN */
	struct in6_addr target_ipv6_addrs[IWL_PROTO_OFFLOAD_NUM_IPV6_ADDRS];
	int num_target_ipv6_addrs;
#endif
#endif

#ifdef CONFIG_IWLWIFI_DEBUGFS
	struct dentry *dbgfs_dir;
	void *dbgfs_data;
#endif
};

static inline struct iwl_mvm_vif *
iwl_mvm_vif_from_mac80211(struct ieee80211_vif *vif)
{
	return (void *)vif->drv_priv;
}

enum iwl_mvm_status {
	IWL_MVM_STATUS_HW_RFKILL,
	IWL_MVM_STATUS_ROC_RUNNING,
	IWL_MVM_STATUS_IN_HW_RESTART,
};

enum iwl_scan_status {
	IWL_MVM_SCAN_NONE,
	IWL_MVM_SCAN_OS,
};

/**
 * struct iwl_nvm_section - describes an NVM section in memory.
 *
 * This struct holds an NVM section read from the NIC using NVM_ACCESS_CMD,
 * and saved for later use by the driver. Not all NVM sections are saved
 * this way, only the needed ones.
 */
struct iwl_nvm_section {
	u16 length;
	const u8 *data;
};

struct iwl_mvm {
	/* for logger access */
	struct device *dev;

	struct iwl_trans *trans;
	const struct iwl_fw *fw;
	const struct iwl_cfg *cfg;
	struct iwl_phy_db *phy_db;
	struct ieee80211_hw *hw;

	/* for protecting access to iwl_mvm */
	struct mutex mutex;
	struct list_head async_handlers_list;
	spinlock_t async_handlers_lock;
	struct work_struct async_handlers_wk;

	struct work_struct roc_done_wk;

	unsigned long status;

	enum iwl_ucode_type cur_ucode;
	bool ucode_loaded;
	bool init_ucode_run;
	u32 error_event_table;
	u32 log_event_table;

	u32 ampdu_ref;

	struct iwl_notif_wait_data notif_wait;

	unsigned long transport_queue_stop;
	u8 queue_to_mac80211[IWL_MAX_HW_QUEUES];
	atomic_t queue_stop_count[IWL_MAX_HW_QUEUES];

	struct iwl_nvm_data *nvm_data;
	/* eeprom blob for debugfs/testmode */
	u8 *eeprom_blob;
	size_t eeprom_blob_size;
	/* NVM sections for 7000 family */
	struct iwl_nvm_section nvm_sections[NVM_NUM_OF_SECTIONS];

	/* EEPROM MAC addresses */
	struct mac_address addresses[IWL_MVM_MAX_ADDRESSES];

	/* data related to data path */
	struct iwl_rx_phy_info last_phy_info;
	struct ieee80211_sta __rcu *fw_id_to_mac_id[IWL_MVM_STATION_COUNT];
	struct work_struct sta_drained_wk;
	unsigned long sta_drained[BITS_TO_LONGS(IWL_MVM_STATION_COUNT)];

	/* configured by mac80211 */
	u32 rts_threshold;

	/* Scan status, cmd (pre-allocated) and auxiliary station */
	enum iwl_scan_status scan_status;
	struct iwl_scan_cmd *scan_cmd;

	/* Internal station */
	struct iwl_mvm_int_sta aux_sta;

	u8 scan_last_antenna_idx; /* to toggle TX between antennas */
	u8 mgmt_last_antenna_idx;

#ifdef CONFIG_IWLWIFI_DEBUGFS
	struct dentry *debugfs_dir;
	u32 dbgfs_sram_offset, dbgfs_sram_len;
	bool prevent_power_down_d3;
#endif

	struct iwl_mvm_phy_ctxt phy_ctxt_roc;

	struct list_head time_event_list;
	spinlock_t time_event_lock;

	/*
	 * A bitmap indicating the index of the key in use. The firmware
	 * can hold 16 keys at most. Reflect this fact.
	 */
	unsigned long fw_key_table[BITS_TO_LONGS(STA_KEY_MAX_NUM)];
	u8 vif_count;

	struct led_classdev led;

	struct ieee80211_vif *p2p_device_vif;

#ifdef CONFIG_PM_SLEEP
	int gtk_ivlen, gtk_icvlen, ptk_ivlen, ptk_icvlen;
#endif

	/* BT-Coex */
	u8 bt_kill_msk;
	struct iwl_bt_coex_profile_notif last_bt_notif;
};

/* Extract MVM priv from op_mode and _hw */
#define IWL_OP_MODE_GET_MVM(_iwl_op_mode)		\
	((struct iwl_mvm *)(_iwl_op_mode)->op_mode_specific)

#define IWL_MAC80211_GET_MVM(_hw)			\
	IWL_OP_MODE_GET_MVM((struct iwl_op_mode *)((_hw)->priv))

extern const u8 iwl_mvm_ac_to_tx_fifo[];

struct iwl_rate_info {
	u8 plcp;	/* uCode API:  IWL_RATE_6M_PLCP, etc. */
	u8 plcp_siso;	/* uCode API:  IWL_RATE_SISO_6M_PLCP, etc. */
	u8 plcp_mimo2;	/* uCode API:  IWL_RATE_MIMO2_6M_PLCP, etc. */
	u8 plcp_mimo3;  /* uCode API:  IWL_RATE_MIMO3_6M_PLCP, etc. */
	u8 ieee;	/* MAC header:  IWL_RATE_6M_IEEE, etc. */
};

/******************
 * MVM Methods
 ******************/
/* uCode */
int iwl_run_init_mvm_ucode(struct iwl_mvm *mvm, bool read_nvm);

/* Utils */
int iwl_mvm_legacy_rate_to_mac80211_idx(u32 rate_n_flags,
					enum ieee80211_band band);
u8 iwl_mvm_mac80211_idx_to_hwrate(int rate_idx);
void iwl_mvm_dump_nic_error_log(struct iwl_mvm *mvm);
u8 first_antenna(u8 mask);
u8 iwl_mvm_next_antenna(struct iwl_mvm *mvm, u8 valid, u8 last_idx);

/* Tx / Host Commands */
int __must_check iwl_mvm_send_cmd(struct iwl_mvm *mvm,
				  struct iwl_host_cmd *cmd);
int __must_check iwl_mvm_send_cmd_pdu(struct iwl_mvm *mvm, u8 id,
				      u32 flags, u16 len, const void *data);
int __must_check iwl_mvm_send_cmd_status(struct iwl_mvm *mvm,
					 struct iwl_host_cmd *cmd,
					 u32 *status);
int __must_check iwl_mvm_send_cmd_pdu_status(struct iwl_mvm *mvm, u8 id,
					     u16 len, const void *data,
					     u32 *status);
int iwl_mvm_tx_skb(struct iwl_mvm *mvm, struct sk_buff *skb,
		   struct ieee80211_sta *sta);
int iwl_mvm_tx_skb_non_sta(struct iwl_mvm *mvm, struct sk_buff *skb);
#ifdef CONFIG_IWLWIFI_DEBUG
const char *iwl_mvm_get_tx_fail_reason(u32 status);
#else
static inline const char *iwl_mvm_get_tx_fail_reason(u32 status) { return ""; }
#endif
int iwl_mvm_flush_tx_path(struct iwl_mvm *mvm, u32 tfd_msk, bool sync);
void iwl_mvm_async_handlers_purge(struct iwl_mvm *mvm);

/* Statistics */
int iwl_mvm_rx_reply_statistics(struct iwl_mvm *mvm,
				struct iwl_rx_cmd_buffer *rxb,
				struct iwl_device_cmd *cmd);
int iwl_mvm_rx_statistics(struct iwl_mvm *mvm,
			  struct iwl_rx_cmd_buffer *rxb,
			  struct iwl_device_cmd *cmd);

/* NVM */
int iwl_nvm_init(struct iwl_mvm *mvm);

int iwl_mvm_up(struct iwl_mvm *mvm);
int iwl_mvm_load_d3_fw(struct iwl_mvm *mvm);

int iwl_mvm_mac_setup_register(struct iwl_mvm *mvm);

/*
 * FW notifications / CMD responses handlers
 * Convention: iwl_mvm_rx_<NAME OF THE CMD>
 */
int iwl_mvm_rx_rx_phy_cmd(struct iwl_mvm *mvm, struct iwl_rx_cmd_buffer *rxb,
			  struct iwl_device_cmd *cmd);
int iwl_mvm_rx_rx_mpdu(struct iwl_mvm *mvm, struct iwl_rx_cmd_buffer *rxb,
		       struct iwl_device_cmd *cmd);
int iwl_mvm_rx_tx_cmd(struct iwl_mvm *mvm, struct iwl_rx_cmd_buffer *rxb,
		      struct iwl_device_cmd *cmd);
int iwl_mvm_rx_ba_notif(struct iwl_mvm *mvm, struct iwl_rx_cmd_buffer *rxb,
			struct iwl_device_cmd *cmd);
int iwl_mvm_rx_radio_ver(struct iwl_mvm *mvm, struct iwl_rx_cmd_buffer *rxb,
			 struct iwl_device_cmd *cmd);
int iwl_mvm_rx_fw_error(struct iwl_mvm *mvm, struct iwl_rx_cmd_buffer *rxb,
			  struct iwl_device_cmd *cmd);
int iwl_mvm_rx_card_state_notif(struct iwl_mvm *mvm,
				struct iwl_rx_cmd_buffer *rxb,
				struct iwl_device_cmd *cmd);
int iwl_mvm_rx_radio_ver(struct iwl_mvm *mvm, struct iwl_rx_cmd_buffer *rxb,
			 struct iwl_device_cmd *cmd);

/* MVM PHY */
int iwl_mvm_phy_ctxt_add(struct iwl_mvm *mvm, struct iwl_mvm_phy_ctxt *ctxt,
			 struct cfg80211_chan_def *chandef,
			 u8 chains_static, u8 chains_dynamic);
int iwl_mvm_phy_ctxt_changed(struct iwl_mvm *mvm, struct iwl_mvm_phy_ctxt *ctxt,
			     struct cfg80211_chan_def *chandef,
			     u8 chains_static, u8 chains_dynamic);
void iwl_mvm_phy_ctxt_remove(struct iwl_mvm *mvm,
			     struct iwl_mvm_phy_ctxt *ctxt);

/* MAC (virtual interface) programming */
int iwl_mvm_mac_ctxt_init(struct iwl_mvm *mvm, struct ieee80211_vif *vif);
void iwl_mvm_mac_ctxt_release(struct iwl_mvm *mvm, struct ieee80211_vif *vif);
int iwl_mvm_mac_ctxt_add(struct iwl_mvm *mvm, struct ieee80211_vif *vif);
int iwl_mvm_mac_ctxt_changed(struct iwl_mvm *mvm, struct ieee80211_vif *vif);
int iwl_mvm_mac_ctxt_remove(struct iwl_mvm *mvm, struct ieee80211_vif *vif);
u32 iwl_mvm_mac_get_queues_mask(struct iwl_mvm *mvm,
				struct ieee80211_vif *vif);
int iwl_mvm_mac_ctxt_beacon_changed(struct iwl_mvm *mvm,
				    struct ieee80211_vif *vif);

/* Bindings */
int iwl_mvm_binding_add_vif(struct iwl_mvm *mvm, struct ieee80211_vif *vif);
int iwl_mvm_binding_remove_vif(struct iwl_mvm *mvm, struct ieee80211_vif *vif);

/* Quota management */
int iwl_mvm_update_quotas(struct iwl_mvm *mvm, struct ieee80211_vif *newvif);

/* Scanning */
int iwl_mvm_scan_request(struct iwl_mvm *mvm,
			 struct ieee80211_vif *vif,
			 struct cfg80211_scan_request *req);
int iwl_mvm_rx_scan_response(struct iwl_mvm *mvm, struct iwl_rx_cmd_buffer *rxb,
			     struct iwl_device_cmd *cmd);
int iwl_mvm_rx_scan_complete(struct iwl_mvm *mvm, struct iwl_rx_cmd_buffer *rxb,
			     struct iwl_device_cmd *cmd);
void iwl_mvm_cancel_scan(struct iwl_mvm *mvm);

/* MVM debugfs */
#ifdef CONFIG_IWLWIFI_DEBUGFS
int iwl_mvm_dbgfs_register(struct iwl_mvm *mvm, struct dentry *dbgfs_dir);
int iwl_mvm_vif_dbgfs_register(struct iwl_mvm *mvm, struct ieee80211_vif *vif,
			       struct dentry *dbgfs_dir);
void iwl_power_get_params(struct iwl_mvm *mvm, struct ieee80211_vif *vif,
			  struct iwl_powertable_cmd *cmd);
#else
static inline int iwl_mvm_dbgfs_register(struct iwl_mvm *mvm,
					 struct dentry *dbgfs_dir)
{
	return 0;
}
#endif /* CONFIG_IWLWIFI_DEBUGFS */

/* rate scaling */
int iwl_mvm_send_lq_cmd(struct iwl_mvm *mvm, struct iwl_lq_cmd *lq,
			u8 flags, bool init);

/* power managment */
int iwl_mvm_power_update_mode(struct iwl_mvm *mvm, struct ieee80211_vif *vif);
int iwl_mvm_power_disable(struct iwl_mvm *mvm, struct ieee80211_vif *vif);

int iwl_mvm_leds_init(struct iwl_mvm *mvm);
void iwl_mvm_leds_exit(struct iwl_mvm *mvm);

/* D3 (WoWLAN, NetDetect) */
int iwl_mvm_suspend(struct ieee80211_hw *hw, struct cfg80211_wowlan *wowlan);
int iwl_mvm_resume(struct ieee80211_hw *hw);
void iwl_mvm_set_wakeup(struct ieee80211_hw *hw, bool enabled);
void iwl_mvm_set_rekey_data(struct ieee80211_hw *hw,
			    struct ieee80211_vif *vif,
			    struct cfg80211_gtk_rekey_data *data);
void iwl_mvm_ipv6_addr_change(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif,
			      struct inet6_dev *idev);
void iwl_mvm_set_default_unicast_key(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif, int idx);

/* BT Coex */
int iwl_send_bt_prio_tbl(struct iwl_mvm *mvm);
int iwl_send_bt_init_conf(struct iwl_mvm *mvm);
int iwl_mvm_rx_bt_coex_notif(struct iwl_mvm *mvm,
			     struct iwl_rx_cmd_buffer *rxb,
			     struct iwl_device_cmd *cmd);

#endif /* __IWL_MVM_H__ */
