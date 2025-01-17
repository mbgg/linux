#include <net/mac80211.h>
#include <net/rtnetlink.h>

#include "ieee80211_i.h"
#include "mesh.h"
#include "driver-ops.h"
#include "led.h"

int __ieee80211_suspend(struct ieee80211_hw *hw, struct cfg80211_wowlan *wowlan)
{
	struct ieee80211_local *local = hw_to_local(hw);
	struct ieee80211_sub_if_data *sdata;
	struct sta_info *sta;

	if (!local->open_count)
		goto suspend;

	ieee80211_scan_cancel(local);

	ieee80211_dfs_cac_cancel(local);

	if (hw->flags & IEEE80211_HW_AMPDU_AGGREGATION) {
		mutex_lock(&local->sta_mtx);
		list_for_each_entry(sta, &local->sta_list, list) {
			set_sta_flag(sta, WLAN_STA_BLOCK_BA);
			ieee80211_sta_tear_down_BA_sessions(
					sta, AGG_STOP_LOCAL_REQUEST);
		}
		mutex_unlock(&local->sta_mtx);
	}

	ieee80211_stop_queues_by_reason(hw,
			IEEE80211_QUEUE_STOP_REASON_SUSPEND);

	/* flush out all packets */
	synchronize_net();

	drv_flush(local, false);

	local->quiescing = true;
	/* make quiescing visible to timers everywhere */
	mb();

	flush_workqueue(local->workqueue);

	/* Don't try to run timers while suspended. */
	del_timer_sync(&local->sta_cleanup);

	 /*
	 * Note that this particular timer doesn't need to be
	 * restarted at resume.
	 */
	cancel_work_sync(&local->dynamic_ps_enable_work);
	del_timer_sync(&local->dynamic_ps_timer);

	local->wowlan = wowlan && local->open_count;
	if (local->wowlan) {
		int err = drv_suspend(local, wowlan);
		if (err < 0) {
			local->quiescing = false;
			local->wowlan = false;
			if (hw->flags & IEEE80211_HW_AMPDU_AGGREGATION) {
				mutex_lock(&local->sta_mtx);
				list_for_each_entry(sta,
						    &local->sta_list, list) {
					clear_sta_flag(sta, WLAN_STA_BLOCK_BA);
				}
				mutex_unlock(&local->sta_mtx);
			}
			ieee80211_wake_queues_by_reason(hw,
					IEEE80211_QUEUE_STOP_REASON_SUSPEND);
			return err;
		} else if (err > 0) {
			WARN_ON(err != 1);
			return err;
		} else {
			goto suspend;
		}
	}

	/* tear down aggregation sessions and remove STAs */
	mutex_lock(&local->sta_mtx);
	list_for_each_entry(sta, &local->sta_list, list) {
		if (sta->uploaded) {
			enum ieee80211_sta_state state;

			state = sta->sta_state;
			for (; state > IEEE80211_STA_NOTEXIST; state--)
				WARN_ON(drv_sta_state(local, sta->sdata, sta,
						      state, state - 1));
		}
	}
	mutex_unlock(&local->sta_mtx);

	/* remove all interfaces */
	list_for_each_entry(sdata, &local->interfaces, list) {
		if (!ieee80211_sdata_running(sdata))
			continue;
		drv_remove_interface(local, sdata);
	}

	sdata = rtnl_dereference(local->monitor_sdata);
	if (sdata)
		drv_remove_interface(local, sdata);

	/*
	 * We disconnected on all interfaces before suspend, all channel
	 * contexts should be released.
	 */
	WARN_ON(!list_empty(&local->chanctx_list));

	/* stop hardware - this must stop RX */
	if (local->open_count)
		ieee80211_stop_device(local);

 suspend:
	local->suspended = true;
	/* need suspended to be visible before quiescing is false */
	barrier();
	local->quiescing = false;

	return 0;
}

/*
 * __ieee80211_resume() is a static inline which just calls
 * ieee80211_reconfig(), which is also needed for hardware
 * hang/firmware failure/etc. recovery.
 */

void ieee80211_report_wowlan_wakeup(struct ieee80211_vif *vif,
				    struct cfg80211_wowlan_wakeup *wakeup,
				    gfp_t gfp)
{
	struct ieee80211_sub_if_data *sdata = vif_to_sdata(vif);

	cfg80211_report_wowlan_wakeup(&sdata->wdev, wakeup, gfp);
}
EXPORT_SYMBOL(ieee80211_report_wowlan_wakeup);
