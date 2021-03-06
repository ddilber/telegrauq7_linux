/*
 * otg_fsm.c - ChipIdea USB IP core OTG FSM driver
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * Author: Jun Li
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * This file mainly handles OTG fsm, it includes OTG fsm operations
 * for HNP and SRP.
 *
 * TODO List
 * - ADP
 * - OTG test device
 */

#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>
#include <linux/usb/chipidea.h>
#include <linux/regulator/consumer.h>

#include "ci.h"
#include "bits.h"
#include "otg.h"
#include "otg_fsm.h"
#include "host.h"
#include "udc.h"

static struct ci_otg_fsm_timer *otg_timer_initializer
(struct ci_hdrc *ci, void (*function)(void *, unsigned long),
			unsigned long expires, unsigned long data)
{
	struct ci_otg_fsm_timer *timer;

	timer = devm_kzalloc(ci->dev, sizeof(struct ci_otg_fsm_timer),
								GFP_KERNEL);
	if (!timer)
		return NULL;
	timer->function = function;
	timer->expires = expires;
	timer->data = data;
	return timer;
}

/* Add for otg: interact with user space app */
static ssize_t
get_a_bus_req(struct device *dev, struct device_attribute *attr, char *buf)
{
	char		*next;
	unsigned	size, t;
	struct ci_hdrc	*ci = dev_get_drvdata(dev);

	next = buf;
	size = PAGE_SIZE;
	t = scnprintf(next, size, "%d\n", ci->fsm.a_bus_req);
	size -= t;
	next += t;

	return PAGE_SIZE - size;
}

static ssize_t
set_a_bus_req(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct ci_hdrc *ci = dev_get_drvdata(dev);

	if (count > 2)
		return -1;

	mutex_lock(&ci->fsm.lock);
	if (buf[0] == '0') {
		ci->fsm.a_bus_req = 0;
	} else if (buf[0] == '1') {
		/* If a_bus_drop is TRUE, a_bus_req can't be set */
		if (ci->fsm.a_bus_drop) {
			mutex_unlock(&ci->fsm.lock);
			return count;
		}
		ci->fsm.a_bus_req = 1;
		if (ci->transceiver->state == OTG_STATE_A_PERIPHERAL) {
			ci->gadget.host_request_flag = 1;
			mutex_unlock(&ci->fsm.lock);
			return count;
		}
	}

	ci_otg_queue_work(ci);
	mutex_unlock(&ci->fsm.lock);

	return count;
}
static DEVICE_ATTR(a_bus_req, S_IRUGO | S_IWUSR, get_a_bus_req, set_a_bus_req);

static ssize_t
get_a_bus_drop(struct device *dev, struct device_attribute *attr, char *buf)
{
	char		*next;
	unsigned	size, t;
	struct ci_hdrc	*ci = dev_get_drvdata(dev);

	next = buf;
	size = PAGE_SIZE;
	t = scnprintf(next, size, "%d\n", ci->fsm.a_bus_drop);
	size -= t;
	next += t;

	return PAGE_SIZE - size;
}

static ssize_t
set_a_bus_drop(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct ci_hdrc	*ci = dev_get_drvdata(dev);

	if (count > 2)
		return -1;

	mutex_lock(&ci->fsm.lock);
	if (buf[0] == '0') {
		ci->fsm.a_bus_drop = 0;
	} else if (buf[0] == '1') {
		ci->fsm.a_bus_drop = 1;
		ci->fsm.a_bus_req = 0;
	}

	ci_otg_queue_work(ci);
	mutex_unlock(&ci->fsm.lock);

	return count;
}
static DEVICE_ATTR(a_bus_drop, S_IRUGO | S_IWUSR, get_a_bus_drop,
						set_a_bus_drop);

static ssize_t
get_b_bus_req(struct device *dev, struct device_attribute *attr, char *buf)
{
	char		*next;
	unsigned	size, t;
	struct ci_hdrc	*ci = dev_get_drvdata(dev);

	next = buf;
	size = PAGE_SIZE;
	t = scnprintf(next, size, "%d\n", ci->fsm.b_bus_req);
	size -= t;
	next += t;

	return PAGE_SIZE - size;
}

static ssize_t
set_b_bus_req(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct ci_hdrc	*ci = dev_get_drvdata(dev);

	if (count > 2)
		return -1;

	mutex_lock(&ci->fsm.lock);
	if (buf[0] == '0')
		ci->fsm.b_bus_req = 0;
	else if (buf[0] == '1') {
		ci->fsm.b_bus_req = 1;
		if (ci->transceiver->state == OTG_STATE_B_PERIPHERAL) {
			ci->gadget.host_request_flag = 1;
			mutex_unlock(&ci->fsm.lock);
			return count;
		}
	}

	ci_otg_queue_work(ci);
	mutex_unlock(&ci->fsm.lock);

	return count;
}
static DEVICE_ATTR(b_bus_req, S_IRUGO | S_IWUSR, get_b_bus_req, set_b_bus_req);

static ssize_t
set_a_clr_err(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct ci_hdrc	*ci = dev_get_drvdata(dev);

	if (count > 2)
		return -1;

	mutex_lock(&ci->fsm.lock);
	if (buf[0] == '1')
		ci->fsm.a_clr_err = 1;

	ci_otg_queue_work(ci);
	mutex_unlock(&ci->fsm.lock);

	return count;
}
static DEVICE_ATTR(a_clr_err, S_IWUSR, NULL, set_a_clr_err);

static struct attribute *inputs_attrs[] = {
	&dev_attr_a_bus_req.attr,
	&dev_attr_a_bus_drop.attr,
	&dev_attr_b_bus_req.attr,
	&dev_attr_a_clr_err.attr,
	NULL,
};

static struct attribute_group inputs_attr_group = {
	.name = "inputs",
	.attrs = inputs_attrs,
};

/*
 * Add timer to active timer list
 */
static void ci_otg_add_timer(struct ci_hdrc *ci, enum ci_otg_fsm_timer_index t)
{
	struct ci_otg_fsm_timer *tmp_timer;
	struct ci_otg_fsm_timer *timer = ci->fsm_timer->timer_list[t];
	struct list_head *active_timers = &ci->fsm_timer->active_timers;

	if (t >= NUM_CI_OTG_FSM_TIMERS)
		return;

	if (!timer) {
		dev_warn(ci->dev, "Unused otg timer, index:%d\n", t);
		return;
	}

	/*
	 * Check if the timer is already in the active list,
	 * if so update timer count
	 */
	list_for_each_entry(tmp_timer, active_timers, list)
		if (tmp_timer == timer) {
			timer->count = timer->expires;
			return;
		}

	if (list_empty(active_timers))
		pm_runtime_get(ci->dev);

	timer->count = timer->expires;
	list_add_tail(&timer->list, active_timers);

	/* Enable 1ms irq */
	if (!(hw_read_otgsc(ci, OTGSC_1MSIE)))
		hw_write_otgsc(ci, OTGSC_1MSIE, OTGSC_1MSIE);
}

/*
 * Remove timer from active timer list
 */
static void ci_otg_del_timer(struct ci_hdrc *ci, enum ci_otg_fsm_timer_index t)
{
	struct ci_otg_fsm_timer *tmp_timer, *del_tmp;
	struct ci_otg_fsm_timer *timer = ci->fsm_timer->timer_list[t];
	struct list_head *active_timers = &ci->fsm_timer->active_timers;
	int flag = 0;

	if (t >= NUM_CI_OTG_FSM_TIMERS)
		return;

	list_for_each_entry_safe(tmp_timer, del_tmp, active_timers, list)
		if (tmp_timer == timer) {
			list_del(&timer->list);
			flag = 1;
		}

	/* Disable 1ms irq if there is no any active timer */
	if (list_empty(active_timers) && (flag == 1)) {
		hw_write_otgsc(ci, OTGSC_1MSIE, 0);
		pm_runtime_put(ci->dev);
	}
}

/*
 * Reduce timer count by 1, and find timeout conditions.
 * Called by otg 1ms timer interrupt
 */
static inline int ci_otg_tick_timer(struct ci_hdrc *ci)
{
	struct ci_otg_fsm_timer *tmp_timer, *del_tmp;
	struct list_head *active_timers = &ci->fsm_timer->active_timers;
	int expired = 0;

	list_for_each_entry_safe(tmp_timer, del_tmp, active_timers, list) {
		tmp_timer->count--;
		/* check if timer expires */
		if (!tmp_timer->count) {
			list_del(&tmp_timer->list);
			tmp_timer->function(ci, tmp_timer->data);
			expired = 1;
		}
	}

	/* disable 1ms irq if there is no any timer active */
	if ((expired == 1) && list_empty(active_timers)) {
		hw_write_otgsc(ci, OTGSC_1MSIE, 0);
		pm_runtime_put(ci->dev);
	}

	return expired;
}

/* The timeout callback function to set time out bit */
static void set_tmout(void *ptr, unsigned long indicator)
{
	*(int *)indicator = 1;
}

static void set_tmout_and_fsm(void *ptr, unsigned long indicator)
{
	struct ci_hdrc *ci = (struct ci_hdrc *)ptr;

	set_tmout(ci, indicator);

	ci_otg_queue_work(ci);
}

static void a_wait_vfall_tmout_func(void *ptr, unsigned long indicator)
{
	struct ci_hdrc *ci = (struct ci_hdrc *)ptr;

	set_tmout(ci, indicator);

	/* Disable port power for detect SRP */
	hw_write(ci, OP_PORTSC, PORTSC_W1C_BITS | PORTSC_PP, 0);
	/* Clear exsiting DP irq */
	hw_write_otgsc(ci, OTGSC_DPIS, OTGSC_DPIS);
	/* Enable data pulse irq */
	hw_write_otgsc(ci, OTGSC_DPIE, OTGSC_DPIE);
	ci_otg_queue_work(ci);
}

static void set_tmout_and_show_msg(void *ptr, unsigned long indicator)
{
	struct ci_hdrc *ci = (struct ci_hdrc *)ptr;

	dev_warn(ci->dev, "Device No Response\n");
	set_tmout_and_fsm(ptr, indicator);
}

static void b_srp_fail_tmout_func(void *ptr, unsigned long indicator)
{
	struct ci_hdrc *ci = (struct ci_hdrc *)ptr;

	dev_warn(ci->dev, "Device No Response\n");
}

static void b_ssend_srp_tmout_func(void *ptr, unsigned long indicator)
{
	struct ci_hdrc *ci = (struct ci_hdrc *)ptr;

	set_tmout(ci, indicator);

	/* only vbus fall below B_sess_vld in b_idle state */
	if (ci->transceiver->state == OTG_STATE_B_IDLE)
		ci_otg_queue_work(ci);
}

static void b_data_pulse_end(void *ptr, unsigned long indicator)
{
	struct ci_hdrc *ci = (struct ci_hdrc *)ptr;

	ci->fsm.b_srp_done = 1;
	ci->fsm.b_bus_req = 0;
	if (ci->fsm.power_up)
		ci->fsm.power_up = 0;

	hw_write_otgsc(ci, OTGSC_HABA, 0);

	ci_otg_queue_work(ci);
}

/*
 * Timer for A-device to turn on Vbus
 * after detecting data pulse from B-device
 */
static void a_wait_dp_end_tmout_func(void *ptr, unsigned long indicator)
{
	struct ci_hdrc *ci = (struct ci_hdrc *)ptr;

	ci->fsm.a_bus_drop = 0;
	ci->fsm.a_srp_det = 1;
	ci_otg_queue_work(ci);
}

static void hnp_polling_timer_work(unsigned long arg)
{
	struct ci_hdrc *ci = (struct ci_hdrc *)arg;

	schedule_work(&ci->hnp_polling_work);
}

static void ci_hnp_polling_work(struct work_struct *work)
{
	struct ci_hdrc *ci = container_of(work, struct ci_hdrc,
						hnp_polling_work);

	pm_runtime_get_sync(ci->dev);
	if (otg_hnp_polling(&ci->fsm) == HOST_REQUEST_FLAG)
		ci_otg_queue_work(ci);
	pm_runtime_put_sync(ci->dev);
}

static void a_tst_maint_tmout_func(void *ptr, unsigned long indicator)
{
	struct ci_hdrc *ci = (struct ci_hdrc *)ptr;

	ci->fsm.tst_maint = 0;
	if (ci->fsm.otg_vbus_off) {
		ci->fsm.otg_vbus_off = 0;
		dev_dbg(ci->dev,
			"test device does not disconnect, end the session!\n");
	}

	/* End the session */
	ci->fsm.a_bus_req = 0;
	ci->fsm.a_bus_drop = 1;
	ci_otg_queue_work(ci);
}

/*
 * otg_srp_reqd feature
 * After A(PET) turn off vbus, B(UUT) should start this timer to do SRP
 * when the timer expires.
 */
static void b_srp_reqd_tmout_func(void *ptr, unsigned long indicator)
{
	struct ci_hdrc *ci = (struct ci_hdrc *)ptr;

	ci->fsm.otg_srp_reqd = 0;
	if (ci->transceiver->state == OTG_STATE_B_IDLE) {
		ci->fsm.b_bus_req = 1;
		ci_otg_queue_work(ci);
	}
}

/*
 * otg_hnp_reqd feature
 * After B(UUT) switch to host, B should hand host role back
 * to A(PET) within TB_TST_SUSP after setting configuration.
 */
static void b_tst_susp_tmout_func(void *ptr, unsigned long indicator)
{
	struct ci_hdrc *ci = (struct ci_hdrc *)ptr;

	if (ci->transceiver->state == OTG_STATE_B_HOST) {
		ci->fsm.b_bus_req = 0;
		ci_otg_queue_work(ci);
	}
}

/* Initialize timers */
static int ci_otg_init_timers(struct ci_hdrc *ci)
{
	struct otg_fsm *fsm = &ci->fsm;

	/* FSM used timers */
	ci->fsm_timer->timer_list[A_WAIT_VRISE] =
		otg_timer_initializer(ci, &set_tmout_and_fsm, TA_WAIT_VRISE,
			(unsigned long)&fsm->a_wait_vrise_tmout);
	if (ci->fsm_timer->timer_list[A_WAIT_VRISE] == NULL)
		return -ENOMEM;

	ci->fsm_timer->timer_list[A_WAIT_VFALL] =
		otg_timer_initializer(ci, &a_wait_vfall_tmout_func,
		TA_WAIT_VFALL, (unsigned long)&fsm->a_wait_vfall_tmout);
	if (ci->fsm_timer->timer_list[A_WAIT_VFALL] == NULL)
		return -ENOMEM;

	ci->fsm_timer->timer_list[A_WAIT_BCON] =
		otg_timer_initializer(ci, &set_tmout_and_show_msg, TA_WAIT_BCON,
				(unsigned long)&fsm->a_wait_bcon_tmout);
	if (ci->fsm_timer->timer_list[A_WAIT_BCON] == NULL)
		return -ENOMEM;

	ci->fsm_timer->timer_list[A_AIDL_BDIS] =
		otg_timer_initializer(ci, &set_tmout_and_fsm, TA_AIDL_BDIS,
				(unsigned long)&fsm->a_aidl_bdis_tmout);
	if (ci->fsm_timer->timer_list[A_AIDL_BDIS] == NULL)
		return -ENOMEM;

	ci->fsm_timer->timer_list[A_BIDL_ADIS] =
		otg_timer_initializer(ci, &set_tmout_and_fsm, TA_BIDL_ADIS,
				(unsigned long)&fsm->a_bidl_adis_tmout);
	if (ci->fsm_timer->timer_list[A_BIDL_ADIS] == NULL)
		return -ENOMEM;

	ci->fsm_timer->timer_list[B_ASE0_BRST] =
		otg_timer_initializer(ci, &set_tmout_and_show_msg, TB_ASE0_BRST,
					(unsigned long)&fsm->b_ase0_brst_tmout);
	if (ci->fsm_timer->timer_list[B_ASE0_BRST] == NULL)
		return -ENOMEM;

	ci->fsm_timer->timer_list[B_SE0_SRP] =
		otg_timer_initializer(ci, &set_tmout_and_fsm, TB_SE0_SRP,
					(unsigned long)&fsm->b_se0_srp);
	if (ci->fsm_timer->timer_list[B_SE0_SRP] == NULL)
		return -ENOMEM;

	ci->fsm_timer->timer_list[B_SSEND_SRP] =
		otg_timer_initializer(ci, &b_ssend_srp_tmout_func, TB_SSEND_SRP,
					(unsigned long)&fsm->b_ssend_srp);
	if (ci->fsm_timer->timer_list[B_SSEND_SRP] == NULL)
		return -ENOMEM;

	ci->fsm_timer->timer_list[B_SRP_FAIL] =
		otg_timer_initializer(ci, &b_srp_fail_tmout_func,
							TB_SRP_FAIL, 0);
	if (ci->fsm_timer->timer_list[B_SRP_FAIL] == NULL)
		return -ENOMEM;

	ci->fsm_timer->timer_list[B_DATA_PLS] =
		otg_timer_initializer(ci, &b_data_pulse_end, TB_DATA_PLS, 0);
	if (ci->fsm_timer->timer_list[B_DATA_PLS] == NULL)
		return -ENOMEM;

	ci->fsm_timer->timer_list[A_DP_END] = otg_timer_initializer(ci,
				&a_wait_dp_end_tmout_func, TA_DP_END, 0);
	if (ci->fsm_timer->timer_list[A_DP_END] == NULL)
		return -ENOMEM;

	ci->fsm_timer->timer_list[B_AIDL_BDIS] = otg_timer_initializer(ci,
					&set_tmout_and_fsm, TB_AIDL_BDIS,
					(unsigned long)&fsm->a_bus_suspend);
	if (ci->fsm_timer->timer_list[B_AIDL_BDIS] == NULL)
		return -ENOMEM;

	setup_timer(&ci->hnp_polling_timer, hnp_polling_timer_work,
							(unsigned long)ci);

	ci->fsm_timer->timer_list[A_TST_MAINT] = otg_timer_initializer(ci,
				&a_tst_maint_tmout_func, TA_TST_MAINT, 0);
	if (ci->fsm_timer->timer_list[A_TST_MAINT] == NULL)
		return -ENOMEM;

	ci->fsm_timer->timer_list[B_SRP_REQD] = otg_timer_initializer(ci,
					&b_srp_reqd_tmout_func, TB_SRP_REQD, 0);
	if (ci->fsm_timer->timer_list[B_SRP_REQD] == NULL)
		return -ENOMEM;

	ci->fsm_timer->timer_list[B_TST_SUSP] = otg_timer_initializer(ci,
					&b_tst_susp_tmout_func, TB_TST_SUSP, 0);
	if (ci->fsm_timer->timer_list[B_TST_SUSP] == NULL)
		return -ENOMEM;

	return 0;
}

static void ci_otg_add_hnp_polling_timer(struct ci_hdrc *ci)
{
	mod_timer(&ci->hnp_polling_timer,
			jiffies + msecs_to_jiffies(T_HOST_REQ_POLL));
}

/* -------------------------------------------------------------*/
/* Operations that will be called from OTG Finite State Machine */
/* -------------------------------------------------------------*/
static void ci_otg_fsm_add_timer(struct otg_fsm *fsm, enum otg_fsm_timer t)
{
	struct ci_hdrc	*ci = container_of(fsm, struct ci_hdrc, fsm);

	if (t < NUM_OTG_FSM_TIMERS) {
		if (t == HNP_POLLING)
			ci_otg_add_hnp_polling_timer(ci);
		else
			ci_otg_add_timer(ci, t);
	}
	return;
}

static void ci_otg_fsm_del_timer(struct otg_fsm *fsm, enum otg_fsm_timer t)
{
	struct ci_hdrc	*ci = container_of(fsm, struct ci_hdrc, fsm);

	if (t < NUM_OTG_FSM_TIMERS)
		ci_otg_del_timer(ci, t);
	return;
}

/*
 * A-device drive vbus: turn on vbus regulator and enable port power
 * Data pulse irq should be disabled while vbus is on.
 */
static void ci_otg_drv_vbus(struct otg_fsm *fsm, int on)
{
	int ret;
	struct ci_hdrc	*ci = container_of(fsm, struct ci_hdrc, fsm);

	if (on) {
		/* Enable power power */
		hw_write(ci, OP_PORTSC, PORTSC_W1C_BITS | PORTSC_PP,
							PORTSC_PP);
		if (ci->platdata->reg_vbus) {
			ret = regulator_enable(ci->platdata->reg_vbus);
			if (ret) {
				dev_err(ci->dev,
				"Failed to enable vbus regulator, ret=%d\n",
				ret);
				return;
			}
		}
		/* Disable data pulse irq */
		hw_write_otgsc(ci, OTGSC_DPIE, 0);

		fsm->a_srp_det = 0;
		fsm->power_up = 0;
	} else {
		if (ci->platdata->reg_vbus)
			regulator_disable(ci->platdata->reg_vbus);

		fsm->a_bus_drop = 1;
		fsm->a_bus_req = 0;
		fsm->b_conn = 0;
	}
}

/*
 * Control data line by Run Stop bit.
 */
static void ci_otg_loc_conn(struct otg_fsm *fsm, int on)
{
	struct ci_hdrc	*ci = container_of(fsm, struct ci_hdrc, fsm);

	if (on)
		hw_write(ci, OP_USBCMD, USBCMD_RS, USBCMD_RS);
	else
		hw_write(ci, OP_USBCMD, USBCMD_RS, 0);
}

/*
 * Generate SOF by host.
 * This is controlled through suspend/resume the port.
 * In host mode, controller will automatically send SOF.
 * Suspend will block the data on the port.
 */
static void ci_otg_loc_sof(struct otg_fsm *fsm, int on)
{
	struct ci_hdrc	*ci = container_of(fsm, struct ci_hdrc, fsm);

	if (on)
		hw_write(ci, OP_PORTSC, PORTSC_W1C_BITS | PORTSC_FPR,
							PORTSC_FPR);
	else
		hw_write(ci, OP_PORTSC, PORTSC_W1C_BITS | PORTSC_SUSP,
							PORTSC_SUSP);
}

/*
 * Start SRP pulsing by data-line pulsing,
 * no v-bus pulsing followed
 */
static void ci_otg_start_pulse(struct otg_fsm *fsm)
{
	struct ci_hdrc	*ci = container_of(fsm, struct ci_hdrc, fsm);

	/* Hardware Assistant Data pulse */
	hw_write_otgsc(ci, OTGSC_HADP, OTGSC_HADP);

	ci_otg_add_timer(ci, B_DATA_PLS);
}

static int ci_otg_start_host(struct otg_fsm *fsm, int on)
{
	struct ci_hdrc	*ci = container_of(fsm, struct ci_hdrc, fsm);

	mutex_unlock(&fsm->lock);
	if (on) {
		/* Disable BSV irq only for A-device */
		if (!ci->fsm.id || ci->transceiver->state > OTG_STATE_B_HOST)
			hw_write_otgsc(ci, OTGSC_BSVIE, 0);
		ci_role_start(ci, CI_ROLE_HOST);
	} else {
		ci_role_stop(ci);
		hw_device_reset(ci);
		ci->role = CI_ROLE_GADGET;
		/* Enable BSV irq only for B-device */
		if (ci->fsm.id)
			hw_write_otgsc(ci, OTGSC_BSVIE, OTGSC_BSVIE);
	}
	mutex_lock(&fsm->lock);
	return 0;
}

static int ci_otg_start_gadget(struct otg_fsm *fsm, int on)
{
	struct ci_hdrc	*ci = container_of(fsm, struct ci_hdrc, fsm);

	mutex_unlock(&fsm->lock);
	if (ci->driver)
		ci_gadget_connect(&ci->gadget, on);
	mutex_lock(&fsm->lock);

	return 0;
}

static struct otg_fsm_ops ci_otg_ops = {
	.drv_vbus = ci_otg_drv_vbus,
	.loc_conn = ci_otg_loc_conn,
	.loc_sof = ci_otg_loc_sof,
	.start_pulse = ci_otg_start_pulse,
	.add_timer = ci_otg_fsm_add_timer,
	.del_timer = ci_otg_fsm_del_timer,
	.start_host = ci_otg_start_host,
	.start_gadget = ci_otg_start_gadget,
};

int ci_otg_fsm_work(struct ci_hdrc *ci)
{
	/*
	 * Handle charger notify in OTG fsm mode
	 */
	if (ci->fsm.id && ci->transceiver->state < OTG_STATE_A_IDLE) {
		unsigned long flags;

		spin_lock_irqsave(&ci->lock, flags);
		if (ci->b_sess_valid_event) {
			ci->b_sess_valid_event = false;
			ci->vbus_active = ci->fsm.b_sess_vld;
			spin_unlock_irqrestore(&ci->lock, flags);
			ci_usb_charger_connect(ci, ci->fsm.b_sess_vld);
			spin_lock_irqsave(&ci->lock, flags);
		}
		spin_unlock_irqrestore(&ci->lock, flags);
		if (!ci->driver)
			return 0;
	}

	pm_runtime_get_sync(ci->dev);
	if (otg_statemachine(&ci->fsm)) {
		if (ci->transceiver->state == OTG_STATE_A_IDLE) {
			/*
			 * Further state change for cases:
			 * a_idle to b_idle, or
			 * a_idle to a_wait_vrise due to ID change(1->0), so
			 * B-dev becomes A-dev can try to start new session
			 * consequently; or
			 * a_idle to a_wait_vrise when power up
			 */
			if ((ci->fsm.id) || (ci->id_event) ||
						(ci->fsm.power_up))
				ci_otg_queue_work(ci);
			if (ci->id_event)
				ci->id_event = false;
		} else if (ci->transceiver->state == OTG_STATE_B_IDLE) {
			if (ci->fsm.b_sess_vld) {
				ci->fsm.power_up = 0;
				/*
				 * Further transite to b_periphearl
				 * when register gadget driver with vbus on
				 */
				ci_otg_queue_work(ci);
			}
		} else if (ci->transceiver->state == OTG_STATE_A_HOST) {
			ci_hdrc_delay_suspend(ci, 2000);
		}
	}
	pm_runtime_put_sync(ci->dev);
	return 0;
}

/*
 * Update fsm variables in each state if catching expected interrupts,
 * called by otg fsm isr.
 */
static void ci_otg_fsm_event(struct ci_hdrc *ci)
{
	u32 intr_sts, otg_bsess_vld, port_conn;
	struct otg_fsm *fsm = &ci->fsm;

	intr_sts = hw_read_intr_status(ci);
	otg_bsess_vld = hw_read_otgsc(ci, OTGSC_BSV);
	port_conn = hw_read(ci, OP_PORTSC, PORTSC_CCS);

	switch (ci->transceiver->state) {
	case OTG_STATE_A_WAIT_BCON:
		if (port_conn) {
			fsm->b_conn = 1;
			fsm->a_bus_req = 1;
			ci_otg_queue_work(ci);
		}
		break;
	case OTG_STATE_B_IDLE:
		if (otg_bsess_vld && (intr_sts & USBi_PCI) && port_conn) {
			fsm->b_sess_vld = 1;
			ci_otg_queue_work(ci);
		}
		break;
	case OTG_STATE_B_PERIPHERAL:
		if ((intr_sts & USBi_SLI) && port_conn && otg_bsess_vld) {
			ci_otg_add_timer(ci, B_AIDL_BDIS);
		} else if (intr_sts & USBi_PCI) {
			ci_otg_del_timer(ci, B_AIDL_BDIS);
			if (fsm->a_bus_suspend == 1)
				fsm->a_bus_suspend = 0;
		}
		break;
	case OTG_STATE_B_HOST:
		if ((intr_sts & USBi_PCI) && !port_conn) {
			fsm->a_conn = 0;
			fsm->b_bus_req = 0;
			ci_otg_queue_work(ci);
		}
		break;
	case OTG_STATE_A_PERIPHERAL:
		if (intr_sts & USBi_SLI) {
			 fsm->b_bus_suspend = 1;
			/*
			 * Init a timer to know how long this suspend
			 * will contine, if time out, indicates B no longer
			 * wants to be host role
			 */
			 ci_otg_add_timer(ci, A_BIDL_ADIS);
		}

		if (intr_sts & USBi_URI)
			ci_otg_del_timer(ci, A_BIDL_ADIS);

		if (intr_sts & USBi_PCI) {
			if (fsm->b_bus_suspend == 1) {
				ci_otg_del_timer(ci, A_BIDL_ADIS);
				fsm->b_bus_suspend = 0;
			}
		}
		break;
	case OTG_STATE_A_SUSPEND:
		if ((intr_sts & USBi_PCI) && !port_conn) {
			fsm->b_conn = 0;

			/* if gadget driver is binded */
			if (ci->driver) {
				/* A device to be peripheral mode */
				ci->gadget.is_a_peripheral = 1;
			}
			ci_otg_queue_work(ci);
		}
		break;
	case OTG_STATE_A_HOST:
		if ((intr_sts & USBi_PCI) && !port_conn) {
			fsm->b_conn = 0;
			if (fsm->tst_maint) {
				ci_otg_del_timer(ci, A_TST_MAINT);
				if (fsm->otg_vbus_off) {
					fsm->a_bus_req = 0;
					fsm->a_bus_drop = 1;
					fsm->otg_vbus_off = 0;
				}
				fsm->tst_maint = 0;
			}
			ci_otg_queue_work(ci);
		}
		break;
	case OTG_STATE_B_WAIT_ACON:
		if ((intr_sts & USBi_PCI) && port_conn) {
			fsm->a_conn = 1;
			ci_otg_queue_work(ci);
		}
		break;
	default:
		break;
	}
}

/*
 * ci_otg_irq - otg fsm related irq handling
 * and also update otg fsm variable by monitoring usb host and udc
 * state change interrupts.
 * @ci: ci_hdrc
 */
irqreturn_t ci_otg_fsm_irq(struct ci_hdrc *ci)
{
	irqreturn_t retval =  IRQ_NONE;
	u32 otgsc, otg_int_src = 0;
	struct otg_fsm *fsm = &ci->fsm;

	otgsc = hw_read_otgsc(ci, ~0);
	otg_int_src = otgsc & OTGSC_INT_STATUS_BITS & (otgsc >> 8);
	fsm->id = (otgsc & OTGSC_ID) ? 1 : 0;

	if (otg_int_src) {
		if (otg_int_src & OTGSC_1MSIS) {
			hw_write_otgsc(ci, OTGSC_1MSIS, OTGSC_1MSIS);
			retval = ci_otg_tick_timer(ci);
			return IRQ_HANDLED;
		} else if (otg_int_src & OTGSC_DPIS) {
			hw_write_otgsc(ci, OTGSC_DPIS, OTGSC_DPIS);
			ci_otg_add_timer(ci, A_DP_END);
		} else if (otg_int_src & OTGSC_IDIS) {
			hw_write_otgsc(ci, OTGSC_IDIS, OTGSC_IDIS);
			if (fsm->id == 0) {
				fsm->a_bus_drop = 0;
				fsm->a_bus_req = 1;
				ci->id_event = true;
			}
		} else if (otg_int_src & OTGSC_BSVIS) {
			hw_write_otgsc(ci, OTGSC_BSVIS, OTGSC_BSVIS);
			if (!(otgsc & OTGSC_BSV) && fsm->b_sess_vld) {
				ci->b_sess_valid_event = true;
				fsm->b_sess_vld = 0;
				if (fsm->id)
					ci_otg_add_timer(ci, B_SSEND_SRP);
				if (fsm->b_bus_req)
					fsm->b_bus_req = 0;
				if (fsm->otg_srp_reqd)
					ci_otg_add_timer(ci, B_SRP_REQD);
			} else {
				ci->vbus_glitch_check_event = true;
			}
		} else if (otg_int_src & OTGSC_AVVIS) {
			hw_write_otgsc(ci, OTGSC_AVVIS, OTGSC_AVVIS);
			if (otgsc & OTGSC_AVV) {
				fsm->a_vbus_vld = 1;
			} else {
				fsm->a_vbus_vld = 0;
				fsm->b_conn = 0;
			}
		}
		ci_otg_queue_work(ci);
		return IRQ_HANDLED;
	}

	ci_otg_fsm_event(ci);

	return retval;
}

void ci_hdrc_otg_fsm_start(struct ci_hdrc *ci)
{
	ci_otg_queue_work(ci);
}

int ci_hdrc_otg_fsm_init(struct ci_hdrc *ci)
{
	int retval = 0;
	struct usb_otg *otg;

	otg = devm_kzalloc(ci->dev,
			sizeof(struct usb_otg), GFP_KERNEL);
	if (!otg) {
		dev_err(ci->dev,
		"Failed to allocate usb_otg structure for ci hdrc otg!\n");
		return -ENOMEM;
	}

	otg->phy = ci->transceiver;
	otg->gadget = &ci->gadget;
	otg->fsm = &ci->fsm;
	if (ci->hcd)
		otg->host = &ci->hcd->self;
	ci->fsm.otg = otg;
	ci->transceiver->otg = ci->fsm.otg;
	ci->fsm.power_up = 1;
	ci->fsm.hnp_polling = 1;
	ci->fsm.id = hw_read_otgsc(ci, OTGSC_ID) ? 1 : 0;
	ci->transceiver->state = OTG_STATE_UNDEFINED;
	ci->fsm.ops = &ci_otg_ops;

	mutex_init(&ci->fsm.lock);

	ci->fsm_timer = devm_kzalloc(ci->dev,
			sizeof(struct ci_otg_fsm_timer_list), GFP_KERNEL);
	if (!ci->fsm_timer) {
		dev_err(ci->dev,
		"Failed to allocate timer structure for ci hdrc otg!\n");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&ci->fsm_timer->active_timers);
	retval = ci_otg_init_timers(ci);
	if (retval) {
		dev_err(ci->dev, "Couldn't init OTG timers\n");
		return retval;
	}

	retval = sysfs_create_group(&ci->dev->kobj, &inputs_attr_group);
	if (retval < 0) {
		dev_dbg(ci->dev,
			"Can't register sysfs attr group: %d\n", retval);
		return retval;
	}

	INIT_WORK(&ci->hnp_polling_work, ci_hnp_polling_work);

	/* Enable A vbus valid irq */
	hw_write_otgsc(ci, OTGSC_AVVIE, OTGSC_AVVIE);

	if (ci->fsm.id) {
		ci->fsm.b_ssend_srp =
			hw_read_otgsc(ci, OTGSC_BSV) ? 0 : 1;
		ci->fsm.b_sess_vld =
			hw_read_otgsc(ci, OTGSC_BSV) ? 1 : 0;
		/* Enable BSV irq */
		hw_write_otgsc(ci, OTGSC_BSVIE, OTGSC_BSVIE);
	}

	return 0;
}

void ci_hdrc_otg_fsm_remove(struct ci_hdrc *ci)
{
	sysfs_remove_group(&ci->dev->kobj, &inputs_attr_group);
	del_timer_sync(&ci->hnp_polling_timer);
}

/* Restart OTG fsm if resume from power lost */
void ci_hdrc_otg_fsm_restart(struct ci_hdrc *ci)
{
	struct otg_fsm *fsm = &ci->fsm;
	int id_status = fsm->id;

	/* Update fsm if power lost in peripheral state */
	if (ci->transceiver->state == OTG_STATE_B_PERIPHERAL) {
		fsm->b_sess_vld = 0;
		otg_statemachine(fsm);
	}

	hw_write_otgsc(ci, OTGSC_IDIE, OTGSC_IDIE);
	hw_write_otgsc(ci, OTGSC_AVVIE, OTGSC_AVVIE);

	/* Update fsm variables for restart */
	fsm->id = hw_read_otgsc(ci, OTGSC_ID) ? 1 : 0;
	if (fsm->id) {
		fsm->b_ssend_srp =
			hw_read_otgsc(ci, OTGSC_BSV) ? 0 : 1;
		fsm->b_sess_vld =
			hw_read_otgsc(ci, OTGSC_BSV) ? 1 : 0;
	} else if (fsm->id != id_status) {
		/* ID changes to be 0 */
		fsm->a_bus_drop = 0;
		fsm->a_bus_req = 1;
		ci->id_event = true;
	}

	if (ci_hdrc_host_has_device(ci) &&
			!hw_read(ci, OP_PORTSC, PORTSC_CCS))
		fsm->b_conn = 0;

	ci_otg_fsm_work(ci);
}
