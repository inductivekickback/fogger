/*
 * Copyright (c) 2020 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <drivers/gpio.h>
#include "fogger.h"

#define SWITCH_DEBOUNCE_MS  50

static const char * const   m_usb_detect_port  = DT_GPIO_LABEL(DT_NODELABEL(usb_detect), gpios);
static const u8_t           m_usb_detect_pin   = DT_GPIO_PIN(DT_NODELABEL(usb_detect),   gpios);
static const u32_t          m_usb_detect_flags = DT_GPIO_FLAGS(DT_NODELABEL(usb_detect), gpios);
static struct device       *m_usb_detect_dev;

static struct gpio_callback m_gpio_cb_data;

static const char * const   m_relay_ctrl_port  = DT_GPIO_LABEL(DT_NODELABEL(relay_ctrl), gpios);
static const u8_t           m_relay_ctrl_pin   = DT_GPIO_PIN(DT_NODELABEL(relay_ctrl),   gpios);
static const u32_t          m_relay_ctrl_flags = DT_GPIO_FLAGS(DT_NODELABEL(relay_ctrl), gpios);
static struct device       *m_relay_ctrl_dev;

static bool             m_initialized = false;
static bool             m_relay_engaged;
static bool             m_machine_ready;
static fogger_status_cb m_status_cb;

static void debounce_timer_expire(struct k_timer *timer_id);

K_TIMER_DEFINE(m_debounce_timer, debounce_timer_expire, NULL);

static struct k_work m_fogger_status_work;
static struct k_work m_fogger_start_work;
static struct k_work m_fogger_stop_work;

static enum fogger_state status_get(void)
{
    if (!m_machine_ready) {
        return FOGGER_STATE_HEATING;
    } else if (m_relay_engaged) {
        return FOGGER_STATE_FOGGING;
    } else {
        return FOGGER_STATE_READY;
    }
}

static bool relay_set(bool state)
{
    if (state) {
        if (!m_relay_engaged) {
            m_relay_engaged = true;
            gpio_pin_set(m_relay_ctrl_dev, m_relay_ctrl_pin, 1);
            return true;
        }
    } else {
        if (m_relay_engaged) {
            m_relay_engaged = false;
            gpio_pin_set(m_relay_ctrl_dev, m_relay_ctrl_pin, 0);
            return true;
        }
    }
    return false;
}

static void workq_status_cb(struct k_work *item)
{
    if (NULL != m_status_cb) {
        m_status_cb(status_get());
    }
}

static void workq_fogger_start(struct k_work *item)
{
    if (m_machine_ready) {
        if (relay_set(true)) {
            if (NULL != m_status_cb) {
                m_status_cb(FOGGER_STATE_FOGGING);
            }
        }
    }
}

static void workq_fogger_stop(struct k_work *item)
{
    if (relay_set(false)) {
        if (NULL != m_status_cb) {
            m_status_cb(status_get());
        }
    }
}

static void debounce_timer_expire(struct k_timer *timer_id)
{
    int val = gpio_pin_get(m_usb_detect_dev, m_usb_detect_pin);
    if (val) {
        if (!m_machine_ready) {
            k_work_submit(&m_fogger_status_work);
        }
        m_machine_ready = true;
    } else {
        if (m_machine_ready) {
            if (m_relay_engaged) {
                k_work_submit(&m_fogger_stop_work);
            } else {
                k_work_submit(&m_fogger_status_work);
            }
        }
        m_machine_ready = false;
    }
}

static void input_changed(struct device *dev, struct gpio_callback *cb, u32_t pins)
{
    k_timer_start(&m_debounce_timer, K_MSEC(SWITCH_DEBOUNCE_MS), K_MSEC(0));
}

static int gpio_init(void)
{
    int ret;

    m_usb_detect_dev = device_get_binding(m_usb_detect_port);
    if (!m_usb_detect_dev) {
        return -ENODEV;
    }

    m_relay_ctrl_dev = device_get_binding(m_relay_ctrl_port);
    if (!m_relay_ctrl_dev) {
        return -ENODEV;
    }

    ret = gpio_pin_configure(m_usb_detect_dev, m_usb_detect_pin, (GPIO_INPUT | m_usb_detect_flags));
    if (ret != 0) {
        return ret;
    }

    ret = gpio_pin_configure(m_relay_ctrl_dev, m_relay_ctrl_pin, (GPIO_OUTPUT | m_relay_ctrl_flags));
    if (ret != 0) {
        return ret;
    }

    ret = gpio_pin_interrupt_configure(m_usb_detect_dev,
                                       m_usb_detect_pin,
                                       GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        return ret;
    }

    gpio_init_callback(&m_gpio_cb_data, input_changed, BIT(m_usb_detect_pin));
    gpio_add_callback(m_usb_detect_dev, &m_gpio_cb_data);

    return 0;
}

int fogger_start(void)
{
    if (!m_initialized) {
        return -1;
    }
    k_work_submit(&m_fogger_start_work);
    return 0;
}

int fogger_stop(void)
{
    if (!m_initialized) {
        return -1;
    }
    k_work_submit(&m_fogger_stop_work);
    return 0;
}

int fogger_state_get(enum fogger_state *p_state)
{
    if (!m_initialized) {
        return -1;
    }

    if (NULL == p_state) {
        return -2;
    }

    *p_state = status_get();
    return 0;
}

int fogger_init(fogger_status_cb p_status_cb)
{
    int err = gpio_init();
    if (err) {
        return -2;
    }

    gpio_pin_set(m_relay_ctrl_dev, m_relay_ctrl_pin, 0);

    k_work_init(&m_fogger_status_work, workq_status_cb);
    k_work_init(&m_fogger_stop_work,   workq_fogger_stop);
    k_work_init(&m_fogger_start_work,  workq_fogger_start);

    m_relay_engaged  = false;
    m_machine_ready  = gpio_pin_get(m_usb_detect_dev, m_usb_detect_pin);
    m_status_cb      = p_status_cb;
    m_initialized    = true;

    return 0;
}
