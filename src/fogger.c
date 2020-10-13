/*
 * Copyright (c) 2020 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <drivers/gpio.h>
#include "fogger.h"

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

static void status_notify(void)
{
    if (NULL != m_status_cb) {
        m_status_cb(status_get());
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

static void input_changed(struct device *dev, struct gpio_callback *cb, u32_t pins)
{
    /* The pins variable is a mask that describes pins in the form (1<<PIN_NUMBER). */
    int val;

    if (dev == m_usb_detect_dev && (pins & BIT(m_usb_detect_pin))) {
        val = gpio_pin_get(dev, m_usb_detect_pin);
        if (val) {
            m_machine_ready = true;
        } else {
            m_machine_ready = false;
            (void) relay_set(false);
        }
        status_notify();
    }
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

    if (m_machine_ready) {
        if (relay_set(true)) {
            status_notify();
        }
    }

    return 0;
}

int fogger_stop(void)
{
    if (!m_initialized) {
        return -1;
    }

    if (relay_set(false)) {
        status_notify();      
    }

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

    m_relay_engaged  = false;
    m_machine_ready  = gpio_pin_get(m_usb_detect_dev, m_usb_detect_pin);
    m_status_cb      = p_status_cb;
    m_initialized    = true;

    return 0;
}
