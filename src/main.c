/*
 * Copyright (c) 2020 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <device.h>
#include <drivers/gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <bluetooth/mesh/dk_prov.h>
#include "model_handler.h"
#include "fogger.h"

#define SWITCH_DEBOUNCE_MS  5

enum fogger_elem {
    FOGGER_ELEM_BUTTON,
    FOGGER_ELEM_READY
};

static const char * const   m_button_port  = DT_GPIO_LABEL(DT_NODELABEL(button0), gpios);
static const u8_t           m_button_pin   = DT_GPIO_PIN(DT_NODELABEL(button0),   gpios);
static const u32_t          m_button_flags = DT_GPIO_FLAGS(DT_NODELABEL(button0), gpios);
static struct device       *m_button_dev;

static struct gpio_callback m_gpio_cb_data;

static const char * const   m_err_led_port  = DT_GPIO_LABEL(DT_NODELABEL(led2), gpios);
static const u8_t           m_err_led_pin   = DT_GPIO_PIN(DT_NODELABEL(led2),   gpios);
static const u32_t          m_err_led_flags = DT_GPIO_FLAGS(DT_NODELABEL(led2), gpios);
static struct device       *m_err_led_dev;

static bool m_button_pressed;

static void debounce_timer_expire(struct k_timer *timer_id);

K_TIMER_DEFINE(m_button_debounce_timer, debounce_timer_expire, NULL);

static void oops(void);
static void fogger_callback(enum fogger_state status);

static struct k_work fogger_status_work;

static void workq_fogger_status(struct k_work *item)
{
    int err;
    enum fogger_state state;

    err = fogger_state_get(&state);
    if (err) {
        oops();
        return;
    }
    fogger_callback(state);
}

static void debounce_timer_expire(struct k_timer *timer_id)
{
    int val = gpio_pin_get(m_button_dev, m_button_pin);
    if (val) {
        m_button_pressed = true;
        fogger_start();
    } else {
        m_button_pressed = false;
        fogger_stop();
    }
}

static void input_changed(struct device *dev, struct gpio_callback *cb, u32_t pins)
{
    k_timer_start(&m_button_debounce_timer, K_MSEC(SWITCH_DEBOUNCE_MS), K_MSEC(0));
}

static int gpio_init(void)
{
	int ret;

    m_button_dev = device_get_binding(m_button_port);
    if (!m_button_dev) {
        return -ENODEV;
    }

    m_err_led_dev = device_get_binding(m_err_led_port);
    if (!m_err_led_dev) {
        return -ENODEV;
    }

    ret = gpio_pin_configure(m_button_dev, m_button_pin, (GPIO_INPUT | m_button_flags));
    if (ret != 0) {
        return ret;
    }

    ret = gpio_pin_configure(m_err_led_dev, m_err_led_pin, (GPIO_OUTPUT | m_err_led_flags));
    if (ret != 0) {
        return ret;
    }

    ret = gpio_pin_interrupt_configure(m_button_dev,
                                       m_button_pin,
                                       GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        return ret;
    }

    gpio_init_callback(&m_gpio_cb_data, input_changed, BIT(m_button_pin));
    gpio_add_callback(m_button_dev, &m_gpio_cb_data);

    return 0;
}

static void oops(void)
{
    gpio_pin_set(m_err_led_dev, m_err_led_pin, 1);
    k_oops();
}

static void model_handler_callback(uint8_t elem_indx, bool status)
{
    switch (elem_indx) {
    case FOGGER_ELEM_BUTTON:
        if (status) {
            fogger_start();
        } else {
            fogger_stop();
        }
        break;
    case FOGGER_ELEM_READY:
        /* Writing to this element doesn't have any effect. Immediately generate a status
           callback to ensure that the value in the mesh is up-to-date. */
        k_work_submit(&fogger_status_work);
        break;
    default:
        oops();
        break;
    }
}

static void fogger_callback(enum fogger_state status)
{
    int err;

    switch (status) {
    case FOGGER_STATE_HEATING:
        err = model_handler_elem_update(FOGGER_ELEM_BUTTON, false);
        if (err) {
            oops();
            return;
        }
        err = model_handler_elem_update(FOGGER_ELEM_READY, false);
        if (err) {
            oops();
            return;
        }
        break;
    case FOGGER_STATE_READY:
        if (m_button_pressed) {
            fogger_start();
        }
        err = model_handler_elem_update(FOGGER_ELEM_BUTTON, false);
        if (err) {
            oops();
            return;
        }
        err = model_handler_elem_update(FOGGER_ELEM_READY, true);
        if (err) {
            oops();
            return;
        }
        break;
    case FOGGER_STATE_FOGGING:
        err = model_handler_elem_update(FOGGER_ELEM_BUTTON, true);
        if (err) {
            oops();
            return;
        }
        err = model_handler_elem_update(FOGGER_ELEM_READY, true);
        if (err) {
            oops();
            return;
        }
        break;
    default:
        oops();
        break;
    }
}

static void bt_ready(int err)
{
    const struct bt_mesh_comp *p_comp;

    if (err) {
        oops();
        return;
    }

    err = model_handler_init(&p_comp, &model_handler_callback);
    if (err) {
        oops();
        return;
    }

    err = bt_mesh_init(bt_mesh_dk_prov_init(), p_comp);
    if (err) {
        oops();
        return;
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    /* This will be a no-op if settings_load() loaded provisioning info */
    bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

    k_work_submit(&fogger_status_work);
}

int main(void)
{
	int err;

    k_work_init(&fogger_status_work,workq_fogger_status);

    err = gpio_init();
    if (err) {
        oops();
    }

    m_button_pressed = gpio_pin_get(m_button_dev, m_button_pin);

    err = fogger_init(&fogger_callback);
    if (err) {
        oops();
    }

    err = bt_enable(bt_ready);
    if (err) {
        oops();
    }

    return 0;
}
