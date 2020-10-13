/*
 * Copyright (c) 2020 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <drivers/gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include "model_handler.h"


static bool                 m_initialized = false;
static model_handler_set_cb m_set_cb = NULL;

static const char * const   m_attention_led_port  = DT_GPIO_LABEL(DT_NODELABEL(led1), gpios);
static const u8_t           m_attention_led_pin   = DT_GPIO_PIN(DT_NODELABEL(led1),   gpios);
static const u32_t          m_attention_led_flags = DT_GPIO_FLAGS(DT_NODELABEL(led1), gpios);
static struct device       *m_attention_led_dev;

static void led_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
            const struct bt_mesh_onoff_set *set,
            struct bt_mesh_onoff_status *rsp);

static void led_get(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
            struct bt_mesh_onoff_status *rsp);

static const struct bt_mesh_onoff_srv_handlers onoff_handlers = {
    .set = led_set,
    .get = led_get,
};

struct led_ctx {
    struct bt_mesh_onoff_srv srv;
    struct k_delayed_work work;
    u32_t remaining;
    bool value;
};

static struct led_ctx led_ctx[ELEMENT_COUNT] = {
    [0 ... 1] = {
        .srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers),
    }
};

static void led_transition_start(struct led_ctx *led)
{
    int led_idx = led - &led_ctx[0];

    /* As long as the transition is in progress, the onoff
     * state is "on":
     */

    m_set_cb(led_idx, 1);
    k_delayed_work_submit(&led->work, K_MSEC(led->remaining));
    led->remaining = 0;
}

static void led_status(struct led_ctx *led, struct bt_mesh_onoff_status *status)
{
    status->remaining_time =
        k_delayed_work_remaining_get(&led->work) + led->remaining;
    status->target_on_off = led->value;
    /* As long as the transition is in progress, the onoff state is "on": */
    status->present_on_off = led->value || status->remaining_time;
}

static void led_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
            const struct bt_mesh_onoff_set *set,
            struct bt_mesh_onoff_status *rsp)
{
    struct led_ctx *led = CONTAINER_OF(srv, struct led_ctx, srv);
    int led_idx = led - &led_ctx[0];

    if (set->on_off == led->value) {
        goto respond;
    }

    led->value = set->on_off;
    led->remaining = set->transition->time;

    if (set->transition->delay > 0) {
        k_delayed_work_submit(&led->work,
                      K_MSEC(set->transition->delay));
    } else if (set->transition->time > 0) {
        led_transition_start(led);
    } else {
        m_set_cb(led_idx, set->on_off);
    }

respond:
    if (rsp) {
        led_status(led, rsp);
    }
}

static void led_get(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
            struct bt_mesh_onoff_status *rsp)
{
    struct led_ctx *led = CONTAINER_OF(srv, struct led_ctx, srv);

    led_status(led, rsp);
}

static void led_work(struct k_work *work)
{
    struct led_ctx *led = CONTAINER_OF(work, struct led_ctx, work.work);
    int led_idx = led - &led_ctx[0];

    if (led->remaining) {
        led_transition_start(led);
    } else {
        m_set_cb(led_idx, led->value);

        /* Publish the new value at the end of the transition */
        struct bt_mesh_onoff_status status;

        led_status(led, &status);
        bt_mesh_onoff_srv_pub(&led->srv, NULL, &status);
    }
}

/** Configuration server definition */
static struct bt_mesh_cfg_srv cfg_srv = {
    .relay = IS_ENABLED(CONFIG_BT_MESH_RELAY),
    .beacon = BT_MESH_BEACON_ENABLED,
    .frnd = IS_ENABLED(CONFIG_BT_MESH_FRIEND),
    .gatt_proxy = IS_ENABLED(CONFIG_BT_MESH_GATT_PROXY),
    .default_ttl = 7,

    /* 3 transmissions with 20ms interval */
    .net_transmit = BT_MESH_TRANSMIT(2, 20),
    .relay_retransmit = BT_MESH_TRANSMIT(2, 20),
};

/* Set up a repeating delayed work to blink the DK's LEDs when attention is
 * requested.
 */
static struct k_delayed_work attention_blink_work;

static void attention_blink(struct k_work *work)
{
    static bool state=true;
    if (m_attention_led_dev) {
        gpio_pin_set(m_attention_led_dev, m_attention_led_pin, state);
        state = !state;
    }
    k_delayed_work_submit(&attention_blink_work, K_MSEC(100));
}

static void attention_on(struct bt_mesh_model *mod)
{
    k_delayed_work_submit(&attention_blink_work, K_NO_WAIT);
}

static void attention_off(struct bt_mesh_model *mod)
{
    k_delayed_work_cancel(&attention_blink_work);
    if (m_attention_led_dev) {
        gpio_pin_set(m_attention_led_dev, m_attention_led_pin, 0);
    }
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
    .attn_on = attention_on,
    .attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(
        1, BT_MESH_MODEL_LIST(
            BT_MESH_MODEL_CFG_SRV(&cfg_srv),
            BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
            BT_MESH_MODEL_ONOFF_SRV(&led_ctx[0].srv)),
        BT_MESH_MODEL_NONE),
    BT_MESH_ELEM(
        2, BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&led_ctx[1].srv)),
        BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
    .cid = CONFIG_BT_COMPANY_ID,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

int model_handler_elem_update(uint8_t elem_indx, bool status)
{
    if (!m_initialized) {
        return -1;
    }

    if (ELEMENT_COUNT <= elem_indx) {
        return -2;
    }

    struct led_ctx *led;
    led            = &led_ctx[elem_indx];
    led->value     = status;
    led->remaining = 0;

    struct bt_mesh_onoff_status onoff_status;
    onoff_status.remaining_time = 0;
    onoff_status.target_on_off  = status;
    onoff_status.present_on_off = status;

    bt_mesh_onoff_srv_pub(&led->srv, NULL, &onoff_status);
    return 0;
};

const int model_handler_init(const struct bt_mesh_comp **p_comp, model_handler_set_cb p_set_cb)
{
    if (NULL == p_set_cb) {
        return -1;
    }

    *p_comp = &comp;

    k_delayed_work_init(&attention_blink_work, attention_blink);

    for (int i = 0; i < ARRAY_SIZE(led_ctx); ++i) {
        k_delayed_work_init(&led_ctx[i].work, led_work);
    }

    m_attention_led_dev = device_get_binding(m_attention_led_port);
    if (m_attention_led_dev) {
        (void) gpio_pin_configure(m_attention_led_dev,
                                  m_attention_led_pin,
                                  (GPIO_OUTPUT | m_attention_led_flags));
    }

    m_set_cb      = p_set_cb;
    m_initialized = true;

    return 0;
}
