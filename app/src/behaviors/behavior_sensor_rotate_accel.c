/*
 * Copyright (c) 2022 The ZMK Contributors
 * Copyright (c) 2026 LiNEA40 contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_sensor_rotate_accel

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/virtual_key_position.h>

#include <dt-bindings/zmk/pointing.h>

#include "behavior_sensor_rotate_common.h"

struct behavior_sensor_rotate_accel_config {
    struct zmk_behavior_binding cw_binding;
    struct zmk_behavior_binding ccw_binding;
    int tap_ms;
    int slow_interval_ms;
    int fast_interval_ms;
    int slow_scale_percent;
    int fast_scale_percent;
};

struct behavior_sensor_rotate_accel_data {
    struct behavior_sensor_rotate_data common;
    int64_t last_trigger_ms[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    struct k_work_delayable release_work;
    struct zmk_behavior_binding active_binding;
    struct zmk_behavior_binding_event active_event;
    bool active;
};

static int select_scale_percent(const struct behavior_sensor_rotate_accel_config *config,
                                struct behavior_sensor_rotate_accel_data *data, int sensor_index,
                                int layer, int triggers) {
    const int64_t now = k_uptime_get();
    const int64_t last_trigger_ms = data->last_trigger_ms[sensor_index][layer];
    int scale_percent = config->slow_scale_percent;

    if (triggers > 1 || triggers < -1 ||
        (last_trigger_ms != 0 && now - last_trigger_ms <= config->fast_interval_ms)) {
        scale_percent = config->fast_scale_percent;
    } else if (last_trigger_ms != 0 && now - last_trigger_ms < config->slow_interval_ms &&
               config->slow_interval_ms > config->fast_interval_ms) {
        const int elapsed_ms = now - last_trigger_ms;
        const int interval_span = config->slow_interval_ms - config->fast_interval_ms;
        const int scale_span = config->slow_scale_percent - config->fast_scale_percent;

        scale_percent =
            config->fast_scale_percent + (scale_span * (elapsed_ms - config->fast_interval_ms)) /
                                             interval_span;
    }

    data->last_trigger_ms[sensor_index][layer] = now;
    return scale_percent;
}

static uint32_t scale_movement(uint32_t movement, int scale_percent) {
    const int16_t x = MOVE_X_DECODE(movement);
    const int16_t y = MOVE_Y_DECODE(movement);

    return MOVE((x * scale_percent) / 100, (y * scale_percent) / 100);
}

static void behavior_sensor_rotate_accel_release(struct k_work *work) {
    struct k_work_delayable *release_work = k_work_delayable_from_work(work);
    struct behavior_sensor_rotate_accel_data *data =
        CONTAINER_OF(release_work, struct behavior_sensor_rotate_accel_data, release_work);

    if (!data->active) {
        return;
    }

    zmk_behavior_invoke_binding(&data->active_binding, data->active_event, false);
    data->active = false;
}

static int behavior_sensor_rotate_accel_init(const struct device *dev) {
    struct behavior_sensor_rotate_accel_data *data = dev->data;

    k_work_init_delayable(&data->release_work, behavior_sensor_rotate_accel_release);

    return 0;
}

static void behavior_sensor_rotate_accel_start(
    const struct behavior_sensor_rotate_accel_config *config,
    struct behavior_sensor_rotate_accel_data *data,
    const struct zmk_behavior_binding *binding,
    struct zmk_behavior_binding_event event) {
    if (data->active) {
        zmk_behavior_invoke_binding(&data->active_binding, data->active_event, false);
        data->active = false;
    }

    data->active_binding = *binding;
    data->active_event = event;

    if (zmk_behavior_invoke_binding(&data->active_binding, data->active_event, true) != 0) {
        return;
    }

    data->active = true;
    k_work_reschedule(&data->release_work, K_MSEC(config->tap_ms));
}

static int behavior_sensor_rotate_accel_process(
    struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event,
    enum behavior_sensor_binding_process_mode mode) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_rotate_accel_config *config = dev->config;
    struct behavior_sensor_rotate_accel_data *data = dev->data;
    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    if (mode != BEHAVIOR_SENSOR_BINDING_PROCESS_MODE_TRIGGER) {
        data->common.triggers[sensor_index][event.layer] = 0;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    int triggers = data->common.triggers[sensor_index][event.layer];
    struct zmk_behavior_binding triggered_binding;

    if (triggers > 0) {
        triggered_binding = config->cw_binding;
        triggered_binding.param1 =
            scale_movement(binding->param1, select_scale_percent(config, data, sensor_index,
                                                                  event.layer, triggers));
    } else if (triggers < 0) {
        triggers = -triggers;
        triggered_binding = config->ccw_binding;
        triggered_binding.param1 =
            scale_movement(binding->param2, select_scale_percent(config, data, sensor_index,
                                                                  event.layer, -triggers));
    } else {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    event.source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    behavior_sensor_rotate_accel_start(config, data, &triggered_binding, event);

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_sensor_rotate_accel_driver_api = {
    .sensor_binding_accept_data = zmk_behavior_sensor_rotate_common_accept_data,
    .sensor_binding_process = behavior_sensor_rotate_accel_process,
};

#define SENSOR_ROTATE_ACCEL_INST(n)                                                               \
    static struct behavior_sensor_rotate_accel_config behavior_sensor_rotate_accel_config_##n = { \
        .cw_binding = {.behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 0))},  \
        .ccw_binding = {.behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 1))}, \
        .tap_ms = DT_INST_PROP(n, tap_ms),                                                        \
        .slow_interval_ms = DT_INST_PROP(n, slow_interval_ms),                                    \
        .fast_interval_ms = DT_INST_PROP(n, fast_interval_ms),                                    \
        .slow_scale_percent = DT_INST_PROP(n, slow_scale_percent),                                \
        .fast_scale_percent = DT_INST_PROP(n, fast_scale_percent),                                \
    };                                                                                            \
    static struct behavior_sensor_rotate_accel_data behavior_sensor_rotate_accel_data_##n = {};   \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_sensor_rotate_accel_init, NULL, &behavior_sensor_rotate_accel_data_##n,               \
                            &behavior_sensor_rotate_accel_config_##n, POST_KERNEL,               \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                 \
                            &behavior_sensor_rotate_accel_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SENSOR_ROTATE_ACCEL_INST)
