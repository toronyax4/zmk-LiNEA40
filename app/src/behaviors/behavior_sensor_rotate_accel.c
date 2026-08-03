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
    int direction_confirm_ms;
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
    int active_scale_percent;
    int active_direction;
    int confirmed_direction;
    int pending_direction;
    int64_t pending_direction_ms;
    bool active;
};

static int select_scale_percent(const struct behavior_sensor_rotate_accel_config *config,
                                struct behavior_sensor_rotate_accel_data *data, int sensor_index,
                                int layer) {
    const int64_t now = k_uptime_get();
    const int64_t last_trigger_ms = data->last_trigger_ms[sensor_index][layer];
    int scale_percent = config->slow_scale_percent;

    if (last_trigger_ms != 0 && now - last_trigger_ms <= config->fast_interval_ms) {
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

static int16_t scale_axis_movement(int16_t axis, int scale_percent, int triggers) {
    const int64_t scaled = ((int64_t)axis * scale_percent * triggers) / 100;

    if (scaled > INT16_MAX) {
        return INT16_MAX;
    }
    if (scaled < INT16_MIN) {
        return INT16_MIN;
    }

    return (int16_t)scaled;
}

static uint32_t scale_movement(uint32_t movement, int scale_percent, int triggers) {
    const int16_t x = MOVE_X_DECODE(movement);
    const int16_t y = MOVE_Y_DECODE(movement);

    return MOVE(scale_axis_movement(x, scale_percent, triggers),
                scale_axis_movement(y, scale_percent, triggers));
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
    data->active_scale_percent = 0;
    data->active_direction = 0;
}

static int behavior_sensor_rotate_accel_init(const struct device *dev) {
    struct behavior_sensor_rotate_accel_data *data = dev->data;

    k_work_init_delayable(&data->release_work, behavior_sensor_rotate_accel_release);

    return 0;
}

static bool behavior_sensor_rotate_accel_confirm_direction(
    const struct behavior_sensor_rotate_accel_config *config,
    struct behavior_sensor_rotate_accel_data *data,
    int direction) {
    const int64_t now = k_uptime_get();

    if (data->confirmed_direction == 0 || direction == data->confirmed_direction) {
        data->confirmed_direction = direction;
        data->pending_direction = 0;
        data->pending_direction_ms = 0;
        return true;
    }

    if (data->pending_direction == direction &&
        now - data->pending_direction_ms <= config->direction_confirm_ms) {
        data->confirmed_direction = direction;
        data->pending_direction = 0;
        data->pending_direction_ms = 0;
        return true;
    }

    data->pending_direction = direction;
    data->pending_direction_ms = now;
    return false;
}

static void behavior_sensor_rotate_accel_start(
    const struct behavior_sensor_rotate_accel_config *config,
    struct behavior_sensor_rotate_accel_data *data,
    const struct zmk_behavior_binding *binding,
    struct zmk_behavior_binding_event event,
    int scale_percent,
    int direction) {
    if (data->active) {
        if (direction == data->active_direction &&
            scale_percent <= data->active_scale_percent) {
            k_work_reschedule(&data->release_work, K_MSEC(config->tap_ms));
            return;
        }

        zmk_behavior_invoke_binding(&data->active_binding, data->active_event, false);
        data->active = false;
        data->active_scale_percent = 0;
        data->active_direction = 0;
    }

    data->active_binding = *binding;
    data->active_event = event;

    if (zmk_behavior_invoke_binding(&data->active_binding, data->active_event, true) != 0) {
        return;
    }

    data->active = true;
    data->active_scale_percent = scale_percent;
    data->active_direction = direction;
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
    int scale_percent;
    int direction;
    uint32_t movement;

    if (triggers > 0) {
        direction = 1;
        triggered_binding = config->cw_binding;
        movement = binding->param1;
    } else if (triggers < 0) {
        direction = -1;
        triggers = -triggers;
        triggered_binding = config->ccw_binding;
        movement = binding->param2;
    } else {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    if (!behavior_sensor_rotate_accel_confirm_direction(config, data, direction)) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    scale_percent = select_scale_percent(config, data, sensor_index, event.layer);
    triggered_binding.param1 = scale_movement(movement, scale_percent, triggers);

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    event.source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    behavior_sensor_rotate_accel_start(config, data, &triggered_binding, event, scale_percent,
                                       direction);

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
        .direction_confirm_ms = DT_INST_PROP(n, direction_confirm_ms),                        \
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
