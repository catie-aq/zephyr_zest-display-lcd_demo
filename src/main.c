/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <math.h>

LOG_MODULE_REGISTER(calibration_app, LOG_LEVEL_INF);

#define REFRESH_RATE 10
#define BACKLIGHT_MAX_BRIGHTNESS 1000000
#define BACKLIGHT_MIN_BRIGHTNESS 0
#define CALIBRATION_POINTS 5
 
struct calibration_coeff {
    double a, b;
};

static struct calibration_coeff cal_coeff_x, cal_coeff_y;

struct calibration_points {
    int16_t raw_x[CALIBRATION_POINTS];
    int16_t raw_y[CALIBRATION_POINTS];
    int16_t screen_x[CALIBRATION_POINTS];
    int16_t screen_y[CALIBRATION_POINTS];
};

static struct calibration_points calib_points = {
    .screen_x = {64, 20, 108, 108, 20},
    .screen_y = {80, 20, 20, 140, 140}
};

static bool is_calibration_mode = false;
static int point_index = 0;
static lv_obj_t *hello_world_label;
static lv_obj_t *cross_label;

static int direction = 1;
static int period = 1000000;
static int increment = 10000;
static int ratio = BACKLIGHT_MAX_BRIGHTNESS;

static const struct pwm_dt_spec pwm_dev =
    PWM_DT_SPEC_GET(DT_NODELABEL(backlight_lcd));
static const struct device *const touch_dev =
    DEVICE_DT_GET(DT_NODELABEL(tsc20030));

static struct {
    size_t x;
    size_t y;
    bool pressed;
} touch_point;

#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button =
    GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

static const struct device *const display_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

static void draw_calibration_marker(int x, int y) {
    printk("MOUVE: X=[%d] Y=[%d]\n", x, y);
    lv_obj_align(cross_label, LV_ALIGN_TOP_LEFT, x, y);
}

void calculate_calibration_coefficients(void) {
    double sum_raw_x = 0, sum_raw_y = 0;
    double sum_screen_x = 0, sum_screen_y = 0;
    double sum_raw_x_screen_x = 0, sum_raw_y_screen_y = 0;
    double sum_raw_x_squared = 0, sum_raw_y_squared = 0;

    for (int i = 0; i < CALIBRATION_POINTS; i++) {
    sum_raw_x = sum_raw_x + calib_points.raw_x[i];
    sum_raw_y = sum_raw_y + calib_points.raw_y[i];
    sum_screen_x = sum_screen_x + calib_points.screen_x[i];
    sum_screen_y = sum_screen_y + calib_points.screen_y[i];
    sum_raw_x_screen_x = sum_raw_x_screen_x + calib_points.raw_x[i] * calib_points.screen_x[i];
    sum_raw_y_screen_y = sum_raw_y_screen_y + calib_points.raw_y[i] * calib_points.screen_y[i];
    sum_raw_x_squared = sum_raw_x_squared + calib_points.raw_x[i] * calib_points.raw_x[i];
    sum_raw_y_squared = sum_raw_y_squared + calib_points.raw_y[i] * calib_points.raw_y[i];
    }

    double denominator_x = CALIBRATION_POINTS * sum_raw_x_squared - sum_raw_x * sum_raw_x;
    double denominator_y = CALIBRATION_POINTS * sum_raw_y_squared - sum_raw_y * sum_raw_y;

    if (denominator_x == 0 || denominator_y == 0) {
        printk("Error : Retry calibration.\n");
        return;
    }

    cal_coeff_x.a = (CALIBRATION_POINTS * sum_raw_x_screen_x - sum_raw_x * sum_screen_x) / denominator_x;
    cal_coeff_x.b = (sum_screen_x * sum_raw_x_squared - sum_raw_x * sum_raw_x_screen_x) / denominator_x;

    cal_coeff_y.a = (CALIBRATION_POINTS * sum_raw_y_screen_y - sum_raw_y * sum_screen_y) / denominator_y;
    cal_coeff_y.b = (sum_screen_y * sum_raw_y_squared - sum_raw_y * sum_raw_y_screen_y) / denominator_y;
    printk("Calibration coefficients:\n");
    printk("ax = %f, bx = %f\n", cal_coeff_x.a, cal_coeff_x.b);
    printk("ay = %f, by = %f\n", cal_coeff_y.a, cal_coeff_y.b);
}

static void touch_event_callback(struct input_event *evt) {
    static int16_t raw_x, raw_y;

    if (evt->code == INPUT_ABS_X) raw_x = evt->value;
    if (evt->code == INPUT_ABS_Y) raw_y = evt->value;
    if (evt->code == INPUT_BTN_TOUCH) {
        touch_point.pressed = evt->value;
        if (touch_point.pressed) {
            touch_point.x = (int16_t)(cal_coeff_x.a * raw_x + cal_coeff_x.b);
            touch_point.y = (int16_t)(cal_coeff_y.a * raw_y + cal_coeff_y.b);
            printk("TOUCH: X=[%d] Y=[%d]\n", touch_point.x, touch_point.y);
        }
    }
}

INPUT_CALLBACK_DEFINE(touch_dev, touch_event_callback);

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    is_calibration_mode = true;
    touch_point.pressed = false;
    point_index = 0;
    cal_coeff_x.a = 1.0;
    cal_coeff_x.b = 0.0;
    cal_coeff_y.a = 1.0;
    cal_coeff_y.b = 0.0;
    lv_label_set_text(cross_label, "x");
    draw_calibration_marker(calib_points.screen_x[point_index], calib_points.screen_y[point_index]);
}

int setup_button(void) {
    int ret;
    if (!gpio_is_ready_dt(&button)) return -ENODEV;

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) return ret;

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) return ret;

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
    return 0;
}

int setup_pwm(void) {
    if (!pwm_is_ready_dt(&pwm_dev)) return -ENODEV;
    return pwm_set_dt(&pwm_dev, period, BACKLIGHT_MAX_BRIGHTNESS);
}

void update_pwm(void) {
    if (ratio >= BACKLIGHT_MAX_BRIGHTNESS) direction = -1;
    else if (ratio <= BACKLIGHT_MIN_BRIGHTNESS) direction = 1;

    ratio += direction * increment;
    int err = pwm_set_dt(&pwm_dev, period, ratio);
    if (err < 0) LOG_ERR("ERROR! [%d]", err);
}

void display_message(const char *message) {
    lv_label_set_text(hello_world_label, message);
}

int main(void) {
    if (!device_is_ready(display_dev)) return 0;

    cross_label = lv_label_create(lv_scr_act());
    lv_label_set_text(cross_label, "x");

    hello_world_label = lv_label_create(lv_scr_act());
    display_message("6TRON BY CATIE!");
    lv_obj_align(hello_world_label, LV_ALIGN_CENTER, 0, 0);

    display_blanking_off(display_dev);
    if (setup_pwm() < 0 || setup_button() < 0) return 0;

    cal_coeff_x.a = cal_coeff_y.a = 1.0;
    cal_coeff_x.b = cal_coeff_y.b = 0.0;

     while (1) {
        if (is_calibration_mode) {
            lv_label_set_text(hello_world_label, "Calibration");

            if (touch_point.pressed) {
              printk("Calibration [%d]\n", point_index);
              touch_point.pressed = false;
              calib_points.raw_x[point_index] = touch_point.x;
              calib_points.raw_y[point_index] = touch_point.y;
                point_index++;
                if (point_index >= CALIBRATION_POINTS) {
                    is_calibration_mode = false;
                    //lv_label_set_text(cross_label, "");
                    calculate_calibration_coefficients();
                    printk("Calibration done.\n");
                } else {
                    draw_calibration_marker(calib_points.screen_x[point_index], calib_points.screen_y[point_index]);
                }
            }
        } else {
            lv_label_set_text(hello_world_label, "6TRON BY CATIE!");
            if (touch_point.pressed) {
                touch_point.pressed = false;
                draw_calibration_marker(touch_point.x, touch_point.y);
            }
        }

        update_pwm();
        lv_task_handler();
        k_msleep(REFRESH_RATE);
    }
    return 0;
}
