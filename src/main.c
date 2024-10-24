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

LOG_MODULE_REGISTER(calibration_app, LOG_LEVEL_INF);

#define REFRESH_RATE 100
#define BACKLIGHT_MAX_BRIGHTNESS 1000000
#define BACKLIGHT_MIN_BRIGHTNESS 0
#define CALIBRATION_POINTS 3

static const int SCREEN_WIDTH = 128;  
static const int SCREEN_HEIGHT = 160; 
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
    .screen_x = {10, 110, 90},
    .screen_y = {10, 130, 110}
};

static bool is_calibration_mode = false;
static int point_index = 0;
static lv_obj_t *hello_world_label;
static lv_obj_t *cross_label;

// PWM variables
static int direction = 1;
static int period = 1000000;
static int increment = 10000;
static int ratio = BACKLIGHT_MAX_BRIGHTNESS;
static const struct pwm_dt_spec pwm_dev =
    PWM_DT_SPEC_GET(DT_NODELABEL(backlight_lcd));

// Touch variables
static const struct device *const touch_dev =
    DEVICE_DT_GET(DT_NODELABEL(tsc20030));

static struct k_sem sync;
static struct {
    size_t x;
    size_t y;
    bool pressed;
} touch_point;

// User button variables
#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button =
    GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

static const struct device *const display_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

static void draw_calibration_marker(int x, int y)
{
    lv_obj_align(cross_label, LV_ALIGN_TOP_LEFT, x, y);
}

static void calculate_calibration_coefficients(void)
{
    int x0 = calib_points.raw_x[0], y0 = calib_points.raw_y[0];
    int x1 = calib_points.raw_x[1], y1 = calib_points.raw_y[1];
    int x2 = calib_points.raw_x[2], y2 = calib_points.raw_y[2];

    int xd0 = calib_points.screen_x[0], yd0 = calib_points.screen_y[0];
    int xd1 = calib_points.screen_x[1], yd1 = calib_points.screen_y[1];
    int xd2 = calib_points.screen_x[2], yd2 = calib_points.screen_y[2];

    int k = (int)((x0 - x2) * (y1 - y2) - (x1 - x2) * (y0 - y2));
    printk("X0=[%d] | X1=[%d] | X2=[%d]\n", x0, x1, x2);
    printk("Y0=[%d] | Y1=[%d] | Y2=[%d]\n", y0, y1, y2);
    printk("K=[%d]\n", k);
    if (k == 0) {
        printk("Error: Please retry calibration.\n");
        is_calibration_mode = true; 
        point_index = 0;  
        return;
    }

    cal_coeff_x.a = ((xd0 - xd2) * (y1 - y2) - (xd1 - xd2) * (y0 - y2)) / (double)k;
    cal_coeff_x.b = ((x0 - x2) * (xd1 - xd2) - (xd0 - xd2) * (x1 - x2)) / (double)k;

    cal_coeff_y.a = ((yd0 - yd2) * (y1 - y2) - (yd1 - yd2) * (y0 - y2)) / (double)k;
    cal_coeff_y.b = ((x0 - x2) * (yd1 - yd2) - (yd0 - yd2) * (x1 - x2)) / (double)k;

    printk("Calibration coefficients:\n");
    printk("ax = %f, bx = %f\n", cal_coeff_x.a, cal_coeff_x.b);
    printk("ay = %f, by = %f\n", cal_coeff_y.a, cal_coeff_y.b);
}

static void clamp_coordinates(int16_t *x, int16_t *y)
{
    // Clamp X
    if (*x < 0) {
        *x = 0;
    } else if (*x > SCREEN_WIDTH) {
        *x = SCREEN_WIDTH;
    }

    // Clamp Y
    if (*y < 0) {
        *y = 0;
    } else if (*y > SCREEN_HEIGHT) {
        *y = SCREEN_HEIGHT;
    }
}

static void touch_event_callback(struct input_event *evt)
{
    static int16_t raw_x, raw_y;

    if (evt->code == INPUT_ABS_X) {
        raw_x = evt->value;
    }
    if (evt->code == INPUT_ABS_Y) {
        raw_y = evt->value;
    }
    if (evt->code == INPUT_BTN_TOUCH) {
        touch_point.pressed = evt->value;
        if (touch_point.pressed) {
            touch_point.x = (int16_t)(cal_coeff_x.a * raw_x + cal_coeff_x.b);
            touch_point.y = (int16_t)(cal_coeff_y.a * raw_y + cal_coeff_y.b);
            clamp_coordinates(&touch_point.x, &touch_point.y);
            printk("TOUCH: X=[%d] Y=[%d]\n", touch_point.x, touch_point.y);
            k_sem_give(&sync);
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

    if (!gpio_is_ready_dt(&button)) {
        LOG_ERR("Error: button device %s is not ready", button.port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Error %d: failed to configure %s pin %d", ret, button.port->name, button.pin);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        LOG_ERR("Error %d: failed to configure interrupt on %s pin %d", ret, button.port->name, button.pin);
        return ret;
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
    printk("Set up button at %s pin %d\n", button.port->name, button.pin);

    return 0;
}

int setup_pwm(void) {
    if (!pwm_is_ready_dt(&pwm_dev)) {
        LOG_ERR("Error: PWM device %s is not ready", pwm_dev.dev->name);
        return -ENODEV;
    }

    int err = pwm_set_dt(&pwm_dev, period, BACKLIGHT_MAX_BRIGHTNESS);
    if (err < 0) {
        LOG_ERR("ERROR! [%d]", err);
        return err;
    }

    return 0;
}

void update_pwm(void) {
    if (ratio >= BACKLIGHT_MAX_BRIGHTNESS) {
        direction = -1;
    } else if (ratio <= BACKLIGHT_MIN_BRIGHTNESS) {
        direction = 1;
    }

    ratio += direction * increment;
    int err = pwm_set_dt(&pwm_dev, period, ratio);
    if (err < 0) {
        LOG_ERR("ERROR! [%d]", err);
    }
}

int main(void) {

    if (!device_is_ready(display_dev)) {
        LOG_ERR("Device not ready, aborting test");
        return 0;
    }

    cross_label = lv_label_create(lv_scr_act());
    lv_label_set_text(cross_label, "");
    hello_world_label = lv_label_create(lv_scr_act());
    lv_label_set_text(hello_world_label, "6TRON BY CATIE!");
    lv_obj_align(hello_world_label, LV_ALIGN_CENTER, 0, 0);
    lv_task_handler();
    display_blanking_off(display_dev);

    if (setup_pwm() < 0) {
        return 0;
    }

    printk("Touch sample for touchscreen: %s\n", touch_dev->name);
    if (!device_is_ready(touch_dev)) {
        LOG_ERR("Device %s not found or not ready. Aborting sample.", touch_dev->name);
        return -ENODEV;
    }

    k_sem_init(&sync, 0, 1);

    if (setup_button() < 0) {
        return 0;
    }

    /* Variable initialization */
    cal_coeff_x.a = 1.0;
    cal_coeff_x.b = 0.0;
    cal_coeff_y.a = 1.0;
    cal_coeff_y.b = 0.0;

    while (1) {
        if (is_calibration_mode) {
            lv_label_set_text(hello_world_label, "Calibration");

            k_sem_take(&sync, K_FOREVER);

            if (touch_point.pressed) {
              printk("Calibration [%d]\n", point_index);
              touch_point.pressed = false;
              calib_points.raw_x[point_index] = touch_point.x;
              calib_points.raw_y[point_index] = touch_point.y;
                point_index++;
                if (point_index >= CALIBRATION_POINTS) {
                    is_calibration_mode = false;
                    lv_label_set_text(cross_label, "");
                    calculate_calibration_coefficients();
                    printk("Calibration complete.\n");
                } else {
                    draw_calibration_marker(calib_points.screen_x[point_index], calib_points.screen_y[point_index]);
                }
            }
        } else {
            lv_label_set_text(hello_world_label, "6TRON BY CATIE!");
        }

        update_pwm();
        lv_task_handler();
        k_msleep(REFRESH_RATE);
    }

    return 0;
}
