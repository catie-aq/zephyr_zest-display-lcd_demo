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

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(app);

#define REFRESH_RATE 100

// User Button variables
#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button =
    GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

// PWM variables
static int direction = 1;
static int period = 1000000;
static int increment = 10000;
static int ratio = 1000000;
static const struct pwm_dt_spec pwm_dev =
    PWM_DT_SPEC_GET(DT_NODELABEL(backlight_lcd));

// Touch variables
static const struct device *const touch_dev =
    DEVICE_DT_GET(DT_NODELABEL(tsc20030));

// Initialize Display
static const struct device *const display_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

static struct k_sem sync;
static struct {
  size_t x;
  size_t y;
  bool pressed;
} touch_point;

// Store a reference to the label object
static lv_obj_t *hello_world_label;

static void touch_event_callback(struct input_event *evt) {
  if (evt->code == INPUT_ABS_X) {
    touch_point.x = evt->value;
  }
  if (evt->code == INPUT_ABS_Y) {
    touch_point.y = evt->value;
  }
  if (evt->code == INPUT_BTN_TOUCH) {
    touch_point.pressed = evt->value;
  }
  if (evt->sync) {
    k_sem_give(&sync);
  }
}

INPUT_CALLBACK_DEFINE(touch_dev, touch_event_callback);

int setup_pwm(void) {
  if (!pwm_is_ready_dt(&pwm_dev)) {
    LOG_ERR("Error: PWM device %s is not ready", pwm_dev.dev->name);
    return -ENODEV;
  }

  int err = pwm_set_dt(&pwm_dev, period, period);
  if (err < 0) {
    LOG_ERR("ERROR! [%d]", err);
    return err;
  }

  return 0;
}

void update_pwm(void) {
  if (ratio >= period) {
    direction = -1;
  } else if (ratio <= 0) {
    direction = 1;
  }

  ratio += direction * increment;
  int err = pwm_set_dt(&pwm_dev, period, ratio);
  if (err < 0) {
    LOG_ERR("ERROR! [%d]", err);
  }
}

static void button_pressed(const struct device *dev, struct gpio_callback *cb,
                           uint32_t pins) {
  LOG_INF("User Button pressed");
  lv_label_set_text(hello_world_label, "User Button pressed");
  lv_task_handler(); 
}

int setup_button(void) {
  int ret;

  if (!gpio_is_ready_dt(&button)) {
    LOG_ERR("Error: button device %s is not ready", button.port->name);
    return -ENODEV;
  }

  ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
  if (ret != 0) {
    LOG_ERR("Error %d: failed to configure %s pin %d", ret, button.port->name,
            button.pin);
    return ret;
  }

  ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret != 0) {
    LOG_ERR("Error %d: failed to configure interrupt on %s pin %d", ret,
            button.port->name, button.pin);
    return ret;
  }

  gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
  gpio_add_callback(button.port, &button_cb_data);
  LOG_INF("Set up button at %s pin %d", button.port->name, button.pin);

  return 0;
}

int main(void) {
  // Initialize Display
  if (!device_is_ready(display_dev)) {
    LOG_ERR("Device not ready, aborting test");
    return 0;
  }

  // Create and initialize the label
  hello_world_label = lv_label_create(lv_scr_act());
  lv_label_set_text(hello_world_label, "6TRON BY CATIE!");
  lv_obj_align(hello_world_label, LV_ALIGN_CENTER, 0, 0);

  lv_task_handler();
  display_blanking_off(display_dev);

  // Initialize PWM
  if (setup_pwm() < 0) {
    return 0;
  }

  // Initialize Touch
  LOG_INF("Touch sample for touchscreen: %s", touch_dev->name);

  if (!device_is_ready(touch_dev)) {
    LOG_ERR("Device %s not found. Aborting sample.", touch_dev->name);
    return 0;
  }

  k_sem_init(&sync, 0, 1);

  // Initialize User Button
  if (setup_button() < 0) {
    return 0;
  }

  while (1) {
    k_msleep(REFRESH_RATE);
    k_sem_take(&sync, K_FOREVER);
    LOG_INF("TOUCH %s X, Y: (%d, %d)", touch_point.pressed ? "PRESS" : "RELEASE",
            touch_point.x, touch_point.y);
    lv_task_handler();
    update_pwm();
    k_sleep(K_MSEC(10));
  }

  return 0;
}
