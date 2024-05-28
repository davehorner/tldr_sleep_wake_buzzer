/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 Nordic Semiconductor ASA
 * Copyright (c) 2024 TecDev, LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM lockup.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <inttypes.h>


#include <nrfx.h>
#include <hal/nrf_power.h>
#if !NRF_POWER_HAS_RESETREAS
#include <hal/nrf_reset.h>
#endif
#include <nfc_t2t_lib.h>
#include <nfc/ndef/msg.h>
#include <nfc/ndef/text_rec.h>

#include <dk_buttons_and_leds.h>

// for NRF GPIO defines required for button wake-up
#include <hal/nrf_gpio.h>

static int making_noise=0;
static const struct pwm_dt_spec buzzer_sig = PWM_DT_SPEC_GET(DT_ALIAS(buzzer_pwm));
/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SLEEP_TIME_MS	1
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */
static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios,
						     {0});

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	if(!making_noise) {
	int ret;
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	ret = pwm_set_dt(&buzzer_sig, PWM_MSEC(1), PWM_MSEC(1)/2);
	making_noise=1;
	if (ret) {
		printk("Error %d: failed to set pulse width\n", ret);
		return;

	}
	}
	// gpio_remove_callback_dt(button.port, &cb);

}
#define SYSTEM_OFF_DELAY_S	3

#define MAX_REC_COUNT		1
#define NDEF_MSG_BUF_SIZE	128

#define NFC_FIELD_LED		DK_LED1
#define SYSTEM_ON_LED		DK_LED2


/* Delayed work that enters system off. */
static struct k_work_delayable system_off_work;


/**
 * @brief Function that receives events from NFC.
 */
static void nfc_callback(void *context,
			 nfc_t2t_event_t event,
			 const uint8_t *data,
			 size_t data_length)
{
	ARG_UNUSED(context);
	ARG_UNUSED(data);
	ARG_UNUSED(data_length);

	switch (event) {
	case NFC_T2T_EVENT_FIELD_ON:
		/* Cancel entering system off */
		k_work_cancel_delayable(&system_off_work);
		dk_set_led_on(NFC_FIELD_LED);
		break;
	case NFC_T2T_EVENT_FIELD_OFF:
		/* Enter system off after delay */
		k_work_reschedule(&system_off_work,
				K_SECONDS(SYSTEM_OFF_DELAY_S));
		dk_set_led_off(NFC_FIELD_LED);
		break;
	default:
		break;
	}
}


/**
 * @brief Function for configuring and starting the NFC.
 */
static int start_nfc(void)
{
	/* Text message in its language code. */
	static const uint8_t en_payload[] = {
		'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd', '!'
	};
	static const uint8_t en_code[] = {'e', 'n'};

	/* Buffer used to hold an NFC NDEF message. */
	static uint8_t buffer[NDEF_MSG_BUF_SIZE];

	NFC_NDEF_TEXT_RECORD_DESC_DEF(nfc_text_rec,
				      UTF_8,
				      en_code,
				      sizeof(en_code),
				      en_payload,
				      sizeof(en_payload));

	NFC_NDEF_MSG_DEF(nfc_text_msg, MAX_REC_COUNT);

	uint32_t len = sizeof(buffer);

	/* Set up NFC */
	if (nfc_t2t_setup(nfc_callback, NULL) < 0) {
		printk("Cannot setup NFC T2T library!\n");
		return -1;
	}

	/* Add record */
	if (nfc_ndef_msg_record_add(&NFC_NDEF_MSG(nfc_text_msg),
				&NFC_NDEF_TEXT_RECORD_DESC(nfc_text_rec)) < 0) {
		printk("Cannot add record!\n");
		return -1;
	}

	/* Encode welcome message */
	if (nfc_ndef_msg_encode(&NFC_NDEF_MSG(nfc_text_msg),
				buffer, &len) < 0) {
		printk("Cannot encode message!\n");
		return -1;
	}

	/* Set created message as the NFC payload */
	if (nfc_t2t_payload_set(buffer, len) < 0) {
		printk("Cannot set payload!\n");
		return -1;
	}

	/* Start sensing NFC field */
	if (nfc_t2t_emulation_start() < 0) {
		printk("Cannot start emulation!\n");
		return -1;
	}

	printk("NFC configuration done\n");
	return 0;
}


/**
 * @brief Function entering system off.
 * System off is delayed to make sure that NFC tag was correctly read.
 */
static void system_off(struct k_work *work)
{
	printk("Powering off.\nApproach a NFC reader to restart.\n");

	//pwm_set_dt(&buzzer_sig, PWM_MSEC(1), PWM_MSEC(1)/4);
	// gpio_pin_configure_dt(&button, GPIO_DISCONNECTED);
	// gpio_pin_configure_dt(&button, GPIO_INPUT);
	//gpio_pin_set_dt(&button,0);
	// gpio_remove_callback_dt(button.port, &button_cb_data);
	// nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_SENSE_HIGH);	
	// k_msleep(SLEEP_TIME_MS);
	dk_set_led_off(SYSTEM_ON_LED);
	//nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_SENSE_HIGH);
	//gpio_pin_interrupt_configure_dt(button.port, GPIO_INT_DISABLE);	
	//		k_msleep(SLEEP_TIME_MS);
	sys_poweroff();
}


int main(void)
{
		/* Configure LED-pins */
	if (dk_leds_init() < 0) {
		printk("Cannot init LEDs!\n");
		return 0;
	}
	dk_set_led_on(SYSTEM_ON_LED);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_SENSE_HIGH);	
	//nrf_gpio_cfg_sense_input(NRF_GPIO_PIN_MAP(0,19), NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH); 


	/* Start NFC */
	if (start_nfc() < 0) {
		printk("ERROR: NFC configuration failed\n");
		return 0;
	}


	int ret;

	printk("tldr PWM-based lockup demonstration\n");

	if (!pwm_is_ready_dt(&buzzer_sig)) {
		printk("Error: PWM device %s is not ready\n",
		       buzzer_sig.dev->name);
		return 0;
	}

		if (!gpio_is_ready_dt(&button)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
					    //   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	if (led.port && !gpio_is_ready_dt(&led)) {
		printk("Error %d: LED device %s is not ready; ignoring it\n",
		       ret, led.port->name);
		led.port = NULL;
	}
	if (led.port) {
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led.port->name, led.pin);
			led.port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", led.port->name, led.pin);
		}
	}
	/* Configure and start delayed work that enters system off */
	k_work_init_delayable(&system_off_work, system_off);
	k_work_reschedule(&system_off_work, K_SECONDS(SYSTEM_OFF_DELAY_S));

	/* Show last reset reason */
	//print_reset_reason();
	/* Exit main function - the rest will be done by the callbacks */
}
