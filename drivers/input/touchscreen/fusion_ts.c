/*
 * based on egalax_ts.c and prism.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>

#define SCAN_COMPLETE_REGISTER 0x11
#define FIRMWARE_VERSION_REGISTER_I 0x0e /* size, year, month */
#define FIRMWARE_VERSION_REGISTER_II 0x0f /* day, version, release */
#define DATA_INFORMATION_REGISTER 0x00 /* number of touch_points */

#define TIP_SWITCH_MASK 0xf
#define NUM_TOUCH_POINTS_MASK 3

#define TOUCH_ID_OFFSET 4
#define TOUCH_POSITION_OFFSET 8

#define MAX_TOUCH_POINTS 2
#define TOUCH_SENSITIVITY 10
#define MAX_PRESSURE 255
#define REGISTERS_PER_TOUCH_POINT 6

#define FUSION_RESET_HOLD 50
#define FUSION_RESET_DELAY 75

struct fusion_ts_priv {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int touchIDs[MAX_TOUCH_POINTS];
	int xmax;
	int ymax;
	int zmax;
	int flip_x;
	int flip_y;
};

struct fusion_ts_event {
	unsigned short x;
	unsigned short y;
	unsigned char z;
	unsigned char tip;
	unsigned char id;
};

/*
 * Multitouch protocol B slots are determinated by order in ev[],
 * which is equivalent to order in ts->touchIDs[]
 * Storing touch IDs is needed, since the registers a specific
 * touch point is stored in do not remain constant.
 *
 * Unassigned indexes in ev[] are pre-initialized to indicate tip==0
 * which leads to removal of the ID
 * from the corresponding index in ts->touchIDs[]
 *
 * Contrary to the datasheet, it has been observed,
 * that interrupts are triggered for the finger up event.
 */
static int fusion_ts_analyze_touch_data(struct fusion_ts_priv* ts,
					struct fusion_ts_event* ev,
					unsigned char* data)
{

	struct fusion_ts_event ev_tmp;
	int i;
	int j;
	int found;
	int num_touch_points;
	unsigned char *ptr;

	num_touch_points = data[0] & NUM_TOUCH_POINTS_MASK;
	if(num_touch_points > MAX_TOUCH_POINTS)
		return -1;

	ptr = data + 1;

	for (i = 0; i < num_touch_points; i++) {
		ev_tmp.tip = ptr[5] & TIP_SWITCH_MASK;

		if(ev_tmp.tip) {
			ev_tmp.x = (ptr[2] << TOUCH_POSITION_OFFSET) + ptr[3];
			ev_tmp.y = (ptr[0] << TOUCH_POSITION_OFFSET) + ptr[1];
			/* formula for z (pressure) transfered from prism.c */
			ev_tmp.z =
			((int)ptr[4] * (int)ptr[4]) / (TOUCH_SENSITIVITY - 5);
			ev_tmp.id = ptr[5] >> TOUCH_ID_OFFSET;

			ev_tmp.z = (ev_tmp.z > ts->zmax) ? ts->zmax : ev_tmp.z;
			ev_tmp.x = (ev_tmp.x > ts->xmax) ? ts->xmax : ev_tmp.x;
			ev_tmp.y = (ev_tmp.y > ts->ymax) ? ts->ymax : ev_tmp.y;

			if(ts->flip_x) {
				ev_tmp.x = ts->xmax - ev_tmp.x;
			}

			if(ts->flip_y) {
				ev_tmp.y = ts->ymax - ev_tmp.y;
			}

			found = 0;
			j = 0;
			while(!found && j < MAX_TOUCH_POINTS) {
				if(ts->touchIDs[j] == ev_tmp.id) {
					ev[j]=ev_tmp;
					found = 1;
				}
				j++;
			}
			j=0;
			while(!found && j < MAX_TOUCH_POINTS) {
				if(ts->touchIDs[j] == -1) {
					ev[j] = ev_tmp;
					ts->touchIDs[j] = ev_tmp.id;
					found = 1;
				}
				j++;
			}
		}
		ptr += REGISTERS_PER_TOUCH_POINT;
	}

	return 0;
}

static int fusion_ts_read_i2c(struct fusion_ts_priv* ts,
				struct fusion_ts_event* ev)
{

	struct i2c_client *client = ts->client;
	unsigned char data[1+REGISTERS_PER_TOUCH_POINT*MAX_TOUCH_POINTS];
	unsigned char start = DATA_INFORMATION_REGISTER;
	int ret;

	struct i2c_msg msg[2] = {
		{ client->addr, client->flags, 1, &start },
		{ client->addr, client->flags | I2C_M_RD, sizeof(data), data }
	};


	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return fusion_ts_analyze_touch_data(ts, ev, data);
}

static irqreturn_t fusion_ts_interrupt(int irq, void *dev_id)
{
	struct fusion_ts_priv *ts = dev_id;
	struct input_dev *input_dev = ts->input_dev;
	struct i2c_client *client = ts->client;
	struct fusion_ts_event ev[MAX_TOUCH_POINTS]={{0}};
	int ret;
	int i;

	ret = fusion_ts_read_i2c(ts, ev);

	if (ret < 0)
		goto end;

	for (i = 0; i < MAX_TOUCH_POINTS; i++) {
		input_mt_slot(input_dev, i);
		input_mt_report_slot_state(input_dev,
						MT_TOOL_FINGER, ev[i].tip);
		if(ev[i].tip) {
			input_report_abs(input_dev, ABS_MT_POSITION_X, ev[i].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, ev[i].y);
			input_report_abs(input_dev, ABS_MT_PRESSURE, ev[i].z);
		} else {
			ts->touchIDs[i] = -1;
		}
	}
	input_mt_report_pointer_emulation(input_dev, true);
	input_sync(input_dev);

	end:

	ret = i2c_smbus_write_byte_data(client, SCAN_COMPLETE_REGISTER, 0);

	if (ret)
		dev_err(&client->dev, "reenabling IRQ failed: %d\n", ret);

	return IRQ_HANDLED;
}

static int fusion_ts_reset(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	int rst_gpio;
	int ret;

	if (!np)
		return -ENODEV;

	rst_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (!gpio_is_valid(rst_gpio))
		return -ENODEV;

	ret = gpio_request(rst_gpio, "fusion_ts_rst");
	if (ret < 0) {
		dev_err(&client->dev,
			"request gpio failed, cannot reset controller: %d\n",
			ret);
		return ret;
	}

	gpio_request(rst_gpio, "fusion_ts_rst");
	gpio_direction_output(rst_gpio, 0);
	gpio_set_value(rst_gpio, 0);
	mdelay(FUSION_RESET_HOLD);
	gpio_set_value(rst_gpio, 1);
	mdelay(FUSION_RESET_DELAY);

	gpio_direction_input(rst_gpio);
	gpio_free(rst_gpio);

	return 0;
}

/*
 * This reads data from the firmware register
 * to determine if communication is possible.
 * Currently there is no analysis of the read data.
 */
static int fusion_ts_firmware_version(struct i2c_client *client)
{
	int ret;
	ret=i2c_smbus_read_byte_data(client, FIRMWARE_VERSION_REGISTER_II);

	if (ret < 0)
		return ret;

	return 0;
}

static int fusion_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	struct fusion_ts_priv *ts;
	struct input_dev *input_dev;
	int ret;
	struct device_node *np;
	int i;

	np = client->dev.of_node;

	if (!np) {
		dev_err(&client->dev, "Failed to access device node\n");
		return -ENODEV;
	}

	ts = devm_kzalloc(&client->dev, sizeof(struct fusion_ts_priv),
				GFP_KERNEL);
	if (!ts) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	for(i = 0; i < MAX_TOUCH_POINTS; i++) {
		ts->touchIDs[i] = -1;
	}

	input_dev = devm_input_allocate_device(&client->dev);
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	ts->client = client;
	ts->input_dev = input_dev;

	ret = fusion_ts_reset(client);
	if (ret) {
		dev_err(&client->dev, "Failed to reset the controller\n");
		return ret;
	}

	ret = fusion_ts_firmware_version(client);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read firmware version\n");
		return ret;
	}

	of_property_read_u32(np, "touch_xmax", &ts->xmax);
	of_property_read_u32(np, "touch_ymax", &ts->ymax);
	ts->zmax = MAX_PRESSURE;
	of_property_read_u32(np, "flip_x", &ts->flip_x);
	of_property_read_u32(np, "flip_y", &ts->flip_y);

	input_dev->name = "fusion Touch Screen";
	input_dev->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, ts->xmax, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ts->ymax, 0, 0);
	input_set_abs_params(input_dev,
				ABS_MT_POSITION_X, 0, ts->xmax, 0, 0);
	input_set_abs_params(input_dev,
				ABS_MT_POSITION_Y, 0, ts->ymax, 0, 0);
	input_mt_init_slots(input_dev, MAX_TOUCH_POINTS, 0);

	input_set_drvdata(input_dev, ts);

	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					fusion_ts_interrupt,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"fusion_ts", ts);

	if (ret < 0) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		return ret;
	}

	ret = input_register_device(ts->input_dev);
	if (ret)
		return ret;

	i2c_set_clientdata(client, ts);
	return 0;
}

static int fusion_ts_remove(struct i2c_client *client)
{
	struct fusion_ts_priv *ts;

	ts = dev_get_drvdata(&client->dev);
	i2c_set_clientdata(client, NULL);

	if (ts->input_dev) {
		input_unregister_device(ts->input_dev);
		input_free_device(ts->input_dev);
	}

	kfree(ts);

	return 0;
}

static const struct i2c_device_id fusion_ts_id[] = {
	{ "fusion4", 0 },
	{ "fusion7", 0 },
	{ "fusion10", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fusion_ts_id);

static struct of_device_id fusion_ts_dt_ids[] = {
	{ .compatible = "touchrev,fusion4"},
	{ .compatible = "touchrev,fusion7"},
	{ .compatible = "touchrev,fusion10"},
	{ /* sentinel */ }
};

static struct i2c_driver fusion_ts_driver = {
	.driver = {
		.name	= "fusion",
		.owner	= THIS_MODULE,
		.of_match_table	= fusion_ts_dt_ids,
	},
	.id_table = fusion_ts_id,
	.probe = fusion_ts_probe,
	.remove = fusion_ts_remove,
};

module_i2c_driver(fusion_ts_driver);

MODULE_AUTHOR("domo");
MODULE_DESCRIPTION("fusion touchscreen driver");
MODULE_LICENSE("GPL");
