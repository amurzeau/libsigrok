/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2012 Martin Ling <martin-git@earth.li>
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2013 Mathias Grimmberger <mgri@zaphod.sax.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <time.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "scpi.h"
#include "protocol.h"

/*
 * This is a unified protocol driver for the DS1000 and DS2000 series.
 *
 * DS1000 support tested with a Rigol DS1102D.
 *
 * DS2000 support tested with a Rigol DS2072 using firmware version 01.01.00.02.
 *
 * The Rigol DS2000 series scopes try to adhere to the IEEE 488.2 (I think)
 * standard. If you want to read it - it costs real money...
 *
 * Every response from the scope has a linefeed appended because the
 * standard says so. In principle this could be ignored because sending the
 * next command clears the output queue of the scope. This driver tries to
 * avoid doing that because it may cause an error being generated inside the
 * scope and who knows what bugs the firmware has WRT this.
 *
 * Waveform data is transferred in a format called "arbitrary block program
 * data" specified in IEEE 488.2. See Agilents programming manuals for their
 * 2000/3000 series scopes for a nice description.
 *
 * Each data block from the scope has a header, e.g. "#900000001400".
 * The '#' marks the start of a block.
 * Next is one ASCII decimal digit between 1 and 9, this gives the number of
 * ASCII decimal digits following.
 * Last are the ASCII decimal digits giving the number of bytes (not
 * samples!) in the block.
 *
 * After this header as many data bytes as indicated follow.
 *
 * Each data block has a trailing linefeed too.
 */

static int parse_int(const char *str, int *ret)
{
	char *e;
	long tmp;

	errno = 0;
	tmp = strtol(str, &e, 10);
	if (e == str || *e != '\0') {
		sr_dbg("Failed to parse integer: '%s'", str);
		return SR_ERR;
	}
	if (errno) {
		sr_dbg("Failed to parse integer: '%s', numerical overflow", str);
		return SR_ERR;
	}
	if (tmp > INT_MAX || tmp < INT_MIN) {
		sr_dbg("Failed to parse integer: '%s', value to large/small", str);
		return SR_ERR;
	}

	*ret = (int)tmp;
	return SR_OK;
}

/* Send a configuration setting. */
SR_PRIV int keysight_config_set(const struct sr_dev_inst *sdi, const char *format, ...)
{
	struct dev_context *devc = sdi->priv;
	va_list args;
	int ret;

	va_start(args, format);
	ret = sr_scpi_send_variadic(sdi->conn, format, args);
	va_end(args);

	if (ret != SR_OK)
		return SR_ERR;

	return sr_scpi_get_opc(sdi->conn);
}

/* Start capturing a new frameset */
SR_PRIV int keysight_capture_start(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	if (!(devc = sdi->priv))
		return SR_ERR;

	const gboolean first_frame = (devc->num_frames == 0);

	uint64_t limit_frames = devc->limit_frames;
	if (limit_frames == 0)
		sr_dbg("Starting data capture for frameset %" PRIu64,
		       devc->num_frames + 1);
	else
		sr_dbg("Starting data capture for frameset %" PRIu64 " of %"
		       PRIu64, devc->num_frames + 1, limit_frames);

	if (sr_scpi_send(sdi->conn, ":DIGitize;*OPC?") != SR_OK)
		return TRUE;

	devc->state = STATE_DIGITIZING;

	return SR_OK;
}

/* Start reading data from the current channel */
SR_PRIV int keysight_channel_start(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_channel *ch;

	if (!(devc = sdi->priv))
		return SR_ERR;

	ch = devc->channel_entry->data;

	sr_dbg("Starting reading data from channel %d", ch->index + 1);

	const gboolean first_frame = (devc->num_frames == 0);

	if (ch->type == SR_CHANNEL_ANALOG) {
		if (keysight_config_set(sdi, ":WAV:SOUR CHAN%d",
				ch->index + 1) != SR_OK)
			return SR_ERR;
	} else {
		if (keysight_config_set(sdi, ":WAV:SOUR POD%d",
				ch->index < 8 ? 1 : 2) != SR_OK)
			return SR_ERR;
	}

	if (first_frame) {
		if (keysight_config_set(sdi, ":WAV:FORM BYTE") != SR_OK)
			return SR_ERR;

		if (keysight_config_set(sdi, ":WAV:POIN:MODE NORM") != SR_OK)
			return SR_ERR;

		/* Required for digital data */
		if (keysight_config_set(sdi, ":WAV:UNS ON") != SR_OK)
			return SR_ERR;

		if (ch->type == SR_CHANNEL_ANALOG) {
			/* Vertical increment. */
			if (sr_scpi_get_float(sdi->conn, ":WAV:YINC?",
					&devc->vert_inc[ch->index]) != SR_OK)
				return SR_ERR;

			/* Vertical origin. */
			if (sr_scpi_get_float(sdi->conn, ":WAV:YOR?",
				&devc->vert_origin[ch->index]) != SR_OK)
				return SR_ERR;

			/* Vertical reference. */
			if (sr_scpi_get_int(sdi->conn, ":WAV:YREF?",
					&devc->vert_reference[ch->index]) != SR_OK)
				return SR_ERR;
		}
	}

	if (sr_scpi_get_int(sdi->conn, ":WAV:POIN?", &devc->num_channel_bytes_total) != SR_OK)
		return SR_ERR;

	devc->num_channel_bytes = 0;
	devc->num_header_bytes = 0;
	devc->num_block_bytes = 0;

	if (sr_scpi_send(sdi->conn, ":WAV:DATA?") != SR_OK)
		return TRUE;
	if (sr_scpi_read_begin(sdi->conn) != SR_OK)
		return TRUE;

	devc->state = STATE_READING_DATA;

	return SR_OK;
}

/* Read the header of a data block */
static int keysight_read_header(struct sr_dev_inst *sdi)
{
	struct sr_scpi_dev_inst *scpi = sdi->conn;
	struct dev_context *devc = sdi->priv;
	char *buf = (char *) devc->buffer;
	size_t header_length;
	int ret;

	/* Try to read the hashsign and length digit. */
	if (devc->num_header_bytes < 2) {
		ret = sr_scpi_read_data(scpi, buf + devc->num_header_bytes,
				2 - devc->num_header_bytes);
		if (ret < 0) {
			sr_err("Read error while reading data header.");
			return SR_ERR;
		}
		devc->num_header_bytes += ret;
	}

	if (devc->num_header_bytes < 2)
		return 0;

	if (buf[0] != '#' || !isdigit(buf[1]) || buf[1] == '0') {
		sr_err("Received invalid data block header '%c%c'.", buf[0], buf[1]);
		return SR_ERR;
	}

	header_length = 2 + buf[1] - '0';

	/* Try to read the length. */
	if (devc->num_header_bytes < header_length) {
		ret = sr_scpi_read_data(scpi, buf + devc->num_header_bytes,
				header_length - devc->num_header_bytes);
		if (ret < 0) {
			sr_err("Read error while reading data header.");
			return SR_ERR;
		}
		devc->num_header_bytes += ret;
	}

	if (devc->num_header_bytes < header_length)
		return 0;

	/* Read the data length. */
	buf[header_length] = '\0';

	if (parse_int(buf + 2, &ret) != SR_OK) {
		sr_err("Received invalid data block length '%s'.", buf + 2);
		return -1;
	}

	sr_dbg("Received data block header: '%s' -> block length %d", buf, ret);

	return ret;
}

SR_PRIV int keysight_receive(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct sr_scpi_dev_inst *scpi;
	struct dev_context *devc;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	struct sr_datafeed_logic logic;
	double vdiv, offset, origin;
	int len, i, vref;
	struct sr_channel *ch;
	gsize expected_data_bytes;

	(void)fd;

	if (!(sdi = cb_data))
		return TRUE;

	if (!(devc = sdi->priv))
		return TRUE;

	scpi = sdi->conn;

	if (!(revents == G_IO_IN || revents == 0))
		return TRUE;

	const gboolean first_frame = (devc->num_frames == 0);

	switch (devc->state) {
	case STATE_IDLE:
		return TRUE;
	case STATE_DIGITIZING:
		if (keysight_channel_start(sdi) != SR_OK)
			return TRUE;
		return TRUE;
	case STATE_READING_DATA:
		// handled bellow
		break;
	default:
		sr_err("BUG: Unknown event target encountered");
		return TRUE;
	}

	ch = devc->channel_entry->data;

	if (devc->num_block_bytes == 0) {
		sr_dbg("New block header expected");
		len = keysight_read_header(sdi);
		if (len == 0)
			/* Still reading the header. */
			return TRUE;
		if (len == -1) {
			sr_err("Error while reading block header, aborting capture.");
			std_session_send_df_frame_end(sdi);
			sr_dev_acquisition_stop(sdi);
			devc->state = STATE_IDLE;
			return TRUE;
		}

		devc->num_block_bytes = len;
		devc->num_block_read = 0;
	}

	len = devc->num_block_bytes - devc->num_block_read;
	if (len > ACQ_BUFFER_SIZE)
		len = ACQ_BUFFER_SIZE;
	sr_dbg("Requesting read of %d bytes", len);

	len = sr_scpi_read_data(scpi, (char *)devc->buffer, len);

	if (len == -1) {
		sr_err("Error while reading block data, aborting capture.");
		std_session_send_df_frame_end(sdi);
		sr_dev_acquisition_stop(sdi);
		devc->state = STATE_IDLE;
		return TRUE;
	}

	sr_dbg("Received %d bytes.", len);

	devc->num_block_read += len;

	if (ch->type == SR_CHANNEL_ANALOG) {
		vref = devc->vert_reference[ch->index];
		vdiv = devc->vert_inc[ch->index];
		origin = devc->vert_origin[ch->index];
		offset = devc->vert_offset[ch->index];
		for (i = 0; i < len; i++)
			devc->data[i] = ((int)devc->buffer[i] - vref - origin) * vdiv;
		float vdivlog = log10f(vdiv);
		int digits = -(int)vdivlog + (vdivlog < 0.0);
		sr_analog_init(&analog, &encoding, &meaning, &spec, digits);
		analog.meaning->channels = g_slist_append(NULL, ch);
		analog.num_samples = len;
		analog.data = devc->data;
		analog.meaning->mq = SR_MQ_VOLTAGE;
		analog.meaning->unit = SR_UNIT_VOLT;
		analog.meaning->mqflags = 0;
		packet.type = SR_DF_ANALOG;
		packet.payload = &analog;
		sr_session_send(sdi, &packet);
		g_slist_free(analog.meaning->channels);
	} else {
		logic.length = len;
		logic.unitsize = 1; // We get only 8 bits of logic data from either POD1 or POD2 but not both
		logic.data = devc->buffer;
		packet.type = SR_DF_LOGIC;
		packet.payload = &logic;
		sr_session_send(sdi, &packet);
	}

	if (devc->num_block_read == devc->num_block_bytes) {
		sr_dbg("Block has been completed");
		/* Discard the terminating linefeed */
		sr_scpi_read_data(scpi, (char *)devc->buffer, 1);

		/* Prepare for possible next block */
		devc->num_header_bytes = 0;
		devc->num_block_bytes = 0;
		devc->num_block_read = 0;

		if (!sr_scpi_read_complete(scpi) && !devc->channel_entry->next) {
			sr_err("Read should have been completed");
		}
	} else {
		sr_dbg("%" PRIu64 " of %" PRIu64 " block bytes read",
			devc->num_block_read, devc->num_block_bytes);
	}

	devc->num_channel_bytes += len;

	if (devc->num_channel_bytes < devc->num_channel_bytes_total) {
		/* Don't have the full data for this channel yet, re-run. */
		return TRUE;
	}

	/* End of data for this channel. */

	if (devc->channel_entry->next) {
		/* We got the frame for this channel, now get the next channel. */
		devc->channel_entry = devc->channel_entry->next;
		keysight_channel_start(sdi);
	} else {
		/* Done with this frame. */
		std_session_send_df_frame_end(sdi);
		devc->state = STATE_IDLE;

		devc->num_frames++;

		/* Get the next frame, starting with the first channel. */
		devc->channel_entry = devc->enabled_channels;

		keysight_capture_start(sdi);

		/* Start of next frame. */
		std_session_send_df_frame_begin(sdi);
	}

	return TRUE;
}

SR_PRIV int keysight_get_dev_cfg(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_channel *ch;
	char *cmd;
	unsigned int i;
	int res;

	devc = sdi->priv;

	/* Analog channel state. */
	for (i = 0; i < devc->model->analog_channels; i++) {
		cmd = g_strdup_printf(":CHAN%d:DISP?", i + 1);
		res = sr_scpi_get_bool(sdi->conn, cmd, &devc->analog_channels[i]);
		g_free(cmd);
		if (res != SR_OK)
			return SR_ERR;
		ch = g_slist_nth_data(sdi->channels, i);
		ch->enabled = devc->analog_channels[i];
	}
	sr_dbg("Current analog channel state:");
	for (i = 0; i < devc->model->analog_channels; i++)
		sr_dbg("CH%d %s", i + 1, devc->analog_channels[i] ? "on" : "off");

	/* Digital channel state. */
	if (devc->model->has_digital) {
		for (i = 0; i < ARRAY_SIZE(devc->digital_channels); i++) {
			cmd = g_strdup_printf(":DIG%d:DISP?", i);
			res = sr_scpi_get_bool(sdi->conn, cmd, &devc->digital_channels[i]);
			g_free(cmd);
			if (res != SR_OK)
				return SR_ERR;
			ch = g_slist_nth_data(sdi->channels, i + devc->model->analog_channels);
			ch->enabled = devc->digital_channels[i];
			sr_dbg("D%d: %s", i, devc->digital_channels[i] ? "on" : "off");
		}
	}

	/* Timebase. */
	if (sr_scpi_get_float(sdi->conn, ":TIM:SCAL?", &devc->timebase) != SR_OK)
		return SR_ERR;
	sr_dbg("Current timebase %g", devc->timebase);

	/* Probe attenuation. */
	for (i = 0; i < devc->model->analog_channels; i++) {
		cmd = g_strdup_printf(":CHAN%d:PROB?", i + 1);

		/* DSO1000B series prints an X after the probe factor, so
		 * we get a string and check for that instead of only handling
		 * floats. */
		char *response;
		res = sr_scpi_get_string(sdi->conn, cmd, &response);
		if (res != SR_OK)
			return SR_ERR;

		int len = strlen(response);
		if (response[len-1] == 'X')
			response[len-1] = 0;

		res = sr_atof_ascii(response, &devc->attenuation[i]);
		g_free(response);
		g_free(cmd);
		if (res != SR_OK)
			return SR_ERR;
	}
	sr_dbg("Current probe attenuation:");
	for (i = 0; i < devc->model->analog_channels; i++)
		sr_dbg("CH%d %g", i + 1, devc->attenuation[i]);

	/* Vertical gain and offset. */
	if (keysight_get_dev_cfg_vertical(sdi) != SR_OK)
		return SR_ERR;

	/* Coupling. */
	for (i = 0; i < devc->model->analog_channels; i++) {
		cmd = g_strdup_printf(":CHAN%d:COUP?", i + 1);
		g_free(devc->coupling[i]);
		devc->coupling[i] = NULL;
		res = sr_scpi_get_string(sdi->conn, cmd, &devc->coupling[i]);
		g_free(cmd);
		if (res != SR_OK)
			return SR_ERR;
	}
	sr_dbg("Current coupling:");
	for (i = 0; i < devc->model->analog_channels; i++)
		sr_dbg("CH%d %s", i + 1, devc->coupling[i]);

	/* Trigger source. */
	g_free(devc->trigger_source);
	devc->trigger_source = NULL;
	if (sr_scpi_get_string(sdi->conn, ":TRIG:EDGE:SOUR?", &devc->trigger_source) != SR_OK)
		return SR_ERR;
	sr_dbg("Current trigger source %s", devc->trigger_source);

	/* Horizontal trigger position. */
	if (sr_scpi_get_float(sdi->conn, ":TIM:POS?", &devc->horiz_triggerpos) != SR_OK)
		return SR_ERR;
	sr_dbg("Current horizontal trigger position %g", devc->horiz_triggerpos);

	/* Trigger slope. */
	g_free(devc->trigger_slope);
	devc->trigger_slope = NULL;
	if (sr_scpi_get_string(sdi->conn, ":TRIG:EDGE:SLOP?", &devc->trigger_slope) != SR_OK)
		return SR_ERR;
	sr_dbg("Current trigger slope %s", devc->trigger_slope);

	/* Trigger level. */
	if (sr_scpi_get_float(sdi->conn, ":TRIG:EDGE:LEV?", &devc->trigger_level) != SR_OK)
		return SR_ERR;
	sr_dbg("Current trigger level %g", devc->trigger_level);

	/* Sample rate. */
	if (sr_scpi_get_float(sdi->conn, ":ACQ:SRAT?", &devc->sample_rate) != SR_OK)
		return SR_ERR;
	sr_dbg("Current sample rate %g", devc->sample_rate);

	return SR_OK;
}

SR_PRIV int keysight_get_dev_cfg_vertical(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	char *cmd;
	unsigned int i;
	int res;

	devc = sdi->priv;

	/* Vertical gain. */
	for (i = 0; i < devc->model->analog_channels; i++) {
		cmd = g_strdup_printf(":CHAN%d:SCAL?", i + 1);
		res = sr_scpi_get_float(sdi->conn, cmd, &devc->vdiv[i]);
		g_free(cmd);
		if (res != SR_OK)
			return SR_ERR;
	}
	sr_dbg("Current vertical gain:");
	for (i = 0; i < devc->model->analog_channels; i++)
		sr_dbg("CH%d %g", i + 1, devc->vdiv[i]);

	/* Vertical offset. */
	for (i = 0; i < devc->model->analog_channels; i++) {
		cmd = g_strdup_printf(":CHAN%d:OFFS?", i + 1);
		res = sr_scpi_get_float(sdi->conn, cmd, &devc->vert_offset[i]);
		g_free(cmd);
		if (res != SR_OK)
			return SR_ERR;
	}
	sr_dbg("Current vertical offset:");
	for (i = 0; i < devc->model->analog_channels; i++)
		sr_dbg("CH%d %g", i + 1, devc->vert_offset[i]);

	return SR_OK;
}
