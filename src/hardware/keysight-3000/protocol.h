/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2012 Martin Ling <martin-git@earth.li>
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
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

#ifndef LIBSIGROK_HARDWARE_RIGOL_DS_PROTOCOL_H
#define LIBSIGROK_HARDWARE_RIGOL_DS_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "rigol-ds"

/* Size of acquisition buffers */
#define ACQ_BUFFER_SIZE (64 * 1024)

/* Maximum number of samples to retrieve at once. */
#define ACQ_BLOCK_SIZE (64 * 1000)

#define MAX_ANALOG_CHANNELS 4
#define MAX_DIGITAL_CHANNELS 16

struct keysight_vendor {
	const char *name;
	const char *full_name;
};

struct keysight_series {
	const struct keysight_vendor *vendor;
	const char *name;
	uint64_t max_timebase[2];
	uint64_t min_vdiv[2];
	int num_horizontal_divs;
	int live_samples;
	int buffer_samples;
};

enum cmds {
	CMD_GET_HORIZ_TRIGGERPOS,
	CMD_SET_HORIZ_TRIGGERPOS,
};

struct keysight_command {
	int cmd;
	const char *str;
};

struct keysight_model {
	const struct keysight_series *series;
	const char *name;
	uint64_t min_timebase[2];
	unsigned int analog_channels;
	bool has_digital;
	const char **trigger_sources;
	unsigned int num_trigger_sources;
};

enum state_e {
	STATE_IDLE = 10000,
	STATE_DIGITIZING,
	STATE_READING_DATA
};

struct dev_context {
	const struct keysight_model *model;

	/* Device properties */
	const uint64_t (*timebases)[2];
	uint64_t num_timebases;
	const uint64_t (*vdivs)[2];
	uint64_t num_vdivs;

	/* Channel groups */
	struct sr_channel_group **analog_groups;
	struct sr_channel_group *digital_group;

	/* Acquisition settings */
	GSList *enabled_channels;
	int limit_frames;

	/* Device settings */
	gboolean analog_channels[MAX_ANALOG_CHANNELS];
	gboolean digital_channels[MAX_DIGITAL_CHANNELS];
	float timebase;
	float sample_rate;
	float attenuation[MAX_ANALOG_CHANNELS];
	float vdiv[MAX_ANALOG_CHANNELS];
	int vert_reference[MAX_ANALOG_CHANNELS];
	float vert_origin[MAX_ANALOG_CHANNELS];
	float vert_offset[MAX_ANALOG_CHANNELS];
	float vert_inc[MAX_ANALOG_CHANNELS];
	char *trigger_source;
	float horiz_triggerpos;
	char *trigger_slope;
	float trigger_level;
	char *coupling[MAX_ANALOG_CHANNELS];

	/* Number of frames received in total. */
	uint64_t num_frames;
	/* GSList entry for the current channel. */
	GSList *channel_entry;
	/* Number of bytes total for current channel. */
	int num_channel_bytes_total;
	/* Number of bytes received for current channel. */
	uint64_t num_channel_bytes;
	/* Number of bytes of block header read */
	uint64_t num_header_bytes;
	/* Number of bytes in current data block, if 0 block header expected */
	uint64_t num_block_bytes;
	/* Number of data block bytes already read */
	uint64_t num_block_read;
	/* What to wait for in *_receive */
	enum state_e state;
	/* Acq buffers used for reading from the scope and sending data to app */
	unsigned char *buffer;
	float *data;
};

SR_PRIV int keysight_config_set(const struct sr_dev_inst *sdi, const char *format, ...);
SR_PRIV int keysight_capture_start(const struct sr_dev_inst *sdi);
SR_PRIV int keysight_channel_start(const struct sr_dev_inst *sdi);
SR_PRIV int keysight_receive(int fd, int revents, void *cb_data);
SR_PRIV int keysight_get_dev_cfg(const struct sr_dev_inst *sdi);
SR_PRIV int keysight_get_dev_cfg_vertical(const struct sr_dev_inst *sdi);

#endif
