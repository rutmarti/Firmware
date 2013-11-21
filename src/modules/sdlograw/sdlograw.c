/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sdlog.c
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 * Simple SD logger for flight data. Buffers new sensor values and
 * does the heavy SD I/O in a low-priority worker thread.
 */

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/prctl.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <systemlib/err.h>
#include <unistd.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>

#include <systemlib/systemlib.h>

#include <mavlink/mavlink_log.h>

#include "sdlograw_buffer.h"

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static const int MAX_NO_LOGFOLDER = 999;	/**< Maximum number of log folders */

static const char *mountpoint = "/fs/microsd";
int sdlograw_file = -1;
struct sdlograw_buffer lb;

/* mutex / condition to synchronize threads */
pthread_mutex_t logbuffer_mutex;
pthread_cond_t logbuffer_cond;

unsigned sdlograw_bytes_written = 0;

/**
 * System state vector log buffer writing
 */
static void *sdlograw_write_thread(void *arg);

/**
 * Create the thread to write the system vector
 */
pthread_t sdlograw_write_start_thread(struct sdlograw_buffer *logbuf);

/**
 * SD log management function.
 */
__EXPORT int sdlograw_main(int argc, char *argv[]);

/**
 * Mainloop of sd log deamon.
 */
int sdlograw_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void sdlograw_usage(const char *reason);

static int sdlograw_file_exist(const char *filename);

/**
 * Print the current status.
 */
static void sdlograw_print_status(void);

/**
 * Create folder for current logging session.
 */
static int sdlograw_create_logfolder(char *folder_path);

static void sdlograw_usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	errx(1, "usage: sdlog {start|stop|status}\n\n");
}

static uint64_t sdlograw_starttime = 0;

/**
 * @return 0 if file exists
 */
int sdlograw_file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer);
}

/**
 * The sd log deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn_cmd().
 */
int sdlograw_main(int argc, char *argv[])
{
	if (argc < 1)
		sdlograw_usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("sdlograw already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("sdlograw",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT - 30,
					 4096,
					 sdlograw_thread_main,
					 (const char **)argv);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (!thread_running) {
			printf("\tsdlograw is not started\n");
		}

		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			sdlograw_print_status();

		} else {
			printf("\tsdlograw not started\n");
		}

		exit(0);
	}

	sdlograw_usage("unrecognized command");
	exit(1);
}

int sdlograw_create_logfolder(char *folder_path)
{
	/* make folder on sdcard */
	uint16_t foldernumber = 1; // start with folder 0001
	int mkdir_ret;

	/* look for the next folder that does not exist */
	while (foldernumber < MAX_NO_LOGFOLDER) {
		/* set up file path: e.g. /mnt/sdcard/sensorfile0001.txt */
		sprintf(folder_path, "%s/log%04u", mountpoint, foldernumber);
		mkdir_ret = mkdir(folder_path, S_IRWXU | S_IRWXG | S_IRWXO);
		/* the result is -1 if the folder exists */

		if (mkdir_ret == 0) {
			/* found directory */
			break;
		} else if (mkdir_ret == -1) {
			/* folder exists already */
			foldernumber++;
		} else {
			warn("failed creating new folder");
			return -1;
		}
	}

	if (foldernumber >= MAX_NO_LOGFOLDER) {
		/* we should not end up here, either we have more than MAX_NO_LOGFOLDER on the SD card, or another problem */
		warn("all %d possible folders exist already", MAX_NO_LOGFOLDER);
		return -1;
	}

	return 0;
}

static void *sdlograw_write_thread(void *arg)
{
	/* set name */
	prctl(PR_SET_NAME, "sdlog microSD I/O", 0);

	struct sdlograw_buffer *logbuf = (struct sdlograw_buffer *)arg;

	int poll_count = 0;

	uint8_t buffer[512];

	while (!thread_should_exit) {
		
		/* make sure threads are synchronized */
		pthread_mutex_lock(&logbuffer_mutex);
		int ret = 0;

		/* only wait if no data is available to process */
		if (sdlograw_buffer_cur_size(logbuf) < 512) {
			/* blocking wait for new data at this line */
			pthread_cond_wait(&logbuffer_cond, &logbuffer_mutex);
		}

		/* only quickly load data, do heavy I/O a few lines down */
		ret = sdlograw_buffer_read(logbuf, buffer, 512);
		/* continue */
		pthread_mutex_unlock(&logbuffer_mutex);

		if (ret)
		{
			sdlograw_bytes_written += write(sdlograw_file, (const char *)buffer, ret);
			fsync(sdlograw_file);
		}
		else
		{
			warnx("not enough data read");
		}
	}

	fsync(sdlograw_file);

	return OK;
}

pthread_t sdlograw_write_start_thread(struct sdlograw_buffer *logbuf)
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	struct sched_param param;
	/* low priority, as this is expensive disk I/O */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 40;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr, 2048);

	pthread_t thread;
	pthread_create(&thread, &receiveloop_attr, sdlograw_write_thread, logbuf);
	return thread;
}

int sdlograw_thread_main(int argc, char *argv[])
{
	if (sdlograw_file_exist(mountpoint) != OK) {
		errx(1, "logging mount point %s not present, exiting.", mountpoint);
	}

	char folder_path[64];

	if (sdlograw_create_logfolder(folder_path))
		errx(1, "unable to create logging folder, exiting.");

	/* string to hold the path to the sensorfile */
	char path_buf[64] = "";

	/* only print logging path, important to find log file later */
	warnx("logging to directory %s\n", folder_path);

	/* set up file path: e.g. /mnt/sdcard/log0001/log.bin */
	sprintf(path_buf, "%s/%s.bin", folder_path, "dat");

	if (0 == (sdlograw_file = open(path_buf, O_CREAT | O_WRONLY | O_DSYNC))) {
		errx(1, "opening %s failed.\n", path_buf);
	}

	/* --- IMPORTANT: DEFINE NUMBER OF ORB STRUCTS TO WAIT FOR HERE --- */
	/* number of messages */
	const ssize_t fdsc = 1;
	/* Sanity check variable and index */
	ssize_t fdsc_count = 0;
	/* file descriptors to wait for */
	struct pollfd fds[fdsc];


	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));

	int sensor_sub;

	/* --- SENSORS RAW VALUE --- */
	/* subscribe to ORB for sensors raw */
	sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	fds[fdsc_count].fd = sensor_sub;
	/* do not rate limit, instead use skip counter (aliasing on rate limit) */
	fds[fdsc_count].events = POLLIN;

	thread_running = true;

	/* initialize log buffer with a size of 2 kbyte */
	sdlograw_buffer_init(&lb, 2048);

	/* initialize thread synchronization */
	pthread_mutex_init(&logbuffer_mutex, NULL);
  	pthread_cond_init(&logbuffer_cond, NULL);

	/* start logbuffer emptying thread */
	pthread_t sysvector_pthread = sdlograw_write_start_thread(&lb);

	sdlograw_starttime = hrt_absolute_time();

	while (!thread_should_exit) {

		/* only poll for sensor_combined */
		int poll_ret = poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* XXX this means none of our providers is giving us data - might be an error? */
		} else if (poll_ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else {

			int ifds = 0;

			/* --- SENSORS RAW VALUE --- */
			if (fds[ifds++].revents & POLLIN) {

				// /* copy sensors raw data into local buffer */
				// orb_copy(ORB_ID(sensor_combined), subs.sensor_sub, &buf.raw);
				// /* write out */
				// sensor_combined_bytes += write(sensorfile, (const char*)&(buf.raw), sizeof(buf.raw));

				/* always copy sensors raw data into local buffer, since poll flags won't clear else */
				orb_copy(ORB_ID(sensor_combined), sensor_sub, &raw);

				struct sdlog_sensVect sensors;
				memcpy(sensors.accel, raw.accelerometer_raw, sizeof(raw.accelerometer_raw));
				memcpy(sensors.gyro, raw.gyro_raw, sizeof(raw.gyro_raw));
				memcpy(sensors.mag, raw.magnetometer_raw, sizeof(raw.magnetometer_raw));

				sensors.tstamp = (uint32_t)raw.timestamp;
				sensors.frameStart = 0x77;
				sensors.frameStop = 0xaa;

				/* put into buffer for later IO */
				pthread_mutex_lock(&logbuffer_mutex);
				sdlograw_buffer_write(&lb, (uint8_t *)&sensors, sizeof(sensors));
				/* signal the other thread new data, but not yet unlock */
				if (sdlograw_buffer_cur_size(&lb) >= 512) {
					/* only request write if several packets can be written at once */
					pthread_cond_signal(&logbuffer_cond);
				}
				/* unlock, now the writer thread may run */
				pthread_mutex_unlock(&logbuffer_mutex);
			}

		}

	}

	sdlograw_print_status();

	/* wake up write thread one last time */
	pthread_mutex_lock(&logbuffer_mutex);
	pthread_cond_signal(&logbuffer_cond);
	/* unlock, now the writer thread may return */
	pthread_mutex_unlock(&logbuffer_mutex);

	/* wait for write thread to return */
	(void)pthread_join(sysvector_pthread, NULL);

  	pthread_mutex_destroy(&logbuffer_mutex);
  	pthread_cond_destroy(&logbuffer_cond);

	warnx("exiting.\n\n");

	thread_running = false;

	return 0;
}

void sdlograw_print_status()
{
	unsigned bytes = sdlograw_bytes_written;
	float mebibytes = bytes / 1024.0f / 1024.0f;
	float seconds = ((float)(hrt_absolute_time() - sdlograw_starttime)) / 1000000.0f;

	warnx("wrote %4.2f MiB (average %5.3f MiB/s).\n", (double)mebibytes, (double)(mebibytes / seconds));
}
