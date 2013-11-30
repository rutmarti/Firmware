/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file sdlog_ringbuffer.h
 * microSD logging
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#ifndef SDLOG_RINGBUFFER_H_
#define SDLOG_RINGBUFFER_H_

struct sdlograw_buffer {
	uint32_t start;
	uint32_t size;
	uint32_t count;
	uint8_t *data;
};

#pragma pack(push, 1)
struct sdlog_sensVect {
	uint32_t frameStart;
	uint32_t tstamp;
	int16_t data[3];
	int16_t temp;
};
#pragma pack(pop)

void sdlograw_buffer_init(struct sdlograw_buffer *lb, uint32_t size);

uint32_t sdlograw_buffer_space_left(struct sdlograw_buffer *lb);

uint32_t sdlograw_buffer_cur_size(struct sdlograw_buffer *lb);

uint32_t sdlograw_buffer_write(struct sdlograw_buffer *lb, const uint8_t *buf, uint32_t bufsize);

uint32_t sdlograw_buffer_read(struct sdlograw_buffer *lb, uint8_t *buf, uint32_t bufsize);

#endif
