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
 * @file sdlog_log.c
 * MAVLink text logging.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <string.h>
#include <stdlib.h>

#include "sdlograw_buffer.h"

void sdlograw_buffer_init(struct sdlograw_buffer *lb, uint32_t size)
{
	lb->size  = size;
	lb->start = 0;
	lb->count = 0;
	lb->data  = (unsigned char *)zalloc(lb->size);
}

uint32_t sdlograw_buffer_space_left(struct sdlograw_buffer *lb)
{
	return (lb->size - lb->count);
}

uint32_t sdlograw_buffer_cur_size(struct sdlograw_buffer *lb)
{
	return lb->count;
}

uint32_t sdlograw_buffer_write(struct sdlograw_buffer *lb, const uint8_t *buf, uint32_t bufsize)
{
	uint32_t wroteSize = 0;
	if ((bufsize > 0) && (sdlograw_buffer_space_left(lb) >= bufsize))
	{
		uint32_t fbIdx = (lb->start + lb->count) % lb->size;
		uint32_t lbIdx = (fbIdx + bufsize - 1);
		if (lbIdx >= lb->size)
		{
			uint32_t fwSize = lb->size - fbIdx;
			uint32_t lwSize = bufsize - fwSize;
			// copy the first chunk at the end of the log buffer
			memcpy(&lb->data[fbIdx], &buf[0], fwSize);
			// copy the rest at the beginning
			memcpy(&lb->data[0], &buf[fwSize], lwSize);
		}
		else
		{
			memcpy(&lb->data[fbIdx], &buf[0], bufsize);
		}
		// increase count
		lb->count += bufsize;
		wroteSize = bufsize;
	}

	return wroteSize;
}

uint32_t sdlograw_buffer_read(struct sdlograw_buffer *lb, uint8_t *buf, uint32_t bufsize)
{
	uint32_t readSize = (lb->count < bufsize) ? lb->count : bufsize;
	if (readSize > 0)
	{
		uint32_t fbIdx = lb->start;
		uint32_t lbIdx = (fbIdx + bufsize - 1);
		if (lbIdx >= lb->size)
		{
			uint32_t fSize = lb->size - fbIdx;
			uint32_t lSize = bufsize - fSize;
			// copy the first chunk at the end of the log buffer
			memcpy(&buf[0], &lb->data[fbIdx], fSize);
			// copy the rest at the beginning
			memcpy(&buf[fSize], &lb->data[0], lSize);
		}
		else
		{
			memcpy(&buf[0], &lb->data[fbIdx], bufsize);
		}
		// decrease count
		lb->count -= bufsize;
		// increase start "pointer"
		lb->start = (lb->start + bufsize) % lb->size;
	}

	return readSize;
}
