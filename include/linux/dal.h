/******************************************************************************
 * Intel mei_dal Linux driver
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2016 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * Contact Information:
 *	Intel Corporation.
 *	linux-mei@linux.intel.com
 *	http://www.intel.com
 *
 * BSD LICENSE
 *
 * Copyright(c) 2016 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef _DAL_H_
#define _DAL_H_

#include <linux/types.h>

#define DAL_VERSION_LEN             32

/**
 * struct dal_version_info -
 *
 * @version: current dal version.
 * @reserved: reserved bytes for future use.
 */
struct dal_version_info {
	char version[DAL_VERSION_LEN];
	u32 reserved[4];
};

#define DAL_KDI_SUCCESS                         0x000
#define DAL_KDI_STATUS_INTERNAL_ERROR           0xA00
#define DAL_KDI_STATUS_INVALID_PARAMS           0xA01
#define DAL_KDI_STATUS_INVALID_HANDLE           0xA02
#define DAL_KDI_STATUS_ILLEGAL_USAGE            0xA03
#define DAL_KDI_STATUS_NOT_INITIALIZED          0xA04
#define DAL_KDI_STATUS_NO_FW_CONNECTION         0xA05
#define DAL_KDI_STATUS_OUT_OF_MEMORY            0xA06
#define DAL_KDI_STATUS_BUFFER_TOO_SMALL         0xA07
#define DAL_KDI_STATUS_OUT_OF_RESOURCE          0xA08
#define DAL_KDI_STATUS_NOT_READY                0xA09
#define DAL_KDI_STATUS_MAX_SESSIONS_REACHED     0xA0A
#define DAL_KDI_STATUS_UNCAUGHT_EXCEPTION       0xA0B
#define DAL_KDI_STATUS_WD_TIMEOUT               0xA0C
#define DAL_KDI_STATUS_APPLET_CRASHED           0xA0D
#define DAL_KDI_STATUS_TA_NOT_FOUND             0xA0E
#define DAL_KDI_STATUS_NON_EXCLUSIVENESS_TA     0xA0F
#define DAL_KDI_STATUS_TA_EXIST                 0xA10
#define DAL_KDI_STATUS_OPERATION_NOT_PERMITTED  0xA11

#define DAL_KDI_INVALID_HANDLE    0

#define KDI_INIT_FLAGS_NONE       0

/**
 * kdi_init -  Init KDI kernel interface.
 * NOTE: Before calling kdi init we assuming that user space api
 *       called.
 *
 * @falgs:    flag for init flow
 * @handle:   pointer to get handle
 *
 * Return: 0 for success < 0 otherwise
 */
int kdi_init(u32 flags, u64 *handle);

/**
 * kdi_deinit -  deinit KDI kernel interface.
 *
 * @handle:   kdi handle
 *
 * Return: 0 for success < 0 otherwise
 */
int kdi_deinit(u64 handle);

/**
 * dal_get_version_info - return DAL version.
 *
 * @version_info:   pointer to KDI version struct
 *
 * Return: 0 for success fail otherwise
 */
int dal_get_version_info(struct dal_version_info *version_info);

/**
 * dal_create_session - will open session to an applet
 *
 * @handle:            kdi handle
 * @session_handle:    pointer to get the session handle
 * @app_id:            applet id
 * @acp_pkg:           applet acp data
 * @acp_pkg_len:       applet acp data size
 * @init_param:		   applet init param
 * @init_param_len:    applet init param size
 *
 * Return: 0 for success fail otherwise
 */
int dal_create_session(u64 handle, u64 *session_handle,
		       const char *app_id, const u8 *acp_pkg,
		       size_t acp_pkg_len, const u8 *init_param,
		       size_t init_param_len);

/**
 * dal_send_and_receive - send and receive data to/from the applet
 *
 * @handle:            kdi handle
 * @session_handle:    session handle
 * @command_id:        command id
 * @input:             send buffer
 * @input_len:         send buffer size
 * @output:            output buffer
 * @output_len:        outout buffer size
 * @response_code:     return code from the applet
 *
 * Return: 0 for success fail otherwise
 */
int dal_send_and_receive(u64 handle, u64 session_handle,
			 int command_id, const u8 *input,
			 size_t input_len, u8 **output,
			 size_t *output_len,
			 int *response_code);

/**
 * dal_close_session - close an open applet session
 *
 * @handle:            kdi handle
 * @session_handle:    session handle
 *
 * Return: 0 for success fail otherwise
 */
int dal_close_session(u64 handle, u64 session_handle);

#endif /* _DAL_H_ */
