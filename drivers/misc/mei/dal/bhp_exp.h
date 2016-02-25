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

#ifndef __BHP_EXP_H__
#define __BHP_EXP_H__

#include <linux/kernel.h>

#include "bh_shared_errcode.h"

typedef int (*bhp_transport_send)(unsigned int handle,
				  unsigned char *buffer,
				  unsigned int length,
				  u64 seq);

typedef int (*bhp_transport_recv)(unsigned int handle,
				  unsigned char *buffer,
				  unsigned int *length);

struct bhp_transport {
	bhp_transport_send send;
	bhp_transport_recv recv;
};

/**
 * Invoke this function before using other API.
 * It will try to connect ME processes(Launcher, SDM and I-VM),
 * and create receiving threads
 * for those process and do other initialization.
 *
 *
 * @return BH_SUCCESS if initialization was successful
 *
 * @return BPE_NO_CONNECTION_TO_FIRMWARE if failed to HECI initialization
 * @return BPE_INTERNAL_ERROR if receiver thread cannot be
 * created or other internal failure
 */
int bhp_init_internal(const struct bhp_transport *transport);


/**
 * Invoke this function before exiting.
 * If BHP_Init is not called, this function will do nothing.
 * If anything goes wrong, please call this function to release resources.
 *
 * @return BH_SUCCESS if success
 */
int bhp_deinit_internal(void);

/**
 * Send Reset command to SDM, Launcher and VM, to let them enter initial state.
 * This function will be blocked until receiving all the responses.
 *
 * @return BH_SUCCESS if success.
 */
int bhp_reset(void);

/**
 * Open Session to specified Java TA.
 * The Firmware side might need to spawn the VM process,
 * and create the TA instance.
 * This function will also connect to the VM process's heci address.
 * This function will block until VM replied the response.
 * Please call BHP_Deinit() to clean up when anything goes wrong.
 *
 * @param session [OUT] the ta session handle, which is used in
 * the function BHP_SendAndRecv.
 * @param ta_id [IN] the applet ID (UUID) to create session.
 * @param ta_pkg [in] TA binary package, i.e, .bpk data.
 * @param pkg_len [in] The length of TA binary package in bytes.
 * @param init_param [IN] the input buffer of the CreateSession command.
 * @param param_len [IN] the length of init_param in bytes
 *
 * @return BH_SUCCESS if success
 *
 */
int bhp_open_ta_session(u64 *session,
			const char *ta_id,
			const u8 *ta_pkg,
			size_t pkg_len,
			const u8 *init_param,
			size_t param_len);

/**
 * Send a CloseTASession command to VM to close the specified Java TA session.
 * This function will be blocked until VM replies the response.
 *
 * @param pSession [IN] the java ta session handle to close.
 *
 * @return BH_SUCCESS if success
 *
 */
int bhp_close_ta_session(const u64 handle);

/**
 * Send a SendAndRecv command to VM. This function will be blocked until VM
 * replies the response.
 *
 * @param handle [IN] the java ta session handle.
 * @param command_id [IN] the command ID.
 * @param input [IN] the input buffer to be sent to TA.
 * @param length [IN] the length of input buffer.
 * @param output [OUT] the pointer to output buffer.
 * @param output_length [IN/OUT] the expected maximum length of output
 * buffer / the actually length of output buffer.
 * @param response_code [OUT] the command result, which is set by
 * IntelApplet.setResponseCode()
 *
 * @return BH_SUCCESS if success
 *
 */
int bhp_send_and_recv(const u64 handle,
		      int command_id,
		      const void *input,
		      size_t length,
		      void **output,
		      size_t *output_length,
		      int *response_code);
#endif /* __BHP_EXP_H__ */
