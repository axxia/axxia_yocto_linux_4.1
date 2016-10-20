/*
 * Copyright (C) 2016 Intel <john.jacques@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 */

#ifndef __DRIVERS_MISC_AXXIA_DSPC_H
#define __DRIVERS_MISC_AXXIA_DSPC_H

/*
  DSP Cluster Control -- Only on 6700
*/

unsigned long axxia_dspc_get_state(void);
void axxia_dspc_set_state(unsigned long);

/*
  ACTLR_EL3/ACTLR_EL2 Access -- For Performance Testing
*/

unsigned long axxia_actlr_el3_get(void);
void axxia_actlr_el3_set(unsigned long);
unsigned long axxia_actlr_el2_get(void);
void axxia_actlr_el2_set(unsigned long);

#endif /* __DRIVERS_MISC_AXXIA_DSPC_H */
