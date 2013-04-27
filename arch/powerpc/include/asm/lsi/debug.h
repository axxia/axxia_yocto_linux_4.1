/*
 * asm/lsi/debug.h
 *
 * Copyright (C) 2010 LSI
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

#ifndef __DRIVERS_LSI_COMMON_DEBUG_H
#define __DRIVERS_LSI_COMMON_DEBUG_H

#define LSI_WARN
#define LSI_ERROR


#undef TRACE
/*#define TRACE*/
#define TRACE_PRINTK
#ifdef TRACE
#ifdef TRACE_PRINTK
#define TRACE_BEGINNING() \
printk(KERN_INFO "nic:%d:%s:Beginning\n", smp_processor_id(), __func__)
#define TRACE_ENDING() \
printk(KERN_INFO "nic:%d:%s:Ending\n", smp_processor_id(), __func__)
#else
#define TRACE_BEGINNING() TRACER_POST("Beginning");
#define TRACE_ENDING() TRACER_POST("Ending");
#endif
#else
#define TRACE_BEGINNING(format, args...)
#define TRACE_ENDING(format, args...)
#endif

#undef PHY_DEBUG
/*#define PHY_DEBUG*/
#if defined(PHY_DEBUG)
#define PHY_DEBUG_PRINT(format, args...) do {\
printk(KERN_INFO "net:%d - PHY_DEBUG - ", __LINE__); \
printk(format, ##args); \
} while (0);
#else
#define PHY_DEBUG_PRINT(format, args...)
#endif

/*
  DEBUG
*/

#if defined(LSI_DEBUG)
#define DEBUG_PRINT(format, args...) do { \
printk(KERN_INFO "%s:%s:%d - DEBUG - ", __FILE__, __func__, __LINE__); \
printk(format, ##args); \
} while (0);
#else
#define DEBUG_PRINT(format, args...) { }
#endif

/*
  WARN
*/

#if defined(LSI_WARN)
#define WARN_PRINT(format, args...) do { \
printk(KERN_WARNING "%s:%s:%d - WARN - ", __FILE__, __func__, __LINE__); \
printk(format, ##args); \
} while (0);
#else
#define WARN_PRINT(format, args...) { }
#endif

/*
  ERROR
*/

#if defined(LSI_ERROR)
#define ERROR_PRINT(format, args...) do { \
printk(KERN_ERR "%s:%s:%d - ERROR - ", __FILE__, __func__, __LINE__); \
printk(format, ##args); \
} while (0);
#else
#define ERROR_PRINT(format, args...) { }
#endif

#endif /* __DRIVERS_LSI_COMMON_DEBUG_H */
