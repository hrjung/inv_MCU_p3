#ifndef  __INCLUDES_H__
#define  __INCLUDES_H__

#define MAJOR_VERSION			1
#define MINOR_VERSION			0

#include    <stdio.h>
#include	<stdint.h>
#include	<stdbool.h>
#include    <string.h>
#include    <ctype.h>
#include    <stdlib.h>
#include    <stdarg.h>


#define SUPPORT_TASK_WATCHDOG

#define SUPPORT_NFC_OLD

#define SUPPORT_PASSWORD

//#define SUPPORT_PARAMETER_BACKUP

#define SUPPORT_INIT_PARAM

#define SUPPORT_DI_AI_CONTROL

//#define SUPPORT_UNIT_TEST

#define SUPPORT_PRODUCTION_TEST_MODE

#ifdef SUPPORT_UNIT_TEST
	#define STATIC
	#undef SUPPORT_DRIVER_HW
	#undef SUPPORT_TASK_WATCHDOG
#else
	#define SUPPORT_DRIVER_HW
	#define STATIC		static
#endif


#endif
