/******************************************************************************
**  FILENAME:       util.h
**
**  PURPOSE:        
**  LAST MODIFIED:  2009.9.30 
******************************************************************************/ 
#ifndef  __UTIL_H__
#define  __UTIL_H__

/*
*********************************************************************************************************
*                                               EXTERNS
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/
#ifdef CPU_CLK72M
#define DELAY_10usec	77		// 72MHz - 77
#define DELAY_100usec	794		// 72Mhz - 794
#define	DELAY_100msec	700000
#else
// 36MHz -> 1/36Mx12(cycle) -> 0.333sec
#define DELAY_1usec		2
#define DELAY_5usec		13
#define DELAY_10usec	30
#define DELAY_50usec	150
#define DELAY_100usec	300
#define DELAY_150usec	450
#define DELAY_200usec	600
#define	DELAY_100msec	300000
#endif


/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            FUNCTION PROTOTYPES
*********************************************************************************************************
*/
float Min_max_cut(float min_val, float max_val, float value);
float Abs(float f);

int	_atoi(char *cp);
int _atoh(char *cp);
float	_atof(char *cp);
//char _toupper(char c);
void delaycnt(int cnt);
unsigned int passTime(unsigned int oldTime);
char *float2str(float f, int place);

#endif                                                          /* End of module include.                               */
