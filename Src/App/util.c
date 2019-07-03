/******************************************************************************
**  FILENAME:       bsp.c
**
**  PURPOSE:        Board support package for BVC-100
** Argument: 
**  LAST MODIFIED:  2009.10. 
******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

/********************************************************************************
* FUNCTION	 : Min_max_cut   
* DESCRIPTION:  	
* ARGUMENTS	 : 
* RETURNS	 : 
********************************************************************************/ 
float Min_max_cut(float min_val, float max_val, float value)
{
	if(value > max_val) value = max_val;
	if(value < min_val) value = min_val;
	
	return(value);
}

/********************************************************************************
* FUNCTION	 : Abs   
* DESCRIPTION:   	
* ARGUMENTS	 : 
* RETURNS	 : 
********************************************************************************/ 
float Abs(float f)
{
	if(f < 0.0)	return -f;
	
	return f;
}

/********************************************************************************
* FUNCTION	 : _atoi   
* DESCRIPTION: ASCII(decimal) to interger converter     	
* ARGUMENTS	 : ASCII(ex: "123")
* RETURNS	 : interger value(ex: 123)
********************************************************************************/ 
int	_atoi(char *cp)
{
	int	result = 0;
	int sign = 1;
	
	while (*cp == ' ') cp++;
	
	if(*cp=='-')
	{
		sign=-1;
		cp++;
	}

	while (*cp != '\0' &&  *cp != ' ')
	{
		result = result*10 + *cp-'0';
		cp++;
	}

	return result*sign;
}

/********************************************************************************
* FUNCTION	 : _atoh   
* DESCRIPTION: ASCII(hex) to interger(hex) converter
* ARGUMENTS	 : ASCII(ex: "1A")
* RETURNS	 : interger value(ex: 0x1a (26)) 
********************************************************************************/ 
int _atoh(char *cp)
{
	int	result = 0;

	while (*cp == ' ') cp++;

	while (*cp != '\0' &&  *cp != ' ')
	{
		if ('0' <= *cp && *cp <='9')
			result = result*16 + *cp-'0';
		else
			result = result*16 + *cp-'A'+10;
		cp++;
	}
	return result;
}

/********************************************************************************
* FUNCTION	 : _atof   
* DESCRIPTION: ASCII(decimal) to float converter     	
* ARGUMENTS	 : ASCII(ex: "123.1")
* RETURNS	 : float value(ex: 123.1)
********************************************************************************/ 
float	_atof(char *cp)
{
	float	result = 0;
	float	result1 = 0;
	float	div = 1;
	int sign = 1;

	if(*cp=='-')
	{
		sign=-1;
		cp++;
	}
	
	while (*cp != '.' && *cp != '\0')
	{
		result = result*10 + *cp-'0';
		cp++;
	}
	cp++;
	while (*cp != '\0')
	{
		result1 = result1*10 + *cp-'0';
		cp++;
		div*=10;
	}
	result+=(result1/div);
	return (result*sign);
}

#if 0
/********************************************************************************
* FUNCTION	 : _toupper   
* DESCRIPTION:   	
* ARGUMENTS	 : 	
* RETURNS	 :          	
********************************************************************************/ 
char _toupper(char c)
{
	if ('a' <= c && c <= 'z')
		return c-0x20;
	return c;	
}
#endif
/********************************************************************************
* FUNCTION	 :      		delay   
* DESCRIPTION:      	
				in case CPU clock 72MHz(0.014usec)
					cnt 1    ->  0.5usec
					cnt 10   ->  1.7usec
					cnt 100  ->  13usec
					cnt 1000 -> 126usec
				in case CPU clock 36MHz(0.014usec)
					cnt 1    ->  1usec
					cnt 10   ->  3.3usec
					cnt 100  ->  26usec
					cnt 1000 -> 252usec
* ARGUMENTS  : 	
* RETURNS	 :          	
********************************************************************************/ 
void delaycnt(int cnt)
{
	volatile int i = cnt;
	for(;i>0;i--)
		;
}

/********************************************************************************
* FUNCTION	 : passTime
* DESCRIPTION: oldTime으로 부터 흐른 시간을 Return, return value*1msec    	
* ARGUMENTS	 : oldTime: 과거 기준 OSTime(OSTimeGet으로 읽어옴)
* RETURNS	 : pass time(/1msec )
********************************************************************************/ 
unsigned int passTime(uint32_t oldTime)
{
	uint32_t	curT;
	curT = HAL_GetTick();
	if(curT>=oldTime) curT -=oldTime;
	else curT+=(0xffffffff - oldTime);
	return curT;
}

/********************************************************************************
* FUNCTION	 : float2str
* DESCRIPTION: 실수 값으로 문자열 생성    	
* ARGUMENTS	 : f: 변환할 실수 값, place: 소수점 이하 자리 수
* RETURNS	 : 문자열
********************************************************************************/ 
char strFloat[20];
char *float2str(float f, int place)
{
	int m=1;
	char strPrintf[9];

	if (f > -1 && f < 0)
		sprintf(strPrintf, "-%%d.%%0%dd", place); // 정수부를 -0으로 표시
	else
		sprintf(strPrintf, "%%d.%%0%dd", place);
	while(place-- > 0) m*=10;
	sprintf(strFloat, strPrintf, (int)f, abs((int)(f * (float)m) % m));

	return strFloat;
}

