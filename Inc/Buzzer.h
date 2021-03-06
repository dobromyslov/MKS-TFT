/*
 * Buzzer.h
 *
 * Created: 13/11/2014 22:56:34
 *  Author: David
 */


#ifndef BUZZER_H_
#define BUZZER_H_

#include "mks_conf.h"
#include "stm32f1xx_hal_tim.h"

#ifdef __cplusplus

namespace Buzzer
{
	void Beep(uint32_t ms);
	bool Noisy();
	void CheckStop();
}
#else
    void BuzzerCheckStop();
#endif

#endif /* BUZZER_H_ */
