/*!
    \file    gd32f30x_it.h
    \brief   interrupt handler declarations
*/

#ifndef GD32F30X_IT_H
#define GD32F30X_IT_H

#include "gd32f30x.h"

/* SysTick_Handler() calls delay_decrement(), so that declaration has to be
   visible here too - otherwise gd32f30x_it.c warns about an implicit
   declaration of delay_decrement(). */
#include "systick.h"

void SysTick_Handler(void);

#endif /* GD32F30X_IT_H */
