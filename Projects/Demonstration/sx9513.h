/******************************************************************************
 * Project        : STM32F030+SX9513
 * File           : sx9513.h
 * Copyright      : 2014 Yosun Singapore Pte Ltd
 ******************************************************************************
  Change History:

    Version 1.0.0 - Aug 2014
    > Initial revision

******************************************************************************/
#ifndef _SX9513_H_
#define _SX9513_H_

void SX9513_Init(void);
void SX9513_IrqSrc(void);
void SX9513_ReadBL0(void);
void SX9513_HandleBL0(void);

#endif
