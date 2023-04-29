#include <stdint.h>
#include "stm32g4xx.h"
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6100100)
    #include "__vectors_ac6.h"
#elif defined(__GNUC__)
    #if defined(__clang__)
        #include "__vectors_llvm.h"
    #else
        //#include "__vectors_gcc.h"
    #endif
#endif
#include "system_ARMCM0.h"
#include "cmsis_compiler.h"

/******************************************************************************
 * @file     startup_<Device>.c
 * @brief    CMSIS-Core(M) Device Startup File for
 *           Device <Device>
 * @version  V1.0.0
 * @date     20. January 2021
 ******************************************************************************/
/*
 * Copyright (c) 2009-2021 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*---------------------------------------------------------------------------
  External References
 *---------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;

#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
extern uint32_t __STACK_SEAL;
#endif

extern __NO_RETURN void __PROGRAM_START(void);

/*---------------------------------------------------------------------------
  Internal References
 *---------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void);
__NO_RETURN void Default_Handler(void);

/* ToDo: Add Cortex exception handler according the used Cortex-Core */
/*---------------------------------------------------------------------------
  Exception / Interrupt Handler
 *---------------------------------------------------------------------------*/
void NMI_Handler                        (void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler                  (void) __attribute__((weak));
void MemManage_Handler                  (void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler                   (void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler                 (void) __attribute__((weak, alias("Default_Handler")));
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
void SVC_Handler                        (void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler                   (void) __attribute__((weak, alias("Default_Handler")));
/*   0                                                                                       */
void PendSV_Handler                     (void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler                    (void) __attribute__((weak, alias("Default_Handler")));
void WWDG_IRQHandler                    (void) __attribute__((weak, alias("Default_Handler")));
void PVD_PVM_IRQHandler                 (void) __attribute__((weak, alias("Default_Handler")));
void RTC_TAMP_LSECSS_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler                (void) __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler                   (void) __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler                     (void) __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler                   (void) __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler                   (void) __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler                   (void) __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler                   (void) __attribute__((weak, alias("Default_Handler")));
void EXTIr_IRQHandler                   (void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler           (void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel2_IRQHandler           (void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel3_IRQHandler           (void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel4_IRQHandler           (void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel5_IRQHandler           (void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel6_IRQHandler           (void) __attribute__((weak, alias("Default_Handler")));
/*   0                                                                                       */
void ADC1_2_IRQHandler                  (void) __attribute__((weak, alias("Default_Handler")));
void USB_HP_IRQHandler                  (void) __attribute__((weak, alias("Default_Handler")));
void USB_LP_IRQHandler                  (void) __attribute__((weak, alias("Default_Handler")));
void FDCAN1_IT0_IRQHandler              (void) __attribute__((weak, alias("Default_Handler")));
void FDCAN1_IT1_IRQHandler              (void) __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler                 (void) __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_TIM15_IRQHandler          (void) __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_TIM16_IRQHandler           (void) __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM17_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler                 (void) __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler                    (void) __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler                    (void) __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler                    (void) __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler                 (void) __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler                 (void) __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler                 (void) __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler                 (void) __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler                    (void) __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler                    (void) __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler                  (void) __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler                  (void) __attribute__((weak, alias("Default_Handler")));
void USART3_IRQHandler                  (void) __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler               (void) __attribute__((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler               (void) __attribute__((weak, alias("Default_Handler")));
void USBWakeUp_IRQHandler               (void) __attribute__((weak, alias("Default_Handler")));
void TIM8_BRK_IRQHandler                (void) __attribute__((weak, alias("Default_Handler")));
void TIM8_UP_IRQHandler                 (void) __attribute__((weak, alias("Default_Handler")));
void TIM8_TRG_COM_IRQHandler            (void) __attribute__((weak, alias("Default_Handler")));
void TIM8_CC_IRQHandler                 (void) __attribute__((weak, alias("Default_Handler")));
/*   0                                                                                       */
/*   0                                                                                       */
void LPTIM1_IRQHandler                  (void) __attribute__((weak, alias("Default_Handler")));
/*   0                                                                                       */
void SPI3_IRQHandler                    (void) __attribute__((weak, alias("Default_Handler")));
void UART4_IRQHandler                   (void) __attribute__((weak, alias("Default_Handler")));
/*   0                                                                                       */
void TIM6_DAC_IRQHandler                (void) __attribute__((weak, alias("Default_Handler")));
void TIM7_IRQHandler                    (void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel1_IRQHandler           (void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel2_IRQHandler           (void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel3_IRQHandler           (void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel4_IRQHandler           (void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel5_IRQHandler           (void) __attribute__((weak, alias("Default_Handler")));
/*   0                                                                                       */
/*   0                                                                                       */
void UCPD1_IRQHandler                   (void) __attribute__((weak, alias("Default_Handler")));
void COMP1_2_3_IRQHandler               (void) __attribute__((weak, alias("Default_Handler")));
void COMP4_IRQHandler                   (void) __attribute__((weak, alias("Default_Handler")));
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
void CRS_IRQHandler                     (void) __attribute__((weak, alias("Default_Handler")));
void SAI1_IRQHandler                    (void) __attribute__((weak, alias("Default_Handler")));
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
void FPU_IRQHandler                     (void) __attribute__((weak, alias("Default_Handler")));
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
/*   0                                                                                       */
void RNG_IRQHandler                     (void) __attribute__((weak, alias("Default_Handler")));
void LPUART1_IRQHandler                 (void) __attribute__((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler                 (void) __attribute__((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler                 (void) __attribute__((weak, alias("Default_Handler")));
void DMAMUX_OVR_IRQHandler              (void) __attribute__((weak, alias("Default_Handler")));
/*   0                                                                                       */
/*   0                                                                                       */
void DMA2_Channel6_IRQHandler           (void) __attribute__((weak, alias("Default_Handler")));
/*   0                                                                                       */
/*   0                                                                                       */
void CORDIC_IRQHandler                  (void) __attribute__((weak, alias("Default_Handler")));
void FMAC_IRQHandler                    (void) __attribute__((weak, alias("Default_Handler")));
/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

/* ToDo: Add Cortex exception vectors according the used Cortex-Core */
extern const VECTOR_TABLE_Type __VECTOR_TABLE[256];
const VECTOR_TABLE_Type __VECTOR_TABLE[256] __VECTOR_TABLE_ATTRIBUTE = {
    (VECTOR_TABLE_Type)(&__INITIAL_SP),                         /*     Initial Stack Pointer */
    (VECTOR_TABLE_Type)&NMI_Handler                                                          ,
    (VECTOR_TABLE_Type)&HardFault_Handler                                                    ,
    (VECTOR_TABLE_Type)&MemManage_Handler                                                    ,
    (VECTOR_TABLE_Type)&BusFault_Handler                                                     ,
    (VECTOR_TABLE_Type)&UsageFault_Handler                                                   ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)&SVC_Handler                                                          ,
    (VECTOR_TABLE_Type)&DebugMon_Handler                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)&PendSV_Handler                                                       ,
    (VECTOR_TABLE_Type)&SysTick_Handler                                                      ,
    (VECTOR_TABLE_Type)&WWDG_IRQHandler                                                      ,
    (VECTOR_TABLE_Type)&PVD_PVM_IRQHandler                                                   ,
    (VECTOR_TABLE_Type)&RTC_TAMP_LSECSS_IRQHandler                                           ,
    (VECTOR_TABLE_Type)&RTC_WKUP_IRQHandler                                                  ,
    (VECTOR_TABLE_Type)&FLASH_IRQHandler                                                     ,
    (VECTOR_TABLE_Type)&RCC_IRQHandler                                                       ,
    (VECTOR_TABLE_Type)&EXTI0_IRQHandler                                                     ,
    (VECTOR_TABLE_Type)&EXTI1_IRQHandler                                                     ,
    (VECTOR_TABLE_Type)&EXTI2_IRQHandler                                                     ,
    (VECTOR_TABLE_Type)&EXTI3_IRQHandler                                                     ,
    (VECTOR_TABLE_Type)&EXTIr_IRQHandler                                                     ,
    (VECTOR_TABLE_Type)&DMA1_Channel1_IRQHandler                                             ,
    (VECTOR_TABLE_Type)&DMA1_Channel2_IRQHandler                                             ,
    (VECTOR_TABLE_Type)&DMA1_Channel3_IRQHandler                                             ,
    (VECTOR_TABLE_Type)&DMA1_Channel4_IRQHandler                                             ,
    (VECTOR_TABLE_Type)&DMA1_Channel5_IRQHandler                                             ,
    (VECTOR_TABLE_Type)&DMA1_Channel6_IRQHandler                                             ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)&ADC1_2_IRQHandler                                                    ,
    (VECTOR_TABLE_Type)&USB_HP_IRQHandler                                                    ,
    (VECTOR_TABLE_Type)&USB_LP_IRQHandler                                                    ,
    (VECTOR_TABLE_Type)&FDCAN1_IT0_IRQHandler                                                ,
    (VECTOR_TABLE_Type)&FDCAN1_IT1_IRQHandler                                                ,
    (VECTOR_TABLE_Type)&EXTI9_5_IRQHandler                                                   ,
    (VECTOR_TABLE_Type)&TIM1_BRK_TIM15_IRQHandler                                            ,
    (VECTOR_TABLE_Type)&TIM1_UP_TIM16_IRQHandler                                             ,
    (VECTOR_TABLE_Type)&TIM1_TRG_COM_TIM17_IRQHandler                                        ,
    (VECTOR_TABLE_Type)&TIM1_CC_IRQHandler                                                   ,
    (VECTOR_TABLE_Type)&TIM2_IRQHandler                                                      ,
    (VECTOR_TABLE_Type)&TIM3_IRQHandler                                                      ,
    (VECTOR_TABLE_Type)&TIM4_IRQHandler                                                      ,
    (VECTOR_TABLE_Type)&I2C1_EV_IRQHandler                                                   ,
    (VECTOR_TABLE_Type)&I2C1_ER_IRQHandler                                                   ,
    (VECTOR_TABLE_Type)&I2C2_EV_IRQHandler                                                   ,
    (VECTOR_TABLE_Type)&I2C2_ER_IRQHandler                                                   ,
    (VECTOR_TABLE_Type)&SPI1_IRQHandler                                                      ,
    (VECTOR_TABLE_Type)&SPI2_IRQHandler                                                      ,
    (VECTOR_TABLE_Type)&USART1_IRQHandler                                                    ,
    (VECTOR_TABLE_Type)&USART2_IRQHandler                                                    ,
    (VECTOR_TABLE_Type)&USART3_IRQHandler                                                    ,
    (VECTOR_TABLE_Type)&EXTI15_10_IRQHandler                                                 ,
    (VECTOR_TABLE_Type)&RTC_Alarm_IRQHandler                                                 ,
    (VECTOR_TABLE_Type)&USBWakeUp_IRQHandler                                                 ,
    (VECTOR_TABLE_Type)&TIM8_BRK_IRQHandler                                                  ,
    (VECTOR_TABLE_Type)&TIM8_UP_IRQHandler                                                   ,
    (VECTOR_TABLE_Type)&TIM8_TRG_COM_IRQHandler                                              ,
    (VECTOR_TABLE_Type)&TIM8_CC_IRQHandler                                                   ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)&LPTIM1_IRQHandler                                                    ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)&SPI3_IRQHandler                                                      ,
    (VECTOR_TABLE_Type)&UART4_IRQHandler                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)&TIM6_DAC_IRQHandler                                                  ,
    (VECTOR_TABLE_Type)&TIM7_IRQHandler                                                      ,
    (VECTOR_TABLE_Type)&DMA2_Channel1_IRQHandler                                             ,
    (VECTOR_TABLE_Type)&DMA2_Channel2_IRQHandler                                             ,
    (VECTOR_TABLE_Type)&DMA2_Channel3_IRQHandler                                             ,
    (VECTOR_TABLE_Type)&DMA2_Channel4_IRQHandler                                             ,
    (VECTOR_TABLE_Type)&DMA2_Channel5_IRQHandler                                             ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)&UCPD1_IRQHandler                                                     ,
    (VECTOR_TABLE_Type)&COMP1_2_3_IRQHandler                                                 ,
    (VECTOR_TABLE_Type)&COMP4_IRQHandler                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)&CRS_IRQHandler                                                       ,
    (VECTOR_TABLE_Type)&SAI1_IRQHandler                                                      ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)&FPU_IRQHandler                                                       ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)&RNG_IRQHandler                                                       ,
    (VECTOR_TABLE_Type)&LPUART1_IRQHandler                                                   ,
    (VECTOR_TABLE_Type)&I2C3_EV_IRQHandler                                                   ,
    (VECTOR_TABLE_Type)&I2C3_ER_IRQHandler                                                   ,
    (VECTOR_TABLE_Type)&DMAMUX_OVR_IRQHandler                                                ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)&DMA2_Channel6_IRQHandler                                             ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)0                                                                     ,
    (VECTOR_TABLE_Type)&CORDIC_IRQHandler                                                    ,
    (VECTOR_TABLE_Type)&FMAC_IRQHandler
};
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

/*---------------------------------------------------------------------------
  Reset Handler called on controller reset
 *---------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void)
{
    __set_PSP((uint32_t)(&__INITIAL_SP));

/* ToDo: Initialize stack limit register for Armv8-M Main Extension based processors*/
//    __set_MSP((uint32_t)(&__STACK_LIMIT));
//    __set_PSP((uint32_t)(&__STACK_LIMIT));

/* ToDo: Add stack sealing for Armv8-M based processors */
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    __TZ_set_STACKSEAL_S((uint32_t *)(&__STACK_SEAL));
#endif

    SystemInit();      /* CMSIS System Initialization */
    __PROGRAM_START(); /* Enter PreMain (C library entry point) */
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

/*---------------------------------------------------------------------------
  Hard Fault Handler
 *---------------------------------------------------------------------------*/
void HardFault_Handler(void)
{
    while (1)
        ;
}

/*---------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *---------------------------------------------------------------------------*/
void Default_Handler(void)
{
    while (1)
        ;
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic pop
#endif
