#include <stdint.h>
#include <stm32l4xx_ll_tim.h>


/* Set TIM1 Frequency ------------------------------------------------------------*/
// Sets the frequency of TIM1s clock and tick triggers.
void bsp_set_TIM1_frequency(uint32_t freq_clock, uint32_t freq_tick) {

    /* Set Timer Prescaler */
    uint32_t k_prescale = (uint32_t)(SystemCoreClock/freq_clock)-1;
    LL_TIM_SetPrescaler(TIM1, k_prescale);
    
    /* Set Timer Auto Reload */
    uint32_t k_reload = (uint32_t)(freq_clock/freq_tick)-1;
    LL_TIM_SetAutoReload(TIM1, k_reload);
}


/* Set TIM1 Compare --------------------------------------------------------------*/
// Sets the compare parameters for TIM1s CH1, CH2 and CH3.
void bsp_set_TIM1_compare(uint32_t comp_CH1, uint32_t comp_CH2, uint32_t comp_CH3) {
    
    /* Set Timer Compare */
    LL_TIM_OC_SetCompareCH1(TIM1, comp_CH1);
    LL_TIM_OC_SetCompareCH2(TIM1, comp_CH2);
    LL_TIM_OC_SetCompareCH3(TIM1, comp_CH3);
}


/* Set TIM2 Interrupt Frequency --------------------------------------------------*/
// Sets the frequency of TIM2s clock and tick triggers.
void bsp_set_TIM2_frequency(uint32_t freq_clock, uint32_t freq_tick) {

    /* Set Timer Prescaler */
    uint32_t k_prescale = (uint32_t)(SystemCoreClock/freq_clock)-1;
    LL_TIM_SetPrescaler(TIM2, k_prescale);
    
    /* Set Timer Auto Reload */
    uint32_t k_reload = (uint32_t)(freq_clock/freq_tick)-1;
    LL_TIM_SetAutoReload(TIM2, k_reload);
}