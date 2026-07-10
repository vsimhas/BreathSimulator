#include "cm7_cpu_config.h"
#include "stm32h7xx.h"

void CM7_ConfigureCpu(void)
{
#if defined(CORE_CM7) && (__FPU_PRESENT == 1)
  SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
#endif
  SCB->CCR &= ~SCB_CCR_UNALIGN_TRP_Msk;
  __DSB();
  __ISB();
}
