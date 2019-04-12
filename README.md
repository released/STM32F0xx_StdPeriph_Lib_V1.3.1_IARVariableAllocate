# STM32F0xx_StdPeriph_Lib_V1.3.1_IARVariableAllocate

update @ 2019/04/12

Add variable allocate example in IAR

variable with specific section and define section in ICF file , will allocate to specific address

1. check id1[3] , id2[3] in Hw_config.c for example.

2. id1[3] , id2[3] will allocate in specific flash address

3. check ID_CODE1 , ID_CODE2 section in stm32f030_flash.icf for example