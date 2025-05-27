# HyCtrl
Stm32 HT22 humidity sensor using TIM capture DMA mode (safe reliable with one pin)

Single Timer input capture i/o used first  as gpio out + pull to initiate HT22 start measure 
then the i/o is turn back to tiemr alternate  tiemr capture function , by dma we then capture 
all ht22 outpur bit.
One all bit get cpatured post timer period processing give result 
unlike polling this method is very safe and relibale and don't consume any cpu .

The timer period shal lbe set to 1Mhz (psc =72 in used STM32f401 blackpill @ 72MHz)
