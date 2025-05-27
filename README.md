# HyCtrl
Stm32 HT22 humidity sensor using TIM capture DMA mode (safe reliable one pin)

Single Timer input capture i/o used first as gpio out + pull control to initiate HT22 start measure.
Then the i/o is turn back to timer alternate timer capture function , and dma input capture started.
One all bit (period) get capture  post period processing translate time to bit &byte 

unlike polling this method is very safe and reliabale and don't consume any cpu .

The timer period shal lbe set to 1Mhz (psc =72 in used STM32f401 blackpill @ 72MHz)
