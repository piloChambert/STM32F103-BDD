# STM32F103-BDD
Bridge Bucket Device emulation with a STM32F103

Provided "AS-IS", not really working well (white noise on ADC input... I don't think it can be fixed).

It's using a 12bits R2R DAC and the one of STM32 adc for audio (the other ADC is used for control pot).
Some LPF filter are needed on input and output (~7khz).
