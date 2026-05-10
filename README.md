A real-time traffic control system implemented on an STM32F4 microcontroller. Built using C and FreeRTOS, this project utilizes hardware level multitasking to maintain precise control over modeled traffic flow. 

A track of LED lights wired to a breadboard act as modelled cars and flow out randomly in response to the read ADC value. About two thirds near the end, the stoplight is positioned in which modelled cars can only pass during green lights. During both yellow and red lights, the cars wait and stack up, before being allowed to pass once the signal turns green.

The core logic can be seen here: [**main.c**](./src/main.c)
