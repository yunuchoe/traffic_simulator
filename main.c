/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wwrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 100

#define red     GPIO_Pin_0
#define amber   GPIO_Pin_1
#define green   GPIO_Pin_2

#define pot     GPIO_Pin_3

#define data    GPIO_Pin_6
#define clock   GPIO_Pin_7
#define reset   GPIO_Pin_8

#define RED    0
#define AMBER  1
#define GREEN  2



/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
static void Traffic_Flow_Adjustment_Task( void *pvParameters );
static void Traffic_Generator_Task( void *pvParameters );
static void Traffic_Light_State_Task( void *pvParameters );
static void System_Display_Task( void *pvParameters );
static void green_timer_handle(xTimerHandle xTimer);
static void amber_timer_handle(xTimerHandle xTimer);
static void red_timer_handle(xTimerHandle xTimer);

static int get_ADC_val( void );
static void clear_lane( void );
static void red_control( int value );
static void amber_control( int value );
static void green_control( int value );
static void shift_reg( int value );



xQueueHandle xQueue_pot = 0;
xQueueHandle xQueue_flow = 0;
xQueueHandle xQueue_led = 0;
xQueueHandle xQueue_traffic = 0;

xTimerHandle green_light_timer;
xTimerHandle amber_light_timer;
xTimerHandle red_light_timer;


/*-----------------------------------------------------------*/

int main(void)
{

    /* Configure the system ready to run the demo.  The clock configuration
    can be done here if it was not done before main() was called. */
    prvSetupHardware();
    clear_lane();


    green_light_timer = xTimerCreate("green_timer", pdMS_TO_TICKS(1000), pdFALSE, (void*)0, green_timer_handle);
    amber_light_timer = xTimerCreate("amber_timer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, amber_timer_handle);
    red_light_timer = xTimerCreate("red_timer", pdMS_TO_TICKS(1000), pdFALSE, (void*)0, red_timer_handle);


    /* Create the queue used by the queue send and queue receive tasks.
    http://www.freertos.org/a00116.html */
    /* The number of items the queue can hold. *//* The size of each item the queue holds. */

    xQueue_pot = xQueueCreate(1, sizeof( uint16_t ));
    xQueue_flow = xQueueCreate(1, sizeof( uint16_t ));
    xQueue_led = xQueueCreate(1, sizeof( uint16_t ));
    xQueue_traffic = xQueueCreate(1, sizeof( uint16_t ));

    /* Add to the registry, for the benefit of kernel aware debugging. */
    //vQueueAddToRegistry( xQueue_handle, "MainQueue" );
    // vQueueAddToRegistry(xQueue_pot, "POT");
    // vQueueAddToRegistry(xQueue_led, "LED");

    xTaskCreate(Traffic_Flow_Adjustment_Task, "Traffic_Flow_Adjustment", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(Traffic_Generator_Task, "Traffic_Generator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(Traffic_Light_State_Task, "Traffic_Light_State", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(System_Display_Task, "System_Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();


    return 0;
}


/*-----------------------------------------------------------*/


static void Traffic_Flow_Adjustment_Task( void *pvParameters )
{
    int adc_value;
    int pot_value;
    while(1)
    {
        adc_value = get_ADC_val();
        pot_value = (int)100*(adc_value-1600)/(4095-1600);

        if (xQueueOverwrite(xQueue_pot, &pot_value)){
            vTaskDelay(500);
        }

    }
}

/*-----------------------------------------------------------*/

static void Traffic_Generator_Task( void *pvParameters )
{
uint16_t pot_val;
    int flow;

    while(1)
    {
        if(xQueuePeek(xQueue_pot, &pot_val, 1000))
        {

            if (xQueueOverwrite(xQueue_flow, &flow))
            {
                vTaskDelay(7000-(66*(pot_val)+1));
            }

        }
    }
}


/*-----------------------------------------------------------*/

static void Traffic_Light_State_Task( void *pvParameters )
{
    int led_val = GREEN;

    printf("green on\n");
    green_control(1);
    xTimerStart(green_light_timer, 0);

    if (xQueueOverwrite(xQueue_led, &led_val)){
        vTaskDelay(500);
    }

    while(1){
    vTaskDelay(1000);
    }

}

static void green_timer_handle( xTimerHandle xTimer )
{
    printf("green over \n");
    green_control(0);
    amber_control(1);
    int led_val = AMBER;


    if (xQueueOverwrite(xQueue_led, &led_val)){
        vTaskDelay(500);
    }

    xTimerStart(amber_light_timer, 0);

}

static void amber_timer_handle( xTimerHandle xTimer )
{
    printf("amber over \n");
    amber_control(0);
    red_control(1);
    int led_val = RED;
    uint16_t pot_val;

    if(xQueueReceive(xQueue_pot, &pot_val, 500))
    {
        if (pot_val >= 95){
            xTimerChangePeriod(red_light_timer, pdMS_TO_TICKS(2500), 0);
        }else if(pot_val <= 5){
            xTimerChangePeriod(red_light_timer, pdMS_TO_TICKS(5000), 0);
        }else if(pot_val > 40 && pot_val < 60){
            xTimerChangePeriod(red_light_timer, pdMS_TO_TICKS(5000), 0);
        }else if(pot_val > 60){
            xTimerChangePeriod(red_light_timer, pdMS_TO_TICKS(3000), 0);
        }else if(pot_val < 40){
            xTimerChangePeriod(red_light_timer, pdMS_TO_TICKS(4000), 0);
        }

        vTaskDelay(500);
    }

    if (xQueueOverwrite(xQueue_led, &led_val)){
        vTaskDelay(500);
    }

    xTimerStart(red_light_timer, 0);

}

static void red_timer_handle( xTimerHandle xTimer )
{
    printf("red over \n");
    red_control(0);
    green_control(1);
    int led_val = GREEN;
    uint16_t pot_val;

    if(xQueueReceive(xQueue_pot, &pot_val, 500) && xTimerIsTimerActive(green_light_timer) == pdFALSE)
    {
        if (pot_val >= 95){
            xTimerChangePeriod(green_light_timer, pdMS_TO_TICKS(5000), 0);
        }else if(pot_val <= 5){
            xTimerChangePeriod(green_light_timer, pdMS_TO_TICKS(2500), 0);
        }else if(pot_val > 40 && pot_val < 60){
            xTimerChangePeriod(green_light_timer, pdMS_TO_TICKS(5000), 0);
        }else if(pot_val > 60){
            xTimerChangePeriod(green_light_timer, pdMS_TO_TICKS(4000), 0);
        }else if(pot_val < 40){
            xTimerChangePeriod(green_light_timer, pdMS_TO_TICKS(3000), 0);
        }
        vTaskDelay(500);
    }



    if (xQueueOverwrite(xQueue_led, &led_val)){
        vTaskDelay(500);
    }

    xTimerStart(green_light_timer, 0);

}

/*-----------------------------------------------------------*/


static void System_Display_Task( void *pvParameters )
{
    int led_val = GREEN;
    int flow;
    int cars[20] = {0};
    while(1)
    {
        // simulate cars moving
        for (int i = 19; i >= 0; i--){
            if (cars[i] == 1){
                shift_reg(1);
            }else{
                shift_reg(0);
            }
        }

        // get led value
        if (xQueueReceive(xQueue_led, &led_val, 500)){
            vTaskDelay(500);
        }

        // green light
        if (led_val == GREEN){
            for (int i = 19; i > 0; i--){
                cars[i] = cars[i-1];
            }
        }
        // amber and red light
        else if (led_val == AMBER || led_val == RED){
            // shifting cars before red light
            for (int i = 7; i > 0; i--)
            {
                if (cars[i] == 0){  // if no car in the lane
                    cars[i] = cars[i-1]; // shift cars
                    cars[i-1] = 0;  // clear the previous car
                }
            }

            // shifting cars after red light
            for (int i = 19; i > 8; i--)
            {
                cars[i] = cars[i-1]; // shift cars after red light
                cars[i-1] = 0;       // clear the previous car
            }

        }

        if (xQueueReceive(xQueue_flow, &flow, 100)){
            cars[0] = 1;    // add car to the lane
        }else{
            cars[0] = 0;    // add nothing to the lane
        }

        vTaskDelay(500);

    }
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* The malloc failed hook is enabled by setting
    configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

    Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected.  pxCurrentTCB can be
    inspected in the debugger if the task name passed into this function is
    corrupt. */
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

    /* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
    FreeRTOSConfig.h.

    This function is called on each cycle of the idle task.  In this case it
    does nothing useful, other than report the amount of FreeRTOS heap that
    remains unallocated. */
    xFreeStackSpace = xPortGetFreeHeapSize();

    if( xFreeStackSpace > 100 )
    {
        /* By now, the kernel has allocated everything it is going to, so
        if there is a lot of heap remaining unallocated then
        the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
        reduced accordingly. */
    }
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
    /* Ensure all priority bits are assigned as preemption priority bits.
    http://www.freertos.org/RTOS-Cortex-M3-M4.html */
    NVIC_SetPriorityGrouping( 0 );

    /* TODO: Setup the clocks, etc. here, if they were not configured before
    main() was called. */


    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);    // enable the GPIOC clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);     // enable the ADC clock

    // configure GPIOC
    GPIO_InitTypeDef GPIO_struct;

    GPIO_struct.GPIO_Pin = red | amber | green | data | clock | reset;  // traffic light pins and shift registers
    GPIO_struct.GPIO_Mode = GPIO_Mode_OUT;                              // set output mode
    GPIO_struct.GPIO_Speed = GPIO_Speed_50MHz;                          // set speed
    GPIO_struct.GPIO_OType = GPIO_OType_PP;                             // set push-pull mode
    GPIO_struct.GPIO_PuPd = GPIO_PuPd_NOPULL;                           // disable pull-up / pull-down
    GPIO_Init(GPIOC, &GPIO_struct);

    GPIO_struct.GPIO_Pin = pot;                 // PC3 pin for POT
    GPIO_struct.GPIO_Mode = GPIO_Mode_AN;       // set analog mode
    GPIO_struct.GPIO_PuPd = GPIO_PuPd_NOPULL;   // disable pull-up / pull-down
    GPIO_Init(GPIOC, &GPIO_struct);

    // configure ADC
    ADC_InitTypeDef pot_ADC_struct;
    ADC_StructInit(&pot_ADC_struct);

    ADC_Init(ADC1, &pot_ADC_struct);

    ADC_Cmd(ADC1, ENABLE);                         // enable ADC1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13 , 1, ADC_SampleTime_144Cycles); // ADC config

}

static int get_ADC_val( void )
{

    ADC_SoftwareStartConv(ADC1);                        // start ADC conversion

    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != 1);  // wait for ADC to finish conversion

    return ADC_GetConversionValue(ADC1);                // get ADC value

}

static void clear_lane( void )
{
    GPIO_ResetBits(GPIOC, reset); // shift reg reset off
    for(int i = 0; i < 10; i++);
    GPIO_SetBits(GPIOC, reset);   // shift reg reset on
}

static void red_control( int value )
{
    if (value == 1){
        GPIO_SetBits(GPIOC, red);
    }else{
        GPIO_ResetBits(GPIOC, red);
    }
}

static void amber_control( int value )
{
    if (value == 1){
        GPIO_SetBits(GPIOC, amber);
    }else{
        GPIO_ResetBits(GPIOC, amber);
    }
}

static void green_control( int value )
{
    if (value == 1){
        GPIO_SetBits(GPIOC, green);
    }else{
        GPIO_ResetBits(GPIOC, green);
    }
}

static void shift_reg( int value )
{
    if (value == 1){
        GPIO_SetBits(GPIOC, data);
        GPIO_ResetBits(GPIOC, clock);
        for(int i = 0; i < 5; i++);
        GPIO_SetBits(GPIOC, clock);
        GPIO_ResetBits(GPIOC, data);
    }else{
        GPIO_ResetBits(GPIOC, data);
        GPIO_ResetBits(GPIOC, clock);
        for(int i = 0; i < 5; i++);
        GPIO_SetBits(GPIOC, clock);
    }
}
