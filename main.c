#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"

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

static void prvSetupHardware( void );


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

    prvSetupHardware();
    clear_lane();

    // create timers
    green_light_timer = xTimerCreate("green_timer", pdMS_TO_TICKS(1000), pdFALSE, (void*)0, green_timer_handle);
    amber_light_timer = xTimerCreate("amber_timer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, amber_timer_handle);
    red_light_timer = xTimerCreate("red_timer", pdMS_TO_TICKS(1000), pdFALSE, (void*)0, red_timer_handle);


  
    // create queues
    xQueue_pot = xQueueCreate(1, sizeof( uint16_t ));
    xQueue_flow = xQueueCreate(1, sizeof( uint16_t ));
    xQueue_led = xQueueCreate(1, sizeof( uint16_t ));
    xQueue_traffic = xQueueCreate(1, sizeof( uint16_t ));

    // create tasks
    xTaskCreate(Traffic_Flow_Adjustment_Task, "Traffic_Flow_Adjustment", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(Traffic_Generator_Task, "Traffic_Generator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(Traffic_Light_State_Task, "Traffic_Light_State", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(System_Display_Task, "System_Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    // lets start!
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
        adc_value = get_ADC_val(); // read adc
        pot_value = (int)100*(adc_value-1600)/(4095-1600); // scale adc val from 0 to 100

        if (xQueueOverwrite(xQueue_pot, &pot_value)){ // send scaled pot val to queue_pot
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
        if(xQueuePeek(xQueue_pot, &pot_val, 1000)) // check queue_pot
        {

            if (xQueueOverwrite(xQueue_flow, &flow)) // update current pot val
            {
                //  high pot val = shorter delay
                // low pot val = longer delay
                vTaskDelay(7000-(66*(pot_val)+1)); 
            }

        }
    }
}


/*-----------------------------------------------------------*/

static void Traffic_Light_State_Task( void *pvParameters )
{
    int led_val = GREEN; // green as default

    green_control(1);
    xTimerStart(green_light_timer, 0); // start green timer
    // time order: green -> amber -> red -> green(loop over again)

    if (xQueueOverwrite(xQueue_led, &led_val)){ // send led color
        vTaskDelay(500);
    }

    while(1){
        vTaskDelay(1000); // ensure this doesnt run again (loop will occur forom timers)
    }

}

static void green_timer_handle( xTimerHandle xTimer )
{
    green_control(0); // green on
    amber_control(1); // amber on
    int led_val = AMBER; // update led


    if (xQueueOverwrite(xQueue_led, &led_val)){ // send led color
        vTaskDelay(500);
    }

    xTimerStart(amber_light_timer, 0); // start amber timer

}

static void amber_timer_handle( xTimerHandle xTimer )
{

    amber_control(0); // amber off
    red_control(1); // red on
    int led_val = RED; // update led
    uint16_t pot_val;

    if(xQueueReceive(xQueue_pot, &pot_val, 500)) // check pot val
    {
        // high pot val = shorter delay
        // low pot val = longer delay
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

    if (xQueueOverwrite(xQueue_led, &led_val)){ // update led color
        vTaskDelay(500);
    }

    xTimerStart(red_light_timer, 0); // start red timer

}

static void red_timer_handle( xTimerHandle xTimer )
{

    red_control(0); // red off
    green_control(1); // green on
    int led_val = GREEN; // update led color
    uint16_t pot_val;

    if(xQueueReceive(xQueue_pot, &pot_val, 500) && xTimerIsTimerActive(green_light_timer) == pdFALSE) // check pot val and if green timer isnt on
    {
        // high pot val = shorter delay
        // low pot val = longer delay
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

    if (xQueueOverwrite(xQueue_led, &led_val)){ // update led val
        vTaskDelay(500);
    }

    xTimerStart(green_light_timer, 0); // start green timer (repeats)

}

/*-----------------------------------------------------------*/


static void System_Display_Task( void *pvParameters )
{
    int led_val = GREEN; // default is green
    int flow;
    int cars[20] = {0}; // empty array of the 20 cars
    while(1)
    {
        // simulate cars moving
        for (int i = 19; i >= 0; i--){ // the array being true means a car is there so we need to move it along every tick
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
        // cars can move freely along
        if (led_val == GREEN){
            for (int i = 19; i > 0; i--){
                cars[i] = cars[i-1]; 
            }
        }
        // amber and red light
        // cars cannot pass stop light
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

        if (xQueueReceive(xQueue_flow, &flow, 100)){ // receive flow
            cars[0] = 1;    // add car to the lane
        }else{
            cars[0] = 0;    // add nothing to the lane
        }

        vTaskDelay(500);

    }
}

// STUFF
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{

    for( ;; );
}

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    for( ;; );
}

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

    xFreeStackSpace = xPortGetFreeHeapSize();

    if( xFreeStackSpace > 100 )
    {
        //
    }
}

// More functions
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
    NVIC_SetPriorityGrouping( 0 );

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

// functions
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

static void red_control( int value ) // toggles red
{
    if (value == 1){
        GPIO_SetBits(GPIOC, red);
    }else{
        GPIO_ResetBits(GPIOC, red);
    }
}

static void amber_control( int value ) // toggles amber
{
    if (value == 1){
        GPIO_SetBits(GPIOC, amber);
    }else{
        GPIO_ResetBits(GPIOC, amber);
    }
}

static void green_control( int value ) // toggles green
{
    if (value == 1){
        GPIO_SetBits(GPIOC, green);
    }else{
        GPIO_ResetBits(GPIOC, green);
    }
}

static void shift_reg( int value ) // shifts registers
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
