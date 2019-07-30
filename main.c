#include "driverlib.h"
#include "Board.h"

// 0 is x-axis, 1 is y-axis
int axis = 1;

// Counters to keep track of how many steps the motor has progressed through (in both x and y)
int stepsx = 0;
int stepsy = 0;

// Boolean flags to check stepper bounds (0 if not hit bound, 1 if bound hit)
int xmin = 0;
int xmax = 0;
int ymin = 0;
int ymax = 0;

// Which pin (i.e. photointerrupter) is being checked
int interrupterPin = 0;

void loop();


// A utility method used to convert integer values to binary addresses corresponding to segments on the LCD
uint8_t intToAddr(int input)
{
     if(input == 0) return 0b11111100;
     if(input == 1) return 0b01100000;
     if(input == 2) return 0b11011011;
     if(input == 3) return 0b11110011;
     if(input == 4) return 0b01100111;
     if(input == 5) return 0b10110111;
     if(input == 6) return 0b10111111;
     if(input == 7) return 0b11100000;
     if(input == 8) return 0b11111111;
     if(input == 9) return 0b11100111;
     else return 0b0;
}

// Parses an integer within the range of [0, 999999] into individual digits and displays on LCD
void LCDNum(int i)
{
    // If the number passed in is not within the range, output zero
    if(i > 999999 || i < 0)
    {
        i = 0;
    }

    // Calculate each individual digit
    int dig1 = i / 100000;
    int dig2 = (i % 100000) / 10000;
    int dig3 = ((i % 100000) % 10000) / 1000;
    int dig4 = (((i % 100000) % 10000) % 1000) / 100;
    int dig5 = ((((i % 100000) % 10000) % 1000) % 100) / 10;
    int dig6 = ((((i % 100000) % 10000) % 1000) % 100) % 10;

    // LCD Pin8-Pin9 for '1'
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_4, intToAddr(dig1));
    // LCD Pin12-Pin13 for '2'
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_6, intToAddr(dig2));
    // LCD Pin16-Pin17 for '3'
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_8, intToAddr(dig3));
    // LCD Pin20-Pin21 for '4'
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_10, intToAddr(dig4));
    // LCD Pin20-Pin21 for '5'
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_2, intToAddr(dig5));
    // LCD Pin20-Pin21 for '6'
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_18, intToAddr(dig6));
    // Turn on LCD
    LCD_E_on(LCD_E_BASE);
}

// Drives motor forwards or backwards
void drive(int dir)
{

    // y-axis
    if(axis)
    {
        // forwards
        if(dir)
        {
            if(!ymax)
            {
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1 + GPIO_PIN3);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1 + GPIO_PIN3);

//                stepsy++;
//                LCDNum(stepsy);

                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3 + GPIO_PIN4);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3 + GPIO_PIN4);

//                stepsy++;
//                LCDNum(stepsy);

                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4 + GPIO_PIN5);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4 + GPIO_PIN5);

//                stepsy++;
//                LCDNum(stepsy);

                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5 + GPIO_PIN1);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5 + GPIO_PIN1);

                stepsy++;
                LCDNum(stepsy);
            }
        }
        // backwards
        else
        {
            if(stepsy < 1)
            {
                stepsy = 1;
                ymin = 1;
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            }

            if(!ymin)
            {
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5 + GPIO_PIN1);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5 + GPIO_PIN1);

//                stepsy--;
//                LCDNum(stepsy);

                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4 + GPIO_PIN5);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4 + GPIO_PIN5);

//                stepsy--;
//                LCDNum(stepsy);

                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3 + GPIO_PIN4);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3 + GPIO_PIN4);

//                stepsy--;
//                LCDNum(stepsy);

                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1 + GPIO_PIN3);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1 + GPIO_PIN3);

                stepsy--;
                LCDNum(stepsy);
            }
        }
    }
    // x-axis
    else
    {
        // forwards
        if(dir)
        {
            if(!xmax)
            {
                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5 + GPIO_PIN7);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5 + GPIO_PIN7);

//                stepsx++;
//                LCDNum(stepsx);

                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
                GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);

//                stepsx++;
//                LCDNum(stepsx);

                GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2 + GPIO_PIN3);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2 + GPIO_PIN3);

//                stepsx++;
//                LCDNum(stepsx);

                GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);

                stepsx++;
                LCDNum(stepsx);
            }

        }
        // backwards
        else
        {
            if(stepsx < 1)
            {
                stepsx = 0;
                xmin = 1;
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            }

            if(!xmin)
            {
                GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);

//                stepsx--;
//                LCDNum(stepsx);

                GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2 + GPIO_PIN3);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2 + GPIO_PIN3);

//                stepsx--;
//                LCDNum(stepsx);

                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
                GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);

//                stepsx--;
//                LCDNum(stepsx);

                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5 + GPIO_PIN7);
                __delay_cycles(2500);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5 + GPIO_PIN7);

                stepsx--;
                LCDNum(stepsx);
            }
        }
    }
}

void photoInterrupter()
{

    //Initialize the ADC Module
    ADC_init(ADC_BASE, ADC_SAMPLEHOLDSOURCE_SC, ADC_CLOCKSOURCE_ADCOSC, ADC_CLOCKDIVIDER_1);
    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE, ADC_CYCLEHOLD_16_CYCLES, ADC_MULTIPLESAMPLESDISABLE);

    ADC_configureMemory(ADC_BASE, interrupterPin, ADC_VREFPOS_AVCC, ADC_VREFNEG_AVSS);
    ADC_clearInterrupt(ADC_BASE, ADC_BELOWTHRESHOLD_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE, ADC_BELOWTHRESHOLD_INTERRUPT);
    ADC_setWindowComp(ADC_BASE, 1023, 50);

    //Enable and Start the conversion
    ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);

}

void changeDir()
{
    int i = 0;
//    LCDNum(88888);
    axis = !axis;
    for(i = 0; i < 500; i++){}
    loop();
}

void loop()
{
    // If we enter the loop and one of the buttons is already pressed, we just wait that out without doing anything
    while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN2) == GPIO_INPUT_PIN_LOW || GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == GPIO_INPUT_PIN_LOW)
    {
        // Debouncing!!
        __delay_cycles(100000);
    }

    // The main loop that repeats, constantly polls for the buttons
    while(1)
    {
        ymax = 0;
        ymin = 0;
        xmax = 0;
        xmin = 0;

        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

//        LCDNum(9999);
        while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN2) == GPIO_INPUT_PIN_LOW)
        {
            // Set which interrupter we're checking
            if(axis)
            {
                interrupterPin = 6;
            }
            else
            {
                interrupterPin = 8;
            }
            //Enable and Start the conversion
            photoInterrupter();
            drive(1);
            if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == GPIO_INPUT_PIN_LOW)
            {
                changeDir();
            }
        }
        while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == GPIO_INPUT_PIN_LOW)
        {
            // Set which interrupter we're checking
            if(axis)
            {
                interrupterPin = 7;
            }
            else
            {
                interrupterPin = 9;
            }
            //Enable and Start the conversion
            photoInterrupter();
            drive(0);
            if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN2) == GPIO_INPUT_PIN_LOW)
            {
                changeDir();
            }
        }
    }
}

void main (void)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    // ------------------------------------------------------------------------------------------------------------------------------
    // LCD
    // ------------------------------------------------------------------------------------------------------------------------------

    //Port select XT1
        GPIO_setAsPeripheralModuleFunctionInputPin
        (
            GPIO_PORT_P4,
            GPIO_PIN1 + GPIO_PIN2,
            GPIO_PRIMARY_MODULE_FUNCTION
        );

        //Set external frequency for XT1
        CS_setExternalClockSource(32768);

        //Select XT1 as the clock source for ACLK with no frequency divider
        CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);

        //Start XT1 with no time out
        CS_turnOnXT1(CS_XT1_DRIVE_0);

        //clear all OSC fault flag
        CS_clearAllOscFlagsWithTimeout(1000);

        //Set P1.6, P1.7, P8.0, P8.1 as analog input pins A6, A7, A8, A9
        GPIO_setAsPeripheralModuleFunctionInputPin
        (
                GPIO_PORT_P1,
                GPIO_PIN6 + GPIO_PIN7,
                GPIO_TERNARY_MODULE_FUNCTION
        );
        GPIO_setAsPeripheralModuleFunctionInputPin
        (
                GPIO_PORT_P8,
                GPIO_PIN0 + GPIO_PIN1,
                GPIO_TERNARY_MODULE_FUNCTION
        );

        // L0~L26 & L36~L39 pins selected
        LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_0, LCD_E_SEGMENT_LINE_26);
        LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_36, LCD_E_SEGMENT_LINE_39);

        LCD_E_initParam initParams = LCD_E_INIT_PARAM;
        initParams.clockDivider = LCD_E_CLOCKDIVIDER_8;
        initParams.muxRate = LCD_E_4_MUX;
        initParams.segments = LCD_E_SEGMENTS_ENABLED;

        // Init LCD as 4-mux mode
        LCD_E_init(LCD_E_BASE, &initParams);

        // LCD Operation - Mode 3, internal 3.08v, charge pump 256Hz
        LCD_E_setVLCDSource(LCD_E_BASE, LCD_E_INTERNAL_REFERENCE_VOLTAGE, LCD_E_EXTERNAL_SUPPLY_VOLTAGE);
        LCD_E_setVLCDVoltage(LCD_E_BASE, LCD_E_REFERENCE_VOLTAGE_3_08V);

        LCD_E_enableChargePump(LCD_E_BASE);
        LCD_E_setChargePumpFreq(LCD_E_BASE, LCD_E_CHARGEPUMP_FREQ_16);

        // Clear LCD memory
        LCD_E_clearAllMemory(LCD_E_BASE);

        // Configure COMs and SEGs
        // L0, L1, L2, L3: COM pins
        // L0 = COM0, L1 = COM1, L2 = COM2, L3 = COM3
        LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_0, LCD_E_MEMORY_COM0);
        LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_1, LCD_E_MEMORY_COM1);
        LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_2, LCD_E_MEMORY_COM2);
        LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_3, LCD_E_MEMORY_COM3);



    // ------------------------------------------------------------------------------------------------------------------------------
    // Buttons
    // ------------------------------------------------------------------------------------------------------------------------------

    //Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    //Set LED1 to output direction
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    //Enable P1.2 internal resistance as pull-up resistance and set as input
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
    //Similarly, enable P2.6 internal resistance as pull-up resistance and set as input
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN6);

    // Unlock LPM5
    PMM_unlockLPM5();



    // ------------------------------------------------------------------------------------------------------------------------------
    // Motor Drivers
    // ------------------------------------------------------------------------------------------------------------------------------

    // Driver 1: Y-Axis
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN1 + GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5 + GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2 + GPIO_PIN3);

    // Driver 2: X-Axis
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1 + GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5 + GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2 + GPIO_PIN3);

    // ------------------------------------------------------------------------------------------------------------------------------
    // ADC
    // -------------------------------------------------------------------------------------------------

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input A6
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */

    __bis_SR_register(GIE);

    loop();
}

#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
{
    if(interrupterPin == 6)
    {
        ymax = 1;
    }
    else if(interrupterPin == 7)
    {
        ymin = 1;
    }
    else if(interrupterPin == 8)
    {
        xmax = 1;
    }
    else if(interrupterPin == 9)
    {
        xmin = 1;
    }
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    ADC_clearInterrupt(ADC_BASE, ADC_BELOWTHRESHOLD_INTERRUPT);
}
