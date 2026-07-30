/* The following program is designed to implement a four-state LED control system using bare-metal Embedded C and direct memory-mapped register access. 
A valid push-button press changes the operating mode in the following order: Running LED → Binary Counter → Alternating LEDs → Flashing LEDs.
while the second push button moves backward through the same modes. */

#include <reg51.h>

// Switch definitions
// SWITCH_I : Output line used while scanning the push button
// SWITCH_O : Input line used to detect button press
sbit SWITCH_I = P2^7;
sbit SWITCH_O = P2^3;

// Variable used to store the current operating mode
// -1 : Initial state
//  0 : Running LED
//  1 : Binary Counter
//  2 : Alternating LEDs
//  3 : Flashing LEDs
int status = -1;


// Delay function used to control LED transition speed
void delayms(unsigned int duration)
{
    unsigned int i, j;

    for(i = 0; i < duration; i++)
        for(j = 0; j < 350; j++);
}


// Push-button detection with software debouncing
// Each valid button press advances the system to the next mode
int ifPressed()
{
    SWITCH_I = 0;        //as the button is pressed

    if(SWITCH_O == 0)
    {
        delayms(20);              // Debounce delay

        if(SWITCH_O == 0)  //if the button is still pressed
        {
            status++;     //increase the status to change to next mode

            // Return to Mode 1 after Mode 4
            if(status > 3)
                status = 0;

            // Wait until button is released
            while(SWITCH_O == 0);

            delayms(20);

            SWITCH_I = 1;     //reinitialize it to its initial state - i.e 1

            return 1;
        }
    }

    SWITCH_I = 1;      //reinitialize outside the if as well

    return 0;
}


void main()
{
    unsigned char count;
    unsigned char value;

    // Initially turn OFF all LEDs
    P1 = 0x00;

    while(1)    //infinite loop
    {
        switch(status)
        {

        // Initial state
        // Wait for the first valid button press
        case -1:

            P1 = 0x00;
            ifPressed();

            break;


        // Mode 1 - Running LED
        // LEDs glow one after another continuously
        case 0:

            P1 = 0x01;

            for(count=0; count<50; count++)
            {
                delayms(10);
                if(ifPressed()) break;
            }

            if(status != 0) break;

            P1 = 0x02;

            for(count=0; count<50; count++)
            {
                delayms(10);
                if(ifPressed()) break;
            }

            if(status != 0) break;

            P1 = 0x04;

            for(count=0; count<50; count++)     //for loop 50 times 
            {
                delayms(10);
                if(ifPressed()) break;
            }

            if(status != 0) break;

            P1 = 0x08;

            for(count=0; count<50; count++)
            {
                delayms(10);
                if(ifPressed()) break;
            }

            break;


        // Mode 2 - Binary Counter
        // LEDs display binary values from 0000 to 1111
        case 1:

            for(value=0; value<16; value++)
            {
                P1 = value;        //value will change it to hexa automatically

                for(count=0; count<50; count++)
                {
                    delayms(10);

                    if(ifPressed())
                        break;
                }

                if(status != 1)
                    break;
            }

            break;


        // Mode 3 - Alternating LED Pattern
        // LEDs alternate between 1010 and 0101
        case 2:

            P1 = 0x0A;

            for(count=0; count<50; count++)
            {
                delayms(10);

                if(ifPressed())
                    break;
            }

            if(status != 2)
                break;

            P1 = 0x05;

            for(count=0; count<50; count++)
            {
                delayms(10);

                if(ifPressed())
                    break;
            }

            break;


        // Mode 4 - Flashing LEDs
        // All LEDs turn ON and OFF repeatedly
        case 3:

            P1 = 0x0F;

            for(count=0; count<50; count++)
            {
                delayms(10);

                if(ifPressed())
                    break;
            }

            if(status != 3)
                break;

            P1 = 0x00;

            for(count=0; count<50; count++)
            {
                delayms(10);

                if(ifPressed())
                    break;
            }

            break;


        // Safety case
        // Reset to initial state if status becomes invalid
        default:

            status = -1;

            break;
        }
    }
}
