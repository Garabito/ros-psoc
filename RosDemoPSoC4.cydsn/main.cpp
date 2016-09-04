/* 
 * RosDemoPSoC4 - rosserial on embedded Cypress PSoC4
 * 
 * Copyright C. Harrison
 * BSD 2-clause license http://opensource.org/licenses/BSD-2-Clause
 *
 */
extern "C" {
#include <project.h>
#include "SysTimer.h"
}

#include <ros.h>
#include "rosserial/rosserial_psoc4/src/ros_lib/examples/HelloWorld/HelloWorld.h"
#include "rosserial/rosserial_psoc4/src/ros_lib/examples/Blink/Blink.h"
#include "ADC_sync_det.h"

//extern void init(void);

ros::NodeHandle  nh;


int main()
{
    systimer_setup(); // sets up millisec() hardware timer
    CyGlobalIntEnable; // Enable interrupts
	
    uint32 reboot_timer = millis();
    uint8 reboot_count =0;
    nh.initNode();
    HelloWorld::setup();
    Blink::setup();
	ADC::setup();

    for(;;)
    {
        HelloWorld::loop();
        Blink::loop();
		ADC::loop();
 
        // watch for pushbutton
        //   Held down 1-2 sec = reboot request
        if ((int32)(millis()-reboot_timer) >= 0) {
            if(Button_Read()) {
                reboot_count = 0;
            } else {
                if(++reboot_count > 1) {
                    Bootloadable_Load();
                }
            }
            reboot_timer += 1000;
        }

        nh.spinOnce();
    }
}

/* [] END OF FILE */
