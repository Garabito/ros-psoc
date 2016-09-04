/* 
 * Analog to Digital conversion for RosDemoPSoC4, with synchronous detection
 * 
 * Copyright C. Harrison
 * BSD 2-clause license http://opensource.org/licenses/BSD-2-Clause
 *
 */
extern "C" {
	#include <project.h>
}
#include <ros.h>
#include <std_msgs/Int32.h>

#include "ADC_proc.h"

extern ros::NodeHandle nh;

namespace ADC {

uint32_t next_report_time;
const uint32_t kReportIntervalMs = 250;
std_msgs::Int32 adc_msg;
ros::Publisher p0DC("adc0DC", &adc_msg);
ros::Publisher p0ACI("adc0ACI", &adc_msg);
ros::Publisher p0ACQ("adc0ACQ", &adc_msg);


volatile int32 adc_DC[ADC_TOTAL_CHANNELS_NUM];
volatile int32 adc_60HzI[ADC_TOTAL_CHANNELS_NUM];
volatile int32 adc_60HzQ[ADC_TOTAL_CHANNELS_NUM];
int32 accum_DC[ADC_TOTAL_CHANNELS_NUM];
int32 accum_60HzI[ADC_TOTAL_CHANNELS_NUM];
int32 accum_60HzQ[ADC_TOTAL_CHANNELS_NUM];
int16 accum_count;
uint8 IQphase;
#define SAMPLE_RATE 3840
#define NUM_ACCUMS (SAMPLE_RATE/60/4)

CY_ISR(ADC_ISR_LOC)
{
    uint32 intr_status;

    /* Read interrupt status registers */
    intr_status = ADC_SAR_INTR_MASKED_REG;
    /* Check for End of Scan interrupt */
    if((intr_status & ADC_EOS_MASK) != 0u)
    {
        if(++accum_count >= NUM_ACCUMS) {
            accum_count = 0;
			if (IQphase == 0x00) { IQphase = 0x01; }
			else if (IQphase == 0x01) { IQphase = 0x11; }
			else if (IQphase == 0x11) { IQphase = 0x10; }
			else { IQphase = 0x00; }
        }
        unsigned int chan;
        for (chan=0; chan<ADC_TOTAL_CHANNELS_NUM; ++chan)
        {
            /* save accumulated reading if ready*/
            if(accum_count==0 && IQphase==0x00) {
                    adc_DC[chan] = accum_DC[chan]/NUM_ACCUMS;
                    accum_DC[chan] = 0;
				    adc_60HzI[chan] = accum_60HzI[chan]/NUM_ACCUMS;
					accum_60HzI[chan] = 0;
					adc_60HzQ[chan] = accum_60HzQ[chan]/NUM_ACCUMS;
					accum_60HzQ[chan] = 0;
            }
			/* accumulate this reading */
			uint16 reading = (uint16)ADC_GetResult16(chan);
            accum_DC[chan] += reading;
			switch (IQphase) {
				case 0x00:
					accum_60HzI[chan] += reading;
					accum_60HzQ[chan] += reading;
					break;
				case 0x01:
                    accum_60HzI[chan] += reading;
                    accum_60HzQ[chan] -= reading;
					break;
				case 0x11:
                    accum_60HzI[chan] -= reading;
                    accum_60HzQ[chan] -= reading;
					break;
				case 0x10:
                    accum_60HzI[chan] -= reading;
                    accum_60HzQ[chan] += reading;
					break;
				default:
					IQphase = 0x00;
			}
        } 
    }    

    /* Clear handled interrupt */
    ADC_SAR_INTR_REG = intr_status;
}

void adc_setup() {
	IQphase = 0x00;
    /* Init and start sequencing SAR ADC */
    ADC_Start();
    ADC_StartConvert();
    /* Enable interrupt and set interrupt handler to local routine */
    ADC_IRQ_StartEx(ADC_ISR_LOC);
}

void setup()
{ 
  nh.advertise(p0DC);
  nh.advertise(p0ACI);
  nh.advertise(p0ACQ);
  next_report_time = millis();
  adc_setup();
}

void loop()
{
  if ((int32_t)(millis()-next_report_time) > 0) {
    next_report_time += kReportIntervalMs;

    adc_msg.data = adc_DC[0];
    p0DC.publish(&adc_msg);
    adc_msg.data = adc_60HzI[0];
    p0ACI.publish(&adc_msg);
    adc_msg.data = adc_60HzQ[0];
    p0ACQ.publish(&adc_msg);
  }
}

} //namespace

