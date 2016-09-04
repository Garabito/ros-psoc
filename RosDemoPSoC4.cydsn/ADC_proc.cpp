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
#include <std_msgs/Int16MultiArray.h>
#include "ADC_proc.h"

extern ros::NodeHandle nh;

namespace ADC {

uint32_t next_report_time;
const uint32_t kReportIntervalMs = 250;

std_msgs::Int16MultiArray adc_msg;
std_msgs::MultiArrayDimension adc_msg_dim[2];
int16_t adc_data[3][ADC_TOTAL_CHANNELS_NUM];
ros::Publisher p("adc", &adc_msg);

int16_t* const& adc_DC = adc_data[0];
int16_t* const& adc_ACI = adc_data[1];
int16_t* const& adc_ACQ = adc_data[2];
int32 accum_DC[ADC_TOTAL_CHANNELS_NUM];
int32 accum_ACI[ADC_TOTAL_CHANNELS_NUM];
int32 accum_ACQ[ADC_TOTAL_CHANNELS_NUM];
int16 accum_count;
uint8 IQphase;
#define SAMPLE_RATE 3840
#define CARRIER_HZ 60
#define NUM_ACCUMS (SAMPLE_RATE/CARRIER_HZ/4)

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
				    adc_ACI[chan] = accum_ACI[chan]/NUM_ACCUMS;
					accum_ACI[chan] = 0;
					adc_ACQ[chan] = accum_ACQ[chan]/NUM_ACCUMS;
					accum_ACQ[chan] = 0;
            }
			/* accumulate this reading */
			uint16 reading = (uint16)ADC_GetResult16(chan);
            accum_DC[chan] += reading;
			if (IQphase & 0x01) {
				accum_ACI[chan] += reading;
			} else {
				accum_ACI[chan] -= reading;
			}
			if (IQphase & 0x10) {
				accum_ACQ[chan] += reading;
			} else {
				accum_ACQ[chan] -= reading;
			}
        } 
    }    

    /* Clear handled interrupt */
    ADC_SAR_INTR_REG = intr_status;
}

void adc_setup() {
    /* Init and start sequencing SAR ADC */
    ADC_Start();
    ADC_StartConvert();
    /* Enable interrupt and set interrupt handler to local routine */
    ADC_IRQ_StartEx(ADC_ISR_LOC);
}

void setup()
{ 
	nh.advertise(p);
	next_report_time = millis()+kReportIntervalMs;
	
	// initialize 2-dimensional array message for ADC data
	adc_msg.data = (int16_t*)adc_data;
	adc_msg.data_length = 3*ADC_TOTAL_CHANNELS_NUM;
	adc_msg.layout.dim_length = 2;
	adc_msg.layout.data_offset = 0;
	adc_msg.layout.dim = adc_msg_dim;
	// outermost dimension is signal type
	adc_msg.layout.dim[0].label = "DC ACI ACQ";
	adc_msg.layout.dim[0].size = 3;
	adc_msg.layout.dim[0].stride = 3*ADC_TOTAL_CHANNELS_NUM;
	// innermost dimension is channel count
	adc_msg.layout.dim[1].label = "chan";
	adc_msg.layout.dim[1].size = ADC_TOTAL_CHANNELS_NUM;
	adc_msg.layout.dim[1].stride = ADC_TOTAL_CHANNELS_NUM;

	adc_setup();
}

void loop()
{
  if ((int32_t)(millis()-next_report_time) > 0) {
    next_report_time += kReportIntervalMs;
    p.publish(&adc_msg);
  }
}

} //namespace

