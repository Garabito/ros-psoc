/* 
 * Analog to Digital conversion for RosDemoPSoC4, with synchronous detection
 *
 * This configuration supports 10 input channels using 8-input SARMUX AD converter
 *  7 channels are sampled directly at full rate and 3 more are submultiplexed
 *
 * System is configured for synchronous detection with ~60Hz carrier frequency
 *   and reports DC, In-phase, and Quadrature components as 16-bit signed values
 *   LSB of reported value is ~ 0.125mV, (ADC LSB)/4, 
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

#define CONVERSION_CHANS ADC_TOTAL_CHANNELS_NUM
#define MEASURE_CHANS (CONVERSION_CHANS + Slow_Mux_CHANNELS - 1)
namespace ADC {

uint32_t next_report_time;
const uint32_t kReportIntervalMs = 250;

std_msgs::Int16MultiArray adc_msg;
std_msgs::MultiArrayDimension adc_msg_dim[2];
int16_t adc_data[3][MEASURE_CHANS];
ros::Publisher p("adc", &adc_msg);

int16_t* const& adc_DC = adc_data[0];
int16_t* const& adc_ACI = adc_data[1];
int16_t* const& adc_ACQ = adc_data[2];
int32 accum_DC[MEASURE_CHANS];
int32 accum_ACI[MEASURE_CHANS];
int32 accum_ACQ[MEASURE_CHANS];
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
			if (IQphase == 0x00) {
				IQphase = 0x01;
				Carrier_out_Write(1);
			}
			else if (IQphase == 0x01) {
				IQphase = 0x11;
			}
			else if (IQphase == 0x11) {
				IQphase = 0x10;
				Carrier_out_Write(0);
			}
			else {
				IQphase = 0x00;
			}
        }
        unsigned int chan;
        for (chan=0; chan<CONVERSION_CHANS; ++chan)
        {
			uint16 reading = (uint16)ADC_GetResult16(chan);
			/* handle slow_max channels */
			uint8_t is_slow_chan = (chan == CONVERSION_CHANS-1);
			if (is_slow_chan) {
				chan += Slow_Mux_GetChannel();
			}
            /* save accumulated reading if ready*/
            if(accum_count==0 && IQphase==0x00) {
                    adc_DC[chan] = accum_DC[chan]/NUM_ACCUMS;
                    accum_DC[chan] = 0;
				    adc_ACI[chan] = accum_ACI[chan]/NUM_ACCUMS;
					accum_ACI[chan] = 0;
					adc_ACQ[chan] = accum_ACQ[chan]/NUM_ACCUMS;
					accum_ACQ[chan] = 0;
					if (is_slow_chan) {
						Slow_Mux_Next();
					}
            }
			/* accumulate this reading */
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
	/* set up sub-mux */
	Mux_Buffer_Start();
	Slow_Mux_Start();
	Slow_Mux_Next();
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
	adc_msg.data_length = 3*MEASURE_CHANS;
	adc_msg.layout.dim_length = 2;
	adc_msg.layout.data_offset = 0;
	adc_msg.layout.dim = adc_msg_dim;
	// outermost dimension is signal type
	adc_msg.layout.dim[0].label = "DC ACI ACQ";
	adc_msg.layout.dim[0].size = 3;
	adc_msg.layout.dim[0].stride = 3*MEASURE_CHANS;
	// innermost dimension is channel count
	adc_msg.layout.dim[1].label = "chan";
	adc_msg.layout.dim[1].size = MEASURE_CHANS;
	adc_msg.layout.dim[1].stride = MEASURE_CHANS;

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

