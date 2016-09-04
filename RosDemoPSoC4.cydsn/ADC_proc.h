/* 
 * Analog to Digital conversion for HiveBio Ministat
 * 
 * Copyright C. Harrison
 * BSD 2-clause license http://opensource.org/licenses/BSD-2-Clause
 *
 */


#ifndef ADC_PROC_H
#define ADC_PROC_H

namespace ADC {

extern volatile int32 adc_DC[];
extern volatile int32 adc_60HzI[];
extern volatile int32 adc_60HzQ[];

void setup();
void loop();

} // namespace

#endif
