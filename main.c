/*
\file   main.c

\brief  Main source file.

(c) 2018 Microchip Technology Inc. and its subsidiaries.

Subject to your compliance with these terms, you may use Microchip software and any
derivatives exclusively with Microchip products. It is your responsibility to comply with third party
license terms applicable to your use of third party software (including open source software) that
may accompany Microchip software.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
FOR A PARTICULAR PURPOSE.

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
SOFTWARE.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mcc_generated_files/application_manager.h"
#include "mcc_generated_files/led.h"
#include "mcc_generated_files/sensors_handling.h"
#include "mcc_generated_files/cloud/cloud_service.h"
#include "mcc_generated_files/debug_print.h"
#include <time.h>
#include "mcc_generated_files/sensors_handling.h"

//This handles messages published from the MQTT server when subscribed
void receivedFromCloud(uint8_t *topic, uint8_t *payload)
{
	LED_flashRed();
	debug_printer(SEVERITY_NONE, LEVEL_NORMAL, "topic: %s", topic);
	debug_printer(SEVERITY_NONE, LEVEL_NORMAL, "payload: %s", payload);
}

// This will get called every 1 second only while we have a valid Cloud connection
void sendToCloud(void)
{
   static char json[200];
   sensor_data dataset = {0,};
   
   // This part runs every CFG_SEND_INTERVAL seconds
   int rawTemperature = SENSORS_getTempValue();
   int light = SENSORS_getLightValue();
   
   // Read the Ultra-Violet, Visible and Near Infrared spectrum data from sparkfun triad
   read_calib_values(&dataset);

   // Add the timestamp to the JSON
   time_t now = time(NULL) + UNIX_OFFSET;

   // Prepare a buffer containing the spectral data to be passed to the cloud
   int len = sprintf(json, 
                      "{\"Timestamp\":%lu,\"Light\":%d,\"Temperature\":\"%d.%02d\","
                      "\"UV0\":\"%d.%02d\",\"UV1\":\"%d.%02d\",\"UV2\":\"%d.%02d\",\"UV3\":\"%d.%02d\",\"UV4\":\"%d.%02d\",\"UV5\":\"%d.%02d\","
                      "\"VS0\":\"%d.%02d\",\"VS1\":\"%d.%02d\",\"VS2\":\"%d.%02d\",\"VS3\":\"%d.%02d\",\"VS4\":\"%d.%02d\",\"VS5\":\"%d.%02d\","
                      "\"IR0\":\"%d.%02d\",\"IR1\":\"%d.%02d\",\"IR2\":\"%d.%02d\",\"IR3\":\"%d.%02d\",\"IR4\":\"%d.%02d\",\"IR5\":\"%d.%02d\"}",
                      now, light, rawTemperature/100,abs(rawTemperature)%100, 
                      (int)dataset.uv[0], abs(dataset.uv[0])%100, (int)dataset.uv[1], abs(dataset.uv[1])%100, (int)dataset.uv[2], abs(dataset.uv[2])%100, 
                      (int)dataset.uv[3], abs(dataset.uv[3])%100, (int)dataset.uv[4], abs(dataset.uv[4])%100, (int)dataset.uv[5], abs(dataset.uv[5])%100, 
                      (int)dataset.vs[0], abs(dataset.vs[0])%100, (int)dataset.vs[1], abs(dataset.vs[1])%100, (int)dataset.vs[2], abs(dataset.vs[2])%100, 
                      (int)dataset.vs[3], abs(dataset.vs[3])%100, (int)dataset.vs[4], abs(dataset.vs[4])%100, (int)dataset.vs[5], abs(dataset.vs[5])%100, 
                      (int)dataset.ir[0], abs(dataset.ir[0])%100, (int)dataset.ir[1], abs(dataset.ir[1])%100, (int)dataset.ir[2], abs(dataset.ir[2])%100, 
                      (int)dataset.ir[3], abs(dataset.ir[3])%100, (int)dataset.ir[4], abs(dataset.ir[4])%100, (int)dataset.ir[5], abs(dataset.ir[5])%100); 
    
   if (len >0) {
      CLOUD_publishData((uint8_t*)json, len);
   }
   
   LED_flashYellow();
}
 
int main(void)
{
   application_init();

   while (1)
   { 
      runScheduler();  
   }
   
   return 0;
}
