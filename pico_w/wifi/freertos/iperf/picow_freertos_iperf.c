/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "tcp_debug.h"
#include "head_tail.h"
#include "mqtt_extra.h"

 
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include <string.h>
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/apps/lwiperf.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#ifndef RUN_FREERTOS_ON_CORE
	#define RUN_FREERTOS_ON_CORE 0
#endif
char remotes[6][8]={"remote1","remote2","remote3","remote4","remote5","remote6"};
 
int rr[6];

#define BATT_TASK_PRIORITY				( tskIDLE_PRIORITY + 11UL )
#define CLOSE_TASK_PRIORITY				( tskIDLE_PRIORITY + 10UL )
#define OPEN_TASK_PRIORITY				( tskIDLE_PRIORITY + 9UL )
#define ADC_TASK_PRIORITY				( tskIDLE_PRIORITY + 8UL )
//#define NTP_TASK_PRIORITY				( tskIDLE_PRIORITY + 5UL )
#define WATCHDOG_TASK_PRIORITY			( tskIDLE_PRIORITY + 1UL )
//#define MQTT_TASK_PRIORITY				( tskIDLE_PRIORITY + 4UL )  
#define SOCKET_TASK_PRIORITY			( tskIDLE_PRIORITY + 6UL )
#define TEST_TASK_PRIORITY				( tskIDLE_PRIORITY + 2UL )
#define BLINK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3UL )

#include "lwip/apps/mqtt.h"
#include "mqtt_example.h"
#include "pico/util/datetime.h"
u8_t lp;
u8_t alarm_hour;
u8_t alarm_min;
u8_t alarm_sec;
u8_t remote_index;
u8_t cmd;
char houralarm[3];
char minalarm[3];
char secalarm[3];
char tmp[80];

char * ptrhead;
char * ptrtail;
char * ptrendofbuf;
char * ptrtopofbuf;
char client_message[BUF_SIZE]; 
//mqtt_request_cb_t pub_mqtt_request_cb_t; 

u16_t mqtt_port = 1883;

char PUB_PAYLOAD_T[]="                                       ";
char PUB_PAYLOAD_SCR_T[]="                                       ";
char PUB_EXTRA_ARG_T[] = "test";
u16_t payload_t_size;



/* Choose 'C' for Celsius or 'F' for Fahrenheit. */
//#define TEMPERATURE_UNITS 'F'
char TEMPERATURE_UNITS;
char  unit;
float retflg;
uint32_t result;
const float conversion_factor = 3.3f / (1 << 12);
float battery;
typedef struct NTP_T_ {
    ip_addr_t ntp_server_address;
    bool dns_request_sent;
    struct udp_pcb *ntp_pcb;
    absolute_time_t ntp_test_time;
    alarm_id_t ntp_resend_alarm;
} NTP_T;

#define NTP_SERVER "pool.ntp.org"
#define NTP_MSG_LEN 48
#define NTP_PORT 123
#define NTP_DELTA 2208988800 // seconds between 1 Jan 1900 and 1 Jan 1970
#define NTP_TEST_TIME (30 * 1000)
#define NTP_RESEND_TIME (10 * 1000)
#define in1 14
#define in2 15
#define enA 16
/*needed for rtc */
/*
datetime_t t;
datetime_t alarm;
datetime_t t_ntp;
datetime_t *pt;
datetime_t *palarm;
datetime_t *pt_ntp;
*/
u8_t rtc_set_flag = 0;
char datetime_buf[256];
char *datetime_str = &datetime_buf[0];
/*needed for rtc */
/*needed for ntp*/
/*needed for GPIO from pico-examples/gpio/hello_7segment/hello_7segment.c
gpio will be an additional freertos task
*/
#define FIRST_GPIO 18
#define BUTTON_GPIO (FIRST_GPIO+7)

uint tbits25;
char bits25[2];
u8_t reset_remote=0;
int val = 0;
int loop;
// This array converts a number 0-9 to a bit pattern to send to the GPIOs
int bits[10] = {
        0x3f,  // 0
        0x3e,  // 1
        0x3d,  // 2
        0x3c,  // 3
        0x3b,  // 4
        0x3a,  // 5
        0x39,  // 6
        0x38,  // 7
        0x37,  // 8
        0x36   // 9
};
// This array converts a number 0-9 to a bit pattern to send to the GPIOs
int32_t mask;

u8_t lp;
u8_t alarm_hour;
u8_t alarm_min;
u8_t alarm_sec;
u8_t remote_index;
u8_t cmd;
char houralarm[3];
char minalarm[3];
char secalarm[3];
char tmp[80];
char * ptrhead;
char * ptrtail;
char * ptrendofbuf;
char * ptrtopofbuf;
char client_message[BUF_SIZE];

/*needed for rtc 
datetime_t t;*/
#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif
 
#include "lwip/apps/mqtt.h"
#include "mqtt_example.h"
#include "pico/util/datetime.h"
static volatile bool fired = false;

u8_t alarm_flg=0;
u8_t open_flg=0;
u8_t close_flg=0;
#if CLIENT_TEST && !defined(IPERF_SERVER_IP)
#error IPERF_SERVER_IP not defined
#endif

// Report IP results and exit
static void iperf_report(void *arg, enum lwiperf_report_type report_type,
                         const ip_addr_t *local_addr, u16_t local_port, const ip_addr_t *remote_addr, u16_t remote_port,
                         u32_t bytes_transferred, u32_t ms_duration, u32_t bandwidth_kbitpsec) {
    static uint32_t total_iperf_megabytes = 0;
    uint32_t mbytes = bytes_transferred / 1024 / 1024;
    float mbits = bandwidth_kbitpsec / 1000.0;

    total_iperf_megabytes += mbytes;

    printf("Completed iperf transfer of %d MBytes @ %.1f Mbits/sec\n", mbytes, mbits);
    printf("Total iperf megabytes since start %d Mbytes\n", total_iperf_megabytes);
}

/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */
float read_onboard_temperature(char unit) {
    
    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    if (unit == 'C') {
        return tempC;
    } else if (unit == 'F') {
        return tempC * 9 / 5 + 32;
    }

    return -50.0f;
}
/*
static void alarm_callback(void) {
    datetime_t t = {0};
    rtc_get_datetime(&t);
    char datetime_buf[256];
    char *datetime_str = &datetime_buf[0];
    datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
    printf("Alarm Fired At %s\n", datetime_str);
    sprintf(tmp,"Alarm Fired %s ",datetime_str);
    ptrhead = head_tail_helper(ptrhead, ptrtail, ptrendofbuf, ptrtopofbuf, tmp);
    //printf("client_message %s\n",client_message);
    stdio_flush();
    fired = true;
    alarm_flg=1;
}
*/

void adc_task(__unused void *params) {
    //bool on = false;
    adc_init();
 
    
    while (true) {
	adc_set_temp_sensor_enabled(true);
    	adc_select_input(4); 
	float temperature = read_onboard_temperature(TEMPERATURE_UNITS);
     
    
        if (temperature != -50.0) {
            //printf("Onboard temperature = %.02f %c\n", temperature, TEMPERATURE_UNITS);

            sprintf(PUB_PAYLOAD_SCR_T,"T = %.02f %c 0x%03x batt = %.02f %s",temperature, TEMPERATURE_UNITS,result,battery, CYW43_HOST_NAME);
            payload_t_size = sizeof(PUB_PAYLOAD_SCR_T);
             
        }
 
       vTaskDelay(22000);
    }

}

void batt_task(__unused void *params) {
    //bool on = false;
    //adc_init();
     
    //adc_select_input(0);
    
    while (true) {   
       adc_select_input(0);
       result = adc_read();
       battery = result * conversion_factor; 	
       sprintf(tmp,"batt task 0x%03x -> %f V",result, battery);
       ptrhead = head_tail_helper(ptrhead, ptrtail, ptrendofbuf, ptrtopofbuf, tmp);	
	 
 
       vTaskDelay(22000);
    }

}
void watchdog_task(__unused void *params) {
    //bool on = false;

    while (true) {
	 
	//if (wifi_connected == 0) watchdog_update();
	watchdog_update();
    
 
       vTaskDelay(100);
    }
}
/*needed for ntp*/
/*
void ntp_task(__unused void *params) {
    //bool on = false;
    //printf("ntp_task starts\n");
	run_ntp_test();
    while (true) {
#if 0 && configNUM_CORES > 1
        static int last_core_id;
        if (portGET_CORE_ID() != last_core_id) {
            last_core_id = portGET_CORE_ID();
            printf("ntp now from core %d\n", last_core_id);
        }
#endif
        //cyw43_arch_gpio_put(0, on);
        //on = !on;
        
        vTaskDelay(2200);
    }
}
*/
void socket_task(__unused void *params) {
	
	//printf("socket_task starts\n");
    TCP_SERVER_T *state = tcp_server_init();
    if (!state) {
        return;
    }
    if (!tcp_server_open(state)) {
        tcp_server_result(state, -1);
        return;
    }
    

    while (true) {
		if(state->complete) {
			free(state);
			sleep_ms(1000);
			TCP_SERVER_T *state = tcp_server_init();
    		if (!state) {
        		return;
    		}
    		if (!tcp_server_open(state)) {
        		tcp_server_result(state, -1);
        		return;
         	}
		}

        vTaskDelay(200);
		
    }
}

void blink_task(__unused void *params) {
    bool on = false;
    printf("blink_task starts\n");
    while (true) {
#if 0 && configNUMBER_OF_CORES > 1
        static int last_core_id;
        if (portGET_CORE_ID() != last_core_id) {
            last_core_id = portGET_CORE_ID();
            printf("blinking now from core %d\n", last_core_id);
        }
#endif
        cyw43_arch_gpio_put(0, on);
        on = !on;
        vTaskDelay(200);
    }
}

void main_task(__unused void *params) {
    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        exit(1);
    } else {
        printf("Connected.\n");
    }
    xTaskCreate(socket_task, "SOCKETThread", configMINIMAL_STACK_SIZE, NULL, SOCKET_TASK_PRIORITY, NULL);
    xTaskCreate(blink_task, "BlinkThread", configMINIMAL_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
    xTaskCreate(adc_task, "ADCThread", configMINIMAL_STACK_SIZE, NULL, ADC_TASK_PRIORITY, NULL);
    //xTaskCreate(batt_task, "BATTThread", configMINIMAL_STACK_SIZE, NULL, BATT_TASK_PRIORITY, NULL);

    cyw43_arch_lwip_begin();
#if CLIENT_TEST
    printf("\nReady, running iperf client\n");
    ip_addr_t clientaddr;
    ip4_addr_set_u32(&clientaddr, ipaddr_addr(xstr(IPERF_SERVER_IP)));
    assert(lwiperf_start_tcp_client_default(&clientaddr, &iperf_report, NULL) != NULL);
#else
    printf("\nReady, running iperf server at %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    lwiperf_start_tcp_server_default(&iperf_report, NULL);
#endif
    cyw43_arch_lwip_end();

    while(true) {
        // not much to do as LED is in another task, and we're using RAW (callback) lwIP API
        vTaskDelay(10000);
    }

    cyw43_arch_deinit();
}

void preptopidata() {
sprintf(client_message,"0123456789012345678901234567890123456789012345678901234567890123\
0123456789012345678901234567890123456789012345678901234567890123\
0123456789012345678901234567890123456789012345678901234567890123\
0123456789012345678901234567890123456789012345678901234567890123\
0123456789012345678901234567890123456789012345678901234567890123\
012345678901234567890123456789012345678901234567890123456789012301234567890123456789012345\
6789012345678901234567890123456789012301234567890123456789012345678901234567890123456789012345678901\n");
 
}
/*
void set_rtc(datetime_t *pt, datetime_t *pt_ntp,datetime_t *palarm) {
	if(rtc_set_flag==0) {
		pt->year = pt_ntp->year;
		pt->month = pt_ntp->month; 
		pt->day = pt_ntp->day;
		//pt->dotw = 0;
		pt->hour = pt_ntp->hour;
		pt->min = pt_ntp->min;
		pt->sec = pt_ntp->sec;
		palarm->year = pt_ntp->year;
		palarm->month = pt_ntp->month;
		palarm->day = pt_ntp->day;
		//palarm->dotw = 0;
		palarm->hour = pt_ntp->hour;
		palarm->min = pt_ntp->min + 1;
		palarm->sec = pt_ntp->sec;
		rtc_set_flag=1;
    	// Start the RTC
    	rtc_init();
    	rtc_set_datetime(&t);
		sleep_us(64);
		rtc_set_alarm(&alarm, &alarm_callback);

	}
        rtc_get_datetime(&t);
        datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
        //printf("\r%s      ", datetime_str);
        //printf("0x%x 0x%x\n",&t,&alarm);
        //printf("pt 0x%x palarm 0x%x\n",pt,palarm);
}
*/

void vLaunch( void) {
    TaskHandle_t task;
    xTaskCreate(main_task, "TestMainThread", configMINIMAL_STACK_SIZE, NULL, TEST_TASK_PRIORITY, &task);

#if NO_SYS && configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    // we must bind the main task to one core (well at least while the init is called)
    // (note we only do this in NO_SYS mode, because cyw43_arch_freertos
    // takes care of it otherwise)
    vTaskCoreAffinitySet(task, 1);
#endif

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

int main( void )
{
    stdio_init_all();
    preptopidata();
	ptrhead = (char *)&client_message[0];
	ptrtail = (char *)&client_message[0];
	ptrtopofbuf = (char *)&client_message[0];
	ptrendofbuf = (char *)&client_message[BUF_SIZE-1];
	printf("0x%x 0x%x 0x%x 0x%x \n", ptrhead, ptrtail, ptrendofbuf, ptrtopofbuf);
    
    /* Configure the hardware ready to run the demo. */
    const char *rtos_name;
#if ( configNUMBER_OF_CORES > 1 )
    rtos_name = "FreeRTOS SMP";
#else
    rtos_name = "FreeRTOS";
#endif

#if ( configNUMBER_OF_CORES == 2 )
    printf("Starting %s on both cores:\n", rtos_name);
    vLaunch();
#elif ( RUN_FREERTOS_ON_CORE == 1 )
    printf("Starting %s on core 1:\n", rtos_name);
    multicore_launch_core1(vLaunch);
    while (true);
#else
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();
#endif
    return 0;
}
