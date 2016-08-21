#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "timers.h"
#include "halHeader.h"
#include "airconfig.h"
#include "protocol.h"
#include "state.h"
#include "io.h"
#include "ota.h"
#include "hal_sleep_manager.h"

#if defined(MTK_SMTCN_ENABLE)
#include "smt_conn.h"
#include "smt_api.h"
#endif
#include "netif.h"
#include "flash_map.h"

#include "sys_init.h"
#include "bsp_gpio_ept_config.h"
#include "network_init.h"
#include "cli_def.h"


uint8_t gin_airconfig_current_channel = 0;
int gin_airconfig_channel_cared[MAX_CHANNEL_CARE];
static TimerHandle_t lelink_airconfig_timer = NULL;
extern void smtcn_evt_handler(wifi_smart_connection_event_t event, void *data);

int airconfig_start(void *pc, uint8_t *prov_key, int prov_key_len);
int airconfig_stop();

int8_t gin_airconfig_ap_connected;
int8_t gin_airconfig_sniffer_got;

#define AIRCONFIG_TIMEOUT (60*60*1000/portTICK_PERIOD_MS)

static void lelink_airconfig_timeout_timer_callback( TimerHandle_t tmr ) {
    if (tmr) {
        xTimerStop(tmr, 0);
        xTimerDelete(tmr, portMAX_DELAY);
        lelink_airconfig_timer = NULL;
    }
    airconfig_stop();
	APPLOG("lelink_airconfig_timeout_timer_callback(). \r\n");
}

static void mtk_thread_airconfig_proc(void *args) {	
    if(wifi_smart_connection_init(NULL, 0, smtcn_evt_handler) < 0){
        return;
    }
    wifi_smart_connection_start(0);
    vTaskDelete(NULL);
}

int airconfig_start(void *ptr, uint8_t *prov_key, int prov_key_len) {
	lelink_airconfig_timer = xTimerCreate( "lelink_airconfig_timer",
                                       AIRCONFIG_TIMEOUT,
                                       pdFALSE,
                                       NULL,
                                       lelink_airconfig_timeout_timer_callback);
    if (lelink_airconfig_timer != NULL) {
        xTimerStart(lelink_airconfig_timer, 0);
    }
	
    if (pdPASS != xTaskCreate(mtk_thread_airconfig_proc,
							  "thread_lelink_proc",
							  1024*2,  //2K stack size;
							  NULL,
							  1,
							  NULL)) {
		LOG_E(common, "create user task fail");
		return 0;
	}
	
    gin_airconfig_sniffer_got = 0;
    return 1;
}

int airconfig_stop() {
    gin_airconfig_current_channel = 0;
    airconfig_reset();
    return 0;
}

static void mtk_thread_lelink_proc(void *args) {
    void *ctxR2R = (void *)lelinkNwNew(REMOTE_BAK_IP, REMOTE_BAK_PORT, 0, NULL);
    void *ctxQ2A = (void *)lelinkNwNew(NULL, 0, NW_SELF_PORT, NULL);

    while (1) {
        lelinkPollingState(100, ctxR2R, ctxQ2A);
    }

    lelinkNwDelete(ctxR2R);
    lelinkNwDelete(ctxQ2A);
}

static int platform_launch_lelink_start(void) {

    if (pdPASS != xTaskCreate(mtk_thread_lelink_proc,
							  "thread_lelink_proc",
							  1024*5,  //16K stack size;
							  NULL,
							  1,
							  NULL)) {
		LOG_E(common, "create user task fail");
		return -1;
	}
	
    return 0;
}

void printForFac(void) {
    uint8_t mac[6] = {0};
    halGetMac(mac, 6);
    APPLOG("mac: %02x:%02x:%02x:%02x:%02x:%02x\r\n", 
        mac[0], 
        mac[1], 
        mac[2], 
        mac[3], 
        mac[4], 
        mac[5]);
}

void platform_and_wlan_init_done(void) {
	int ret; 
    printForFac();
	
    ret = lelinkStorageInit(CM4_FLASH_LELINK_CUST_ADDR, FLASH_LELINK_CUST_SIZE, 0x1000);//CM4 buff slim:128KB + fota buff slim:128KB;->totalSize:0x40000
    if (0 > ret) {
        APPLOGE("lelinkStorageInit ret[%d]\r\n", ret);
        return;
    }
    // protocol
    ret = lelinkInit(NULL);
    if (0 > ret) {
        APPLOGE("lelinkInit ret[%d]\r\n", ret);
        return;
    }
    platform_launch_lelink_start();
}

uint8_t le_ota(uint8_t len, char *param[]) {
    int type = 2;
    char url[64] = "http://115.182.63.167/zhiwei/mt7687_le_demo.bin";
    if (len < 1) {
        APPLOGE("need url address\n");
        return 0;
    }
    type = atoi(param[0]);
    if (len == 2) {
        strncpy(url, param[1], sizeof(url));
        APPLOG("url address %s\n",param[1]);
    }
   
    leOTA(type, url, NULL, 0);
    return 0;
}

void le_gpio_test(char data)
{
    #if 0
    uint8_t buff[24];
    switch (data)
    {
        case 1:
            APPLOG("all init \n");
            break;
        case 2:
            halGPIOWrite(0, 0, 1);
            break;
        case 3:
            halGPIOWrite(0, 0, 0);
            break;
        case 4:
            halGPIOClose(0);
            break;
        case 5:
            halUartWrite(0, "hello world!\n", 12);
            break;
        case 6:
            memset(buff, 0, 24);
            halUartRead(0, buff, 24);
            halUartWrite(0, "receive data:", 13);
            halUartWrite(0, buff, 24);
            break;
        case 7:
            while(1)
            {
                if(flag)
                    halPWMSetDuty(3, 255);
                else
                    halPWMSetDuty(3, 0);
                flag = !flag;
                vTaskDelay(1000);
            }
            break;
        default:
            APPLOGE("wrong param = %c", data);
            break;
    }
    #endif
}

extern void HWhalAES(void);
extern void mbedtls_aes_test();
uint8_t le_reboot(uint8_t len, char *param[]) {
    int type = atoi(param[0]);
    //halReboot();
    //mbedtls_aes_test();
    //HWhalAES();
    APPLOG("le_gpio_test len[%d] param[%d] \n", len, type);
    le_gpio_test(type);
    return 0;
}

uint8_t g_ipv4_addr[4] = {0};

extern void tickless_init(void);

void wifi_socket_ip_ready_callback(const struct netif *netif) {	
	APPLOG("[wifibox]begin to create wifi_audio_box_socket_task. \n");
	//xTaskHandle xHandle;

#if BYTE_ORDER == BIG_ENDIAN
	 g_ipv4_addr[0] = (((netif->ip_addr).addr) >> 24) & 0xff ;
	 g_ipv4_addr[1] = (((netif->ip_addr).addr) >> 16) & 0xff ;
	 g_ipv4_addr[2] = (((netif->ip_addr).addr) >> 8) & 0xff ;
	 g_ipv4_addr[3] = ((netif->ip_addr).addr) & 0xff ;
	 APPLOG("[BIG_ENDIAN]:wifi_box_socket_test(), ip_addr = %d.%d.%d.%d \n",g_ipv4_addr[0], g_ipv4_addr[1], g_ipv4_addr[2], g_ipv4_addr[3]);
#else
	 g_ipv4_addr[0] = ((netif->ip_addr).addr) & 0xff ;
	 g_ipv4_addr[1] = (((netif->ip_addr).addr) >> 8) & 0xff ;
	 g_ipv4_addr[2] = (((netif->ip_addr).addr) >> 16) & 0xff ;
	 g_ipv4_addr[3] = (((netif->ip_addr).addr) >> 24) & 0xff ;
	 APPLOG("[LITTLE_ENDIAN]:wifi_box_socket_test(), ip_addr = %d.%d.%d.%d \n",g_ipv4_addr[0], g_ipv4_addr[1], g_ipv4_addr[2], g_ipv4_addr[3]);
#endif
     gin_airconfig_ap_connected = 1;
}

void mtk_platform_init() {
	/* Do system initialization, eg: hardware, nvdm, logging and random seed. */
    system_init();

    /* bsp_ept_gpio_setting_init() under driver/board/mt76x7_hdk/ept will initialize the GPIO settings
     * generated by easy pinmux tool (ept). ept_*.c and ept*.h are the ept files and will be used by
     * bsp_ept_gpio_setting_init() for GPIO pinumux setup.
     */
    bsp_ept_gpio_setting_init();

    /* Initialize network process,  includes: load the wifi profile setting from NVDM,  
     * wifi stack initaization,  and some wifi event handler register,   
     * tcpip stack and net interface initialization,  dhcp client, dhcp server process initialization
     * notes:  the wifi initial process will be implemented and finished while system task scheduler is running,
     *            when it is done , the WIFI_EVENT_IOT_INIT_COMPLETE event will be triggered */
    network_full_init();
    
#if configUSE_TICKLESS_IDLE == 2
    if (hal_sleep_manager_init() == HAL_SLEEP_MANAGER_OK) {
        tickless_init();
    }
#endif

#if defined(MTK_MINICLI_ENABLE)
    /* Initialize cli task to enable user input cli command from uart port.*/
    cli_def_create();
    cli_task_create();
#endif

#ifdef MTK_HOMEKIT_ENABLE
    homekit_init();
#endif
	wifi_register_ip_ready_callback(wifi_socket_ip_ready_callback);
    //vTaskDelay(1000);
}

int main() {
    //modules_init();
    mtk_platform_init();

    platform_and_wlan_init_done();
	
    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following line
    will never be reached.  If the following line does execute, then there was
    insufficient FreeRTOS heap memory available for the idle and/or timer tasks
    to be created.  See the memory management section on the FreeRTOS web site
    for more details. */
    for ( ;; );
}
