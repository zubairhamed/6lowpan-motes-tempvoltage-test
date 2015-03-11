
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include <stdio.h>

#include "debug.h"

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#include "dev/adc-sensor.h"
#include "dev/leds.h"

#define SEND_INTERVAL		60 * CLOCK_SECOND
#define MAX_PAYLOAD_LEN		40

#define GLOBAL_CONN_PORT 3002

#define SERVER_PORT 10001

static char buf[MAX_PAYLOAD_LEN];
static struct uip_udp_conn *conn;
static struct sensors_sensor *sensor;

PROCESS(mote_process, "Mote Process");
AUTOSTART_PROCESSES(&mote_process);

static void send_packet(void)
{
    static int rv;
    static float sane = 0;
    static int dec;
    static float frac;

    static int tempDec;
    static float tempFrac;
    static int vddDec;
    static int vddFrac;

    sensor = sensors_find(ADC_SENSOR);
    if(sensor) {
        rv = sensor->value(ADC_SENSOR_TYPE_TEMP);
        if(rv != -1) {
            sane = 25 + ((rv - 1480) / 4.5);
            dec = sane;
            frac = sane - dec;

            tempDec = dec;
            tempFrac = frac;
        }

        rv = sensor->value(ADLEDS_CONF_ALLC_SENSOR_TYPE_VDD);
        if(rv != -1) {
            sane = rv * 3.75 / 2047;
            dec = sane;
            frac = sane - dec;

            vddDec = dec;
            vddFrac = frac;
        }

        // Create CoAP Payload and send as NON CONFIRMABLE


        sprintf(buf, "%d.%02u|%d.%02u", tempDec, (unsigned int)(tempFrac*100), vddDec, vddFrac);
        uip_udp_packet_send(conn, buf, strlen(buf));
    }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mote_process, ev, data)
{
    static struct etimer et;
    uip_ipaddr_t server_addr;

    PROCESS_BEGIN();
    PRINTF("Mote Voltage/Temperature Demo Started.\n");

    leds_off(LEDS_ALL);
    uip_ip6addr(&server_addr, 0x2406, 0x3003, 0x201c, 0x456, 0x82e6, 0x50ff, 0xfe07, 0xea2e);

    conn = udp_new(&server_addr, UIP_HTONS(SERVER_PORT), NULL);
    if(!conn) {
        PRINTF("Error creating new UDP Global Connection\n");
    }
    udp_bind(conn, UIP_HTONS(GLOBAL_CONN_PORT));

    PRINTF("Server address ");
    PRINT6ADDR(&conn->ripaddr);
    PRINTF(" Port %u\n", UIP_HTONS(conn->rport));

    etimer_set(&et, SEND_INTERVAL);
    while(1) {
        PROCESS_YIELD();
        if(etimer_expired(&et)) {
            send_packet();
            etimer_restart(&et);
        }
    }
    PROCESS_END();
}
