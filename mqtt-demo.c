/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/** \addtogroup cc2538-examples
 * @{
 *
 * \defgroup cc2538-mqtt-demo CC2538 MQTT Demo Project
 *
 * Demonstrates MQTT functionality. Works with IBM Quickstart as well as
 * mosquitto.
 * @{
 *
 * \file
 * An MQTT example for the cc2538-based platforms
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "rpl/rpl-private.h"
#include "mqtt.h"
#include "net/rpl/rpl.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-icmp6.h"
#include "net/ipv6/sicslowpan.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "dev/cc2538-sensors.h"
#include "dev/als-sensor.h"
#include "dev/cc2538-temp-sensor.h"
#include "dev/vdd3-sensor.h"
#include "dev/adc.h"


#include "ontology.h"

#include <string.h>

#define GET_VALUE_LIGHT (als_sensor.value(0))

//value function takes an int input (0 means raw temp value and 1 means converted)
#define GET_TEMPERATURE (cc2538_temp_sensor.value(1)) 

#define GET_VOLTAGE (vdd3_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED)) 



/*---------------------------------------------------------------------------*/
/*
 * IBM server: messaging.quickstart.internetofthings.ibmcloud.com
 * (184.172.124.189) mapped in an NAT64 (prefix 64:ff9b::/96) IPv6 address
 * Note: If not able to connect; lookup the IP address again as it may change.
 *
 * Alternatively, publish to a local MQTT broker (e.g. mosquitto) running on
 * the node that hosts your border router
 */
#ifdef MQTT_DEMO_BROKER_IP_ADDR
static const char *broker_ip = MQTT_DEMO_BROKER_IP_ADDR;
#define DEFAULT_ORG_ID              "mqtt-demo"
#else
static const char *broker_ip = "0064:ff9b:0000:0000:0000:0000:b8ac:7cbd";
#define DEFAULT_ORG_ID              "quickstart"
#endif
/*---------------------------------------------------------------------------*/
/*
 * A timeout used when waiting for something to happen (e.g. to connect or to
 * disconnect)
 */
#define CONFIG_SEND_WAIT (CLOCK_SECOND * 5)
#define STATE_MACHINE_PERIODIC     (CLOCK_SECOND >> 1)
/*---------------------------------------------------------------------------*/
/* Provide visible feedback via LEDS during various states */
/* When connecting to broker */
#define CONNECTING_LED_DURATION    (CLOCK_SECOND >> 2)

/* Each time we try to publish */
#define PUBLISH_LED_ON_DURATION    (CLOCK_SECOND)
/*---------------------------------------------------------------------------*/
/* Connections and reconnections */
#define RETRY_FOREVER              0xFF
#define RECONNECT_INTERVAL         (CLOCK_SECOND * 2)

/*
 * Number of times to try reconnecting to the broker.
 * Can be a limited number (e.g. 3, 10 etc) or can be set to RETRY_FOREVER
 */
#define RECONNECT_ATTEMPTS         RETRY_FOREVER
#define CONNECTION_STABLE_TIME     (CLOCK_SECOND * 5)
static struct timer connection_life;
static uint8_t connect_attempt;
/*---------------------------------------------------------------------------*/
/* Various states */
static uint8_t state;
static uint8_t counter;   //introduced state
#define STATE_INIT            0
#define STATE_REGISTERED      1
#define STATE_CONNECTING      2
#define STATE_CONNECTED       3
#define STATE_PUBLISHING      4
#define STATE_DISCONNECTED    5
#define STATE_NEWCONFIG       6
#define STATE_ONEM2MREGISTER  7
#define STATE_CONFIG_ERROR 0xFE
#define STATE_ERROR        0xFF
/*---------------------------------------------------------------------------*/
#define CONFIG_ORG_ID_LEN        32
#define CONFIG_TYPE_ID_LEN       32
#define CONFIG_AUTH_TOKEN_LEN    32
#define CONFIG_EVENT_TYPE_ID_LEN 32
#define CONFIG_CMD_TYPE_LEN       8
#define CONFIG_IP_ADDR_STR_LEN   64
/*---------------------------------------------------------------------------*/
#define RSSI_MEASURE_INTERVAL_MAX 86400 /* secs: 1 day */
#define RSSI_MEASURE_INTERVAL_MIN     5 /* secs */
#define PUBLISH_INTERVAL_MAX      86400 /* secs: 1 day */
#define PUBLISH_INTERVAL_MIN          5 /* secs */
/*---------------------------------------------------------------------------*/
/* A timeout used when waiting to connect to a network */
#define NET_CONNECT_PERIODIC        (CLOCK_SECOND >> 2)
#define NO_NET_LED_DURATION         (NET_CONNECT_PERIODIC >> 1)
/*---------------------------------------------------------------------------*/
/* Default configuration values */
#define DEFAULT_TYPE_ID             "cc2538"
#define DEFAULT_AUTH_TOKEN          "AUTHZ"
#define DEFAULT_EVENT_TYPE_ID       "status"
#define DEFAULT_SUBSCRIBE_CMD_TYPE  "+"
#define DEFAULT_BROKER_PORT         1883
#define DEFAULT_PUBLISH_INTERVAL    (5 * CLOCK_SECOND) //changed , initially 30
#define DEFAULT_KEEP_ALIVE_TIMER    60
#define DEFAULT_RSSI_MEAS_INTERVAL  (CLOCK_SECOND * 30)
/*---------------------------------------------------------------------------*/
/* Take a sensor reading on button press */
#define PUBLISH_TRIGGER &button_sensor

/* Payload length of ICMPv6 echo requests used to measure RSSI with def rt */
#define ECHO_REQ_PAYLOAD_LEN   20
/*---------------------------------------------------------------------------*/
PROCESS_NAME(mqtt_demo_process);
AUTOSTART_PROCESSES(&mqtt_demo_process);
/*---------------------------------------------------------------------------*/
/**
 * \brief Data structure declaration for the MQTT client configuration
 */
typedef struct mqtt_client_config {
  char org_id[CONFIG_ORG_ID_LEN];
  char type_id[CONFIG_TYPE_ID_LEN];
  char auth_token[CONFIG_AUTH_TOKEN_LEN];
  char event_type_id[CONFIG_EVENT_TYPE_ID_LEN];
  char broker_ip[CONFIG_IP_ADDR_STR_LEN];
  char cmd_type[CONFIG_CMD_TYPE_LEN];
  clock_time_t pub_interval;
  int def_rt_ping_interval;
  uint16_t broker_port;
} mqtt_client_config_t;
/*---------------------------------------------------------------------------*/
/* Maximum TCP segment size for outgoing segments of our socket */
#define MAX_TCP_SEGMENT_SIZE    32
/*---------------------------------------------------------------------------*/
#define STATUS_LED LEDS_RED
/*---------------------------------------------------------------------------*/
/*
 * Buffers for Client ID and Topic.
 * Make sure they are large enough to hold the entire respective string
 *
 * d:quickstart:status:EUI64 is 32 bytes long
 * iot-2/evt/status/fmt/json is 25 bytes
 * We also need space for the null termination
 */
#define BUFFER_SIZE 64
static char client_id[BUFFER_SIZE];
/*---------------------------------------------------------------------------*/
/*
 * The main MQTT buffers.
 * We will need to increase if we start publishing more data.
 */
#define APP_BUFFER_SIZE 8192
static struct mqtt_connection conn;
static char app_buffer[APP_BUFFER_SIZE];
static char button_ON[3]="ON";
static char button_OFF[4]="OFF";
/*---------------------------------------------------------------------------*/
#define QUICKSTART "quickstart"
/*---------------------------------------------------------------------------*/
static struct mqtt_message *msg_ptr = 0;
static struct etimer publish_periodic_timer;
static struct ctimer ct;
static char *buf_ptr;
static uint16_t seq_nr_value = 0;
/*---------------------------------------------------------------------------*/
/* Parent RSSI functionality */
static struct uip_icmp6_echo_reply_notification echo_reply_notification;
static struct etimer echo_request_timer;
static int def_rt_rssi = 0;
/*---------------------------------------------------------------------------*/
static mqtt_client_config_t conf;

static char sent_config = 0;
/*---------------------------------------------------------------------------*/
PROCESS(mqtt_demo_process, "MQTT Demo");
/*---------------------------------------------------------------------------*/
int
ipaddr_sprintf(char *buf, uint8_t buf_len, const uip_ipaddr_t *addr)
{
  uint16_t a;
  uint8_t len = 0;
  int i, f;
  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) {
        len += snprintf(&buf[len], buf_len - len, "::");
      }
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
        len += snprintf(&buf[len], buf_len - len, ":");
      }
      len += snprintf(&buf[len], buf_len - len, "%x", a);
    }
  }

  return len;
}
/*---------------------------------------------------------------------------*/
static void
echo_reply_handler(uip_ipaddr_t *source, uint8_t ttl, uint8_t *data,
                   uint16_t datalen)
{
  if(uip_ip6addr_cmp(source, uip_ds6_defrt_choose())) {
    def_rt_rssi = sicslowpan_get_last_rssi();
  }
}
/*---------------------------------------------------------------------------*/
static void
publish_led_off(void *d)
{
  leds_off(STATUS_LED);
}
/*---------------------------------------------------------------------------*/

static char state_buffer[16] = "OFF";

static int
ctrl_leds(const char * chunk) 
{
	if(strncmp(chunk, state_buffer, 16) == 0)
		return 1; //NOOP
	
	if(strncmp(chunk, "ON", 2) == 0) {
      leds_on(LEDS_ALL);
	} else if(strncmp(chunk, "COLOR", 5) == 0) {
		char * delim = strchr(chunk,'=');
		return ctrl_leds(&(delim[1]));
	} else if(strncmp(chunk, "GREEN", 5) == 0) {
      leds_off(LEDS_ALL);
	  leds_on(LEDS_GREEN);
	} else if(strncmp(chunk, "RED", 3) == 0) {
      leds_off(LEDS_ALL);
	  leds_on(LEDS_RED);
	} else if(strncmp(chunk, "YELLOW", 6) == 0) {
      leds_off(LEDS_ALL);
	  leds_on(LEDS_YELLOW);
	} else if(strncmp(chunk, "ORANGE", 6) == 0) {
      leds_off(LEDS_ALL);
	  leds_on(LEDS_ORANGE);
    } else if(strncmp(chunk, "OFF", 3) == 0) {
      leds_off(LEDS_ALL);
    } else 
		return 255; //ERROR
	
	strncpy(state_buffer, chunk, 16);
	
	return 0; //OK
}
/*---------------------------------------------------------------------------*/
static char reqid_buffer[BUFFER_SIZE];


static void
send_response (const char * response) 
{
	
	//TODO ERROR HANDLING
	snprintf(app_buffer, APP_BUFFER_SIZE-1, "%s;%s", reqid_buffer, response);
	
	mqtt_publish(&conn, NULL, RESPONSE_TOPIC, (uint8_t *)app_buffer,
               strlen(app_buffer), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF);
}
/*---------------------------------------------------------------------------*/
static void
pub_handler(const char *topic, uint16_t topic_len, const uint8_t *chunk,
            uint16_t chunk_len)
{
  char * delim = strchr(chunk,';');
  int id_len = (int)(delim - (char *)chunk);
  
  snprintf(reqid_buffer, id_len+1, "%s", chunk);

  printf("Got CTRL MSG: %s\n",chunk);
  
  if(strncmp(topic, "led", 3) == 0) {
	switch (ctrl_leds(&(delim[1]))) {
		case 0:
			send_response("OK");
			break;
		case 1:
			send_response("NOOP");
			break;
		default:
			send_response("ERROR");
			break;
	}		
    return;
  }
}
/*---------------------------------------------------------------------------*/
static void
mqtt_event(struct mqtt_connection *m, mqtt_event_t event, void *data)
{
  switch(event) {
  case MQTT_EVENT_CONNECTED: {
    DBG("APP - Application has a MQTT connection\n");
    timer_set(&connection_life, CONNECTION_STABLE_TIME);
    state = STATE_CONNECTED;
    break;
  }
  case MQTT_EVENT_DISCONNECTED: {
    DBG("APP - MQTT Disconnect. Reason %u\n", *((mqtt_event_t *)data));

    state = STATE_DISCONNECTED;
    process_poll(&mqtt_demo_process);
    break;
  }
  case MQTT_EVENT_PUBLISH: {
    msg_ptr = data;

    /* Implement first_flag in publish message? */
    if(msg_ptr->first_chunk) {
      msg_ptr->first_chunk = 0;
      DBG("APP - Application received a publish on topic '%s'. Payload "
          "size is %i bytes. Content:\n\n",
          msg_ptr->topic, msg_ptr->payload_length);
    }

    pub_handler(msg_ptr->topic, strlen(msg_ptr->topic), msg_ptr->payload_chunk,
                msg_ptr->payload_length);
    break;
  }
  case MQTT_EVENT_SUBACK: {
    DBG("APP - Application is subscribed to topic successfully\n");
    break;
  }
  case MQTT_EVENT_UNSUBACK: {
    DBG("APP - Application is unsubscribed to topic successfully\n");
    break;
  }
  case MQTT_EVENT_PUBACK: {
    DBG("APP - Publishing complete.\n");
    break;
  }
  default:
    DBG("APP - Application got a unhandled MQTT event: %i\n", event);
    break;
  }
}
/*---------------------------------------------------------------------------*/
static int
construct_client_id(void)
{
  int len = snprintf(client_id, BUFFER_SIZE, "d:%s:%s:%02x%02x%02x%02x%02x%02x",
                     conf.org_id, conf.type_id,
                     linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
                     linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[5],
                     linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

  /* len < 0: Error. Len >= BUFFER_SIZE: Buffer too small */
  if(len < 0 || len >= BUFFER_SIZE) {
    printf("Client ID: %d, Buffer %d\n", len, BUFFER_SIZE);
    return 0;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
static void
update_config(void)
{
  if(construct_client_id() == 0) {
    /* Fatal error. Client ID larger than the buffer */
    state = STATE_CONFIG_ERROR;
    return;
  }

  /*if(construct_sub_topic() == 0) {
    // Fatal error. Topic larger than the buffer 
    state = STATE_CONFIG_ERROR;
    return;
  }

  if(construct_pub_topic() == 0) {
    // Fatal error. Topic larger than the buffer 
    state = STATE_CONFIG_ERROR;
    return;
  }*/

  /* Reset the counter */
  seq_nr_value = 0;

  state = STATE_INIT;

  /*
   * Schedule next timer event ASAP
   *
   * If we entered an error state then we won't do anything when it fires.
   *
   * Since the error at this stage is a config error, we will only exit this
   * error state if we get a new config.
   */
  etimer_set(&publish_periodic_timer, 0);

  return;
}
/*---------------------------------------------------------------------------*/
static int
init_config()
{
  /* Populate configuration with default values */
  memset(&conf, 0, sizeof(mqtt_client_config_t));

  memcpy(conf.org_id, DEFAULT_ORG_ID, strlen(DEFAULT_ORG_ID));
  memcpy(conf.type_id, DEFAULT_TYPE_ID, strlen(DEFAULT_TYPE_ID));
  memcpy(conf.auth_token, DEFAULT_AUTH_TOKEN, strlen(DEFAULT_AUTH_TOKEN));
  memcpy(conf.event_type_id, DEFAULT_EVENT_TYPE_ID,
         strlen(DEFAULT_EVENT_TYPE_ID));
  memcpy(conf.broker_ip, broker_ip, strlen(broker_ip));
  memcpy(conf.cmd_type, DEFAULT_SUBSCRIBE_CMD_TYPE, 1);

  conf.broker_port = DEFAULT_BROKER_PORT;
  conf.pub_interval = DEFAULT_PUBLISH_INTERVAL;
  conf.def_rt_ping_interval = DEFAULT_RSSI_MEAS_INTERVAL;

  return 1;
}
/*---------------------------------------------------------------------------*/
static void
subscribe(void)
{
  /* Publish MQTT topic in IBM quickstart format */
  mqtt_status_t status;

  printf("subscribing to topic %s\n", CTRL_TOPIC);
  status = mqtt_subscribe(&conn, NULL, CTRL_TOPIC, MQTT_QOS_LEVEL_0);

  DBG("APP - Subscribing!\n");
  if(status == MQTT_STATUS_OUT_QUEUE_FULL) {
    DBG("APP - Tried to subscribe but command queue was full!\n");
  }
}
/*---------------------------------------------------------------------------*/

static void
publish(void)  //called from state_machine()
{
  int len;
  int remaining = APP_BUFFER_SIZE;

  buf_ptr = app_buffer;

  len = snprintf(buf_ptr, remaining,
                 "%d",
                 GET_VALUE_LIGHT); //GET_VALUE_LIGHT //GET_TEMPERATURE //GET_VOLTAGE

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }

  remaining -= len;
  buf_ptr += len;

 
  mqtt_publish(&conn, NULL, SENSOR_BRIGHT, (uint8_t *)app_buffer, //SENSOR_BRIGHT SENSOR_TEMP SENSOR_VOLT
               strlen(app_buffer), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF); //originally retain was ON
  
  DBG("APP - Publish!\n");
}

//REVISED PUBLISH: divided into 3 different functions for each sensor

static void
publish_temp(void)  //called from state_machine()
{
  int len;
  int remaining = APP_BUFFER_SIZE;

  buf_ptr = app_buffer;

  len = snprintf(buf_ptr, remaining,
                 "%d",
                 GET_TEMPERATURE); 

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }

  remaining -= len;
  buf_ptr += len;


  mqtt_publish(&conn, NULL, SENSOR_TEMP, (uint8_t *)app_buffer,  
               strlen(app_buffer), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF);
}


static void
publish_voltage(void)  //called from state_machine()
{
  int len;
  int remaining = APP_BUFFER_SIZE;

  buf_ptr = app_buffer;

  len = snprintf(buf_ptr, remaining,
                 "%d",
                 GET_VOLTAGE); 

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }

  remaining -= len;
  buf_ptr += len;


  mqtt_publish(&conn, NULL, SENSOR_VOLT, (uint8_t *)app_buffer,  
               strlen(app_buffer), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF);
}

static void publish_select_button(void){
   mqtt_publish(&conn, NULL, SENSOR_SELECT_BUTTON, (uint8_t *)button_ON,  
               strlen(button_ON), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF);
	
     
}

/*---------------------------------------------------------------------------*/
static void
connect_to_broker(void)
{
  /* Connect to MQTT server */
  mqtt_connect(&conn, conf.broker_ip, conf.broker_port,
               conf.pub_interval * 3);

  state = STATE_CONNECTING;
}
/*---------------------------------------------------------------------------*/
static void
ping_parent(void)
{
  if(uip_ds6_get_global(ADDR_PREFERRED) == NULL) {
    return;
  }

  uip_icmp6_send(uip_ds6_defrt_choose(), ICMP6_ECHO_REQUEST, 0,
                 ECHO_REQ_PAYLOAD_LEN);
}
/*---------------------------------------------------------------------------*/
static long remaining_ontology = -1;
static char * pos_ontology;
static char end_delimiter_send = 0;

static void
send_oneM2M_definition(void)
{
	int len;
	
	if(conn.out_queue_full) {
		printf("MQTT QUEUE FULL, retrying later\n");
		return;
	}
	
	if (end_delimiter_send)
	{
		sent_config = 1;
		return;
	}
	

	if (remaining_ontology > 0) {
		printf("sending partial-config -> broker\n");
		len = snprintf(app_buffer, APP_BUFFER_SIZE-1,"%s;%s",DEVICE_UUID, pos_ontology);
		
		if (len > APP_BUFFER_SIZE-1)
			len = APP_BUFFER_SIZE-1;
		
		len -= strlen(DEVICE_UUID)+1;
	
		mqtt_publish(&conn, NULL, CONFIG_TOPIC, (uint8_t *)app_buffer,
		strlen(app_buffer), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF);
			
		remaining_ontology -= len;
		pos_ontology = & (pos_ontology[len-1]);
		printf("DONE sending partial-config -> broker\n");
		return;
	} 
	
	printf("sending config-end-delimiter -> broker\n");
	snprintf(app_buffer,APP_BUFFER_SIZE-1,"%s;END",DEVICE_UUID);
	
	mqtt_publish(&conn, NULL, CONFIG_TOPIC, (uint8_t *)app_buffer,
		strlen(app_buffer), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF);
		
	end_delimiter_send = 1;
	

	DBG("APP - Publish Config!\n");
	   
}
/*---------------------------------------------------------------------------*/
static void
state_machine(void)
{
  
  switch(state) {
  case STATE_INIT:
    /* If we have just been configured register MQTT connection */
    mqtt_register(&conn, &mqtt_demo_process, client_id, mqtt_event,
                  MAX_TCP_SEGMENT_SIZE);

    /*
     * If we are not using the quickstart service (thus we are an IBM
     * registered device), we need to provide user name and password
     */
    if(strncasecmp(conf.org_id, QUICKSTART, strlen(conf.org_id)) != 0) {
      if(strlen(conf.auth_token) == 0) {
        printf("User name set, but empty auth token\n");
        state = STATE_ERROR;
        break;
      } else {
        mqtt_set_username_password(&conn, "use-token-auth",
                                   conf.auth_token);
      }
    }

    /* _register() will set auto_reconnect. We don't want that. */
    conn.auto_reconnect = 0;
    connect_attempt = 1;

    state = STATE_REGISTERED;
    DBG("Init\n");
    /* Continue */
  case STATE_REGISTERED:
    if(uip_ds6_get_global(ADDR_PREFERRED) != NULL) {
      /* Registered and with a public IP. Connect */
      DBG("Registered. Connect attempt %u\n", connect_attempt);
      ping_parent();
      connect_to_broker();
    } else {
      leds_on(STATUS_LED);
      ctimer_set(&ct, NO_NET_LED_DURATION, publish_led_off, NULL);
    }
    etimer_set(&publish_periodic_timer, NET_CONNECT_PERIODIC);
    return;
    break;
  case STATE_CONNECTING:
    leds_on(STATUS_LED);
    ctimer_set(&ct, CONNECTING_LED_DURATION, publish_led_off, NULL);
    /* Not connected yet. Wait */
    DBG("Connecting (%u)\n", connect_attempt);
    break;
  case STATE_CONNECTED:
    /* Don't subscribe unless we are a registered device */
    if(strncasecmp(conf.org_id, QUICKSTART, strlen(conf.org_id)) == 0) {
      DBG("Using 'quickstart': Skipping subscribe\n");
      state = STATE_PUBLISHING;
    }
    /* Continue */
  case STATE_ONEM2MREGISTER:
	state = STATE_ONEM2MREGISTER;
    /*Send config to broker*/
	if (!sent_config)
	{
		if (remaining_ontology == -1) {
			printf("initial config for ontology sending\n");
			remaining_ontology = strlen(ontology);
			pos_ontology = ontology;
		}
		
		send_oneM2M_definition();
		
		etimer_set(&publish_periodic_timer, CONFIG_SEND_WAIT);
		return;
	
	}
	printf("DONE sending partial-config -> broker\n");
	/* Continue */
  case STATE_PUBLISHING:
    /* If the timer expired, the connection is stable. */
    if(timer_expired(&connection_life)) {
      /*
       * Intentionally using 0 here instead of 1: We want RECONNECT_ATTEMPTS
       * attempts if we disconnect after a successful connect
       */
      connect_attempt = 0;
    }

    if(mqtt_ready(&conn) && conn.out_buffer_sent) {
      /* Connected. Publish */
      if(state == STATE_ONEM2MREGISTER) {
        subscribe();
        state = STATE_PUBLISHING;
      } else {
        if (counter==0){
          publish();
          counter++;
        }
        else {
          if (counter==1){
            publish_temp();
            counter++;
          }
	  else {
          if (counter==2){
            publish_voltage();
            counter=0;
            } else
               counter=0;
          }	
        }
         
        //publish();    //PUBLISH FUNCTION IS CALLED HERE +++++++++++++++++++++++++++++++++++
	
        //publish_temp();
      }
      etimer_set(&publish_periodic_timer, conf.pub_interval);

      DBG("Publishing\n");
      /* Return here so we don't end up rescheduling the timer */
      return;
    } else {
      /*
       * Our publish timer fired, but some MQTT packet is already in flight
       * (either not sent at all, or sent but not fully ACKd).
       *
       * This can mean that we have lost connectivity to our broker or that
       * simply there is some network delay. In both cases, we refuse to
       * trigger a new message and we wait for TCP to either ACK the entire
       * packet after retries, or to timeout and notify us.
       */
      DBG("Publishing... (MQTT state=%d, q=%u)\n", conn.state,
          conn.out_queue_full);
    }
    break;
  case STATE_DISCONNECTED:
    DBG("Disconnected\n");
    if(connect_attempt < RECONNECT_ATTEMPTS ||
       RECONNECT_ATTEMPTS == RETRY_FOREVER) {
      /* Disconnect and backoff */
      clock_time_t interval;
      mqtt_disconnect(&conn);
      connect_attempt++;

      interval = connect_attempt < 3 ? RECONNECT_INTERVAL << connect_attempt :
        RECONNECT_INTERVAL << 3;

      DBG("Disconnected. Attempt %u in %lu ticks\n", connect_attempt, interval);

      etimer_set(&publish_periodic_timer, interval);

      state = STATE_REGISTERED;
      return;
    } else {
      /* Max reconnect attempts reached. Enter error state */
      state = STATE_ERROR;
      DBG("Aborting connection after %u attempts\n", connect_attempt - 1);
    }
    break;
  case STATE_CONFIG_ERROR:
    /* Idle away. The only way out is a new config */
    printf("Bad configuration.\n");
    return;
  case STATE_ERROR:
  default:
    leds_on(STATUS_LED);
    /*
     * 'default' should never happen.
     *
     * If we enter here it's because of some error. Stop timers. The only thing
     * that can bring us out is a new config event
     */
    printf("Default case: State=0x%02x\n", state);
    return;
  }

  /* If we didn't return so far, reschedule ourselves */
  etimer_set(&publish_periodic_timer, STATE_MACHINE_PERIODIC);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mqtt_demo_process, ev, data)
{

  PROCESS_BEGIN();

  printf("MQTT Demo Process\n");

  if(init_config() != 1) {
    PROCESS_EXIT();
  }

  update_config();
  def_rt_rssi = 0x8000000;
  uip_icmp6_echo_reply_callback_add(&echo_reply_notification,
                                    echo_reply_handler);
  etimer_set(&echo_request_timer, conf.def_rt_ping_interval);

  /* Main loop */
  while(1) {

    PROCESS_YIELD();

    if(ev == sensors_event && data == PUBLISH_TRIGGER) {  //sensors_event originates from sensors.c
      if(state == STATE_ERROR) {
        connect_attempt = 1;
        state = STATE_REGISTERED;
      }
    }
    //binding the response from buttons
    if (ev == sensors_event && data == &button_select_sensor){
       publish_select_button();       
    }

    if((ev == PROCESS_EVENT_TIMER && data == &publish_periodic_timer) ||
       ev == PROCESS_EVENT_POLL ||
       (ev == sensors_event && data == PUBLISH_TRIGGER)) {
      state_machine();
    }

    if(ev == PROCESS_EVENT_TIMER && data == &echo_request_timer) {
      ping_parent();
      etimer_set(&echo_request_timer, conf.def_rt_ping_interval);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
