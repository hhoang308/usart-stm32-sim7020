#ifndef PARAMETER_H_
#define PARAMETER_H_

typedef enum StatusType {
	STATUS_SUCCESS = 0, /**< The operation was successful. */
	STATUS_ERROR, /**< The operation was fail. */
	STATUS_MODEM_NOT_READY, /**< The modem is not ready yet. */
	STATUS_TIMEOUT, /**< The operation timed out. */
	STATUS_MQTT_NOT_CONNECTED, /**< The supplied socket is not connected. */
} StatusType;

#ifndef MQTT_CLIENT_ID 
#define MQTT_CLIENT_ID "SmartWaterMeter"
#endif

#ifndef MQTT_SERVER 
#define MQTT_SERVER "171.244.173.204"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1884
#endif

#ifndef MQTT_USERNAME
#define MQTT_USERNAME "admin"
#endif

#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD "admin"
#endif

#ifndef COMMAND_TIMEOUT
#define COMMAND_TIMEOUT 5000
#endif

#endif /* PARAMETER_H_ */