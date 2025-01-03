#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>
#include <MQTTClient.h>

#define SERVER_PORT 8080
#define BUFFER_SIZE 1024

// real time scheduling
#define VERIFY_DATA_PERIOD 35
#define VERIFY_DATA_EXECUTION 10
#define SERVER_PERIOD 40
#define SERVER_CAPACITY 6

// priorities
#define VERIFY_DATA_PRIORITY 5
#define SERVER_PRIORITY 4
#define MQTT_PRIORITY 3
#define PREFERENCE_DETECTION_PRIORITY 2
#define SENSOR_VERIFICATION_PRIORITY 1

// MQTT
#define QOS 1
#define TIMEOUT 10000L
#define MQTT_CLIENT_ID "ESP32-G12012"
const char *mqtt_server = "ssl://e75ba1e7000741ea8a244cfdd6af41cd.s1.eu.hivemq.cloud:8883";
const char *mqtt_username = "Arduino";
const char *mqtt_password = "lalala99";

MQTTClient mqtt_client;
MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
MQTTClient_SSLOptions ssl_opts = MQTTClient_SSLOptions_initializer;

struct SharedData
{
    float previous_temperature;
    float temperature;
    int light_level;
    pthread_mutex_t data_mutex;
    int client_sock;
    int light_preference;
    float temp_preference;
};

struct SystemState
{
    int blinds_state;
    int lights_state;
    int heater_state;
    pthread_mutex_t state_mutex;
};

struct SharedData shared_data;
struct SystemState system_state;

void initialize_preferences(struct SharedData *data)
{
    pthread_mutex_lock(&data->data_mutex);
    data->light_preference = 2000; // Default light level
    data->temp_preference = 20.0;  // Default temperature in Celsius
    data->previous_temperature = 0;
    pthread_mutex_unlock(&data->data_mutex);
}

void add_ms_to_timespec(struct timespec *ts, long ms)
{
    ts->tv_sec += ms / 1000;
    ts->tv_nsec += (ms % 1000) * 1000000;
    if (ts->tv_nsec >= 1000000000)
    {
        ts->tv_sec += 1;
        ts->tv_nsec -= 1000000000;
    }
}

void set_thread_priority(pthread_t thread, int priority)
{
    struct sched_param param;
    param.sched_priority = priority;
    pthread_setschedparam(thread, SCHED_FIFO, &param);
}

void *verify_data_task(void *arg)
{
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);

    while (1)
    {
        struct timespec start, end;
        clock_gettime(CLOCK_MONOTONIC, &start);

        char buffer[BUFFER_SIZE] = {0};
        int bytes_read = recv(shared_data.client_sock, buffer, BUFFER_SIZE - 1, 0);

        if (bytes_read > 0)
        {
            float temp;
            int light;
            if (sscanf(buffer, "%f,%d", &temp, &light) == 2)
            {
                pthread_mutex_lock(&shared_data.data_mutex);
                shared_data.previous_temperature = shared_data.temperature;
                shared_data.temperature = temp;
                shared_data.light_level = light;
                pthread_mutex_unlock(&shared_data.data_mutex);

                printf("Received - Temperature: %.1f°C, Light: %d\n", temp, light);
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &end);
        long elapsed_us = (end.tv_sec - start.tv_sec) * 1000000 +
                          (end.tv_nsec - start.tv_nsec) / 1000;
        if (elapsed_us < VERIFY_DATA_EXECUTION * 1000)
        {
            usleep(VERIFY_DATA_EXECUTION * 1000 - elapsed_us);
        }

        add_ms_to_timespec(&next_period, VERIFY_DATA_PERIOD);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }
    return NULL;
}

void *command_server_task(void *arg)
{
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);

    while (1)
    {
        struct timespec start, end;
        clock_gettime(CLOCK_MONOTONIC, &start);

        pthread_mutex_lock(&shared_data.data_mutex);
        float temp = shared_data.temperature;
        int light = shared_data.light_level;
        pthread_mutex_unlock(&shared_data.data_mutex);

        pthread_mutex_lock(&system_state.state_mutex);

        // Blinds control
        if (light < shared_data.light_preference + 1000 && system_state.blinds_state != 0)
        {
            send(shared_data.client_sock, "BLINDS_OPEN\n", strlen("BLINDS_OPEN\n"), 0);
            printf("Sent command: BLINDS_OPEN\n");
            system_state.blinds_state = 0;
            sendAlertToMQTT("/comcs/g12012/alerts", "Blinds are opening!");
        }
        else if (light >= shared_data.light_preference + 1000 && system_state.blinds_state != 1)
        {
            send(shared_data.client_sock, "BLINDS_CLOSE\n", strlen("BLINDS_CLOSE\n"), 0);
            printf("Sent command: BLINDS_CLOSE\n");
            system_state.blinds_state = 1;
            sendAlertToMQTT("/comcs/g12012/alerts", "Blinds are closing!");
        }

        // Lights control
        if (light < shared_data.light_preference && system_state.lights_state != 1)
        {
            send(shared_data.client_sock, "LIGHT_ON\n", strlen("LIGHT_ON\n"), 0);
            printf("Sent command: LIGHT_ON\n");
            system_state.lights_state = 1;
            sendAlertToMQTT("/comcs/g12012/alerts", "Lights are on!");
        }
        else if (light >= shared_data.light_preference && system_state.lights_state != 0)
        {
            send(shared_data.client_sock, "LIGHT_OFF\n", strlen("LIGHT_OFF\n"), 0);
            printf("Sent command: LIGHT_OFF\n");
            system_state.lights_state = 0;
            sendAlertToMQTT("/comcs/g12012/alerts", "Lights are off!");
        }

        // Heater control
        if (temp < shared_data.temp_preference && system_state.heater_state != 1)
        {
            send(shared_data.client_sock, "HEATER_ON\n", strlen("HEATER_ON\n"), 0);
            printf("Sent command: HEATER_ON\n");
            system_state.heater_state = 1;
            sendAlertToMQTT("/comcs/g12012/alerts", "Heater is on!");
        }
        else if (temp >= shared_data.temp_preference && system_state.heater_state != 0)
        {
            send(shared_data.client_sock, "HEATER_OFF\n", strlen("HEATER_OFF\n"), 0);
            printf("Sent command: HEATER_OFF\n");
            system_state.heater_state = 0;
            sendAlertToMQTT("/comcs/g12012/alerts", "Heater is off!");
        }

        pthread_mutex_unlock(&system_state.state_mutex);

        clock_gettime(CLOCK_MONOTONIC, &end);
        long elapsed_us = (end.tv_sec - start.tv_sec) * 1000000 +
                          (end.tv_nsec - start.tv_nsec) / 1000;
        if (elapsed_us < SERVER_CAPACITY * 1000)
        {
            usleep(SERVER_CAPACITY * 1000 - elapsed_us);
        }

        add_ms_to_timespec(&next_period, SERVER_PERIOD);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }
    return NULL;
}

char *toSmartData(float temp, float humid)
{
    // Variables for time
    time_t currentTime;
    struct tm *timeInfo;
    char timeString[30]; // Buffer for formatted time

    // Get the current time and format it as ISO 8601 (e.g., "2025-01-01T12:34:56Z")
    time(&currentTime);
    timeInfo = gmtime(&currentTime); // Use gmtime for UTC time or localtime for local time
    strftime(timeString, sizeof(timeString), "%Y-%m-%dT%H:%M:%SZ", timeInfo);

    // Convert temperature and humidity to strings with 2 decimal places
    char tempStr[10];
    char humidStr[10];
    snprintf(tempStr, sizeof(tempStr), "%.2f", temp);
    snprintf(humidStr, sizeof(humidStr), "%.2f", humid);

    // Define the location
    const char *location = "Porto";

    // Create the JSON-like payload
    char *payload = (char *)malloc(512); // Allocate enough memory for the payload
    if (payload == NULL)
    {
        fprintf(stderr, "Memory allocation failed\n");
        return NULL;
    }

    snprintf(payload, 512,
             "{ \"id\": \"%s_%ld\", \"dateObserved\": \"%s\", \"location\": \"%s\", \"temperature\": \"%s\", \"illuminance\": \"%s\" }",
             location, currentTime, timeString, location, tempStr, humidStr);

    return payload;
}

void sendAlertToMQTT(const char *topic, const char *message)
{
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    pubmsg.payload = (void *)message;
    pubmsg.payloadlen = (int)strlen(message);
    pubmsg.qos = QOS;
    pubmsg.retained = 0;
    int err = MQTTClient_publishMessage(mqtt_client, topic, &pubmsg, NULL);
    if (err != MQTTCLIENT_SUCCESS)
    {
        fprintf(stderr, "Failed to publish MQTT message, error code: %d\n\n", err);
    }
}

int messageArrived(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    (void)context;  // Suppress unused parameter warning
    (void)topicLen; // Suppress unused parameter warning

    if (strcmp(topicName, "/comcs/g12012/tempPreference") == 0)
    {
        pthread_mutex_lock(&shared_data.data_mutex);
        shared_data.temp_preference = atof((char *)message->payload);
        pthread_mutex_unlock(&shared_data.data_mutex);
        printf("Temperature preference has changed to ->  %.1f\n", shared_data.temp_preference);
    }
    else if (strcmp(topicName, "/comcs/g12012/lumPreference") == 0)
    {
        pthread_mutex_lock(&shared_data.data_mutex);
        shared_data.light_preference = atoi((char *)message->payload);
        pthread_mutex_unlock(&shared_data.data_mutex);
        printf("Light preference has changed to -> %d\n", shared_data.light_preference);
    }

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

void *preference_detection_task(void *arg)
{
    (void)arg; // Suppress unused parameter warning
    while (1)
    {
        MQTTClient_yield();
    }
    return NULL;
}

void *mqtt_task(void *arg)
{
    (void)arg; // Suppress unused parameter warning
    while (1)
    {
        pthread_mutex_lock(&shared_data.data_mutex);
        float temp = shared_data.temperature;
        int light = shared_data.light_level;
        pthread_mutex_unlock(&shared_data.data_mutex);

        char *payload = toSmartData(temp, light);
        if (payload != NULL)
        {
            sendAlertToMQTT("/comcs/g12012/data", payload);
            free(payload);
        }
        usleep(5000000);
    }
    return NULL;
}
void sensor_verification_task(void *arg)
{
    (void)arg; // Suppress unused parameter warning
    while (1)
    {
        // verify if the current and the previous temperatures have significant difference
        pthread_mutex_lock(&shared_data.data_mutex);

        float temp = shared_data.temperature;
        float previous_temp = shared_data.previous_temperature;

        // if the temperature difference is greater than 1.5 degrees, send an alert

        if ((temp - previous_temp) > 1.5 || (previous_temp - temp) > 1.5)
        {
            sendAlertToMQTT("/comcs/g12012/alerts", "Sensor is experiencing high temperature disparity! Something may be wrong with the sensor.");
        }

        pthread_mutex_unlock(&shared_data.data_mutex);

        usleep(5000000);
    }
    return NULL;
}

int main()
{

    initialize_preferences(&shared_data);

    MQTTClient_create(&mqtt_client, mqtt_server, MQTT_CLIENT_ID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    conn_opts.username = mqtt_username;
    conn_opts.password = mqtt_password;
    ssl_opts.enableServerCertAuth = 1;
    conn_opts.ssl = &ssl_opts;

    MQTTClient_setCallbacks(mqtt_client, NULL, NULL, messageArrived, NULL);

    int err = MQTTClient_connect(mqtt_client, &conn_opts);
    if (err != MQTTCLIENT_SUCCESS)
    {
        fprintf(stderr, "Failed to connect to MQTT broker, error code: %d\n\n", err);
        exit(EXIT_FAILURE);
    }
    else
    {
        puts("Connected to MQTT broker successfully.");
    }

    int subscribeTemperature = MQTTClient_subscribe(mqtt_client, "/comcs/g12012/tempPreference", QOS);
    if (subscribeTemperature != MQTTCLIENT_SUCCESS)
    {
        fprintf(stderr, "Failed to subscribe to MQTT topic, error code: %d\n\n", err);
        exit(EXIT_FAILURE);
    }
    else
    {
        puts("Subscribed to MQTT topic successfully.");
    }

    int subscribeLuminosity = MQTTClient_subscribe(mqtt_client, "/comcs/g12012/lumPreference", QOS);
    if (subscribeLuminosity != MQTTCLIENT_SUCCESS)
    {
        fprintf(stderr, "Failed to subscribe to MQTT topic, error code: %d\n\n", err);
        exit(EXIT_FAILURE);
    }
    else
    {
        puts("Subscribed to MQTT topic successfully.");
    }

    pthread_mutex_init(&shared_data.data_mutex, NULL);
    pthread_mutex_init(&system_state.state_mutex, NULL);

    system_state.blinds_state = 0;
    system_state.lights_state = 0;
    system_state.heater_state = 0;

    int server_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(struct sockaddr_in);

    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0)
    {
        perror("Socket creation failed");
        return EXIT_FAILURE;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(SERVER_PORT);

    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("Binding failed");
        return EXIT_FAILURE;
    }

    listen(server_sock, 1);
    printf("Waiting for client connection...\n");

    shared_data.client_sock = accept(server_sock,
                                     (struct sockaddr *)&client_addr,
                                     &addr_len);
    printf("Client connected.\n");

    pthread_t verify_thread, server_thread, mqtt_thread, sensor_verification_thread, preference_detection_thread;

    pthread_create(&verify_thread, NULL, verify_data_task, NULL);
    pthread_create(&server_thread, NULL, command_server_task, NULL);
    pthread_create(&mqtt_thread, NULL, mqtt_task, NULL);
    pthread_create(&preference_detection_thread, NULL, preference_detection_task, NULL);
    pthread_create(&sensor_verification_thread, NULL, sensor_verification_task, NULL);

    set_thread_priority(verify_thread, VERIFY_DATA_PRIORITY);
    set_thread_priority(server_thread, SERVER_PRIORITY);
    set_thread_priority(mqtt_thread, MQTT_PRIORITY);
    set_thread_priority(preference_detection_thread, PREFERENCE_DETECTION_PRIORITY);
    set_thread_priority(sensor_verification_thread, SENSOR_VERIFICATION_PRIORITY);

    pthread_join(verify_thread, NULL);
    pthread_join(server_thread, NULL);
    pthread_join(mqtt_thread, NULL);
    pthread_join(preference_detection_thread, NULL);
    pthread_join(sensor_verification_thread, NULL);

    close(shared_data.client_sock);
    close(server_sock);
    pthread_mutex_destroy(&shared_data.data_mutex);
    pthread_mutex_destroy(&system_state.state_mutex);

    MQTTClient_disconnect(mqtt_client, TIMEOUT);
    MQTTClient_destroy(&mqtt_client);

    return 0;
}