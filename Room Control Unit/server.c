#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <ctype.h>
#include <sys/time.h>

#define SERVER_PORT 8080
#define BUFFER_SIZE 1024

// State tracking structure (same as before)
struct SystemState
{
    int blinds_state; // 0 = open, 1 = closed
    int lights_state; // 0 = off, 1 = on
    int heater_state; // 0 = off, 1 = on
};

// Data verification function with timing
int verify_data(const char *buffer, float *temperature, int *ldr_value)
{
    struct timeval start, end;
    gettimeofday(&start, NULL);

    // Existing verification logic
    char trimmed[BUFFER_SIZE];
    strncpy(trimmed, buffer, BUFFER_SIZE - 1);
    trimmed[BUFFER_SIZE - 1] = '\0';

    char *newline = strchr(trimmed, '\r');
    if (newline)
        *newline = '\0';
    newline = strchr(trimmed, '\n');
    if (newline)
        *newline = '\0';

    char *token = strtok(trimmed, ",");
    if (token == NULL)
    {
        printf("Invalid data format: No comma found\n");
        return 0;
    }

    *temperature = atof(token);

    if (*temperature == 0.0 && strcmp(token, "0") != 0)
    {
        printf("Invalid temperature value\n");
        return 0;
    }

    token = strtok(NULL, ",");
    if (token == NULL)
    {
        printf("Invalid data format: Missing LDR value\n");
        return 0;
    }

    *ldr_value = atoi(token);

    if (*ldr_value == 0 && strcmp(token, "0") != 0)
    {
        printf("Invalid LDR value\n");
        return 0;
    }

    gettimeofday(&end, NULL);
    long data_verification_time = (end.tv_sec - start.tv_sec) * 1000000 +
                                  (end.tv_usec - start.tv_usec);

    printf("Data Verification Time: %ld microseconds\n", data_verification_time);

    return 1;
}

// Command generation function with timing
int generate_commands(float temperature, int ldr_value,
                      struct SystemState *current_state,
                      char *commands, int commands_size)
{
    struct timeval start, end;
    gettimeofday(&start, NULL);

    // Existing command generation logic (from previous version)
    int command_len = 0;
    int state_changed = 0;

    // Blinds Control
    if (ldr_value < 3000)
    {
        if (current_state->blinds_state != 0)
        {
            command_len += snprintf(commands + command_len,
                                    commands_size - command_len,
                                    "BLINDS_OPEN\n");
            current_state->blinds_state = 0;
            state_changed = 1;
        }
    }
    else if (ldr_value >= 3000)
    {
        if (current_state->blinds_state != 1)
        {
            command_len += snprintf(commands + command_len,
                                    commands_size - command_len,
                                    "BLINDS_CLOSE\n");
            current_state->blinds_state = 1;
            state_changed = 1;
        }
    }

    // Lights Control
    if (ldr_value < 2000)
    {
        if (current_state->lights_state != 1)
        {
            command_len += snprintf(commands + command_len,
                                    commands_size - command_len,
                                    "LIGHT_ON\n");
            current_state->lights_state = 1;
            state_changed = 1;
        }
    }
    else if (ldr_value >= 2000)
    {
        if (current_state->lights_state != 0)
        {
            command_len += snprintf(commands + command_len,
                                    commands_size - command_len,
                                    "LIGHT_OFF\n");
            current_state->lights_state = 0;
            state_changed = 1;
        }
    }

    // Heater Control
    if (temperature < 20.0)
    {
        if (current_state->heater_state != 1)
        {
            command_len += snprintf(commands + command_len,
                                    commands_size - command_len,
                                    "HEATER_ON\n");
            current_state->heater_state = 1;
            state_changed = 1;
        }
    }
    else
    {
        if (current_state->heater_state != 0)
        {
            command_len += snprintf(commands + command_len,
                                    commands_size - command_len,
                                    "HEATER_OFF\n");
            current_state->heater_state = 0;
            state_changed = 1;
        }
    }

    gettimeofday(&end, NULL);
    long command_generation_time = (end.tv_sec - start.tv_sec) * 1000000 +
                                   (end.tv_usec - start.tv_usec);

    printf("Command Generation Time: %ld microseconds\n", command_generation_time);

    return state_changed ? command_len : 0;
}

void handle_client(int client_sock)
{
    char buffer[BUFFER_SIZE];
    struct SystemState current_state = {0, 0, 0};

    while (1)
    {
        struct timeval start_receive, end_receive;
        gettimeofday(&start_receive, NULL);

        memset(buffer, 0, BUFFER_SIZE);
        int bytes_read = recv(client_sock, buffer, BUFFER_SIZE - 1, 0);

        gettimeofday(&end_receive, NULL);
        long data_receive_time = (end_receive.tv_sec - start_receive.tv_sec) * 1000000 +
                                 (end_receive.tv_usec - start_receive.tv_usec);

        printf("Data Receive Time: %ld microseconds\n", data_receive_time);

        if (bytes_read <= 0)
        {
            printf("Client disconnected or error.\n");
            break;
        }

        buffer[bytes_read] = '\0';
        printf("Raw data received: '%s'\n", buffer);

        // Verify data
        float temperature;
        int ldr_value;
        if (!verify_data(buffer, &temperature, &ldr_value))
        {
            continue;
        }

        // Generate commands
        char commands[BUFFER_SIZE] = {0};
        int command_len = generate_commands(temperature, ldr_value,
                                            &current_state,
                                            commands,
                                            sizeof(commands));

        // Send commands if any
        if (command_len > 0)
        {
            struct timeval start_send, end_send;
            gettimeofday(&start_send, NULL);

            if (send(client_sock, commands, command_len, 0) < 0)
            {
                perror("Send failed");
                break;
            }

            gettimeofday(&end_send, NULL);
            long command_send_time = (end_send.tv_sec - start_send.tv_sec) * 1000000 +
                                     (end_send.tv_usec - start_send.tv_usec);

            printf("Command Send Time: %ld microseconds\n", command_send_time);
            printf("Sent commands to client: %s", commands);
        }
    }
}

int main()
{
    int server_sock, client_sock;
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

    client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &addr_len);
    if (client_sock < 0)
    {
        perror("Connection failed");
        return EXIT_FAILURE;
    }

    printf("Client connected.\n");
    handle_client(client_sock);

    close(client_sock);
    close(server_sock);
    return EXIT_SUCCESS;
}
