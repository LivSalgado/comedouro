#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwipopts.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/err.h" // Inclui definições de erros do lwIP
#include "dht.h" // Include DHT11 library
#include "tusb.h"
#include "common.h"
#include "hx711.h"

#define PRINT_ARR(arr, len)                                                                    \
  do {                                                                                         \
    for (size_t i = 0; i < len; ++i) {                                                         \
      printf("hx711_multi_t chip %i: %li\n", i, arr[i]);                                      \
    }                                                                                          \
  } while (0)

// Calibration values (adjust these based on your load cell and setup)
float LOAD_CELL_ZERO_OFFSET = 0.0f; // Value when there's no weight
float LOAD_CELL_SCALE_FACTOR = 1.0f; // Adjust this to convert readings to grams

// HX711 pins (adjust if needed)
#define HX711_CLOCK_PIN 14
#define HX711_DATA_PIN 15

// Number of readings for averaging during calibration
#define NUM_CALIBRATION_READINGS 10

// Known weight for calibration (in grams)
#define KNOWN_WEIGHT 400.0f

// HC-SR04 Sensor Pins
#define TRIGGER_PIN 17
#define ECHO_PIN 16

// Servo Pins and Configuration
#define SERVO_PIN 19
#define PWM_FREQUENCY 50
#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500

// Distance Thresholds
#define CLOSE_DISTANCE 50.0f  // cm
#define FAR_DISTANCE 100.0f  // cm

// Servo states
typedef enum {
    SERVO_0_DEG,
    SERVO_180_DEG
} servo_state;

// --- Configurações ---
#define WIFI_SSID "IJKL_QUARTOS"
#define WIFI_PASSWORD "31290304"
#define API_HOST "192.168.0.29"
#define API_PORT 5000
#define API_ENDPOINT "/dados"
#define CONNECTION_TIMEOUT_MS 10000 // Timeout para a conexão TCP
#define RETRY_INTERVAL_MS 5000 // Intervalo entre tentativas de reconexão
#define DNS_TIMEOUT_MS 5000 // Timeout para a resolução DNS

// --- Estrutura para dados do cliente ---
typedef struct {
    struct tcp_pcb *pcb;
    bool connected;
    bool request_sent;
    ip_addr_t server_ip; // Armazena o IP do servidor
    char post_data[256];
    char request_buffer[512];
} ClientData;

static ClientData client_data = {NULL, false, false, {0}, "", ""}; // Inicializa a estrutura

// Function declarations
uint64_t get_pulse_duration();
float get_distance_cm();
void set_servo_angle(float angle);
void setup();
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_client_err(void *arg, err_t err);
static err_t send_post_request(ClientData *client);
static void connect_to_server(ClientData *client);
static void dns_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg);
static bool resolve_dns(ClientData *client);
static void close_connection(ClientData *client);
bool connect_to_wifi();
void print_ip_address();
float read_weight(hx711_t *hx);
void tare_scale(hx711_t *hx);
void calibrate_scale_factor(hx711_t *hx);

// DHT11 Sensor Configuration
static const dht_model_t DHT_MODEL = DHT11;
static const uint DHT_DATA_PIN = 18; // Changed DATA_PIN to DHT_DATA_PIN to avoid conflicts

// Wi-Fi Configuration (Replace with your credentials)
#define WIFI_SSID "IJKL_QUARTOS"  // Replace with your Wi-Fi SSID
#define WIFI_PASSWORD "31290304" // Replace with your Wi-Fi password

// Function to connect to Wi-Fi
bool connect_to_wifi() {
    printf("Connecting to Wi-Fi...\n");
    int retries = 0;
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) {
        printf("Wi-Fi connection failed. Retrying...\n");
        retries++;
        if (retries > 5) {
            printf("Failed to connect after multiple retries.\n");
            return false;
        }
    }

    printf("Connected to Wi-Fi\n");
    return true;
}

// Function to get the IP address
void print_ip_address() {
    struct netif *netif = netif_default; // Get the default network interface

    if (netif != NULL) {
        ip4_addr_t *ip4 = netif_ip4_addr(netif); // Get the IPv4 address pointer
        printf("IP Address: %s\n", ip4addr_ntoa(ip4)); // Print the IP address
    } else {
        printf("No network interface found.\n");
    }
}

// --- Callback para resolução DNS assíncrona ---
static void dns_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
    ClientData *client = (ClientData *)callback_arg;

    if (ipaddr != NULL) {
        client->server_ip = *ipaddr;
        printf("Endereço IP resolvido: %s\n", ipaddr_ntoa(ipaddr));
        // Agora que temos o IP, podemos conectar
        connect_to_server(client);
    } else {
        printf("Falha na resolução DNS.\n");
        close_connection(client); // Fecha qualquer conexão existente
    }
}

// --- Função para resolver o DNS (agora assíncrona) ---
static bool resolve_dns(ClientData *client) {
    client->server_ip.addr = 0; // Reseta o IP
    err_t dns_err = dns_gethostbyname(API_HOST, &client->server_ip, dns_callback, client);

    if (dns_err == ERR_OK) {
        // IP já resolvido (improvável, mas possível)
        printf("Endereço IP já resolvido (cache).\n");
        return true;
    } else if (dns_err == ERR_INPROGRESS) {
        // Resolução em andamento.  Esperar no loop principal.
        printf("Resolução DNS em andamento...\n");
        return false;
    } else {
        printf("Falha na resolução DNS: %d\n", dns_err);
        return false;
    }
}

// --- Função para fechar a conexão TCP ---
static void close_connection(ClientData *client) {
    if (client->pcb) {
        tcp_arg(client->pcb, NULL);
        tcp_recv(client->pcb, NULL);
        tcp_err(client->pcb, NULL);
        tcp_close(client->pcb);
        client->pcb = NULL;
    }
    client->connected = false;
    client->request_sent = false;
}

// --- Callbacks lwIP (agora usando a estrutura ClientData) ---

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    ClientData *client = (ClientData *)arg;

    if (err == ERR_OK) {
        printf("Conectado ao servidor!\n");
        client->connected = true;
        client->pcb = tpcb; // Atualiza o PCB na estrutura
        return send_post_request(client); // Envia o request imediatamente após conectar
    } else {
        printf("Erro ao conectar: %d\n", err);
        close_connection(client);
        return err;
    }
}

static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    ClientData *client = (ClientData *)arg;

    if (err == ERR_OK && p != NULL) {
        printf("Resposta recebida:\n");
        char *data = (char *)p->payload;
        for (int i = 0; i < p->len; i++) {
            putchar(data[i]);
        }
        printf("\n");

        tcp_recved(tpcb, p->len);
        pbuf_free(p);
        close_connection(client); // Fecha após receber a resposta
    } else if (p == NULL) {
        printf("Servidor fechou a conexão.\n");
        close_connection(client);
    } else {
        printf("Erro ao receber dados: %d\n", err);
        if (p != NULL) {
            pbuf_free(p);
        }
        close_connection(client);
    }
    return ERR_OK;
}

static void tcp_client_err(void *arg, err_t err) {
    ClientData *client = (ClientData *)arg;
    printf("Erro TCP: %d\n", err);
    close_connection(client);
}

// --- Função para enviar a requisição POST (agora usando ClientData) ---
static err_t send_post_request(ClientData *client) {
    if (!client->connected || client->pcb == NULL) {
        printf("Não conectado para enviar a requisição.\n");
        return ERR_CONN;
    }

    snprintf(client->request_buffer, sizeof(client->request_buffer),
             "POST %s HTTP/1.1\r\n"
             "Host: %s:%d\r\n"
             "Content-Type: application/json\r\n"
             "Content-Length: %d\r\n"
             "Connection: close\r\n"
             "\r\n"
             "%s",
             API_ENDPOINT, API_HOST, API_PORT, (int)strlen(client->post_data), client->post_data);

    err_t err = tcp_write(client->pcb, client->request_buffer, strlen(client->request_buffer), TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        printf("Erro ao enviar dados: %d\n", err);
        close_connection(client); // Fecha em caso de erro
        return err;
    }

    err = tcp_output(client->pcb);
    if (err != ERR_OK) {
        printf("Erro tcp_output: %d\n", err);
        close_connection(client); // Fecha em caso de erro
        return err;
    }
    client->request_sent = true;
    return ERR_OK;
}

// --- Função para conectar ao servidor (agora usando ClientData) ---
static void connect_to_server(ClientData *client) {
    if (client->pcb != NULL) {
        // Já existe uma conexão (ou tentativa), não faz nada
        return;
    }

    client->pcb = tcp_new();
    if (client->pcb == NULL) {
        printf("Erro ao criar PCB\n");
        return;
    }

    tcp_arg(client->pcb, client); // Passa a estrutura ClientData para os callbacks
    tcp_err(client->pcb, tcp_client_err);
    tcp_recv(client->pcb, tcp_client_recv);

    // Usa o IP resolvido
    err_t err = tcp_connect(client->pcb, &client->server_ip, API_PORT, tcp_client_connected);
    if (err != ERR_OK) {
        printf("Erro ao iniciar a conexão: %d\n", err);
        close_connection(client);
    }
}

// Function to read the weight and apply calibration
float read_weight(hx711_t *hx) {
  int32_t raw_value = hx711_get_value(hx);
  printf("Raw Value: %ld\n", raw_value); // Imprime o valor bruto
  float weight = (((float)(raw_value - LOAD_CELL_ZERO_OFFSET) / LOAD_CELL_SCALE_FACTOR) / 2.0) - KNOWN_WEIGHT;
  return weight;
}

// Function to perform tare calibration
void tare_scale(hx711_t *hx) {
  printf("Taring scale. Remove all weight.\n");
  sleep_ms(3000); // Give time to remove weight
  int32_t zero_sum = 0;
  for (int i = 0; i < NUM_CALIBRATION_READINGS; ++i) {
    zero_sum += hx711_get_value(hx);
    sleep_ms(50);
  }
  LOAD_CELL_ZERO_OFFSET = (float)zero_sum / NUM_CALIBRATION_READINGS; // Get the average
  printf("New zero offset: %f\n", LOAD_CELL_ZERO_OFFSET);
}

// Function to perform scale factor calibration
void calibrate_scale_factor(hx711_t *hx) {
  printf("Place a known weight (%f g) on the load cell.\n", KNOWN_WEIGHT);
  sleep_ms(5000); // Give time to place weight

  int32_t weight_sum = 0;
  for (int i = 0; i < NUM_CALIBRATION_READINGS; ++i) {
    weight_sum += hx711_get_value(hx);
    sleep_ms(50);
  }
  float weighted_value = (float)weight_sum / NUM_CALIBRATION_READINGS;

  LOAD_CELL_SCALE_FACTOR = (weighted_value - LOAD_CELL_ZERO_OFFSET) / KNOWN_WEIGHT;
  printf("New scale factor: %f\n", LOAD_CELL_SCALE_FACTOR);
}

// Function to setup hardware
void setup() {
    gpio_init(TRIGGER_PIN);
    gpio_init(ECHO_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    // Configure the GPIO pin for PWM
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config config = pwm_get_default_config();
    float clk_div = (float)clock_get_hz(clk_sys) / (PWM_FREQUENCY * 20000);
    pwm_init(slice_num, &config, false);
    pwm_set_wrap(slice_num, 20000);
    pwm_set_clkdiv(slice_num, clk_div);
    pwm_set_enabled(slice_num, true);
}

// Get pulse duration from the HC-SR04
uint64_t get_pulse_duration() {
    // Send a 10-microsecond pulse on the TRIGGER pin
    gpio_put(TRIGGER_PIN, 1);
    sleep_us(10);
    gpio_put(TRIGGER_PIN, 0);

    uint64_t start_time = time_us_64();
    // Wait for the ECHO pin to go high
    while (gpio_get(ECHO_PIN) == 0);
    // Wait for the ECHO pin to go low
    while (gpio_get(ECHO_PIN) == 1);
    uint64_t end_time = time_us_64();

    return end_time - start_time;
}

// Calculate distance in cm using pulse duration
float get_distance_cm() {
    uint64_t pulse_duration = get_pulse_duration();
    float distance_cm = (pulse_duration * 0.0343) / 2;
    return distance_cm;
}

// Function to set the servo angle
void set_servo_angle(float angle) {
    if (angle > 180) angle = 180;
    if (angle < 0) angle = 0;

    uint32_t pulse_width = MIN_PULSE_WIDTH + (uint32_t)((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * (angle / 180.0f));

    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(SERVO_PIN), pulse_width);
}

int main() {
    stdio_init_all();

    // Initialize the CYW43 driver
    if (cyw43_arch_init()) {
        printf("Failed to initialise cyw43_arch\n");
        return -1;
    }

    // Enable Wi-Fi station mode
    cyw43_arch_enable_sta_mode();

    // Connect to Wi-Fi
    if (!connect_to_wifi()) {
        printf("Failed to connect to Wi-Fi. Halting.\n");
        return -1;
    }

    // Get and print the IP address
    print_ip_address();

    while (!tud_cdc_connected()) {
        sleep_ms(1);
    }

    printf("HX711 Balanca - Raspberry Pi Pico\n");

    hx711_config_t hxcfg;
    hx711_get_default_config(&hxcfg);

    hxcfg.clock_pin = HX711_CLOCK_PIN;
    hxcfg.data_pin = HX711_DATA_PIN;

    hx711_t hx;

    // 1. Initialise
    hx711_init(&hx, &hxcfg);

    // 2. Power up the hx711 and set gain on chip
    hx711_power_up(&hx, hx711_gain_128);

    // 4. Wait for readings to settle
    hx711_wait_settle(hx711_rate_80);

    // Calibrate the scale
    tare_scale(&hx);
    calibrate_scale_factor(&hx);

    printf("Ready to measure weight!\n");

    setup(); // Initialize GPIO and PWM

    servo_state current_servo_state = SERVO_0_DEG;
    set_servo_angle(0); // Start at the 0 degree position

    uint64_t object_detected_time = 0; // Timestamp of when the object was detected
    const uint32_t return_delay_ms = 2000; // 1 minute delay in milliseconds

    // Initialize DHT11 sensor
    dht_t dht;
    dht_init(&dht, DHT_MODEL, pio0, DHT_DATA_PIN, true /* pull_up */);

    uint32_t last_retry_time = 0; // Controla o tempo da última tentativa

    while (true) {
        // Read the weight
        float weight = read_weight(&hx);

        // Print the weight to the serial console
         printf("Weight: %.2f g\n", weight); // Adjust unit as needed

        // Small delay
        sleep_ms(100);

        float distance = get_distance_cm();
        printf("Distance: %.2f cm\n", distance);

        // DHT11 sensor reading
        dht_start_measurement(&dht);
        float humidity;
        float temperature_c;
        dht_result_t result = dht_finish_measurement_blocking(&dht, &humidity, &temperature_c);

        if (result == DHT_RESULT_OK) {
            printf("Temperature: %.1f C, Humidity: %.1f%%\n", temperature_c, humidity); // Print in Celsius
        } else if (result == DHT_RESULT_TIMEOUT) {
            printf("DHT sensor not responding. Please check your wiring.\n");
        } else {
            printf("Bad DHT checksum\n");
        }

        if (distance < CLOSE_DISTANCE) {
            if (current_servo_state != SERVO_180_DEG) {
                set_servo_angle(180);
                current_servo_state = SERVO_180_DEG;
                object_detected_time = to_ms_since_boot(get_absolute_time()); // Record the time of detection
            }
        } else {
            // Check if enough time has passed since the object was detected
            if (current_servo_state == SERVO_180_DEG &&
                to_ms_since_boot(get_absolute_time()) - object_detected_time >= return_delay_ms) {
                set_servo_angle(0);
                current_servo_state = SERVO_0_DEG;
            }
        }

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(500);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(500); // Take readings more frequently

        // Prepara os dados para o POST
        snprintf(client_data.post_data, sizeof(client_data.post_data),
                 "{\"weight\": %.2f, \"temperature\": %.1f, \"humidity\": %.1f}",
                 weight, temperature_c, humidity);

        if (!client_data.connected) {
            // Tenta resolver o DNS e conectar (assíncrono)
            if (client_data.server_ip.addr == 0) { // Se o IP ainda não foi resolvido
                if (!resolve_dns(&client_data)) {
                    // Espera a resolução DNS, ou um timeout
                    uint32_t start_time = time_us_32();
                    while(client_data.server_ip.addr == 0 && (time_us_32() - start_time < DNS_TIMEOUT_MS * 1000)) {
                        cyw43_arch_poll();
                        sleep_ms(10);
                    }
                    if(client_data.server_ip.addr == 0){
                        printf("Timeout na resolução DNS.\n");
                        // Tratar o timeout (tentar novamente, por exemplo)
                        if (time_us_32() - last_retry_time > RETRY_INTERVAL_MS * 1000) {
                            printf("Tentando novamente...\n");
                            last_retry_time = time_us_32();
                        }
                    }
                }
            } else {
                // Se o IP já foi resolvido, tenta conectar
                connect_to_server(&client_data);
                 // Adiciona um timeout para a conexão
                uint32_t start_time = time_us_32();
                while (!client_data.connected && (time_us_32() - start_time < CONNECTION_TIMEOUT_MS * 1000)) {
                    cyw43_arch_poll();
                    sleep_ms(10);
                }
                if (!client_data.connected) {
                    printf("Timeout na conexão TCP.\n");
                    close_connection(&client_data); // Fecha a conexão em caso de timeout
                    // Tratar o timeout (tentar novamente, por exemplo)
                    if (time_us_32() - last_retry_time > RETRY_INTERVAL_MS * 1000) {
                        printf("Tentando novamente...\n");
                        last_retry_time = time_us_32();
                        client_data.server_ip.addr = 0; // Força uma nova resolução DNS
                    }
                }
            }
        }
        cyw43_arch_poll();
        sleep_ms(10);

    }
    cyw43_arch_deinit();
    return 0;
}