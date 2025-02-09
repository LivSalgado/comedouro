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
#include "lwip/err.h" // Inclui definicoes de erros do lwIP
#include "dht.h" 

#include "tusb.h"
#include "common.h"
#include "hx711.h"

#define PRINT_ARR(arr, len)                                                                    \
  do {                                                                                         \
    for (size_t i = 0; i < len; ++i) {                                                         \
      printf("hx711_multi_t chip %i: %li\n", i, arr[i]);                                      \
    }                                                                                          \
  } while (0)

// Valores de calibracao (ajuste-os com base na sua celula de carga e configuracao)
float LOAD_CELL_ZERO_OFFSET = 0.0f; // Valor quando nao há peso
float LOAD_CELL_SCALE_FACTOR = 1.0f; // Ajuste isto para converter leituras em gramas

// Pinos HX711 (ajuste se necessario)
#define HX711_CLOCK_PIN 14
#define HX711_DATA_PIN 15

// Numero de leituras para calculo da media durante a calibracao
#define NUM_CALIBRATION_READINGS 10

// Peso conhecido para calibracao (em gramas)
#define KNOWN_WEIGHT 400.0f

// Pinos do sensor HC-SR04
#define TRIGGER_PIN 17
#define ECHO_PIN 16

// Pinos Servo e Configuracao
#define SERVO_PIN 19
#define PWM_FREQUENCY 50
#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500

// Limites de distancia
#define CLOSE_DISTANCE 50.0f // cm
#define FAR_DISTANCE 100.0f  // cm

// Servos estados
typedef enum {
  SERVO_0_DEG,
  SERVO_180_DEG
} servo_state;

// --- Configuracoes ---
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define API_HOST "192.168.0.29"
#define API_PORT 5000
#define API_ENDPOINT "/dados"
#define CONNECTION_TIMEOUT_MS 10000 // Timeout para a conexao TCP
#define RETRY_INTERVAL_MS 5000      // Intervalo entre tentativas de reconexao
#define DNS_TIMEOUT_MS 5000         // Timeout para a resolucao DNS

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

// Declaracoes de funcao
uint64_t get_pulse_duration();
float get_distance_cm();
void set_servo_angle(float angle);
float get_current_servo_angle();
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

// Configuracao do sensor DHT11
static const dht_model_t DHT_MODEL = DHT11;
static const uint DHT_DATA_PIN = 18; // DATA_PIN alterado para DHT_DATA_PIN para evitar conflitos

// Variável global para armazenar o ângulo atual do servo
static float current_servo_angle = 0.0f;

// Funcao para conectar-se ao Wi-Fi
bool connect_to_wifi() {
  printf("Conectando ao Wi-Fi...\n");
  int retries = 0;
  while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) {
    printf("A conexao Wi-Fi falhou. Tentando novamente..\n");
    retries++;
    if (retries > 5) {
      printf("Falha ao conectar para conectar após varias tentativas.\n");
      return false;
    }
  }

  printf("Conectado ao Wi-Fi\n");
  return true;
}

// Funcao para obter o endereco IP
void print_ip_address() {
  struct netif *netif = netif_default; // Obtenha a interface de rede padrao

  if (netif != NULL) {
    ip4_addr_t *ip4 = netif_ip4_addr(netif); // Obtenha o endereco IPv4
    printf("Endereco IP: %s\n", ip4addr_ntoa(ip4));
  } else {
    printf("Nenhuma interface de rede encontrada.\n");
  }
}

// --- Callback para resolucao DNS assincrona ---
static void dns_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
  ClientData *client = (ClientData *)callback_arg;

  if (ipaddr != NULL) {
    client->server_ip = *ipaddr;
    printf("Endereço IP resolvido: %s\n", ipaddr_ntoa(ipaddr));
    // Agora que temos o IP, podemos conectar
    connect_to_server(client);
  } else {
    printf("Falha na resolucao DNS.\n");
    close_connection(client); // Fecha qualquer conexao existente
  }
}

// --- Funcao para resolver o DNS (agora assíncrona ---
static bool resolve_dns(ClientData *client) {
  client->server_ip.addr = 0; // Reseta o IP
  err_t dns_err = dns_gethostbyname(API_HOST, &client->server_ip, dns_callback, client);

  if (dns_err == ERR_OK) {
    // IP já resolvido (improvável, mas possível)
    printf("Endereço IP já resolvido (cache).\n");
    return true;
  } else if (dns_err == ERR_INPROGRESS) {
    // Resolucao em andamento.  Esperar no loop principal.
    printf("Resolucao DNS em andamento...\n");
    return false;
  } else {
    printf("Falha na resolucao DNS: %d\n", dns_err);
    return false;
  }
}

// --- Funcao para fechar a conexao TCP ---
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
    printf("Servidor fechou a conexao.\n");
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

// --- Funcao para enviar a requisicao POST (agora usando ClientData) ---
static err_t send_post_request(ClientData *client) {
  if (!client->connected || client->pcb == NULL) {
    printf("nao conectado para enviar a requisicao.\n");
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

// --- Funcao para conectar ao servidor (agora usando ClientData) ---
static void connect_to_server(ClientData *client) {
  if (client->pcb != NULL) {
    // Já existe uma conexao (ou tentativa), nao faz nada
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
    printf("Erro ao iniciar a conexao: %d\n", err);
    close_connection(client);
  }
}

// Funcao para ler o peso e aplicar calibracao
float read_weight(hx711_t *hx) {
  int32_t raw_value = hx711_get_value(hx);
  printf("Valor bruto: %ld\n", raw_value); // Imprime o valor bruto
  float weight = (((float)(raw_value - LOAD_CELL_ZERO_OFFSET) / LOAD_CELL_SCALE_FACTOR) / 2.0) - KNOWN_WEIGHT;
  return weight;
}

// Funcao para realizar calibracao de tara
void tare_scale(hx711_t *hx) {
  printf("Escala de tara. Remova todo o peso.\n");
  sleep_ms(3000); // Dê tempo para remover peso
  int32_t zero_sum = 0;
  for (int i = 0; i < NUM_CALIBRATION_READINGS; ++i) {
    zero_sum += hx711_get_value(hx);
    sleep_ms(50);
  }
  LOAD_CELL_ZERO_OFFSET = (float)zero_sum / NUM_CALIBRATION_READINGS; // Obtenha a media
  printf("Novo zero offset: %f\n", LOAD_CELL_ZERO_OFFSET);
}

// Funcao para realizar calibracao do fator de escala
void calibrate_scale_factor(hx711_t *hx) {
  printf("Coloque um peso conhecido (%f g) na celula de carga.\n", KNOWN_WEIGHT);
  sleep_ms(5000); // De tempo para colocar peso

  int32_t weight_sum = 0;
  for (int i = 0; i < NUM_CALIBRATION_READINGS; ++i) {
    weight_sum += hx711_get_value(hx);
    sleep_ms(50);
  }
  float weighted_value = (float)weight_sum / NUM_CALIBRATION_READINGS;

  LOAD_CELL_SCALE_FACTOR = (weighted_value - LOAD_CELL_ZERO_OFFSET) / KNOWN_WEIGHT;
  printf("Novo fator de escala: %f\n", LOAD_CELL_SCALE_FACTOR);
}

// Funcao para configurar hardware
void setup() {
  gpio_init(TRIGGER_PIN);
  gpio_init(ECHO_PIN);
  gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
  gpio_set_dir(ECHO_PIN, GPIO_IN);

  // Configure o pino GPIO para PWM
  gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
  pwm_config config = pwm_get_default_config();
  float clk_div = (float)clock_get_hz(clk_sys) / (PWM_FREQUENCY * 20000);
  pwm_init(slice_num, &config, false);
  pwm_set_wrap(slice_num, 20000);
  pwm_set_clkdiv(slice_num, clk_div);
  pwm_set_enabled(slice_num, true);
}

// Obtenha a duracao do pulso do HC-SR04
uint64_t get_pulse_duration() {
  // Envie um pulso de 10 microssegundos no pino TRIGGER
  gpio_put(TRIGGER_PIN, 1);
  sleep_us(10);
  gpio_put(TRIGGER_PIN, 0);

  uint64_t start_time = time_us_64();
  // Espere o pino ECHO ficar alto
  while (gpio_get(ECHO_PIN) == 0)
    ;
  // Espere o pino ECHO ficar baixo
  while (gpio_get(ECHO_PIN) == 1)
    ;
  uint64_t end_time = time_us_64();

  return end_time - start_time;
}

// Calcule a distancia em cm usando a duracao do pulso
float get_distance_cm() {
  uint64_t pulse_duration = get_pulse_duration();
  float distance_cm = (pulse_duration * 0.0343) / 2;
  return distance_cm;
}

// Funcao para definir o angulo do servo
void set_servo_angle(float angle) {
  if (angle > 180)
    angle = 180;
  if (angle < 0)
    angle = 0;

  uint32_t pulse_width = MIN_PULSE_WIDTH + (uint32_t)((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * (angle / 180.0f));

  uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
  pwm_set_chan_level(slice_num, pwm_gpio_to_channel(SERVO_PIN), pulse_width);

  // Atualize a variavel global para refletir o angulo atual do servo
  current_servo_angle = angle;
}

// Funcao para obter o angulo atual do servo
float get_current_servo_angle() {
  return current_servo_angle;
}

int main() {
  stdio_init_all();

  // Inicialize o driver CYW43
  if (cyw43_arch_init()) {
    printf("Falha ao inicializar cyw43_arch\n");
    return -1;
  }

  // Ative o modo de estacao Wi-Fi
  cyw43_arch_enable_sta_mode();

  // Conecte-se ao Wi-Fi
  if (!connect_to_wifi()) {
    printf("Falha ao conectar ao Wi-Fi. Parando.\n");
    return -1;
  }

  // Obtenha e imprima o endereco IP
  print_ip_address();

  while (!tud_cdc_connected()) {
    sleep_ms(1);
  }

  printf("Balanca do comedouro inteligente iOT");

  hx711_config_t hxcfg;
  hx711_get_default_config(&hxcfg);

  hxcfg.clock_pin = HX711_CLOCK_PIN;
  hxcfg.data_pin = HX711_DATA_PIN;

  hx711_t hx;

  // 1. Inicializar
  hx711_init(&hx, &hxcfg);

  // 2. Ligue o hx711 e configure o ganho no chip
  hx711_power_up(&hx, hx711_gain_128);

  // 4. Espere que as leituras sejam resolvidas
  hx711_wait_settle(hx711_rate_80);

  // Calibrar a balanca
  tare_scale(&hx);
  calibrate_scale_factor(&hx);

  printf("Pronto para medir o peso!\n");

  setup(); // Inicialize GPIO e PWM

  servo_state current_servo_state = SERVO_0_DEG;
  set_servo_angle(0); // Comece na posicao de 0 graus

  uint64_t object_detected_time = 0; // Carimbo de data e hora de quando o objeto foi detectado
  const uint32_t return_delay_ms = 120000; // Atraso de 2 minutos em milissegundos

  // Initialize DHT11 sensor
  dht_t dht;
  dht_init(&dht, DHT_MODEL, pio0, DHT_DATA_PIN, true /* pull_up */);

  uint32_t last_retry_time = 0; // Controla o tempo da ultima tentativa

  bool object_was_close = false; // Acompanhe se o objeto estava proximo na iteracao anterior

  float last_servo_angle = get_current_servo_angle(); // Acompanhe o ultimo angulo do servo

  while (true) {
    // Leia o peso
    float weight = read_weight(&hx);

    // Imprima o peso no console serial
    printf("Peso: %.2f g\n", weight); // Ajuste a unidade conforme necessario

    // Pequeno atraso
    sleep_ms(100);

    float distance = get_distance_cm();
    printf("Distancia: %.2f cm\n", distance);

    // Leitura do sensor DHT11
    dht_start_measurement(&dht);
    float humidity;
    float temperature_c;
    dht_result_t result = dht_finish_measurement_blocking(&dht, &humidity, &temperature_c);

    if (result == DHT_RESULT_OK) {
      printf("Temperatura: %.1f C, Umidade: %.1f%%\n", temperature_c, humidity); // Imprimir em Celsius
    } else if (result == DHT_RESULT_TIMEOUT) {
      printf("O sensor DHT nao esta respondendo. Por favor, verifique sua fiacao.\n");
    } else {
      printf("Soma de verificacao DHT incorreta\n");
    }

    // Verifique se o objeto esta proximo
    bool is_close = (distance < CLOSE_DISTANCE);

    if (is_close && !object_was_close) { // O objeto acabou de se aproximar
      if (current_servo_state != SERVO_180_DEG) {
        set_servo_angle(180);
        current_servo_state = SERVO_180_DEG;
        object_detected_time = to_ms_since_boot(get_absolute_time()); // Registre o tempo de deteccao
      }
    } else if (!is_close && object_was_close) { // O objeto acabou de ser movido
      // Verifique se ja passou tempo suficiente desde que o objeto foi detectado
      if (current_servo_state == SERVO_180_DEG &&
          to_ms_since_boot(get_absolute_time()) - object_detected_time >= return_delay_ms) {
        set_servo_angle(0);
        current_servo_state = SERVO_0_DEG;
      }
    }

    object_was_close = is_close; // Atualize o estado para a proxima iteracao

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(500);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(500); // Faca leituras com mais frequencia

    // Prepare os dados para o POST
    snprintf(client_data.post_data, sizeof(client_data.post_data),
             "{\"peso\": %.2f, \"temperatura\": %.1f, \"umidade\": %.1f}",
             weight, temperature_c, humidity);

    // Verifique se o angulo do servo mudou
    if (get_current_servo_angle() != last_servo_angle) {
      printf("Angulo do servo alterado. Enviando dados para API.\n");
      last_servo_angle = get_current_servo_angle(); // Atualize o ultimo angulo do servo

      if (!client_data.connected) {
        // Tente resolver o DNS e conectar (assincrono)
        if (client_data.server_ip.addr == 0) { // Se o IP ainda nao foi resolvido
          if (!resolve_dns(&client_data)) {
            // Aguarde a resolucao do DNS ou um tempo limite
            uint32_t start_time = time_us_32();
            while (client_data.server_ip.addr == 0 && (time_us_32() - start_time < DNS_TIMEOUT_MS * 1000)) {
              cyw43_arch_poll();
              sleep_ms(10);
            }
            if (client_data.server_ip.addr == 0) {
              printf("Tempo limite na resolucao DNS.\n");
              // Lidar com o tempo limite (tente novamente, por exemplo)
              if (time_us_32() - last_retry_time > RETRY_INTERVAL_MS * 1000) {
                printf("Tentando novamente...\n");
                last_retry_time = time_us_32();
              }
            }
          }
        } else {
          // Se o IP ja foi resolvido, tente conectar
          connect_to_server(&client_data);
          // Adicione um tempo limite para a conexao
          uint32_t start_time = time_us_32();
          while (!client_data.connected && (time_us_32() - start_time < CONNECTION_TIMEOUT_MS * 1000)) {
            cyw43_arch_poll();
            sleep_ms(10);
          }
          if (!client_data.connected) {
            printf("Tempo limite na conexao TCP.\n");
            close_connection(&client_data); // Feche a conexao em caso de tempo limite
            // Lida com o tempo limite (tente novamente, por exemplo)
            if (time_us_32() - last_retry_time > RETRY_INTERVAL_MS * 1000) {
              printf("Tentando novamente...\n");
              last_retry_time = time_us_32();
              client_data.server_ip.addr = 0; // Forcar uma nova resolucao DNS
            }
          }
        }
      }
      cyw43_arch_poll();
      sleep_ms(10);
    } // Fim da verificacao do angulo do servo.
  }   // Fim do loop while principal
  cyw43_arch_deinit();
  return 0;
}