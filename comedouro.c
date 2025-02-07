#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwipopts.h"


// Wi-Fi Configuration (Replace with your credentials)
#define WIFI_SSID "IJKL_QUARTOS"
#define WIFI_PASSWORD "31290304"

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

    // Blink the LED to indicate success
    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); // Turn LED on
        sleep_ms(500);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); // Turn LED off
        sleep_ms(500);
    }

    return 0;
}