#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include "hardware/i2c.h"

// Definições para conexão Wi-Fi
#define WIFI_SSID "073AFibra 2.4Ge5G" // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASS "dCz6pRQ5"          // Substitua pela senha da sua rede Wi-Fi

// Definições do AHT10 e I2C0
#define AHT10_ADDR 0x38  // Endereço I2C do AHT10
#define SDA_PIN 4        // Pino SDA (I2C0)
#define SCL_PIN 5        // Pino SCL (I2C0)

// Buffer para respostas HTTP
char http_response[512]; 

// Função para ler a temperatura do AHT10
float ler_temperatura()
{
    uint8_t cmd_measure[3] = {0xAC, 0x33, 0x00}; // Comando para iniciar medição
    uint8_t data[6]; // Buffer para armazenar os dados recebidos

    // Envia o comando de medição
    i2c_write_blocking(i2c0, AHT10_ADDR, cmd_measure, 3, false);
    sleep_ms(80); // Aguarda a medição (~75ms recomendado)

    // Lê os 6 bytes de resposta do AHT10
    if (i2c_read_blocking(i2c0, AHT10_ADDR, data, 6, false) != 6)
    {
        printf("Erro ao ler dados do AHT10\n");
        return -99.99; // Retorna um valor de erro
    }

    // Extrai os dados de temperatura dos bytes recebidos
    uint32_t temp_raw = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];
    float temperatura = ((temp_raw * 200.0) / 1048576) - 50;

    return temperatura;
}

// Função de callback para processar requisições HTTP
static err_t http_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (p == NULL)
    {
        tcp_close(tpcb);
        return ERR_OK;
    }

    tcp_recved(tpcb, p->tot_len); // Notifica o TCP que os dados foram processados

    float temperature = ler_temperatura();

    // Cria a resposta HTTP
    snprintf(http_response, sizeof(http_response),
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: text/html\r\n"
             "Connection: close\r\n\r\n"
             "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='5'></head><body>"
             "<h1>Temperatura do ambiente</h1>"
             "<p>Temperatura: %.2f C</p>"
             "</body></html>\r\n",
             temperature);

    tcp_write(tpcb, http_response, strlen(http_response), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb); // Garante que os dados são enviados
    tcp_close(tpcb);  // Fecha a conexão corretamente
    pbuf_free(p);

    return ERR_OK;
}

// Callback de conexão: associa o http_callback à conexão
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, http_callback); // Associa o callback HTTP
    return ERR_OK;
}

// Função de setup do servidor HTTP
static void start_http_server(void)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb)
    {
        printf("Erro ao criar PCB\n");
        return;
    }

    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK)
    {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }

    pcb = tcp_listen(pcb);                // Coloca o PCB em modo de escuta
    tcp_accept(pcb, connection_callback); // Associa o callback de conexão
    printf("Servidor HTTP rodando na porta 80...\n");
}

int main()
{
    stdio_init_all(); // Inicializa a saída padrão

    // Inicializa o I2C0 para comunicação com o AHT10
    i2c_init(i2c0, 100 * 1000); // Configura o I2C para 100 kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    
    sleep_ms(10000);
    printf("Iniciando servidor HTTP\n");

    // Inicializa o Wi-Fi
    if (cyw43_arch_init())
    {
        printf("Erro ao inicializar o Wi-Fi\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    printf("Conectando ao Wi-Fi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000))
    {
        printf("Falha ao conectar ao Wi-Fi\n");
        return 1;
    }
    else
    {
        printf("Connected.\n");
        uint8_t *ip_address = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
        printf("Endereço IP %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }

    printf("Wi-Fi conectado!\n");
    printf("Para verificar a temperatura, acesse o IP no navegador.\n");

    // Inicia o servidor HTTP
    start_http_server();

    // Loop principal
    while (true)
    {
        cyw43_arch_poll(); // Necessário para manter o Wi-Fi ativo
        sleep_ms(1000);
    }

    cyw43_arch_deinit(); // Desliga o Wi-Fi (não será chamado, pois o loop é infinito)
    return 0;
}
