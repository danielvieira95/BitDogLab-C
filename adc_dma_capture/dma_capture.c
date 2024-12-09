#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2818b.pio.h"

// Configurações do ADC e DMA
#define MIC_PIN 28 // Pino do microfone (GPIO 28)
#define ADC_CHANNEL 2 // Canal ADC correspondente ao MIC_PIN
#define DMA_BUFFER_SIZE 256 // Tamanho do buffer do DMA
#define SAMPLING_FREQUENCY 30000 // Frequência de amostragem em Hz

uint16_t adc_buffer[DMA_BUFFER_SIZE]; // Buffer para armazenar dados do ADC
int dma_channel; // Canal do DMA

// Configurações da matriz de LEDs
#define LED_COUNT 25 // Número de LEDs na matriz
#define LED_PIN 7 // Pino de controle dos LEDs
#define LED_CENTER 12 // Índice do LED central

// Estrutura de pixel GRB
typedef struct {
    uint8_t G, R, B;
} npLED_t;
npLED_t leds[LED_COUNT]; // Buffer de LEDs

// Variáveis PIO
PIO np_pio;
uint sm;

/**
 * Inicializa o ADC para leitura do microfone.
 */
void configure_adc() {
    adc_init();
    adc_gpio_init(MIC_PIN);
    adc_select_input(ADC_CHANNEL);

    // Configura a frequência do clock do ADC para 30 kHz
    uint32_t clock_divider = clock_get_hz(clk_adc) / SAMPLING_FREQUENCY;
    adc_set_clkdiv((float)clock_divider);
}

/**
 * Configura o DMA para transferir valores do ADC para o buffer.
 */
void configure_dma() {
    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_16);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, true);

    dma_channel_configure(
        dma_channel,
        &config,
        &adc_buffer,
        &adc_hw->fifo,
        DMA_BUFFER_SIZE,
        true
    );

    adc_fifo_setup(true, true, 1, false, true);
    adc_run(true);
}

/**
 * Inicializa a matriz de LEDs utilizando PIO.
 */
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;

    sm = pio_claim_unused_sm(np_pio, false);
    if (sm < 0) {
        np_pio = pio1;
        sm = pio_claim_unused_sm(np_pio, true);
    }

    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

    for (uint i = 0; i < LED_COUNT; ++i) {
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
    }
}

/**
 * Atribui uma cor a um LED.
 */
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
}

/**
 * Limpa todos os LEDs.
 */
void npClear() {
    for (uint i = 0; i < LED_COUNT; ++i) {
        npSetLED(i, 0, 0, 0);
    }
}

/**
 * Atualiza os LEDs com base no buffer.
 */
void npWrite() {
    for (uint i = 0; i < LED_COUNT; ++i) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
}

/**
 * Processa os dados do microfone e atualiza os LEDs.
 */
void update_led_matrix() {
    uint32_t sum = 0;

    // Calcula a média do buffer
    for (int i = 0; i < DMA_BUFFER_SIZE; i++) {
        sum += adc_buffer[i];
    }
    uint16_t average_value = sum / DMA_BUFFER_SIZE;

    // Define o limiar para ausência de som
    uint16_t threshold = 200;

    if (average_value < threshold) {
        // Sem som, acende o LED central
        npClear();
        npSetLED(LED_CENTER, 255, 0, 0); // Vermelho para o LED central
    } else {
        // Atualiza LEDs com base no valor médio
        uint8_t led_index = (average_value * LED_COUNT) / 4096;
        for (int i = 0; i < LED_COUNT; i++) {
            if (i <= led_index) {
                npSetLED(i, 0, 255, 0); // Verde para LEDs acesos
            } else {
                npSetLED(i, 0, 0, 0); // LEDs apagados
            }
        }
    }
    npWrite();
}

/**
 * Função principal.
 */
int main() {
    stdio_init_all();

    // Configurações iniciais
    configure_adc();
    configure_dma();
    npInit(LED_PIN);

    while (true) {
        // Aguarda o término da transferência DMA
        dma_channel_wait_for_finish_blocking(dma_channel);

        // Atualiza a matriz de LEDs
        update_led_matrix();

        // Reinicia o DMA para nova coleta
        dma_channel_configure(
            dma_channel, NULL, &adc_buffer, &adc_hw->fifo, DMA_BUFFER_SIZE, true
        );
    }

    return 0;
}
