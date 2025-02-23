#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"

 
#define ADC_NUM 2                                   // Set to 2 according BitDogLab's microphone board 
#define ADC_PIN (26 + ADC_NUM)
#define ADC_VREF 3.3                                // 3.3 V power supplied
#define ADC_RANGE (1 << 12)                         // 2^12 where 12 is the resolution of the ADC
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))

#define I2C_SDA 14
#define I2C_SCL 15
 
const uint8_t SOFT_LEVEL = 11;
const uint8_t MID_LEVEL = 12;
const uint8_t HIGH_LEVEL = 13;
const uint16_t PERIOD = ADC_RANGE;  // Valor máximo do contador PWM
const float BASE_VOLTAGE = 1.64f;   // Valor medido na GPIO 28, referente ao microfone
const uint8_t BASE_DB = 54;         // Valor coletado em medições do ambiente  
const uint8_t PWM_BASE_FREQUENCY = 1000;
const uint8_t AVG_SAMPLE_RATE_MS = 100;
const uint8_t DISPLAY_COUNTDOWN_LIMIT = 100;

volatile float buffer[200];
uint16_t count = 0;                // Quantidade de amostras ADC coletadas no determinado intervalo
uint16_t display_countdown = 0;    // Contador para atualização do display
uint8_t db_value_set = 0;
uint8_t led_selection;
uint8_t slice_bled, slice_gled, slice_rled;

volatile uint8_t ssd[ssd1306_buffer_length];
// Preparar área de renderização para o display (ssd1306_width pixels por ssd1306_n_pages páginas)
volatile struct render_area frame_area = {
    start_column : 0,
    end_column : ssd1306_width - 1,
    start_page : 0,
    end_page : ssd1306_n_pages - 1
};

void print_to_display(uint8_t *ssd, char *string) {
    int16_t x = 10;
    int16_t y = 25;
    for (int i = 0; i < strlen(string); i++) {
        if (string[i] == '\n') {
            x = 0;
            y += 8;
        } else {
            ssd1306_draw_char(ssd, x, y, string[i]);
            x += 8;
        }
    }
    render_on_display(ssd, &frame_area);
}

void clear_display(uint8_t *ssd) {
    for (int i = 0; i < ssd1306_buffer_length; i++) {
        ssd[i] = 0;
    }
    render_on_display(ssd, &frame_area);
}

/**
 * @brief Configura o pino do LED para funcionar com PWM
 * @param led_gpio Pino do LED
 * @param frequency Frequência do PWM em Hz
 * @param slice Número do slice do PWM
 * @param value Valor inicial do LED
 */
void setup_pwm(uint8_t led_gpio, uint16_t frequency, uint8_t *slice, float value) {
    gpio_set_function(led_gpio, GPIO_FUNC_PWM); 
    *slice = pwm_gpio_to_slice_num(led_gpio);
    // Cálculo para encontrar o divisor de cada LED, caso precisem oscilar em frequências diferentes
    float divider = 125e6 / (frequency * PERIOD);
    pwm_set_clkdiv(*slice, divider);
    pwm_set_wrap(*slice, PERIOD);
    pwm_set_enabled(*slice, true);
    pwm_set_gpio_level(led_gpio, value);
}

void setup_display() {
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // inicializa OLED SSD1306
    ssd1306_init();
    calculate_render_area_buffer_length(&frame_area);
    // zera o display inteiro
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);
}

void setup_adc() {
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_NUM);
}

int remove_outliers() {
    float sum = 0;
    int outliers = 0;
    for (int i = 0; i < count; i++) {
        if (buffer[i] < BASE_VOLTAGE) {
            printf("outlier: %d: %.2f\n", i, buffer[i]);
            outliers++;
            continue;
        }
        sum += buffer[i];
    }
    count -= outliers;
    printf("sum: %.2f\n", sum);
    return sum;
}

void set_led_selection(uint8_t db_value) {
    if (db_value_set > BASE_DB && db_value_set < 70) {
        led_selection = SOFT_LEVEL;
    } else if (db_value_set >= 70 && db_value_set < 88) {
        // *led_selection = (uint8_t[]) {SOFT_LEVEL, HIGH_LEVEL};
        led_selection = MID_LEVEL;
    } else if (db_value_set >= 88) {
        led_selection = HIGH_LEVEL;
    } else {
        led_selection = SOFT_LEVEL;
    }
}

/**
 * Função de callback para o temporizador repetitivo
 * Calcula o valor médio das amostras coletadas e ajusta o valor do LED
 * @param t Ponteiro para a estrutura do temporizador
 */
bool samples_callback(struct repeating_timer *t) {
    // Etapa necessária pois foi entendido que o ruído do microfone causa leituras imprecisas, 
    // com valores menores do que o valor de referência
    float samples_sum = 0;
    int outliers = 0;
    // Etapa necessária pois como o trimmer não estava ajustando, foi entendido que o ruído do microfone causa leituras imprecisas, 
    // com valores menores do que o valor de referência. Talvez o trimmer acabou ficando num volume alto e acaba captando muito ruído.
    // NOTA: tentei modularizar para um método que retornava o valor de 'sum' porém estava ocorrendo uma inconsistência que fazia divergir o cálculo
    for (int i = 0; i < count; i++) {
        if (buffer[i] < BASE_VOLTAGE) {
            printf("outlier: %d: %.2f\n", i, buffer[i]);
            outliers++;
            continue;
        }
        samples_sum += buffer[i];
    }
    count -= outliers;
    printf("buffer count: %d\n", count);
    float average = samples_sum / count;
    printf("average: %.2f\n", average);
    float ratio = average / BASE_VOLTAGE;
    printf("ratio: %.2f\n", ratio);
    // Fatores de ajuste para suprir falta de ajuste via trimmer
    float factor = (average - BASE_VOLTAGE)*100;
    float sample_exchange;
    if (average <= (BASE_VOLTAGE + 0.04)) {
        printf("Base sound\n");
        sample_exchange = ratio;
        printf("adjust: %.2f\n", sample_exchange);
    } else if (average > (BASE_VOLTAGE + 0.04) && average <= (BASE_VOLTAGE + 0.3)) {
        printf("High sound\n");
        sample_exchange = pow((average), 2);
        printf("adjust: %.2f\n", sample_exchange);
    } else {
        printf("Very high sound\n");
        sample_exchange = pow((average), 3);
        printf("adjust: %.2f\n", sample_exchange);
    }
    // Cálculo do valor em decibéis
    db_value_set = BASE_DB + 20 * log10(sample_exchange) + factor;
    printf("DB: %d\n", db_value_set);
    set_led_selection(db_value_set);
    count = 0;
    return true;
}

void turn_off_leds() {    
    pwm_set_gpio_level(SOFT_LEVEL, 0);
    pwm_set_gpio_level(MID_LEVEL, 0);
    pwm_set_gpio_level(HIGH_LEVEL, 0);
}

void set_led_level(uint8_t led_selection, float value) {
    turn_off_leds();
    // for (int i = 0; i < sizeof(led_selection) / sizeof(led_selection[0]); i++) {
    if (led_selection == MID_LEVEL) {
        pwm_set_gpio_level(SOFT_LEVEL, (value - 500));
        pwm_set_gpio_level(HIGH_LEVEL, (value - 1000));
    } else {
        pwm_set_gpio_level(led_selection, value);
    }
    // }
}

int main() {
    stdio_init_all();
    printf("ADC_CONVERT: %.5f\n", ADC_CONVERT);

    setup_display();
    setup_adc();
    setup_pwm(SOFT_LEVEL, PWM_BASE_FREQUENCY, &slice_gled, BASE_VOLTAGE);
    setup_pwm(MID_LEVEL, PWM_BASE_FREQUENCY, &slice_bled, 0);
    setup_pwm(HIGH_LEVEL, PWM_BASE_FREQUENCY, &slice_rled, 0);

    // Nível de coloração mais amarelado
    pwm_set_gpio_level(SOFT_LEVEL, 1500);
    pwm_set_gpio_level(HIGH_LEVEL, 1000);
    print_to_display(ssd, "Inicializando...");
    sleep_ms(1000);
    
    // Configura o temporizador repetitivo para retirar uma média das amostras a cada 100 milisegundos
    struct repeating_timer timer;
    add_repeating_timer_ms(AVG_SAMPLE_RATE_MS, samples_callback, NULL, &timer);

    uint16_t adc_raw;
    while (1) {
        adc_raw = adc_read();
        printf("ADC raw: %d\n", adc_raw);
        float adc_vref_value = adc_raw * ADC_CONVERT;
        printf("%.2f\n", adc_vref_value);
        buffer[count] = adc_vref_value;
        count++;
        display_countdown++;
        
        set_led_level(led_selection, adc_raw);

        if (display_countdown == DISPLAY_COUNTDOWN_LIMIT) {
            display_countdown = 0;
            clear_display(ssd);
            char vdb[10];
            // Convert db_value_set to a string
            sprintf(vdb, "%u", db_value_set);
            // Concatenate the string with " [dB]"
            strcat(vdb, " dB");
            // char vdb = db_value_set + " [dB]";
            printf("updating display with %s\n", vdb);
            print_to_display(ssd, vdb);
        }
        sleep_ms(10);
    }
}