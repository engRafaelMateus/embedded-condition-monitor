/**
 * @file main.c
 * @brief Embedded Condition Monitoring Platform
 *
 * @project embedded-condition-monitor
 *
 * Plataforma embarcada para monitoramento, diagnóstico e telemetria
 * de equipamentos, desenvolvida com ESP32, ESP-IDF, FreeRTOS e C.
 *
 * O sistema realiza aquisição de dados de sensores digitais e analógicos,
 * processamento das medições e classificação automática da condição
 * operacional do equipamento nos estados:
 *
 *   - NORMAL
 *   - WARNING
 *   - CRITICAL
 *
 * Arquitetura atual:
 *
 *   MPU6050 (I2C) ----\
 *                      \
 *   Sensor ADC ---------> taskSensores
 *                            |
 *                            | FreeRTOS Queue
 *                            v
 *                       taskControle
 *                            |
 *               +------------+-------------+
 *               |            |             |
 *             LEDs        Buzzer PWM     Alarm ACK
 *                                          ^
 *                                          |
 *                                      taskBotao
 *                                          ^
 *                                          |
 *                                      GPIO ISR
 *
 * A infraestrutura UART também está configurada para evolução
 * da camada de telemetria e comunicação bidirecional.
 *
 * ---------------------------------------------------------------------------
 * FUNCIONALIDADES IMPLEMENTADAS
 * ---------------------------------------------------------------------------
 *
 * - ESP32 utilizando SDK nativo ESP-IDF
 * - Arquitetura multitarefa com FreeRTOS
 * - Aquisição de temperatura pelo MPU6050 via I2C
 * - Aquisição de aceleração nos eixos X, Y e Z
 * - Aquisição analógica utilizando ADC Oneshot
 * - Média de múltiplas amostras ADC
 * - Validação e descarte de ciclos de aquisição inválidos
 * - Comunicação entre tasks utilizando FreeRTOS Queue
 * - Máquina de estados NORMAL / WARNING / CRITICAL
 * - Sinalização visual com LEDs
 * - Alarme sonoro utilizando PWM/LEDC
 * - Reconhecimento de alarme pelo operador
 * - GPIO interrupt para tratamento do botão
 * - Debounce realizado fora da ISR
 * - Task Notifications entre ISR, taskBotao e taskControle
 * - Tratamento inicial de falhas de aquisição I2C e ADC
 * - Configuração de duas interfaces UART para comunicação bidirecional
 *
 * ---------------------------------------------------------------------------
 * EM DESENVOLVIMENTO
 * ---------------------------------------------------------------------------
 *
 * - Task dedicada de telemetria
 * - Telemetria estruturada via UART
 * - Separação entre aquisição, controle e comunicação
 * - Frames contendo medições, estado operacional e status de alarme
 * - Heartbeat e identificação do dispositivo
 * - Sequenciamento de mensagens
 * - Detecção de timeout e perda de comunicação
 * - Tratamento centralizado de falhas
 *
 * ---------------------------------------------------------------------------
 * ROADMAP
 * ---------------------------------------------------------------------------
 *
 * Firmware / Comunicação:
 * - CAN/TWAI entre nós ESP32
 * - RS-485 para comunicação industrial
 * - Definição de protocolo de mensagens
 * - Detecção e recuperação de falhas de comunicação
 * - Watchdog
 * - Persistência de configurações utilizando NVS
 * - Logs estruturados com ESP_LOG
 * - Organização modular do firmware em arquivos .c/.h
 * - CMake e organização do processo de build
 *
 * Telemetria / Conectividade:
 * - Gateway Linux embarcado desenvolvido em C++
 * - TCP/IP e UDP
 * - MQTT
 * - HTTP/REST
 * - Integração com serviços externos/backend
 * - Reconexão automática e tratamento de indisponibilidade
 *
 * Linux Embarcado:
 * - Serviço/daemon em C++ moderno
 * - Threads
 * - Mutexes
 * - Condition variables
 * - IPC
 * - systemd
 * - journal
 *
 * Hardware / Validação:
 * - Testes de bancada
 * - Multímetro
 * - Osciloscópio
 * - Analisador lógico
 * - Leitura de datasheets e esquemáticos
 * - Prototipagem física
 * - Diagnóstico de sinais
 * - Captura de esquemático e PCB utilizando KiCad
 *
 * Qualidade / Robustez:
 * - Testes de integração
 * - Testes de falhas
 * - Recuperação automática
 * - Atualização OTA
 * - Documentação de ensaios e resultados
 * - Versionamento com Git/GitHub
 *
 * ---------------------------------------------------------------------------
 * FLUXO PRINCIPAL
 * ---------------------------------------------------------------------------
 *
 * Hardware
 *    -> aquisição de sensores
 *    -> taskSensores
 *    -> FreeRTOS Queue
 *    -> taskControle
 *    -> classificação do estado
 *    -> atuação local
 *    -> telemetria
 *    -> comunicação externa
 *
 * @note Projeto em evolução contínua para estudo e aplicação prática de
 *       conceitos utilizados em firmware, hardware e sistemas embarcados.
 */
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>
#include <driver/i2c_master.h>
#include <esp_adc/adc_oneshot.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/uart.h>
#include <driver/twai.h>
#include "esp_err.h"
#include<math.h>

#define I2C_PORT I2C_NUM_0
#define I2C_SDA 21
#define I2C_SCL 22
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_I2C_ADDR 0x68

#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

#define CAN_MODO_TESTE 1

#define LED_NORMAL   25
#define LED_WARNING  26
#define LED_CRITICAL 27

#define BOTAO 4
#define BUZZER 16

#define UART1_PORT UART_NUM_1
#define UART1_TX 18
#define UART1_RX 19

#define UART2_PORT UART_NUM_2
#define UART2_TX 32
#define UART2_RX 33

static TaskHandle_t taskBotaoHandle = NULL;
static TaskHandle_t taskControleHandle = NULL;
static TaskHandle_t taskTelemetriaHandle = NULL;



typedef struct{
  i2c_master_dev_handle_t mpu_handle;
  adc_oneshot_unit_handle_t adc_handle;
  QueueHandle_t fila;
} sensor_task_context_t;

typedef struct {
  int16_t temperatura_raw;
  int16_t ax;
  int16_t ay;
  int16_t az;

  float temperatura_c;
  float ax_g;
  float ay_g;
  float az_g;

  int valor_adc;
} sensor_data_t;

typedef enum {
    STATE_NORMAL  = 0,
    STATE_WARNING = 1,
    STATE_CRITICAL= 2
} system_state_t;

typedef struct {
    QueueHandle_t fila_sensores;
    QueueHandle_t fila_telemetria;
} control_task_context_t;

typedef struct{
  sensor_data_t dados;
  system_state_t estado;
  bool alarme_reconhecido;
} telemetry_data_t;


  esp_err_t ler_temperatura_raw(
                                i2c_master_dev_handle_t dev_handle, 
                                int16_t *temperatura_raw){
  
    uint8_t data[2] = {0};
    uint8_t reg = MPU6050_REG_TEMP_OUT_H;
    
      esp_err_t resultado = i2c_master_transmit_receive(
        dev_handle, 
        &reg, 
        1, 
        data, 
        2, 
        -1
      );      

      if (resultado == ESP_OK){            
        *temperatura_raw = (int16_t)((data[0] << 8) | data[1]);
        }

    return resultado;
  }

  esp_err_t ler_aceleracao_raw(i2c_master_dev_handle_t dev_handle,  int16_t *ax, 
                                                                    int16_t *ay, 
                                                                    int16_t *az){

    uint8_t data[6];                 
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;                                                 

    esp_err_t resultado = i2c_master_transmit_receive(
        dev_handle, 
        &reg, 
        1, 
        data, 
        6, 
        -1
    );  

    if(resultado == ESP_OK){
      *ax = (int16_t)((data[0] << 8) | data[1]);
      *ay = (int16_t)((data[2] << 8) | data[3]);
      *az = (int16_t)((data[4] << 8) | data[5]);
    }

    return resultado;
  }

  /*
    abaixo de 45 °C     → NORMAL
    45 até abaixo de 60 → WARNING
    60 °C ou mais       → CRITICAL
  
    0 até 2499      → NORMAL
    2500 até 3299   → WARNING
    3300 ou mais    → CRITICAL  
  */
  system_state_t avaliar_estado(sensor_data_t *dados){

    if(dados->valor_adc >= 3300 || dados->temperatura_c >= 60){
      
      return STATE_CRITICAL;
    }
    else if(dados->valor_adc >= 2500 || dados->temperatura_c >= 45){
      
      return STATE_WARNING;
    }
    else{
      
      return STATE_NORMAL;
    }
  }

  esp_err_t mpu6050_init(i2c_master_dev_handle_t dev_handle){
    uint8_t dados[2] = {
      MPU6050_REG_PWR_MGMT_1,
      0x00
    };

    return i2c_master_transmit(
        dev_handle,
        dados,
        2,
        -1
    ); 
  }  

  void taskSensores(void *pvParameters){

    sensor_task_context_t *ctx = (sensor_task_context_t *)pvParameters;

    while(true){

      sensor_data_t leitura = {0};

      esp_err_t resultado = ler_temperatura_raw(ctx->mpu_handle, &leitura.temperatura_raw);

        if (resultado != ESP_OK)
        {
          printf("Erro ao ler temperatura\n");
          vTaskDelay(pdMS_TO_TICKS(1000));
          continue;          
        }
            
        leitura.temperatura_c = (leitura.temperatura_raw / 340.0f) + 36.53f;
        printf("Temperatura Sensor: %.2f C\n", leitura.temperatura_c);


      resultado = ler_aceleracao_raw(
                                      ctx->mpu_handle, 
                                      &leitura.ax, 
                                      &leitura.ay, 
                                      &leitura.az
                                      );

        if(resultado != ESP_OK){

          printf("Erro ao ler acelerometro\n");
          vTaskDelay(pdMS_TO_TICKS(1000));
          continue;
        }

        leitura.ax_g = leitura.ax / 16384.0f;
        leitura.ay_g = leitura.ay / 16384.0f;
        leitura.az_g = leitura.az / 16384.0f;

        printf("AX: %.2f g\n", leitura.ax_g);
        printf("AY: %.2f g\n", leitura.ay_g);
        printf("AZ: %.2f g\n", leitura.az_g);

        int soma = 0;
        int leituras_validas = 0;
        int tentativas = 0;

        while(leituras_validas < 10 && tentativas < 20){

          resultado = adc_oneshot_read(
            ctx->adc_handle,
            ADC_CHANNEL_6,
            &leitura.valor_adc
          );

          tentativas++;
        
          if(resultado == ESP_OK){
            soma += leitura.valor_adc;
            leituras_validas++;
          }           
        }

        if(leituras_validas == 10){      
          leitura.valor_adc = soma / 10;
          printf("Valor ADC: %d\n", leitura.valor_adc);
        }
        else{
            printf("Falha: Não foi possível obter 10 leituras válidas.\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        xQueueSend(
          ctx->fila,   //Onde enviar (Handle da Fila)
          &leitura,    //O que enviar (Ponteiro para o Dado)
          pdMS_TO_TICKS(100)
        );


      vTaskDelay(pdMS_TO_TICKS(1000));
    }

  }
  
  void atualiza_leds(system_state_t estado){

    if(estado == STATE_NORMAL){
      gpio_set_level(LED_NORMAL, 1);
      gpio_set_level(LED_WARNING, 0);
      gpio_set_level(LED_CRITICAL, 0);

    }else if(estado == STATE_WARNING){
      gpio_set_level(LED_NORMAL, 0);
      gpio_set_level(LED_WARNING, 1);
      gpio_set_level(LED_CRITICAL, 0);

    }else if (estado == STATE_CRITICAL){
      gpio_set_level(LED_NORMAL, 0);
      gpio_set_level(LED_WARNING, 0);
      gpio_set_level(LED_CRITICAL, 1);
      

    }
  }

  void taskControle(void *pvParameters){

    control_task_context_t *ctx = (control_task_context_t *)pvParameters;

    sensor_data_t leitura = {0};
    telemetry_data_t telemetria = {0};
    bool alarme_reconhecido = false;


    while(true){

      BaseType_t recebeu = xQueueReceive(
                                          ctx->fila_sensores,
                                          &leitura,
                                          pdMS_TO_TICKS(100)
                                        );

      if(recebeu != pdTRUE){
        continue; //tratar depois
      }

      telemetria.dados = leitura;

      system_state_t estado = avaliar_estado(&leitura);
      
      telemetria.estado = estado;

      atualiza_leds(estado); 

      if (estado != STATE_CRITICAL) {
          alarme_reconhecido = false;
      }

      uint32_t aviso = ulTaskNotifyTake(
                                pdTRUE,
                                0
                              );
    
      if(aviso > 0 && estado == STATE_CRITICAL){
        alarme_reconhecido = true;
      }

      telemetria.alarme_reconhecido = alarme_reconhecido;

      if (xQueueSend(ctx->fila_telemetria, &telemetria, 0) != pdTRUE) {
          printf("Fila de telemetria cheia: mensagem descartada\n");
      }

      if (estado == STATE_CRITICAL && !alarme_reconhecido) {

          ledc_set_duty(
              LEDC_LOW_SPEED_MODE,
              LEDC_CHANNEL_0,
              2048
          );

          ledc_update_duty(
              LEDC_LOW_SPEED_MODE,
              LEDC_CHANNEL_0
          );

      } else {

          ledc_set_duty(
              LEDC_LOW_SPEED_MODE,
              LEDC_CHANNEL_0,
              0
          );

          ledc_update_duty(
              LEDC_LOW_SPEED_MODE,
              LEDC_CHANNEL_0
          );
      }

      if (estado == STATE_NORMAL){
        printf("Estado: NORMAL\n");
      }
      else if (estado == STATE_WARNING){
          printf("Estado: WARNING\n");
      }
      else if (estado == STATE_CRITICAL){

        printf("Estado: CRITICAL\n");

        if (alarme_reconhecido){
            printf("Alarme: RECONHECIDO\n");
        }
        else {
            printf("Alarme: NAO RECONHECIDO\n");
        }
      }    
    }
  }

  /* PROTOCOLO CAN DE TELEMETRIA
    *
    * CAN classico, IDs padrao de 11 bits, frames de dados (extd=0, rtr=0).
    * Velocidade configurada: 500 kbit/s.
    * Publicacao dos quatro frames a cada telemetria processada, sem consulta
    * previa do receptor. A saida UART permanece ativa.
    *
    * REPRESENTACAO
    * - Campos de 16 bits: byte mais significativo primeiro (big-endian).
    * - Temperatura e aceleracao: inteiros com sinal em complemento de dois.
    * - Valores float sao escalados e arredondados com lroundf() no envio.
    * - O receptor recupera o sinal antes de dividir pela escala.
    *
    * ID 0x100 — ESTADO OPERACIONAL — DLC 3
    * data[0]   : estado: 0=NORMAL, 1=WARNING, 2=CRITICAL.
    * data[1]   : alarme reconhecido: 0=nao, 1=sim.
    * data[2]   : validade do estado: 0=valido,
    *             1=indisponivel por falha de aquisicao.
    * Reconhecer o alarme nao elimina a condicao CRITICAL.
    *
    * ID 0x200 — TEMPERATURA — DLC 3
    * data[0..1]: int16, temperatura em graus Celsius multiplicada por 100.
    * data[2]   : status da aquisicao: 0=valida, 1=falha.
    * Resolucao do transporte: 0,01 C. Faixa: -327,68 a +327,67 C.
    * Exemplo: 24,00 C -> 2400 -> 09 60.
    *
    * ID 0x201 — ADC — DLC 3
    * data[0..1]: uint16, valor ADC disponivel na telemetria.
    * data[2]   : status da aquisicao: 0=valida, 1=falha.
    * Valor sem conversao para unidade fisica de pressao.
    * Exemplo: 2048 -> 08 00.
    *
    * ID 0x202 — ACELERACAO — DLC 7
    * data[0..1]: int16, eixo X em g multiplicado por 1000.
    * data[2..3]: int16, eixo Y em g multiplicado por 1000.
    * data[4..5]: int16, eixo Z em g multiplicado por 1000.
    * data[6]   : status da aquisicao: 0=valida, 1=falha.
    * Resolucao do transporte: 0,001 g. Faixa: -32,768 a +32,767 g.
    * Exemplo: -0,250 g -> -250 -> FF 06.
    *
    * VALIDACAO E LIMITACOES
    * - Conferir ID, formato, DLC e status antes de interpretar o payload.
    * - Estado deve estar entre 0 e 2; reconhecimento deve ser 0 ou 1.
    * - Status diferente de zero impede usar a medicao/estado como valido.
    * - Atualmente apenas ciclos validos chegam a telemetria: status enviado=0.
    *   A publicacao de falhas ainda precisa ser implementada.
    * - Conversoes pressupõem valores validos dentro das faixas representaveis.
    * - Frames separados nao possuem contador de sequencia para associar ciclos.
    *
    * MODOS DE EXECUCAO
    * CAN_MODO_TESTE=1: imprime bytes e chama o interpretador localmente.
    * CAN_MODO_TESTE=0: inicializa TWAI e solicita envio com twai_transmit().
    * O retorno ESP_OK indica aceitacao pelo driver, nao entrega ao receptor.
    *
    * Codificacao e interpretacao testadas localmente.
    * Recepcao via twai_receive() e validacao fisica do barramento pendentes.
    */


  void processar_frame_can(const twai_message_t *message){

    if(message->identifier == 0x200 && 
      message->extd == 0 && 
      message->rtr == 0 && 
      message->data_length_code == 3)
    {
      if (message->data[2] != 0) {
        printf("Temperatura indisponivel: status de aquisicao %u\n",
              (unsigned int)message->data[2]);
        return;
      }

        uint16_t bits = (message->data[0] * 256u) + message->data[1];
        int32_t temperatura_recebida_x100 = bits;

        if (bits >= 32768u) {
            temperatura_recebida_x100 -= 65536;
        }

        float conversao = temperatura_recebida_x100 / 100.0f;
        printf("[TESTE CAN RX] Temperatura = %.2f C\n", conversao);

    }else if( message->identifier == 0x201 &&
              message->extd == 0 &&
              message->rtr == 0 &&
              message->data_length_code == 3
            )
    {
      if (message->data[2] != 0) {
        printf("Leitura indisponivel: status de aquisicao %u\n",
              (unsigned int)message->data[2]);
        return;
      }

      uint16_t ADC = (message->data[0] * 256u) + message->data[1];
      printf("[TESTE CAN RX] ADC = %d\n", ADC);

    }else if( message->identifier == 0x202 &&
              message->extd == 0 &&
              message->rtr == 0 &&
              message->data_length_code == 7
            )
    {
      if (message->data[6] != 0) {
        printf("Leitura indisponivel: status de aquisicao %u\n",
              (unsigned int)message->data[6]);
        return;
      }

      uint16_t bits = (message->data[0] * 256u) + message->data[1];
      int32_t ax_g_x1000 = bits;

        if (bits >= 32768u) {
            ax_g_x1000 -= 65536;
        }

        float conversao_x = ax_g_x1000 / 1000.0f;

        bits = (message->data[2] * 256u) + message->data[3];
        int32_t ay_g_x1000 = bits;

          if (bits >= 32768u) {
              ay_g_x1000 -= 65536;
          }

        float conversao_y = ay_g_x1000 / 1000.0f;

        bits = (message->data[4] * 256u) + message->data[5];
        int32_t az_g_x1000 = bits;

          if (bits >= 32768u) {
              az_g_x1000 -= 65536;
          }

        float conversao_z = az_g_x1000 / 1000.0f;


        printf("[TESTE CAN RX]\nAcel_x = %.3f g\nAcel_y = %.3f g\nAcel_z = %.3f g\n", conversao_x, conversao_y, conversao_z);

    }else if( message->identifier == 0x100 &&
              message->extd == 0 &&
              message->rtr == 0 &&
              message->data_length_code == 3
            )
    {
      if (message->data[2] != 0) {
        printf("Leitura indisponivel: status de aquisicao %u\n",
              (unsigned int)message->data[2]);
        return;
      }

      if (message->data[0] > 2 || message->data[1] > 1) {
        printf("Mensagem rejeitada: estado ou reconhecimento invalido\n");
        return;
      }

      uint8_t estado  = message->data[0];
      bool alarme_reconhecido = message->data[1];

      printf("[TESTE CAN RX]\nSTATUS = %d\nALARME = %d\n", estado, alarme_reconhecido);

    }else{
        printf("Mensagem rejeitada: formato inesperado\n");
    }
  }
  
  void enviar_mensagem(const twai_message_t *message){

    if(message->data_length_code > 8){
      printf("Mensagem Rejeita\n");
      return;
    }

    #if CAN_MODO_TESTE

        printf("[TESTE CAN TX] ID=0x%X DLC=%d DATA=", (unsigned int)message->identifier, message->data_length_code);

        for (int i = 0; i < message->data_length_code; i++) {
            printf("%02X ", (unsigned int)message->data[i]);
        }

        printf("\n");
        processar_frame_can(message);

    #else

        esp_err_t resultado = twai_transmit(
                                              message,
                                              pdMS_TO_TICKS(100)
                                            );

        if (resultado != ESP_OK) {
            printf("[CAN TX] Erro ao solicitar envio: %s\n",
            esp_err_to_name(resultado));
            return;
        }

        printf("[CAN TX] Frame aceito para transmissao: ID=0x%X\n",
          (unsigned int)message->identifier);

    #endif

  }

  void enviar_telemetria_can(const telemetry_data_t *telemetria){

    twai_message_t temp_message = {
      .identifier = 0x200,
      .extd = 0,
      .rtr = 0,
      .ss = 1,
      .data_length_code = 3,
      .data = {0},
    };

    int16_t temperatura_x100 = (int16_t)lroundf(telemetria->dados.temperatura_c * 100.0f);
    uint16_t bits = (uint16_t)temperatura_x100;

    temp_message.data[0] = bits >> 8;
    temp_message.data[1] = bits & 0xFF;
    temp_message.data[2] = 0;

    enviar_mensagem(&temp_message);
    
    twai_message_t adc_message = {
      .identifier = 0x201,
      .extd = 0,
      .rtr = 0,
      .ss = 1,
      .data_length_code = 3,
      .data = {0},
    };

    uint16_t valor_ADC = telemetria->dados.valor_adc;
    adc_message.data[0] = valor_ADC >> 8;
    adc_message.data[1] = valor_ADC & 0xFF;
    adc_message.data[2] = 0;

    enviar_mensagem(&adc_message);

    twai_message_t acel_message = {
      .identifier = 0x202,
      .extd = 0,
      .rtr = 0,
      .ss = 1,
      .data_length_code = 7,
      .data = {0},
    };

    int16_t ax_x1000 = (int16_t)lroundf(telemetria->dados.ax_g * 1000.0f);
    uint16_t bits_ax = (uint16_t)ax_x1000;

    acel_message.data[0] = bits_ax >> 8;
    acel_message.data[1] = bits_ax & 0xFF;

    int16_t ay_x1000 = (int16_t)lroundf(telemetria->dados.ay_g * 1000.0f);
    uint16_t bits_ay = (uint16_t)ay_x1000;

    acel_message.data[2] = bits_ay >> 8;
    acel_message.data[3] = bits_ay & 0xFF;

    int16_t az_x1000 = (int16_t)lroundf(telemetria->dados.az_g * 1000.0f);
    uint16_t bits_az = (uint16_t)az_x1000;

    acel_message.data[4] = bits_az >> 8;
    acel_message.data[5] = bits_az & 0xFF;
    
    
    acel_message.data[6] = 0;

    enviar_mensagem(&acel_message);


    twai_message_t status_op_message = {
      .identifier = 0x100,
      .extd = 0,
      .rtr = 0,
      .ss = 1,
      .data_length_code = 3,
      .data = {0},
    };

    status_op_message.data[0] = telemetria->estado;
    status_op_message.data[1] = telemetria->alarme_reconhecido;
    status_op_message.data[2] = 0;

    enviar_mensagem(&status_op_message);

  }
  #if !CAN_MODO_TESTE

  //Recebe frames do driver TWAI e entrega ao interpretador do protocolo.

  void taskReceberCAN(void *pvParameters)
  {
      (void)pvParameters;

      twai_message_t message = {0};

      while (true) {
          esp_err_t resultado = twai_receive(
              &message,
              pdMS_TO_TICKS(1000)
          );

          if (resultado == ESP_OK) {
              processar_frame_can(&message);
          }
          else if (resultado == ESP_ERR_TIMEOUT) {
              // Nenhum frame chegou durante a espera.
              continue;
          }
          else {
              printf("[CAN RX] Erro ao receber: %s\n", esp_err_to_name(resultado));
              vTaskDelay(pdMS_TO_TICKS(100));
          }
      }
  }

#endif

  void taskTelemetria(void *pvParameters){

    QueueHandle_t fila_telemetria = (QueueHandle_t)pvParameters;    

    telemetry_data_t telemetria = {0};


      while(true){

        BaseType_t recebeu = xQueueReceive(
                                            fila_telemetria,
                                            &telemetria,
                                            portMAX_DELAY
                                          );

        if(recebeu == pdTRUE){

          enviar_telemetria_can(&telemetria);

          uart_write_bytes(
                            UART1_PORT,
                            &telemetria,
                            sizeof(telemetria)
                          );
        }
      vTaskDelay(pdMS_TO_TICKS(500));
      }
  }

  void taskLerTelemetria(void *pvParameters){

    telemetry_data_t buffer = {0};
    size_t total_recebido = 0;

    while(true){

      int bytes_lidos = uart_read_bytes(
                                          UART2_PORT,
                                          (uint8_t *)&buffer + total_recebido,
                                          sizeof(buffer) - total_recebido,
                                          pdMS_TO_TICKS(100)
                                        );     

      //Verificação se chegaram todos os bytes de telemetria antes de fazer a leitura 
      if(bytes_lidos > 0){ 
        total_recebido += bytes_lidos;
              
        if(total_recebido == sizeof(buffer)){

          printf("UART: Temperatura = %.2f C\n", buffer.dados.temperatura_c);
          total_recebido = 0; 
        }
        }else if (bytes_lidos < 0){
            printf("ERRO ao receber os bytes\n");
        }
      
    }
  }

  void taskBotao(void *pvParameters){

    while(true){

      ulTaskNotifyTake(
        pdTRUE,
        portMAX_DELAY
      );

      vTaskDelay(pdMS_TO_TICKS(50));

      ulTaskNotifyTake(
            pdTRUE,
            0
      );

      if (gpio_get_level(BOTAO) == 1){

        if(taskControleHandle != NULL){

          xTaskNotifyGive(taskControleHandle);
        }
      }
    }
  }

  static void IRAM_ATTR botao_isr(void *arg){

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(
      taskBotaoHandle,
      &xHigherPriorityTaskWoken
    );


    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }



void app_main() {

  i2c_master_bus_config_t i2c_config ={
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT,
    .scl_io_num = I2C_SCL,
    .sda_io_num = I2C_SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
  };

  i2c_master_bus_handle_t bus_handle;

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &bus_handle));

  i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MPU6050_I2C_ADDR,
    .scl_speed_hz = 100000,
  };

  i2c_master_dev_handle_t dev_handle;

  ESP_ERROR_CHECK(
    i2c_master_bus_add_device(
      bus_handle, 
      &dev_cfg, 
      &dev_handle
    )
  );

  ESP_ERROR_CHECK(mpu6050_init(dev_handle) );

  adc_oneshot_unit_handle_t adc1_handle;

  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_12,
  };

  ESP_ERROR_CHECK(adc_oneshot_config_channel(
    adc1_handle, 
    ADC_CHANNEL_6, 
    &config
    )
  );

  QueueHandle_t fila_sensores;

  fila_sensores = xQueueCreate(
    5,
    sizeof(sensor_data_t)
  );

  if(fila_sensores == NULL){
    printf("Erro ao criar fila de sensores\n");
    return;
  }

  static sensor_task_context_t sensor_ctx;

  sensor_ctx.mpu_handle = dev_handle;
  sensor_ctx.adc_handle = adc1_handle;
  sensor_ctx.fila = fila_sensores;

  QueueHandle_t fila_telemetria;

  fila_telemetria = xQueueCreate(
    5,
    sizeof(telemetry_data_t)
  );

  if(fila_telemetria == NULL){
    printf("Erro ao criar fila de telemetria\n");
    return;
  }

  static control_task_context_t control_ctx;

  control_ctx.fila_sensores = fila_sensores;
  control_ctx.fila_telemetria = fila_telemetria;



  gpio_config_t led_config = {
    .pin_bit_mask = 
                    (1ULL << LED_NORMAL) | 
                    (1ULL << LED_WARNING) | 
                    (1ULL << LED_CRITICAL),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };

  ESP_ERROR_CHECK(gpio_config(&led_config));

  gpio_config_t botao_config = {
                                .pin_bit_mask = 1ULL << BOTAO,
                                .mode = GPIO_MODE_INPUT,
                                .pull_up_en = GPIO_PULLUP_DISABLE,
                                .pull_down_en = GPIO_PULLDOWN_ENABLE,
                                .intr_type = GPIO_INTR_POSEDGE
                              };


  ESP_ERROR_CHECK(gpio_config(&botao_config));

  ESP_ERROR_CHECK(gpio_install_isr_service(0));


  ledc_timer_config_t timer_config = {
                                    .speed_mode = LEDC_LOW_SPEED_MODE,
                                    .timer_num = LEDC_TIMER_0,
                                    .duty_resolution = LEDC_TIMER_12_BIT,
                                    .freq_hz = 1000,
                                    .clk_cfg = LEDC_AUTO_CLK
                                  };

  esp_err_t erro = ledc_timer_config(&timer_config);
    if(erro != ESP_OK){
        printf("Erro ao configurar LEDC_TIMER_CONFIG\n");
        abort();
    }

  ledc_channel_config_t channel_config = {
                                  .gpio_num = BUZZER,
                                  .speed_mode = LEDC_LOW_SPEED_MODE,
                                  .channel = LEDC_CHANNEL_0,
                                  .timer_sel = LEDC_TIMER_0,
                                  .duty = 0,
                                  .hpoint = 0
                                };
    
  erro = ledc_channel_config(&channel_config);  
    if(erro != ESP_OK){
        printf("Erro ao configurar LEDC_CHANNEL_CONFIG\n");
        abort();
    }

    uart_config_t uart1_config = {
                                  .baud_rate = 115200,
                                  .data_bits = UART_DATA_8_BITS,
                                  .parity = UART_PARITY_DISABLE,
                                  .stop_bits = UART_STOP_BITS_1,
                                  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                                  .source_clk = UART_SCLK_DEFAULT
                              };

    ESP_ERROR_CHECK(uart_param_config(UART1_PORT, &uart1_config));  

    ESP_ERROR_CHECK(uart_set_pin( 
                                  UART1_PORT, 
                                  UART1_TX, 
                                  UART1_RX, 
                                  UART_PIN_NO_CHANGE, 
                                  UART_PIN_NO_CHANGE
                                )
    );

    ESP_ERROR_CHECK(uart_driver_install(  
                                          UART1_PORT, 
                                          256, 
                                          0, 
                                          0, 
                                          NULL, 
                                          0
                                        )
    );

    uart_config_t uart2_config = {
                                  .baud_rate = 115200,
                                  .data_bits = UART_DATA_8_BITS,
                                  .parity = UART_PARITY_DISABLE,
                                  .stop_bits = UART_STOP_BITS_1,
                                  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                                  .source_clk = UART_SCLK_DEFAULT
                                };

  ESP_ERROR_CHECK(uart_param_config(UART2_PORT, &uart2_config)); 

  ESP_ERROR_CHECK(uart_set_pin( 
                                  UART2_PORT, 
                                  UART2_TX, 
                                  UART2_RX, 
                                  UART_PIN_NO_CHANGE, 
                                  UART_PIN_NO_CHANGE)
                                );

  ESP_ERROR_CHECK(uart_driver_install(  
                                        UART2_PORT, 
                                        1024, 
                                        0, 
                                        0, 
                                        NULL, 
                                        0
                                      )
  ); 

  #if !CAN_MODO_TESTE

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT( GPIO_NUM_17, 
                                                                  GPIO_NUM_5, 
                                                                  TWAI_MODE_NORMAL
                                                                );

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t resultado = twai_driver_install(&g_config, &t_config, &f_config);

      if (resultado == ESP_OK) {
          printf("Driver installed\n");
      } else {
          printf("Failed to install driver\n");
          return;
      }  

      if (twai_start() == ESP_OK) {
          printf("Driver started\n");
      } else {
          printf("Failed to start driver\n");
          return;
      }

      BaseType_t task_criada = xTaskCreate(
        taskReceberCAN,
        "taskReceberCAN",
        4096,
        NULL,
        5,
        NULL
    );

    if (task_criada != pdPASS) {
        printf("[CAN RX] Erro ao criar task de recepcao\n");
        return;
    }

  #endif


  BaseType_t task_result = xTaskCreate(
                                    taskControle,
                                    "taskControle",
                                    4096,
                                    &control_ctx,
                                    5,
                                    &taskControleHandle
                                  );

  if( task_result != pdPASS )
    {
        printf("Erro ao Criar a taskControle\n");
        abort();
    }

  task_result = xTaskCreate(
                          taskSensores,
                          "taskSensores",
                          4096,
                          &sensor_ctx,
                          5,
                          NULL
                        );

  if( task_result != pdPASS )
    {
        printf("Erro ao Criar a taskSensores\n");
        abort();
    }


  task_result = xTaskCreate(
                          taskBotao,
                          "taskBotao",
                          2048,
                          NULL,
                          5,
                          &taskBotaoHandle
                        );

  if( task_result != pdPASS )
    {
        printf("Erro ao Criar a taskBotao\n");
        abort();
    }

  task_result = xTaskCreate(
                          taskTelemetria,
                          "taskTelemetria",
                          2048,
                          fila_telemetria,
                          5,
                          &taskTelemetriaHandle
                        );

  if( task_result != pdPASS )
    {
        printf("Erro ao Criar a taskTelemetria\n");
        abort();
    }    

    task_result = xTaskCreate(
                                taskLerTelemetria,
                                "taskLerTelemetria",
                                2048,
                                NULL,
                                5,
                                NULL
                            );

    if (task_result != pdPASS) {
        printf("Erro ao criar taskLerTelemetria\n");
        abort();
    }

  ESP_ERROR_CHECK(gpio_isr_handler_add(
                                        BOTAO,
                                        botao_isr,
                                        NULL
                                      )
  );


}