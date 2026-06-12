#include <MeMegaPi.h>
#include <Arduino_FreeRTOS.h>
#include "queue.h"
#include "semphr.h"

// Definición de Baudrate para registros UART0 del ATmega2560
#define F_CPU 16000000UL
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

// Mapeo del pin A12 en el ATmega2560 (Puerto K, bit 4 //-> PCINT20)
#define SWITCH_PIN_BIT   PK4  
#define SWITCH_DDR       DDRK
#define SWITCH_PORT      PORTK
#define SWITCH_PIN_REG   PINK

// Instancias de actuadores (De la librería MeMegaPi)
MeMegaPiDCMotor motor_1(1);
MeMegaPiDCMotor motor_9(9);
MeMegaPiDCMotor motor_2(2);
MeMegaPiDCMotor motor_10(10);

int16_t velocidad_base = 80; // Velocidad de avance constante (vy)

// --- Handles de FreeRTOS ---
QueueHandle_t xAngleQueue;
SemaphoreHandle_t xEmergencySemaphore; 

// Prototipos de funciones
void vMoveControlTask(void *pvParameters);
void vUARTCommunicationTask(void *pvParameters);
void move_control(int16_t vx, int16_t vy, int16_t vw);
void USART_Transmit_String(const char *pdata);
int16_t aplicar_constraints(int16_t velocidad);

void motor_foward_left_run(int16_t speed)  { motor_10.run(-speed); }
void motor_foward_right_run(int16_t speed) { motor_1.run(speed); }
void motor_back_left_run(int16_t speed)    { motor_2.run(-speed); }
void motor_back_right_run(int16_t speed)   { motor_9.run(speed); }

void setup() {
    // 1. Configuración del puerto serial UART0 por registros
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0);  

    // 2. Configuración de Pin A12 (PK4) como entrada con Pull-Up
    SWITCH_DDR &= ~(1 << SWITCH_PIN_BIT);   
    SWITCH_PORT |= (1 << SWITCH_PIN_BIT);   

    // 3. Inicialización de objetos FreeRTOS
    xAngleQueue = xQueueCreate(5, sizeof(int16_t));
    xEmergencySemaphore = xSemaphoreCreateBinary();

    // 4. Configuración de la Interrupción PCINT2
    if(xEmergencySemaphore != NULL) {
        // Habilitar grupo de interrupciones 2 (Puerto K)
        PCICR |= (1 << PCIE2);
        // Habilitar interrupción específica PCINT20 (Pin A12)
        PCMSK2 |= (1 << PCINT20);
    }

    // Ajustes de temporizadores originales para los PWM del MeMegaPi
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);
    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
    
    // 5. Creación de tareas
    if (xAngleQueue != NULL && xEmergencySemaphore != NULL) {
        xTaskCreate(vMoveControlTask, "MOVE CONTROL", 192, NULL, 2, NULL);
        xTaskCreate(vUARTCommunicationTask, "UART COMM", 192, NULL, 1, NULL);
    }
    
    // Habilitar interrupciones globales
    sei();
}

// ISR (Rutina de Interrupción) para el grupo PCINT2_vect (Puerto K / Pin A12)
ISR(PCINT2_vect) {
    // Verificar si el pin A12 está en estado LOW (switch presionado)
    if (!(SWITCH_PIN_REG & (1 << SWITCH_PIN_BIT))) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        
        // Entregar semáforo desde la interrupción
        xSemaphoreGiveFromISR(xEmergencySemaphore, &xHigherPriorityTaskWoken);
        
        // Forzar cambio de contexto si se despertó una tarea de mayor prioridad
        if (xHigherPriorityTaskWoken == pdTRUE) {
            taskYIELD();
        }
    }
}

// Tarea 1: Control cinemático (Handler Task para la interrupción)
void vMoveControlTask(void *pvParameters) {
    int16_t angulo_recibido = 0;
    
    while(1) {
        // A) Revisar Semáforo de Emergencia (Giro 180°)
        if (xSemaphoreTake(xEmergencySemaphore, 0) == pdPASS) {
            // Rutina de 180° activada instantáneamente
            move_control(0, 0, 80); 
            vTaskDelay(pdMS_TO_TICKS(1100)); // Retardo no bloqueante
            
            move_control(0, 0, 0); 
            vTaskDelay(pdMS_TO_TICKS(200));

            // Limpiar la cola para desechar comandos enviados durante el giro
            xQueueReset(xAngleQueue); 
        }

        // B) Leer comandos de dirección desde la Cola (Enviados por UART)
        if (xQueueReceive(xAngleQueue, &angulo_recibido, pdMS_TO_TICKS(20)) == pdPASS) {
            int16_t vy = velocidad_base;
            int16_t vw = angulo_recibido; // El ángulo de la cámara se traduce a velocidad angular
            
            move_control(0, vy, vw);
        }
    }
}

// Tarea 2: Recepción UART segura y sin bloqueos
void vUARTCommunicationTask(void *pvParameters) {
    char rxBuffer[16];
    uint8_t bufferIdx = 0;
    
    while(1) {
        // Revisa si hay nuevo dato RX en UART
        if((UCSR0A & (1 << RXC0)) != 0) {
            char incomingByte = UDR0;
            
            // Salto de línea indica fin del dato enviado desde Python
            if (incomingByte == '\n' || incomingByte == '\r' || bufferIdx >= 15) {
                if (bufferIdx > 0) {
                    rxBuffer[bufferIdx] = '\0';
                    
                    // Convertir el string recibido a entero (ángulo de conducción)
                    int16_t angulo_conduccion = atoi(rxBuffer);
                    
                    // Enviar dato procesado a la Queue
                    xQueueSend(xAngleQueue, &angulo_conduccion, 0);
                    
                    // Transmisión de validación a la Jetson Nano / PC
                    char txBuffer[30];
                    sprintf(txBuffer, "OK-> Angulo: %d\r\n", angulo_conduccion);
                    USART_Transmit_String(txBuffer);
                    
                    bufferIdx = 0; // Reiniciar buffer
                }
            } else {
                rxBuffer[bufferIdx++] = incomingByte;
            }
        }
        
        // Bloqueo ligero para evitar inanición de CPU
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

int16_t aplicar_constraints(int16_t velocidad) {
    if (velocidad > 255) return 255;
    if (velocidad < -255) return -255;
    return velocidad;
}

// Control cinemático
void move_control(int16_t vx, int16_t vy, int16_t vw) {
    vw = vw * 2.8;
    int16_t foward_left_speed  = vy + vx + vw;
    int16_t foward_right_speed = vy - vx - vw;
    int16_t back_left_speed    = vy - vx + vw;
    int16_t back_right_speed   = vy + vx - vw;

    foward_right_speed = foward_right_speed + 7; // Compensación física

    foward_left_speed  = aplicar_constraints(foward_left_speed);
    foward_right_speed = aplicar_constraints(foward_right_speed);
    back_left_speed    = aplicar_constraints(back_left_speed);
    back_right_speed   = aplicar_constraints(back_right_speed);

    motor_foward_left_run(foward_left_speed);
    motor_foward_right_run(foward_right_speed);
    motor_back_left_run(back_left_speed);
    motor_back_right_run(back_right_speed);
}

// Transmisión de UART nativa
void USART_Transmit_String(const char *pdata) {
    for (size_t i = 0; i < strlen(pdata); i++) {
        while(!(UCSR0A & (1 << UDRE0))); // Esperar a que el buffer esté vacío
        UDR0 = pdata[i];                 
    }
}

// loop vacío
void loop() {}