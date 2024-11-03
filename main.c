#include "stm32f4xx.h"

// Define HX711 pins
#define HX711_DT_PIN       8   // PB8
#define HX711_SCK_PIN      9   // PB9

#define SCALE_FACTOR       0.000000003  // Replace with your calibration factor
#define OFFSET             8392000 // Set this value after initial calibration

#define UART_BAUDRATE 9600

void UART_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;     // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;    // Enable USART1 clock

    GPIOA->MODER &= ~(3UL << (2 * 9));       // Clear mode bits for PA9
    GPIOA->MODER |= (2UL << (2 * 9));        // Set PA9 to Alternate function
    GPIOA->AFR[1] |= (7 << (4 * (9 - 8)));   // Set AF7 (USART1_TX) for PA9

    GPIOA->MODER &= ~(3UL << (2 * 10));      // Clear mode bits for PA10
    GPIOA->MODER |= (2UL << (2 * 10));       // Set PA10 to Alternate function
    GPIOA->AFR[1] |= (7 << (4 * (10 - 8)));  // Set AF7 (USART1_RX) for PA10

    USART1->BRR = (SystemCoreClock/UART_BAUDRATE); // Set baud rate
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable TX, RX, and USART1
}

void UART_Send_String(char* str) {
    while (*str) {
        while (!(USART1->SR & USART_SR_TXE)); // Wait until TXE is set
        USART1->DR = *str++; // Send character
    }
}

char UART_Receive(void) {
    while (!(USART1->SR & USART_SR_RXNE));  // Wait until RXNE is set
    return USART1->DR;
}

void HX711_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Configure PB8 as input (DT)
    GPIOB->MODER &= ~(3UL << (2 * HX711_DT_PIN));
    
    // Configure PB9 as output (SCK)
    GPIOB->MODER |= (1UL << (2 * HX711_SCK_PIN));
    GPIOB->ODR &= ~(1UL << HX711_SCK_PIN); // Set SCK low initially

    // Configure EXTI for PB8 (DT pin) to trigger interrupt on falling edge
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PB; // Map EXTI8 to PB8
    EXTI->IMR |= (1UL << HX711_DT_PIN);           // Unmask EXTI8
    EXTI->FTSR |= (1UL << HX711_DT_PIN);          // Trigger on falling edge for EXTI8

    // Enable NVIC interrupt for EXTI9_5 (covers EXTI5 to EXTI9)
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void delay_ms(uint32_t ms) {
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 5;
    for (uint32_t i = 0; i < ms; i++) {
        while (!(SysTick->CTRL & (1 << 16)));
    }
    SysTick->CTRL = 0;
}

uint32_t HX711_Read(void) {
    uint32_t count = 0;
    GPIOB->ODR &= ~(1UL << HX711_SCK_PIN);

    //while (GPIOB->IDR & (1UL << HX711_DT_PIN));

    for (int i = 0; i < 24; i++) {
        GPIOB->ODR |= (1UL << HX711_SCK_PIN);  // Set SCK high
        count = count << 1;
        GPIOB->ODR &= ~(1UL << HX711_SCK_PIN); // Set SCK low
        if (GPIOB->IDR & (1UL << HX711_DT_PIN)) {
            count++;
        }
    }

    GPIOB->ODR |= (1UL << HX711_SCK_PIN);
    GPIOB->ODR &= ~(1UL << HX711_SCK_PIN); // Set SCK low again

    return count;
}

float Get_Weight(void) {
    uint32_t raw_value = HX711_Read();
    float weight = (float)(raw_value) * SCALE_FACTOR;
    return weight;
}

void Float_To_String(float value, char* str) {
    int intPart = (int)value;
    float decimalPart = value - intPart;
    int decimalInt = (int)(decimalPart * 1000);

    int index = 0;

    if (intPart == 0) {
        str[index++] = '0';
    } else {
        int reverseIntPart = 0, tempInt = intPart;
        while (tempInt > 0) {
            reverseIntPart = reverseIntPart * 10 + tempInt % 10;
            tempInt /= 10;
        }
        while (reverseIntPart > 0) {
            str[index++] = (reverseIntPart % 10) + '0';
            reverseIntPart /= 10;
        }
    }

    str[index++] = '.'; // Decimal point

    if (decimalInt < 10) {
        str[index++] = '0';
        str[index++] = '0';
        str[index++] = '0' + decimalInt;
    } else if (decimalInt < 100) {
        str[index++] = '0';
        str[index++] = '0' + (decimalInt / 10);
        str[index++] = '0' + (decimalInt % 10);
    } else if (decimalInt < 1000) {
        str[index++] = '0' + (decimalInt / 100);
        str[index++] = '0' + ((decimalInt / 10) % 10);
        str[index++] = '0' + (decimalInt % 10);
    } else {
        str[index++] = '0' + (decimalInt / 1000);
        str[index++] = '0' + ((decimalInt / 100) % 10);
        str[index++] = '0' + ((decimalInt / 10) % 10);
        str[index++] = '0' + (decimalInt % 10);
    }

    str[index++] = ' ';
    str[index++] = 'k';
    str[index++] = 'g';
    str[index++] = '\n';
    str[index] = '\0';
}

void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & (1UL << HX711_DT_PIN)) {  // Check if the interrupt is from PB8 (DT)
        float weight = Get_Weight();
        char buffer[50];
	if (USART1->SR & USART_SR_RXNE) {
            char received = UART_Receive();
            if (received == 'O') {
                float difference = 0.5 - weight;
                Float_To_String(difference,buffer);
                UART_Send_String(buffer);
            }
        }
			
        Float_To_String(weight, buffer);
        UART_Send_String(buffer);
			

        if (weight < 0.04) {             // 300 grams threshold
            GPIOA->ODR &= ~(1UL << 5);   // Turn off LED at PA5
            GPIOA->ODR |= (1UL << 6);    // Turn on LED at PA6
						if (weight==0.000){
							UART_Send_String("Out of stock\r\n");
						}else{
							UART_Send_String("Only few in stock\r\n");
						}
        } else {
            GPIOA->ODR &= ~(1UL << 6);   // Turn off LED at PA6
            GPIOA->ODR |= (1UL << 5);    // Turn on LED at PA5
						UART_Send_String("In Stock\r\n");
        }

        EXTI->PR |= (1UL << HX711_DT_PIN);  // Clear the EXTI interrupt pending bit
    }
}

// Exception handlers
void HardFault_Handler(void) {
    UART_Send_String("Hard Fault occurred!\r\n");
    while (1);  // Stay in this loop to halt the program
}

void MemManage_Handler(void) {
    UART_Send_String("Memory Management Fault occurred!\r\n");
    while (1);
}

void BusFault_Handler(void) {
    UART_Send_String("Bus Fault occurred!\r\n");
    while (1);
}

void UsageFault_Handler(void) {
    UART_Send_String("Usage Fault occurred!\r\n");
    while (1);
}

// Functions to trigger each exception

void Trigger_Reset(void) {
    NVIC_SystemReset();  // Triggers a system reset
}

void Trigger_HardFault(void) {
    volatile int *p = (int*)0x0;  // Null pointer dereference
    *p = 42;                      // This will cause a Hard Fault
}

void Trigger_MemoryManagementFault(void) {
    volatile int *p = (int*)0xE000ED90; // Invalid memory address in system region
    *p = 0;  // This will cause a Memory Management Fault
}

void Trigger_BusFault(void) {
    volatile int *p = (int*)0x40000002; // Unaligned access to a peripheral address
    *p = 1;  // This will cause a Bus Fault
}

void Trigger_UsageFault(void) {
    volatile int a = 5;
    volatile int b = 0;
    volatile int c = a / b; // Division by zero, causes a Usage Fault
}

int main(void) {
    HX711_Init();
    UART_Init();

    // Trigger_Reset();               // Uncomment to trigger a reset
    // Trigger_HardFault();           // Uncomment to trigger a hard fault
    // Trigger_MemoryManagementFault(); // Uncomment to trigger a memory management fault
    // Trigger_BusFault();            // Uncomment to trigger a bus fault
    // Trigger_UsageFault();  

    while (1) {
        // Main loop does other tasks; HX711 handled via interrupt on PB8.
    }
}
