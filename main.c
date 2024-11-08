#include "stm32f4xx.h"  // Adjust header if needed

#define UART_BAUDRATE 9600

#define M 0.0013    // Slope from calibration (example value)
#define B -0.49

volatile float weight;

float CalculateWeight(uint16_t adc_value) {
    return (M * adc_value) + B;
}

void ADC_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;     // Enable ADC1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // Enable GPIOA clock

    GPIOA->MODER |= GPIO_MODER_MODE1;       // Set PA1 to analog mode
    ADC1->CR2 = 0;                          // Disable ADC to configure
    ADC1->SQR3 = 1;                         // Set channel 1 for conversion
    ADC1->CR2 |= ADC_CR2_ADON;              // Enable ADC
}

uint16_t ADC_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;           // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));       // Wait for conversion to complete
    return ADC1->DR;                        // Return ADC value
}

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
	
		USART1->CR1 |= USART_CR1_RXNEIE;  // Enable RXNE interrupt
    NVIC_EnableIRQ(USART1_IRQn);       // Enable USART1 interrupt in NVIC
}

void UART_SendChar(char c) {
    while (!(USART1->SR & USART_SR_TXE));   // Wait until transmit data register is empty
    USART1->DR = c;                         // Send character
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

// Function to convert integer to string
void IntToStr(uint16_t value, char *str) {
    int i = 0;
    do {
        str[i++] = (value % 10) + '0';   // Get last digit and convert to character
        value /= 10;
    } while (value);

    str[i] = '\0';                       // Null-terminate the string

    // Reverse the string
    for (int j = 0; j < i / 2; j++) {
        char temp = str[j];
        str[j] = str[i - j - 1];
        str[i - j - 1] = temp;
    }
}

void Float_To_String(float value, char* str) {
		int intPart = (int)value;
    float decimalPart = value - intPart;
    int decimalInt = (int)(decimalPart * 1000); // Scale up to 3 decimal places

    int index = 0;

    // Convert the integer part
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

    // Convert the decimal part
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

    // Append units
    str[index++] = ' ';
    str[index++] = 'k';
    str[index++] = 'g';
    str[index++] = '\n';
    str[index] = '\0'; // Null-terminate the string
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

void USART1_IRQHandler(void) {
    // Check if the RXNE flag is set, indicating data received
    if (USART1->SR & USART_SR_RXNE) {
        char received = USART1->DR;  // Read the received data, clears RXNE flag
        if (received == 'O') {
            float difference = 3.0 - weight;
						if (difference<0){
							difference=0.0;
						}
            char buffer[20];  // Buffer for the string
            Float_To_String(difference, buffer);  // Convert difference to string
            UART_Send_String(buffer);  // Send the buffer over UART
						UART_Send_String("To be ordered\r\n");
						UART_SendChar('\r');                 // Newline for readability
						UART_SendChar('\n');  
        }
    }
}

int main(void) {
    ADC_Init();
    UART_Init();
	
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    GPIOA->MODER &= ~((3UL << (2 * 5)) | (3UL << (2 * 6)));
    GPIOA->MODER |= (1UL << (2 * 5)) | (1UL << (2 * 6));

    char buffer[10];

    while (1) {
        uint16_t adc_value = ADC_Read();    // Read FSR value
        weight = CalculateWeight(adc_value);  // Convert ADC value to weight
				if (weight<0.0){
					weight=0.0;
				}
				Float_To_String(weight,buffer);

        //IntToStr(adc_value, buffer);         // Convert ADC value to string manually
        UART_Send_String(buffer);             // Send ADC value over Bluetooth
				if (weight < 1.0) {             // 1 kg threshold
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
				
				
				UART_SendChar('\r');                 // Newline for readability
				UART_SendChar('\n');  
				
        delay_ms(500);
    }
}
