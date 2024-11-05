#include "stm32f4xx.h"

// Function to toggle the LED
void SignalExcep(void) {
    GPIOA->ODR ^= (1 << 5);
	  GPIOA->ODR ^= (1 << 6);
}

// Configure the MPU to generate a Memory Management Fault
void configureMPU(void) {
    // Disable MPU before configuration
    MPU->CTRL = 0;

    // Configure MPU region for no access
    MPU->RNR = 0; // Region number 0
    MPU->RBAR = 0x20000000; // Base address (e.g., start of SRAM)
    MPU->RASR = (0x0 << MPU_RASR_AP_Pos)  // No access permission
                | (0x1F << MPU_RASR_SIZE_Pos) // Set region size (e.g., 512MB)
                | MPU_RASR_ENABLE_Msk; // Enable the region

    // Enable MPU with default memory map background region disabled
    MPU->CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk;

    // Enable Memory Management Fault
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
}

// Function to trigger a Hard Fault by division by zero
void triggerHardFault(void) {
		SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk; // Set the DIV_0_TRP bit to trap divide-by-zero errors
    volatile int divisor = 0;
    volatile int result = 10 / divisor; // Division by zero to trigger Hard Fault
	  (void)result;
}

// Function to trigger a Memory Management Fault
void triggerMemManageFault(void) {
    // Access the MPU-protected region to trigger a Memory Management Fault
    volatile int *pProtectedRegion = (int *)0x20000000; // Protected region
    *pProtectedRegion = 0; // Write to restricted memory region
}

// Function to trigger a Bus Fault
void triggerBusFault(void) {
    // Enable Bus Fault handler
    SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
    
    // Access an invalid peripheral address to trigger a Bus Fault
    volatile int *pInvalidPeriphAddress = (int *)0x60000000; // Invalid peripheral address
    *pInvalidPeriphAddress = 0; // Write to invalid peripheral space
}

// Function to trigger a Usage Fault
void triggerUsageFault(void) {
    // Enable Usage Fault and Divide-by-Zero trap
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk; // Set the DIV_0_TRP bit to trap divide-by-zero errors
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk; // Enable Usage Fault
    
    // Perform a divide-by-zero operation to trigger the Usage Fault
    volatile int a = 10;
    volatile int b= 0;
    volatile int c = a / b; // This will now trigger a Usage Fault

    (void)c; // Prevent unused variable warning
}

// Function to trigger a PendSV Fault
void triggerPendSV(void) {
    // Set PendSV interrupt pending to trigger PendSV_Handler
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

// Hard Fault Handler
void HardFault_Handler(void) {
    while (1) {
        SignalExcep(); // Signal to indicate Hard Fault
        for (volatile int i = 0; i < 100000; i++); // Simple delay
    }
}

// Memory Management Fault Handler
void MemManage_Handler(void) {
    while (1) {
        SignalExcep(); // Signal to indicate Memory Management Fault
        for (volatile int i = 0; i < 100000; i++); // Simple delay
    }
}

// Bus Fault Handler
void BusFault_Handler(void) {
    while (1) {
        SignalExcep(); // Signal to indicate Bus Fault
        for (volatile int i = 0; i < 100000; i++); // Simple delay
    }
}

// Usage Fault Handler
void UsageFault_Handler(void) {
    while (1) {
        SignalExcep(); // Signal to indicate Usage Fault
        for (volatile int i = 0; i < 100000; i++); // Simple delay
    }
}
// PendSV Handler
void PendSV_Handler(void) {
    while (1) {
        SignalExcep(); // Signal to indicate PendSV
        for (volatile int i = 0; i < 100000; i++); // Simple delay
    }
}

// Example system initialization function
void initializeSystem(void) {
    // Enable the GPIO clock for port A
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PC13 as output (LED)
    GPIOA->MODER |= (1 << (5 * 2)); // Set mode to output
		GPIOA->MODER |= (1 << (6 * 2));
    
    
}

// Main function
int main(void) {
    // Initialize system (e.g., configure clocks, peripherals)
    initializeSystem();
     //configureMPU(); // Configure MPU for Memory Management Faults

    // Infinite loop
    while (1) {
        // Uncomment one of the following lines to test specific faults
				// triggerHardFault();
        // triggerBusFault();
        // triggerMemManageFault();
        // triggerUsageFault();
        // triggerPendSV();
    }
}
