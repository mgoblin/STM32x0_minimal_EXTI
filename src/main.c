#include <stm32f1xx.h>

#define BUTTON_PORT   GPIOC
#define BUTTON_PIN    14
#define BUTTON_EXTI   EXTI14
#define LED_PORT      GPIOC
#define LED_PIN       13

// Task LED target state. 
// This variable is used to communicate between irq handler and main.
volatile uint8_t target_led_state = 0;

/*
* Enables the clock for GPIO Port C and Alternate Function I/O (AFIO) peripheral.
*/
static void clock_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // Enable clock for PORT C
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable clock for AFIO
}

/*
* GPIO initialization
*
* Setup LED1 pin as push-pull output and Button pin as pull-up input
*/
static void gpio_init(void)
{
  // Next two code lines set PC13: output mode, 2 MHz, push-pull
  GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13); // set push-pull by clearing GPIO_CRH_CNF13 bits
  GPIOC->CRH |= GPIO_CRH_MODE13_1; // set output mode with 2 MHz speed

  // Set PC14: input with pull-up
  GPIOC->CRH &= ~(GPIO_CRH_MODE14 | GPIO_CRH_CNF14);
  GPIOC->CRH |= GPIO_CRH_CNF14_1; // CNF14 = 10 → input with PU/PD
  GPIOC->ODR |= (1 << BUTTON_PIN); // Enable pull-up
}

/*
 * Turns LED to active state.
 * LED is active-low and connected to the PC13.
 *
 * This function clears the corresponding bit in the GPIOC ODR (Output Data Register)
 * to drive the LED pin low, which lights on the LED connected to LED_PIN.
 */
static void led_on(void)
{
  GPIOC->ODR &= ~(1 << LED_PIN);
}

/*
 * Turns LED to passive state.
 * LED is active-low and connected to the PC13.
 *
 * This function sets the corresponding bit in the GPIOC ODR (Output Data Register)
 * to high, which lights off the LED connected to LED_PIN. 
 */
static void led_off(void)
{
  GPIOC->ODR |= (1 << LED_PIN);
}

/*
* Interrupt initialization on PC14 falling edge.
*/
static void interrupt_init(void)
{
  // Connect the EXTI_14 line to pin PC14.
  // AFIO->EXTICR[3] is the register for AFIO_EXTICR4, that hosts
  // AFIO->EXTICR[index] is a form to access EXTICR 1-4 registers.
  // AFIO->EXTICR[0] is a synonym for AFIO_EXTICR1 and so on.
  // EXTI_14 has inputs from pins 14 of all ports.
  // To select PC as input, we need to write 0b0010 to EXTICR4
  // pin 14 position 4-bits.
  // Clear the EXTI_14 port selection bits 4-bits 
  // and set 0b0010 for select PC as input.
  AFIO->EXTICR[3] &= ~(AFIO_EXTICR4_EXTI14);
  AFIO->EXTICR[3] |=  AFIO_EXTICR4_EXTI14_PC;
  // Enable the EXTI_14 interrupt.
  EXTI->IMR |= EXTI_IMR_MR14;
  // Setup EXTI interrupts for falling input on the button pin PC14.
  // Disable the 'rising edge' trigger (button release).
  EXTI->RTSR &= ~(EXTI_RTSR_TR14);
  // Enable the 'falling edge' trigger (button press).
  EXTI->FTSR |= EXTI_FTSR_TR14;
  // Set interrupt at minimum priority.
  NVIC_SetPriority(EXTI15_10_IRQn, 0x03);
  // Enable the NVIC interrupt.
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/*
 * Main program.
 */
int main(void) {
  // Initialize global variables.
  led_off();

  // Enable clock for the GPIOC and AFIO peripheral.
  clock_init();

  // Initialize PC13 as push-pull output and PC14 as pull-up input.
  gpio_init();

  // Initialize interrupts.
  interrupt_init();

  // Light the button only if it should be on.
  while (1) {
    // Arrange the LED state with target.
    target_led_state ? led_on() : led_off();
    //__WFI(); Uncomment this line get stuck STM32F103 MCU. Known issue.
  }
}

/*
* Interrupt handler for EXTI15_10_IRQn.
*
* Handles the button press interrupt.
*/
void EXTI15_10_IRQHandler(void)
{
  // Check that EXTI 14 is the line that triggered.
  if(EXTI->PR & EXTI_PR_PR14) 
  {
    // If it was, clear the interrupt flag.
    EXTI->PR |= EXTI_PR_PR14; // Clear by writing 1 to pin 14 position.
    // Toggle the LED target state.
    target_led_state = !target_led_state;
  }
}
