/*
* Interrupts handling demo.
* 
* Set LED connected to PC13 on/off 
* when ecnoder button is pressed or encoder rotated.
*
* Encoder pins state change on falling edge is linked 
* to interrupt handler.
*/
#include <stm32f1xx.h>

#define BUTTON_PORT   GPIOC
#define BUTTON_PIN    14
#define ENCODER_PORT   GPIOC
#define ENCODER_PIN   15
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

  // Set PC14 and PC15: input with pull-up
  GPIOC->CRH &= ~(
      GPIO_CRH_MODE14 | GPIO_CRH_CNF14
    | GPIO_CRH_MODE15 | GPIO_CRH_CNF15
  );
  GPIOC->CRH |= GPIO_CRH_CNF14_1 | GPIO_CRH_CNF15_1; // CNF14/CNF15 = 10 → input with PU/PD
  GPIOC->ODR |= (1 << BUTTON_PIN); // Enable pull-up for PC14
  GPIOC->ODR |= (1 << ENCODER_PIN); // Enable pull-up for PC14
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
  AFIO->EXTICR[3] &= ~(AFIO_EXTICR4_EXTI14 | AFIO_EXTICR4_EXTI15);
  AFIO->EXTICR[3] |=  AFIO_EXTICR4_EXTI14_PC | AFIO_EXTICR4_EXTI15_PC;
  // Enable the EXTI_14 interrupt.
  EXTI->IMR |= EXTI_IMR_MR14 | EXTI_IMR_MR15;
  // Setup EXTI interrupts for falling input on the button pin PC14.
  // Disable the 'rising edge' trigger (button release).
  EXTI->RTSR &= ~(EXTI_RTSR_TR14 | EXTI_RTSR_TR15);
  // Enable the 'falling edge' trigger (button press).
  EXTI->FTSR |= EXTI_FTSR_TR14 | EXTI_FTSR_TR15;
  // Set interrupt at minimum priority.
  NVIC_SetPriority(EXTI15_10_IRQn, 0x03);
  // Enable the NVIC interrupt.
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/*
 * Main program.
 */
int main(void) {
  // Enable clock for the GPIOC and AFIO peripheral.
  clock_init();

  // Initialize PC13 as push-pull output and PC14 as pull-up input.
  gpio_init();
  
  led_off();

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
* Handles the button press or encoder rotate interrupt.
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

  // Check that EXTI 14 is the line that triggered.
  if(EXTI->PR & EXTI_PR_PR15) 
  {
    // If it was, clear the interrupt flag.
    EXTI->PR |= EXTI_PR_PR15; // Clear by writing 1 to pin 14 position.
    // Toggle the LED target state.
    target_led_state = !target_led_state;
  }
}
