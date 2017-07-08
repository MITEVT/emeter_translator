#include "chip.h"
#include "sysinit.h"
#include <string.h>
#include <stdlib.h>

const uint32_t OscRateIn = 24000000;

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/


#define LED0 2, 8
#define LED1 2, 10

#define LINE_BUF_SIZE 128
#define UART_RX_BUFFER_SIZE 8

volatile uint32_t msTicks;

static uint8_t uart_rx_buf[UART_RX_BUFFER_SIZE];
static uint8_t line_buf[LINE_BUF_SIZE];
static uint8_t line_buf_count;

typedef struct {
	int32_t mAh;
	int32_t mV;
	int32_t mA;
	int32_t mph;
	int32_t m;
} emeter_state_t;

static emeter_state_t emeter_state;

#ifdef DEBUG_ENABLE
	#define DEBUG_Print(str) Chip_UART_SendBlocking(LPC_USART, str, strlen(str))
	#define DEBUG_Write(str, count) Chip_UART_SendBlocking(LPC_USART, str, count)
#else
	#define DEBUG_Print(str)
	#define DEBUG_Write(str, count) 
#endif

/*****************************************************************************
 * Private functions
 ****************************************************************************/

void SysTick_Handler(void) {
	msTicks++;
}

static void LED_Init(uint8_t port, uint8_t pin) {
	Chip_GPIO_WriteDirBit(LPC_GPIO, port, pin, true);
	Chip_GPIO_SetPinState(LPC_GPIO, port, pin, false);

}

static void LED_Write(uint8_t port, uint8_t pin, uint8_t val) {
	Chip_GPIO_SetPinState(LPC_GPIO, port, pin, val);
}

/* String of ASCII digits, possibly
 * preceded by white space.  For bases
 * greater than 10, either lower- or
 * upper-case digits may be used.
 */
int32_t my_atoi(uint8_t *string)	{
    int32_t result = 0;
    uint8_t digit;
    uint8_t sign;

    /*
     * Check for a sign.
     */

    if (*string == '-') {
		sign = 1;
		string += 1;
    } else {
		sign = 0;
		if (*string == '+') {
		    string += 1;
		}
    }

    for ( ; ; string += 1) {
		digit = *string - '0';
		if (digit > 9) {
		    break;
		}
		result = (10*result) + digit;
    }

    if (sign) {
		return -result;
    }
    return result;
}

// Sample line 
// 3.128	42.12	10.04	15.32	8.9132

int8_t str2milli(uint8_t *str, int32_t *num) {
	uint8_t idx = 0;
	// bool negative;
	// if (str[idx] == '-') {
	// 	negative = true;
	// 	idx++;
	// }

	uint8_t dot_idx = 0;
	while (str[idx] != '\0') {
		if (str[idx] == '.') {
			dot_idx = idx;
		}
		idx++;
	}

	if (dot_idx == 0) return -1; // Not integer? Parse error
	
	uint8_t first[5], second[5];
	uint8_t len = idx;
	uint8_t int_len = dot_idx;
	uint8_t dec_len = len - dot_idx - 1;

	// Split integer and decimal
	memcpy(first, str, int_len);
	memcpy(second, str+int_len+1, dec_len);
	first[int_len] = '\0';
	second[dec_len] = '\0';

	// Fix decimal to be 3 digits for milli
	if (dec_len < 3) {
		memset(second+dec_len, '0', 3-dec_len);
	} else if (dec_len > 3) {
		second[3] = '\0';
	}

	int32_t num_first = my_atoi(first) * 1000;
	int32_t num_dec = my_atoi(second);

	if (num_first < 0) {
		*num = num_first - num_dec;
	} else {
		*num = num_first+num_dec;
	}
	
	return 0;

}

int8_t parse_line(void) {
	if (line_buf_count < 2) {
		return -1; // Strange prob just second new line character
	}


	uint8_t data[5][10]; // amp_hours, volts, amps, speed, distance;

	int i = 0, last_ind = 0, tab_count = 0;
	while (1) { // tokenize?
		if (line_buf[i] == '\t') {
			memcpy(data[tab_count], line_buf+last_ind, i - last_ind);
			data[tab_count][i - last_ind] = '\0';
			tab_count++;
			last_ind = i + 1;
		}
		i++;
		if (i == line_buf_count) {
			if (tab_count != 4) {
				return -2; // Parse error
			}
			memcpy(data[tab_count], line_buf+last_ind, i - last_ind);
			break;
		}
	}

	str2milli(data[0], &emeter_state.mAh);
	str2milli(data[1], &emeter_state.mV);
	str2milli(data[2], &emeter_state.mA);
	str2milli(data[3], &emeter_state.mph);
	str2milli(data[4], &emeter_state.m);

	line_buf_count = 0;

	return 0;
}


int main(void)
{
	SysTick_Config (TicksPerMS);

	LPC_SYSCTL->CLKOUTSEL = 0x03; 		// Main CLK (Core CLK) Out
	LPC_SYSCTL->CLKOUTUEN = 0x00; 		// Toggle Update CLKOUT Source
	LPC_SYSCTL->CLKOUTUEN = 0x01;
	while(!(LPC_SYSCTL->CLKOUTUEN & 0x1)); // Wait until updated
	LPC_SYSCTL->CLKOUTDIV = 0x04; 		// No division

	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_1, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_OPENDRAIN_EN)); /*CLKOUT*/

	//---------------
	//UART
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT));/* RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT));/* TXD */

	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, UART_BAUD);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);

	line_buf_count = 0;
	//---------------

	DEBUG_Print("Started up\n\r");

	LED_Init(LED0);
	LED_Init(LED1);

	LED_Write(LED0, true);

	int i;
	while (1) {
		uint8_t count;
		if ((count = Chip_UART_Read(LPC_USART, uart_rx_buf, UART_RX_BUFFER_SIZE)) != 0) {
			// Chip_UART_SendBlocking(LPC_USART, uart_rx_buf, count);
			for (i = 0; i < count; i++) {
				switch(uart_rx_buf[i]) {
					case '\r':
					case '\n':
						// Found new line, pop everything in ring buffer and begin parse
						parse_line();
						break;
					default:
						line_buf[line_buf_count] = uart_rx_buf[i];
						line_buf_count++;
						break;
				}
			}
		}

		// if (msTicks - last_count > 1000) {
		// 	last_count = msTicks;
		// 	DEBUG_Print("PING\r\n");
		// }
	}
}
