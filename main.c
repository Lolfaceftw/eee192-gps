/**
 * @file main.c
 * @brief Module 5 Sample: "Keystroke Hexdump"
 *
 * @author Alberto de Villa <alberto.de.villa@eee.upd.edu.ph>
 * @date 28 Oct 2024
 */

// Common include for the XC32 compiler
#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "platform.h"

/////////////////////////////////////////////////////////////////////////////

/*
 * Copyright message printed upon reset
 * 
 * Displaying author information is optional; but as always, must be present
 * as comments at the top of the source file for copyright purposes.
 * 
 * FIXME: Modify this prompt message to account for additional instructions.
 */
static const char banner_msg[] =
"\033[0m\033[2J\033[1;1H"
"+--------------------------------------------------------------------+\r\n"
"| EEE 192: Electrical and Electronics Engineering Laboratory VI      |\r\n"
"|          Academic Year 2024-2025, Semester 2                       |\r\n"
"|                                                                    |\r\n"
"| Sensor: GPS Module                                                 |\r\n"
"|                                                                    |\r\n"
"| Author:  Estrada (Supplemented by EEE 158 AY 24-25 1S)             |\r\n"
"| Date:    2025                                                      |\r\n"
"+--------------------------------------------------------------------+\r\n"
"\r\n"
"Data: ";

static const char ESC_SEQ_KEYP_LINE[] = "\033[1;1H";
static const char ESC_SEQ_IDLE_INF[]  = "\033[20;1H";

//////////////////////////////////////////////////////////////////////////////

// Program state machine
typedef struct prog_state_type
{
	// Flags for this program
#define PROG_FLAG_BANNER_PENDING        0x0001	// Waiting to transmit the banner
#define PROG_FLAG_UPDATE_PENDING        0x0002	// Waiting to transmit updates
#define PROG_FLAG_GPS_UPDATE_PENDING	0x0004	// Waiting to transmit updates
#define PROG_FLAG_GEN_COMPLETE      0x8000	// Message generation has been done, but transmission has not occurred
    
	uint16_t flags;
	
	// Transmit stuff
	platform_usart_tx_bufdesc_t tx_desc[4];
	char tx_buf[64];
	uint16_t tx_blen;
	
	// Receiver stuff
	platform_usart_rx_async_desc_t rx_desc;
	uint16_t rx_desc_blen;
	char rx_desc_buf[16];
    
    // Receive from GPS
    platform_usart_rx_async_desc_t gps_rx_desc;
    uint16_t gps_rx_desc_blen;
    char gps_rx_desc_buf[64];
    
} prog_state_t;

/*
 * Initialize the main program state
 * 
 * This style might be familiar to those accustomed to he programming
 * conventions employed by the Arduino platform.
 */
static void prog_setup(prog_state_t *ps)
{
	memset(ps, 0, sizeof(*ps));
	
	platform_init();
	
    // SERCOM3 - Keyb + PIC32
    
	ps->rx_desc.buf     = ps->rx_desc_buf;
	ps->rx_desc.max_len = sizeof(ps->rx_desc_buf);
	
	platform_usart_cdc_rx_async(&ps->rx_desc);
    
    // SERCOM1 - GPS

    ps->gps_rx_desc.buf = ps->gps_rx_desc_buf;
    ps->gps_rx_desc.max_len = sizeof(ps->gps_rx_desc_buf);
    
    gps_platform_usart_cdc_rx_async(&ps->gps_rx_desc);
	return;
}

double convert_to_decimal(const char* raw, char direction, int is_lat) {
    int deg_len = is_lat ? 2 : 3;
    char deg_str[4] = {0};
    strncpy(deg_str, raw, deg_len);
    double degrees = atof(deg_str);
    double minutes = atof(raw + deg_len);
    double decimal = degrees + (minutes / 60.0);
    if (direction == 'S' || direction == 'W') decimal *= -1;
    return decimal;
}

int parse_gpgll_to_buffer(const char* sentence, char* out_buf) {
    char buf[128];
    strncpy(buf, sentence, sizeof(buf));
    buf[sizeof(buf) - 1] = '\0';

    char* token = strtok(buf, ",");
    if (!token || strncmp(token, "$GPGLL", 6) != 0) return 0;

    char *lat = strtok(NULL, ",");
    char *lat_dir = strtok(NULL, ",");
    char *lon = strtok(NULL, ",");
    char *lon_dir = strtok(NULL, ",");
    char *time = strtok(NULL, ",");
    char *status = strtok(NULL, "*");

    if (!lat || !lat_dir || !lon || !lon_dir || !time || !status) return 0;

    double latitude = convert_to_decimal(lat, lat_dir[0], 1);
    double longitude = convert_to_decimal(lon, lon_dir[0], 0);

    sprintf(out_buf,
        "Latitude: %.6f\nLongitude: %.6f\nTime: %.2s:%.2s:%.2s\nStatus: %c\n",
        latitude, longitude, time, time+2, time+4, status[0]);

    return 1;
}

/*
 * Do a single loop of the main program
 * 
 * This style might be familiar to those accustomed to he programming
 * conventions employed by the Arduino platform.
 */
static void prog_loop_one(prog_state_t *ps)
{
	uint16_t a = 0;
	
	// Do one iteration of the platform event loop first.
	platform_do_loop_one();
	
	// Something happened to the pushbutton?
	if ((a = platform_pb_get_event()) != 0) {
		if ((a & PLATFORM_PB_ONBOARD_PRESS) != 0) {
			// Print out the banner
			ps->flags |= PROG_FLAG_BANNER_PENDING;
		}
		a = 0;
	}
	
	////////////////////////////////////////////////////////////////////
	
	// Process any pending flags (BANNER)
	do {
		if ((ps->flags & PROG_FLAG_BANNER_PENDING) == 0)
			break;
		
		if (platform_usart_cdc_tx_busy())
			break;
		
		if ((ps->flags & PROG_FLAG_GEN_COMPLETE) == 0) {
			// Message has not been generated.
			ps->tx_desc[0].buf = banner_msg;
			ps->tx_desc[0].len = sizeof(banner_msg)-1;
			ps->flags |= PROG_FLAG_GEN_COMPLETE;
		}
		
		if (platform_usart_cdc_tx_async(&ps->tx_desc[0], 1)) {
			ps->flags &= ~(PROG_FLAG_BANNER_PENDING | PROG_FLAG_GEN_COMPLETE);
		}
	} while (0);
	
    // Something from the SERCOM1 UART?
	if (ps->gps_rx_desc.compl_type == PLATFORM_USART_RX_COMPL_DATA) {
        PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 15);
        ps->flags |= PROG_FLAG_GPS_UPDATE_PENDING;
        ps->gps_rx_desc_blen = ps->gps_rx_desc.compl_info.data_len;
    }
    
    // Process any pending GPS update flags
    do {
		if ((ps->flags & PROG_FLAG_GPS_UPDATE_PENDING) == 0)
			break;
		
		if (platform_usart_cdc_tx_busy())
			break;
		
		if ((ps->flags & PROG_FLAG_GEN_COMPLETE) == 0) {
			ps->tx_desc[0].buf = ps->gps_rx_desc_buf;;
			ps->tx_desc[0].len = ps->gps_rx_desc_blen;
		}
        
		if (platform_usart_cdc_tx_async(&ps->tx_desc[0], 2)) {
			ps->gps_rx_desc.compl_type = PLATFORM_USART_RX_COMPL_NONE;
			gps_platform_usart_cdc_rx_async(&ps->gps_rx_desc);
			ps->flags &= ~(PROG_FLAG_GPS_UPDATE_PENDING | PROG_FLAG_GEN_COMPLETE);
		}
	} while (0);
	
	// Done
	return;
}

// main() -- the heart of the program
int main(void)
{
	prog_state_t ps;
	
	// Initialization time	
	prog_setup(&ps);
	
	/*
	 * Microcontroller main()'s are supposed to never return (welp, they
	 * have none to return to); hence the intentional infinite loop.
	 */
	for (;;) {
		prog_loop_one(&ps);
	}
    
    // This line must never be reached
    return 1;
}
