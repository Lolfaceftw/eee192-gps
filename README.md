# Embedded GPS NMEA GPGLL Parser and Terminal Display

## Overview

This project is an embedded C application developed for the **Microchip PIC32CM5164LS00048** microcontroller (based on the **ARM Cortex-M23** core). It demonstrates the implementation of a basic GPS data logger and parser, focusing on extracting and displaying location and time information from NMEA GPGLL sentences.

The application reads raw NMEA data from a GPS module connected via a dedicated UART interface, parses the GPGLL sentences to extract latitude, longitude, and UTC time, converts the time to a local timezone, and formats the data for human-readable output. This output is then displayed on a connected serial terminal (via the microcontroller's CDC/virtual COM port), utilizing ANSI escape codes for a structured and dynamic display.

This project serves as a practical example showcasing fundamental embedded systems development skills, including low-level peripheral configuration, asynchronous communication handling, data parsing, and application state management in a bare-metal environment.

## Features

*   **Hardware Initialization:** Configures system clocks, GPIO, SysTick timer, External Interrupt Controller (EIC), and two independent USART peripherals.
*   **Dual USART Communication:**
    *   Dedicated UART for receiving raw NMEA data from a GPS module.
    *   CDC (Communication Device Class) interface over USB for terminal output and debugging.
*   **Asynchronous I/O:** Utilizes a descriptor-based approach for asynchronous USART receive and transmit operations, allowing the main application loop to remain non-blocking.
*   **NMEA Parsing:** Specifically targets and parses **GPGLL** sentences.
*   **Data Extraction and Conversion:**
    *   Extracts Latitude, Longitude, and UTC Time from GPGLL.
    *   Converts NMEA coordinate format (DDmm.mmmm, DDDmm.mmmm) to decimal degrees.
    *   Converts UTC time to a configurable local timezone (default UTC+8).
*   **Terminal User Interface:**
    *   Displays a startup banner with project information.
    *   Displays parsed GPS data (Local Time, Latitude, Longitude) on a fixed line using ANSI cursor positioning and line clearing (configurable).
    *   Optional debug mode to print all received raw NMEA sentences.
*   **Button Interaction:** Detects a press on the evaluation board's button (SW0) to trigger a re-display of the banner.
*   **Activity Indicator:** Blinks an LED (on-board LED) upon receiving GPS data.
*   **Robustness:** Includes basic handling for buffer overflows in data assembly and formatting, and uses atomic read techniques (`volatile`, cookie pattern) for shared timing data.
*   **Development/Testing Feature:** Includes an option to use predefined fake GPGLL sentences instead of real GPS data for testing purposes (`USE_FAKE_GPS_DATA` macro).

## Hardware

*   **Microcontroller:** Microchip PIC32CM5164LS00048 (ARM Cortex-M23)
*   **Development Board:** Microchip PIC32CM Curiosity Nano Evaluation Kit or similar board featuring the PIC32CM5164LS00048.
*   **Peripherals Used:**
    *   **USART3:** Configured for CDC (Virtual COM Port) communication with a PC terminal.
        *   TX: PB08 (SERCOM3 PAD[0])
        *   RX: PB09 (SERCOM3 PAD[1])
    *   **USART1:** Configured for communication with the GPS module.
        *   RX: PB17 (SERCOM1 PAD[1]) - *Note: TX (PB16, SERCOM1 PAD[0]) is configured but not actively used by the application code if only receiving.*
    *   **SysTick Timer:** Provides the system's primary time base for tick-based processing.
    *   **GPIO:** PA15 (On-board LED), PA23 (On-board Button SW0).
    *   **EIC:** External Interrupt Controller used for button press detection on PA23 (EXTINT[2]).
*   **External Hardware:**
    *   **GPS Module:** Any GPS module outputting NMEA sentences (specifically GPGLL) via UART.
    *   UART Connection: Connect the GPS module's TX pin to the microcontroller's GPS RX pin (PB17). Ensure compatible voltage levels or use level shifting.
    *   USB Cable: Standard micro-USB or USB-C (depending on the board) for programming and CDC.

## Software Architecture

The project follows a modular architecture, separating low-level hardware interaction from data processing and presentation logic.

## How It Works (Simplified Data Flow)

1.  **Initialization (`main()` -> `prog_setup()` -> `platform_init()`):** All hardware peripherals (Clocks, USARTs, GPIO, SysTick, EIC) are configured. Asynchronous receive operations are started on both CDC and GPS UARTs. The initial program state is set up.
2.  **Main Loop (`main()` -> `for(;;)` -> `prog_loop_one()`):** The application enters an infinite loop.
3.  **Platform Events (`handle_platform_events()`):** Calls the platform layer's tick handler (`platform_do_loop_one()`) to service low-level tasks. Checks for button presses to queue banner display.
4.  **GPS Reception (`handle_gps_reception()`):** Periodically checks if the GPS UART driver has completed a receive buffer chunk. If so, appends the data to an assembly buffer. Sets a flag if a complete NMEA sentence (`\r\n`) is found. Restarts the async receive for the next chunk.
5.  **Sentence Processing (`handle_gps_sentence_processing()`):** If the "GPS Update Pending" flag is set, extracts the first complete NMEA sentence from the assembly buffer. If `DEBUG_MODE_PRINT_RAW_GPS` is true, sends the raw sentence to the UI for display. If the sentence is GPGLL and no GPGLL is pending parsing, it's copied to a dedicated buffer and flagged for parsing. The processed sentence is removed from the assembly buffer.
6.  **GPGLL Parsing and Display (`handle_gpgll_parsing_and_request_display()`):** If the "Parsed GPGLL Pending" flag is set, the stored GPGLL sentence is passed to the `nmea_parse_gpgll_and_format()` function in the `nmea_parser` module. The formatted output string is then sent to the UI (`ui_handle_parsed_data_transmission()`) for transmission via the CDC UART.
7.  **UI Transmission (`ui_handle_banner_transmission()`, `ui_handle_parsed_data_transmission()`, `ui_handle_raw_data_transmission()`):** If a message (banner, parsed data, or raw data) needs to be sent and the CDC USART is not busy, the UI module prepares the message in the main TX buffer and initiates an asynchronous transmission via the platform CDC driver (`platform_usart_cdc_tx_async()`). Application-level TX flags are used to prevent race conditions with the main TX buffer.
8.  **LED Activity:** The activity LED is turned on when GPS data is received and turned off at the beginning of each main loop iteration, creating a blink effect.

## Prerequisites

*   **Hardware:**
    *   Microchip PIC32CM Curiosity Nano Evaluation Kit (or compatible board with PIC32CM5164LS00048).
    *   GPS Module with UART output (configured for 38400 baud, 8N1, 2 stop bits typically, but verify your module's specs and adjust `platform/gps_usart.c` if needed).
    *   Appropriate cables (USB for board power/programming/CDC, wires for connecting GPS TX to PB17 on the board).
    *   (Optional) Logic level converter if your GPS module's voltage levels don't match the microcontroller's I/O voltage (typically 3.3V for the PIC32CM Nano board).
*   **Software:**
    *   Microchip MPLAB X IDE (latest version recommended).
    *   Microchip XC32 C/C++ Compiler (latest version recommended).
    *   A serial terminal program (e.g., CoolTerm, PuTTY, Tera Term) configured for the CDC port at 9600 baud, 8N1. Ensure your terminal supports ANSI escape codes for the best display experience.

## Building and Running

1.  **Clone the Repository:**
    ```bash
    git clone <repository_url>
    cd <repository_directory>
    ```
2.  **Open in MPLAB X IDE:** Open the project (`.X` folder) in MPLAB X IDE.
3.  **Configure Project:** Ensure the correct device (PIC32CM5164LS00048) and compiler (XC32) are selected for the project. Select your connected debugging/programming tool (e.g., PKOB4).
4.  **Connect Hardware:**
    *   Connect your PIC32CM Curiosity Nano board to your computer via USB.
    *   Connect the TX pin of your GPS module to the RX pin of the GPS UART on the board (PB17).
    *   Ensure common ground between the GPS module and the board.
    *   Power up the GPS module.
5.  **Build:** Clean and Build the project (e.g., `Run -> Clean and Build Main Project`).
6.  **Program:** Download and Run the project onto the microcontroller (e.g., `Run -> Run Main Project`). This will typically build, program, and start execution.
7.  **Open Terminal:** Open your serial terminal program and connect to the CDC (Virtual COM Port) created by the Curiosity Nano board. Configure the terminal connection:
    *   **Baud Rate:** 9600
    *   **Data Bits:** 8
    *   **Parity:** None
    *   **Stop Bits:** 1
    *   **Flow Control:** None
    *   Ensure **ANSI escape code support** is enabled in your terminal settings if necessary.
8.  **Observe Output:**
    *   The banner should appear on the terminal.
    *   As the GPS module acquires a fix and sends GPGLL sentences, parsed location and time data should update on the terminal, or raw NMEA data will be displayed if `DEBUG_MODE_PRINT_RAW_GPS` is `true`.
    *   Pressing the on-board button should re-display the banner.
    *   The on-board LED should blink when GPS data is received.

## Configuration Notes

*   **Local Timezone:** Modify `#define LOCAL_TIMEZONE_OFFSET_HOURS` in `nmea_parser.c` to adjust the timezone conversion.
*   **Debug Raw Output:** Toggle `static const bool DEBUG_MODE_PRINT_RAW_GPS` in `main.c` to enable/disable printing of raw NMEA sentences.
*   **Fake Data:** Toggle `#define USE_FAKE_GPS_DATA` in `main.c` to switch between using real GPS data and cycling through predefined fake GPGLL sentences for testing. When using fake data, the `fake_data_interval` constant controls the update rate.
*   **Baud Rates:** The CDC baud rate (115200) is typically fixed by the Curiosity Nano firmware. The GPS UART baud rate (38400) is configured in `platform/gps_usart.c`. If your GPS module uses a different rate, you will need to calculate and update the `SERCOM_BAUD` register value in `gps_platform_usart_init()`.

## Project Files

The project source code is organized into logical modules. Key files and directories include:

*   `main.c` / `main.h`: Contains the primary application logic, state management structures, and the main execution loop.
*   `nmea_parser.c` / `nmea_parser.h`: Implements the NMEA sentence parsing functionality.
*   `terminal_ui.c` / `terminal_ui.h`: Handles the display logic and formatting for the serial terminal interface.
*   `platform.h`: Provides a generic interface layer for platform-specific functionalities like GPIO, SysTick timing, and USART communication.
*   `platform/`: This directory contains the specific implementations for the PIC32CM5164LS00048 hardware:
    *   `platform/gpio.c`: Handles system clock setup, GPIO configuration, button (EIC) setup, and NVIC initialization.
    *   `platform/systick.c`: Implements the SysTick timer driver and time utility functions.
    *   `platform/usart.c`: Implements the USART driver for the CDC (terminal) interface.
    *   `platform/gps_usart.c`: Implements the USART driver specifically configured for communication with the GPS module.
*   Other files (`.X`, `.gitignore`, etc.) are part of the MPLAB X IDE project setup and source control.

## Development Environment

This project was developed using:

*   MPLAB X IDE
*   XC32 C/C++ Compiler
*   Target Device: PIC32CM5164LS00048

## License

This project is licensed under the MIT License. See the separate [LICENSE.md](LICENSE.md) file for details.


## Acknowledgements

*   Original structure and portions by Alberto de Villa, EEE UP Diliman.
*   Supplements by EEE 158 and EEE 192 students (AY 2024-2025).

---

