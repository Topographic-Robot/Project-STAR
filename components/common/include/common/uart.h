/* components/common/include/common/uart.h */

#ifndef TOPOROBO_UART_H
#define TOPOROBO_UART_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"

/* Constants ******************************************************************/

extern const uint32_t uart_timeout_ticks; /* Timeout for UART commands in ticks */

/* Private Functions **********************************************************/

/**
 * @brief Initialize the UART interface with the specified parameters.
 *
 * This function configures the UART bus with the provided TX and RX pins,
 * baud rate, and UART port number. It sets the UART mode to master, initializes
 * the driver, and sets up communication parameters such as baud rate, data bits,
 * parity, and stop bits.
 *
 * @param[in] tx_io Pin number for the UART TX (transmit) line.
 * @param[in] rx_io Pin number for the UART RX (receive) line.
 * @param[in] baud_rate Baud rate for UART communication (e.g., 9600, 115200).
 * @param[in,out] uart_num UART port number to use for communication (e.g., UART_NUM_1).
 * @param[in] tag The tag for logging errors and events.
 *
 * @return
 *   - ESP_OK on successful initialization.
 *   - An error code from the esp_err_t enumeration on failure.
 *
 * @note This function does not require any buffers for the UART driver,
 *       so it sets the buffer size to 0 for both TX and RX.
 */
esp_err_t priv_uart_init(uint8_t tx_io, uint8_t rx_io, uint32_t baud_rate,
                         uart_port_t uart_num, const char *tag);

/**
 * @brief Read data from the UART interface and return the length of data read.
 *
 * This function reads up to `len` bytes from the UART interface specified by `uart_num`.
 * It uses a timeout mechanism to wait for data to be available. If data is received,
 * the length of the data read is stored in `out_length`, and the data is returned
 * in the `data` buffer.
 *
 * **Logic and Flow:**
 * - Attempts to read data from the UART interface using `uart_read_bytes`.
 * - If data is read successfully, stores the length in `out_length` and logs the data.
 * - If the read fails or times out, sets `out_length` to zero and returns an error.
 *
 * @param[out] data Pointer to the buffer where the read data will be stored.
 * @param[in] len The maximum number of bytes to read from the UART interface.
 * @param[out] out_length Pointer to an integer where the length of data read will be stored.
 * @param[in] uart_num UART port number to communicate over (e.g., UART_NUM_1).
 * @param[in] tag The tag for logging errors and events.
 *
 * @return
 *   - ESP_OK if data is successfully read.
 *   - ESP_FAIL if reading fails or times out.
 */
esp_err_t priv_uart_read(uint8_t *data, size_t len, int32_t *out_length,
                         uart_port_t uart_num, const char *tag);

#endif /* TOPOROBO_UART_H */

