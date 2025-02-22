/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "mcu_periph/uart.h"

#if PERIODIC_TELEMETRY && (USE_UART0 || USE_UART1 || USE_UART2 || USE_UART3 || USE_UART4 || USE_UART5 || USE_UART6 || USE_UART7 || USE_UART8)
#include "modules/datalink/telemetry.h"

static void send_uartx_err(struct transport_tx *trans, struct link_device *dev, struct uart_periph* uart)
{
  uint16_t ore    = uart->ore;
  uint16_t ne_err = uart->ne_err;
  uint16_t fe_err = uart->fe_err;
  uint8_t _bus0 = 0;
  pprz_msg_send_UART_ERRORS(trans, dev, AC_ID,
                            &ore, &ne_err, &fe_err, &_bus0);
}
#endif

#if USE_UART0
struct uart_periph uart0;
#endif

#if USE_UART1
struct uart_periph uart1;
#endif

#if USE_UART2
struct uart_periph uart2;
#endif

#if USE_UART3
struct uart_periph uart3;
#endif

#if USE_UART4
struct uart_periph uart4;
#endif

#if USE_UART5
struct uart_periph uart5;
#endif

#if USE_UART6
struct uart_periph uart6;
#endif

#if USE_UART7
struct uart_periph uart7;
#endif

#if USE_UART8
struct uart_periph uart8;
#endif

#if PERIODIC_TELEMETRY
static void send_uart_err(struct transport_tx *trans __attribute__ ((unused)),
                          struct link_device *dev __attribute__ ((unused)))
{
  static uint8_t uart_nb_cnt = 0;
  switch (uart_nb_cnt) {
#if USE_UART0
    case 0:
      send_uartx_err(trans, dev, &uart0); break;
#endif
#if USE_UART1
    case 1:
      send_uartx_err(trans, dev, &uart1); break;
#endif
#if USE_UART2
    case 2:
      send_uartx_err(trans, dev, &uart2); break;
#endif
#if USE_UART3
    case 3:
      send_uartx_err(trans, dev, &uart3); break;
#endif
#if USE_UART4
    case 4:
      send_uartx_err(trans, dev, &uart4); break;
#endif
#if USE_UART5
    case 5:
      send_uartx_err(trans, dev, &uart5); break;
#endif
#if USE_UART6
    case 6:
      send_uartx_err(trans, dev, &uart6); break;
#endif
#if USE_UART7
    case 7:
      send_uartx_err(trans, dev, &uart7); break;
#endif
#if USE_UART8
    case 8:
      send_uartx_err(trans, dev, &uart8); break;
#endif
    default: break;
  }
  uart_nb_cnt++;
  if (uart_nb_cnt == 9) {
    uart_nb_cnt = 0;
  }
}
#endif

void uart_periph_init(struct uart_periph *p)
{
  p->rx_insert_idx = 0;
  p->rx_extract_idx = 0;
  p->tx_insert_idx = 0;
  p->tx_extract_idx = 0;
  p->tx_running = false;
  p->ore = 0;
  p->ne_err = 0;
  p->fe_err = 0;
  p->device.periph = (void *)p;
  p->device.check_free_space = (check_free_space_t) uart_check_free_space;
  p->device.put_byte = (put_byte_t) uart_put_byte;
  p->device.put_buffer = (put_buffer_t) uart_put_buffer;
  p->device.send_message = (send_message_t) uart_send_message;
  p->device.char_available = (char_available_t) uart_char_available;
  p->device.get_byte = (get_byte_t) uart_getch;
  p->device.set_baudrate = (set_baudrate_t) uart_periph_set_baudrate;

#if PERIODIC_TELEMETRY
  // the first to register do it for the others
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_UART_ERRORS, send_uart_err);
#endif
}

int WEAK uart_check_free_space(struct uart_periph *p, long *fd __attribute__((unused)), uint16_t len)
{
  int space = p->tx_extract_idx - p->tx_insert_idx - 1;
  if (space < 0) {
    space += UART_TX_BUFFER_SIZE;
  }
  return space >= len ? space : 0;
}

// Weak implementation of put_buffer, byte by byte
void WEAK uart_put_buffer(struct uart_periph *p, long fd, const uint8_t *data, uint16_t len)
{
  int i = 0;
  for (i = 0; i < len; i++) {
    uart_put_byte(p, fd, data[i]);
  }
}

// Weak implementation of send_message, not needed for stream operation
void WEAK uart_send_message(struct uart_periph *p __attribute__((unused)), long fd __attribute__((unused)))
{
}

uint8_t WEAK uart_getch(struct uart_periph *p)
{
  uint8_t ret = p->rx_buf[p->rx_extract_idx];
  p->rx_extract_idx = (p->rx_extract_idx + 1) % UART_RX_BUFFER_SIZE;
  return ret;
}

int WEAK uart_char_available(struct uart_periph *p)
{
  int available = p->rx_insert_idx - p->rx_extract_idx;
  if (available < 0) {
    available += UART_RX_BUFFER_SIZE;
  }
  return available;
}

void WEAK uart_arch_init(void)
{
}

void WEAK uart_periph_invert_data_logic(struct uart_periph *p __attribute__((unused)), bool invert_rx __attribute__((unused)), bool invert_tx __attribute__((unused)))
{
}

