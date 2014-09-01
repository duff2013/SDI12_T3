/*
 ||
 || @file 		utils.h
 || @version 	1
 || @author 	Colin Duffy
 || @contact 	cmduffy@engr.psu.edu
 || @license
 || | Copyright (c) 2014 Colin Duffy
 || | This library is free software; you can redistribute it and/or
 || | modify it under the terms of the GNU Lesser General Public
 || | License as published by the Free Software Foundation; version
 || | 2.1 of the License.
 || |
 || | This library is distributed in the hope that it will be useful,
 || | but WITHOUT ANY WARRANTY; without even the implied warranty of
 || | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 || | Lesser General Public License for more details.
 || |
 || | You should have received a copy of the GNU Lesser General Public
 || | License along with this library; if not, write to the Free Software
 || | Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 || #
 ||
 */

#ifndef utils_h
#define utils_h

#define BITBAND_ADDR(reg, bit) (((uint32_t)&(reg) - 0x40000000) * 32 + (bit) * 4 + 0x42000000)
#define BITBAND_U32(reg, bit) (*(uint32_t *)BITBAND_ADDR((reg), (bit)))

#define SCGC7_DMA_BIT       1
#define SCGC6_DMAMUX_BIT    1
#define SCGC4_UART0_BIT     10
#define SCGC4_UART1_BIT     11
#define SCGC4_UART2_BIT     12

#define IRQ_PRIORITY        64  // 0 = highest priority, 255 = lowest

#define UART_C3_TXDIR       (uint8_t)0x20
#define UART_C3_TXINV       (uint8_t)0x10
#define UART_C3_ORIE        (uint8_t)0x08
#define UART_C3_FEIE        (uint8_t)0x02
#define UART_C3_PEIE        (uint8_t)0x01

#define UART_S2_RXINV       (uint8_t)0x10

#define C2_ENABLE           UART_C2_TE | UART_C2_RE | UART_C2_RIE
#define C2_TX_ACTIVE		C2_ENABLE | UART_C2_TIE
#define C2_TX_COMPLETING	C2_ENABLE | UART_C2_TCIE
#define C2_TX_INACTIVE		C2_ENABLE

#define UART_TX_DIR_OUT     UART_C3_TXDIR
#define UART_TX_DIR_IN      ~UART_C3_TXDIR
#define SDI12_ENABLE        UART_C1_LOOPS | UART_C1_RSRC | UART_C1_PE

#define RUN_ONCE_BEGIN      \
struct run_once {           \
run_once() {

#define RUN_ONCE_END        \
}};                         \
static run_once __o;

#define TX_BUFFER_SIZE      32
#define RX_BUFFER_SIZE      128

#define BLOCKING            1
#define NON_BLOCKING        2
// --------------------------------------------------------------------------------------------
#define ENABLE_UART0 ({                                                             \
    BITBAND_U32(SIM_SCGC4, SCGC4_UART0_BIT) = 0x01;                                 \
    CORE_PIN0_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);  \
    CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);               \
    int divisor = BAUD2DIV(1200);                                                   \
    UART0_BDH = (divisor >> 13) & 0x1F;                                             \
    UART0_BDL = (divisor >> 5) & 0xFF;                                              \
    UART0_C4 = divisor & 0x1F;                                                      \
    UART0_C1 = SDI12_ENABLE;                                                        \
    UART0_PFIFO = 0;                                                                \
    UART0_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE;                               \
    UART0_S2 = UART_S2_RXINV;                                                       \
    UART0_C3 = UART_C3_TXINV;                                                       \
    NVIC_SET_PRIORITY(IRQ_UART0_STATUS, IRQ_PRIORITY);                              \
})

#define ENABLE_UART1 ({                                                             \
    BITBAND_U32(SIM_SCGC4, SCGC4_UART1_BIT) = 0x01;                                 \
    CORE_PIN9_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);  \
    CORE_PIN10_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);              \
    int divisor = BAUD2DIV(1200);                                                   \
    UART1_BDH = (divisor >> 13) & 0x1F;                                             \
    UART1_BDL = (divisor >> 5) & 0xFF;                                              \
    UART1_C4 = divisor & 0x1F;                                                      \
    UART1_C1 = SDI12_ENABLE;                                                        \
    UART1_PFIFO = 0;                                                                \
    UART1_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE;                               \
    UART1_S2 = UART_S2_RXINV;                                                       \
    UART1_C3 = UART_C3_TXINV;                                                       \
    NVIC_SET_PRIORITY(IRQ_UART1_STATUS, IRQ_PRIORITY);                              \
})

#define ENABLE_UART2 ({                                                             \
    BITBAND_U32(SIM_SCGC4, SCGC4_UART2_BIT) = 0x01;                                 \
    CORE_PIN7_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);  \
    CORE_PIN8_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);               \
    int divisor = BAUD2DIV3(1200);                                                  \
    UART2_BDH = (divisor >> 13) & 0x1F;                                             \
    UART2_BDL = (divisor >> 5) & 0xFF;                                              \
    UART2_C4 = divisor & 0x1F;                                                      \
    UART2_C1 = SDI12_ENABLE;                                                        \
    UART2_PFIFO = 0;                                                                \
    UART2_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE;                               \
    UART2_S2 = UART_S2_RXINV;                                                       \
    UART2_C3 = UART_C3_TXINV;                                                       \
    NVIC_SET_PRIORITY(IRQ_UART2_STATUS, IRQ_PRIORITY);                              \
})
#endif
