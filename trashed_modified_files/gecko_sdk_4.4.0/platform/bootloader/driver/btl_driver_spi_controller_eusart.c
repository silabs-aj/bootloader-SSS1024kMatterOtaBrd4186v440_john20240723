/***************************************************************************//**
 * @file
 * @brief Universal SPI controller eusart driver for the Silicon Labs Bootloader.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc.  Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.  This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "api/btl_interface.h"
#include "config/btl_config.h"
#include "debug/btl_debug.h"
#include "btl_driver_spi_controller.h"
#include "em_cmu.h"
#include "em_eusart.h"
#include "em_gpio.h"
#include "btl_spi_controller_eusart_driver_cfg.h"
#include "btl_driver_util.h"

#define BTL_DRIVER_SPI_EUSART_TXLOC \
  (SL_EUSART_EXTFLASH_TX_LOC << _EUSART_ROUTELOC0_TXLOC_SHIFT)
#define BTL_DRIVER_SPI_EUSART_RXLOC \
  (SL_EUSART_EXTFLASH_RX_LOC << _EUSART_ROUTELOC0_RXLOC_SHIFT)
#define BTL_DRIVER_SPI_EUSART_CLKLOC \
  (SL_EUSART_EXTFLASH_SCLK_LOC << _EUSART_ROUTELOC0_CLKLOC_SHIFT)

#if !defined(SL_EUSART_EXTFLASH_PERIPHERAL)
#error "SL_EXTFLASH_PERIPHERAL not defined"
#endif

#if SL_EUSART_EXTFLASH_PERIPHERAL_NO == 0
#define BTL_DRIVER_SPI_EUSART          EUSART0
#define BTL_DRIVER_SPI_EUSART_NUM      0
#define BTL_DRIVER_SPI_EUSART_CLOCK    cmuClock_EUSART0

#if defined(BOOTLOADER_INTERFACE_TRUSTZONE_AWARE)
#if defined(SMU_PPUSATD0_EUSART0)
#define BTL_DRIVER_SPI_PPUSATD_NUM     0UL
#define BTL_DRIVER_SPI_PPUSATD         SMU_PPUSATD0_EUSART0
#elif defined(SMU_PPUSATD1_EUSART0)
#define BTL_DRIVER_SPI_PPUSATD_NUM     1UL
#define BTL_DRIVER_SPI_PPUSATD         SMU_PPUSATD1_EUSART0
#endif
#endif // BOOTLOADER_INTERFACE_TRUSTZONE_AWARE

#elif SL_EUSART_EXTFLASH_PERIPHERAL_NO == 1
#define BTL_DRIVER_SPI_EUSART          EUSART1
#define BTL_DRIVER_SPI_EUSART_NUM      1
#define BTL_DRIVER_SPI_EUSART_CLOCK    cmuClock_EUSART1

#if defined(BOOTLOADER_INTERFACE_TRUSTZONE_AWARE)
#if defined(SMU_PPUSATD0_EUSART1)
#define BTL_DRIVER_SPI_PPUSATD_NUM     0UL
#define BTL_DRIVER_SPI_PPUSATD         SMU_PPUSATD0_EUSART1
#elif defined(SMU_PPUSATD1_EUSART1)
#define BTL_DRIVER_SPI_PPUSATD_NUM     1UL
#define BTL_DRIVER_SPI_PPUSATD         SMU_PPUSATD1_EUSART1
#endif
#endif // BOOTLOADER_INTERFACE_TRUSTZONE_AWARE

#elif SL_EUSART_EXTFLASH_PERIPHERAL_NO == 2
#define BTL_DRIVER_SPI_EUSART          EUSART2
#define BTL_DRIVER_SPI_EUSART_NUM      2
#define BTL_DRIVER_SPI_EUSART_CLOCK    cmuClock_EUSART2

#if defined(BOOTLOADER_INTERFACE_TRUSTZONE_AWARE)
#if defined(SMU_PPUSATD0_EUSART2)
#define BTL_DRIVER_SPI_PPUSATD_NUM     0UL
#define BTL_DRIVER_SPI_PPUSATD         SMU_PPUSATD0_EUSART2
#elif defined(SMU_PPUSATD1_EUSART2)
#define BTL_DRIVER_SPI_PPUSATD_NUM     1UL
#define BTL_DRIVER_SPI_PPUSATD         SMU_PPUSATD1_EUSART2
#endif
#endif // BOOTLOADER_INTERFACE_TRUSTZONE_AWARE

#elif SL_EUSART_EXTFLASH_PERIPHERAL_NO == 3
#define BTL_DRIVER_SPI_EUSART          EUSART3
#define BTL_DRIVER_SPI_EUSART_NUM      3
#define BTL_DRIVER_SPI_EUSART_CLOCK    cmuClock_EUSART3

#if defined(BOOTLOADER_INTERFACE_TRUSTZONE_AWARE)
#if defined(SMU_PPUSATD0_EUSART3)
#define BTL_DRIVER_SPI_PPUSATD_NUM     0UL
#define BTL_DRIVER_SPI_PPUSATD         SMU_PPUSATD0_EUSART3
#elif defined(SMU_PPUSATD1_EUSART3)
#define BTL_DRIVER_SPI_PPUSATD_NUM     1UL
#define BTL_DRIVER_SPI_PPUSATD         SMU_PPUSATD1_EUSART3
#endif
#endif // BOOTLOADER_INTERFACE_TRUSTZONE_AWARE

#elif SL_EUSART_EXTFLASH_PERIPHERAL_NO == 4
#define BTL_DRIVER_SPI_EUSART          EUSART4
#define BTL_DRIVER_SPI_EUSART_NUM      4
#define BTL_DRIVER_SPI_EUSART_CLOCK    cmuClock_EUSART4

#if defined(BOOTLOADER_INTERFACE_TRUSTZONE_AWARE)
#if defined(SMU_PPUSATD0_EUSART4)
#define BTL_DRIVER_SPI_PPUSATD_NUM     0UL
#define BTL_DRIVER_SPI_PPUSATD         SMU_PPUSATD0_EUSART4
#elif defined(SMU_PPUSATD1_EUSART4)
#define BTL_DRIVER_SPI_PPUSATD_NUM     1UL
#define BTL_DRIVER_SPI_PPUSATD         SMU_PPUSATD1_EUSART4
#endif
#endif // BOOTLOADER_INTERFACE_TRUSTZONE_AWARE

#else
#error "Invalid SL_EXTFLASH_PERIPHERAL"
#endif

#ifndef BTL_DRIVER_SPI_PPUSATD
#define BTL_DRIVER_SPI_PPUSATD_NUM 0xFFFFFFFFUL
#define BTL_DRIVER_SPI_PPUSATD     0UL
#endif

// Permits a custom crystal frequency as a build option
#ifndef BTL_HFXO_FREQ
#define BTL_HFXO_FREQ   39000000
#endif

uint32_t get_grpc_clk_freq(void)
{
  const uint8_t frequencies[] = { 4, 0, 0, 7, 0, 0, 13, 16, 19, 0, 26, 32, 38, 48, 56, 64, 80 };

  // Default to 80 MHz so divider is large enough if a non-supported oscillator is selected
  uint32_t freq = 80000000;
  uint32_t grpc_src = CMU->EM01GRPCCLKCTRL & _CMU_EM01GRPCCLKCTRL_CLKSEL_MASK;
  uint32_t band, cal, hfrco_div, mdiv, ndiv;

  switch (grpc_src)
  {
    case CMU_EM01GRPCCLKCTRL_CLKSEL_HFXO:
      freq = BTL_HFXO_FREQ;
      break;
    case CMU_EM01GRPCCLKCTRL_CLKSEL_HFRCODPLL:
      // Check if DPLL is enabled and locked
      CMU->CLKEN0_SET = CMU_CLKEN0_DPLL0;
      if (DPLL0->STATUS & DPLL_STATUS_RDY) {
        // If so, calculate frequency from M/N dividers assuming HFXO as reference
        mdiv = DPLL0->CFG1 & _DPLL_CFG1_M_MASK;
        ndiv = (DPLL0->CFG1 & _DPLL_CFG1_N_MASK) >> _DPLL_CFG1_N_SHIFT;
        freq = (BTL_HFXO_FREQ / mdiv) * ndiv;
      }
      else {
        // DPLL not engaged; calculate frequency from tuning band
        CMU->CLKEN0_SET = CMU_CLKEN0_HFRCO0;
        cal = HFRCO0->CAL;
        band = (cal & _HFRCO_CAL_FREQRANGE_MASK) >> _HFRCO_CAL_FREQRANGE_SHIFT;
        hfrco_div = (cal & _HFRCO_CAL_CLKDIV_MASK) >> _HFRCO_CAL_CLKDIV_SHIFT;

        // Calculate frequency from band and CLKDIV divider
        freq = (frequencies[band] * 1000000UL) / (1 << hfrco_div);
      }
      // Disable DPLL and HFRCO register clocks
      CMU->CLKEN0_CLR = ( CMU_CLKEN0_DPLL0 | CMU_CLKEN0_HFRCO0);
      break;
    default:
      BTL_DEBUG_PRINT_WORD_HEX(grpc_src);
      BTL_DEBUG_PRINTLN("not supported as EM01GRPCCLK source.\n");
  }

  return freq;
}

static void eusart_sync(EUSART_TypeDef *eusart, uint32_t mask)
{
  // Wait for any pending previous write operation to have been completed
  // in the low-frequency domain.
  while ((eusart->SYNCBUSY & mask) != 0U) {
  }
}

static void clk_enable(void)
{
#if defined(CMU_CTRL_HFPERCLKEN)
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(BTL_DRIVER_SPI_EUSART_CLOCK, true);
#endif
#if defined(_CMU_CLKEN0_MASK)
  CMU->CLKEN0_SET = CMU_CLKEN0_GPIO;
#if SL_EUSART_EXTFLASH_PERIPHERAL_NO == 0
  CMU->CLKEN1_SET = CMU_CLKEN1_EUSART0;
#elif SL_EUSART_EXTFLASH_PERIPHERAL_NO == 1
  CMU->CLKEN1_SET = CMU_CLKEN1_EUSART1;
#elif SL_EUSART_EXTFLASH_PERIPHERAL_NO == 2
  CMU->CLKEN1_SET = CMU_CLKEN1_EUSART2;
#else
#error "Invalid SL_USART_EXTFLASH_PERIPHERAL"
#endif
#endif
}

void spi_init(void)
{
  BTL_DEBUG_PRINTLN("spi_init......");
  clk_enable();

  // MOSI
  GPIO_PinModeSet(SL_EUSART_EXTFLASH_TX_PORT,
                  SL_EUSART_EXTFLASH_TX_PIN,
                  gpioModePushPull,
                  0);
  // MISO
  GPIO_PinModeSet(SL_EUSART_EXTFLASH_RX_PORT,
                  SL_EUSART_EXTFLASH_RX_PIN,
                  gpioModeInputPull,
                  0);
  // CLK
  GPIO_PinModeSet(SL_EUSART_EXTFLASH_SCLK_PORT,
                  SL_EUSART_EXTFLASH_SCLK_PIN,
                  gpioModePushPull,
                  0);
  // CS#
  GPIO_PinModeSet(SL_EUSART_EXTFLASH_CS_PORT,
                  SL_EUSART_EXTFLASH_CS_PIN,
                  gpioModePushPull,
                  1);

  EUSART_Reset(BTL_DRIVER_SPI_EUSART);

#if defined(_EUSART_IF_MASK)
  BTL_DRIVER_SPI_EUSART->IF_SET = _EUSART_IF_MASK;
#else
  BTL_DRIVER_SPI_EUSART->IF_CLR = _EUSART_IF_MASK;
#endif

  // Get EM01GRPCCLK frequency
  uint32_t grpc_freq = get_grpc_clk_freq();

  // Calculate SPI clock divider and do a quick and dirty round up
  uint32_t sdiv = (10 * grpc_freq) / SL_EUSART_EXTFLASH_FREQUENCY;
  sdiv = ((sdiv + 5) / 10) - 1;

  // Set up for SPI
  BTL_DRIVER_SPI_EUSART->CFG2 = _EUSART_CFG2_RESETVALUE
                                | (sdiv << _EUSART_CFG2_SDIV_SHIFT)
                                | EUSART_CFG2_CLKPOL_IDLELOW
                                | EUSART_CFG2_CLKPHA_SAMPLELEADING
                                | EUSART_CFG2_MASTER;

  BTL_DRIVER_SPI_EUSART->CFG0 =  _EUSART_CFG0_RESETVALUE
                                | EUSART_CFG0_SYNC
                                | EUSART_CFG0_MSBF;

  // Configure databits, leave stopbits and parity at reset default (not used)
  BTL_DRIVER_SPI_EUSART->FRAMECFG = _EUSART_FRAMECFG_RESETVALUE
                                    | EUSART_FRAMECFG_DATABITS_EIGHT;

  EUSART_Enable(BTL_DRIVER_SPI_EUSART, eusartEnable);
//  EUSART_BaudrateSet(BTL_DRIVER_SPI_EUSART, 0, SL_EUSART_EXTFLASH_FREQUENCY);

#if defined(_EUSART_ROUTEPEN_RESETVALUE)
  BTL_DRIVER_SPI_EUSART->ROUTEPEN = EUSART_ROUTEPEN_TXPEN
                                    | EUSART_ROUTEPEN_RXPEN
                                    | EUSART_ROUTEPEN_CLKPEN;

  BTL_DRIVER_SPI_EUSART->ROUTELOC0 = BTL_DRIVER_SPI_EUSART_TXLOC
                                     | BTL_DRIVER_SPI_EUSART_RXLOC
                                     | BTL_DRIVER_SPI_EUSART_CLKLOC;
#else
  GPIO->EUSARTROUTE[BTL_DRIVER_SPI_EUSART_NUM].TXROUTE = 0
                                                         | (SL_EUSART_EXTFLASH_TX_PORT << _GPIO_EUSART_TXROUTE_PORT_SHIFT)
                                                         | (SL_EUSART_EXTFLASH_TX_PIN << _GPIO_EUSART_TXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[BTL_DRIVER_SPI_EUSART_NUM].RXROUTE = 0
                                                         | (SL_EUSART_EXTFLASH_RX_PORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
                                                         | (SL_EUSART_EXTFLASH_RX_PIN << _GPIO_EUSART_RXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[BTL_DRIVER_SPI_EUSART_NUM].SCLKROUTE = 0
                                                           | (SL_EUSART_EXTFLASH_SCLK_PORT << _GPIO_EUSART_SCLKROUTE_PORT_SHIFT)
                                                           | (SL_EUSART_EXTFLASH_SCLK_PIN << _GPIO_EUSART_SCLKROUTE_PIN_SHIFT);

  GPIO->EUSARTROUTE[BTL_DRIVER_SPI_EUSART_NUM].ROUTEEN = 0
                                                         | GPIO_EUSART_ROUTEEN_TXPEN
                                                         | GPIO_EUSART_ROUTEEN_RXPEN
                                                         | GPIO_EUSART_ROUTEEN_SCLKPEN;
#endif

  BTL_DRIVER_SPI_EUSART->CMD = EUSART_CMD_RXEN | EUSART_CMD_TXEN;
  eusart_sync(BTL_DRIVER_SPI_EUSART, _EUSART_SYNCBUSY_RXEN_MASK | _EUSART_SYNCBUSY_TXEN_MASK);
}

void spi_deinit(void)
{
  clk_enable();
  EUSART_Enable(BTL_DRIVER_SPI_EUSART, eusartDisable);
}

void spi_writeByte(uint8_t data)
{
  clk_enable();
  EUSART_Spi_TxRx(BTL_DRIVER_SPI_EUSART, data);
}

void spi_writeHalfword(uint16_t data)
{
  clk_enable();
  EUSART_Tx(BTL_DRIVER_SPI_EUSART, (data >> 8) & 0xFF);
  EUSART_Tx(BTL_DRIVER_SPI_EUSART, data & 0xFF);
  EUSART_Rx(BTL_DRIVER_SPI_EUSART);
  EUSART_Rx(BTL_DRIVER_SPI_EUSART);
}

void spi_write3Byte(uint32_t data)
{
  clk_enable();
  EUSART_Tx(BTL_DRIVER_SPI_EUSART, (data >> 16) & 0xFF);
  EUSART_Tx(BTL_DRIVER_SPI_EUSART, (data >> 8) & 0xFF);
  EUSART_Tx(BTL_DRIVER_SPI_EUSART, data & 0xFF);
  EUSART_Rx(BTL_DRIVER_SPI_EUSART);
  EUSART_Rx(BTL_DRIVER_SPI_EUSART);
  EUSART_Rx(BTL_DRIVER_SPI_EUSART);
}

uint8_t spi_readByte(void)
{
  clk_enable();
  return EUSART_Spi_TxRx(BTL_DRIVER_SPI_EUSART, 0xFF);
}

uint16_t spi_readHalfword(void)
{
  clk_enable();
  uint16_t retval = 0;
  EUSART_Tx(BTL_DRIVER_SPI_EUSART, 0xFF);
  EUSART_Tx(BTL_DRIVER_SPI_EUSART, 0xFF);
  retval = EUSART_Rx(BTL_DRIVER_SPI_EUSART) << 8;
  retval |= EUSART_Rx(BTL_DRIVER_SPI_EUSART);

  return retval;
}

void spi_setCsActive(void)
{
  clk_enable();
  GPIO_PinOutClear(SL_EUSART_EXTFLASH_CS_PORT, SL_EUSART_EXTFLASH_CS_PIN);
}

void spi_setCsInactive(void)
{
  clk_enable();
  GPIO_PinOutSet(SL_EUSART_EXTFLASH_CS_PORT, SL_EUSART_EXTFLASH_CS_PIN);
}

uint32_t spi_getUsartPPUSATD(uint32_t *ppusatdNr)
{
  *ppusatdNr = BTL_DRIVER_SPI_PPUSATD_NUM;
  return BTL_DRIVER_SPI_PPUSATD;
}
