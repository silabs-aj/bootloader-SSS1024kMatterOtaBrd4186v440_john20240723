/***************************************************************************//**
 * @file
 * @brief Configuration header for bootloader Spi Controller Eusart Driver
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
#ifndef BTL_SPI_CONTROLLER_EUSART_DRIVER_CONFIG_H
#define BTL_SPI_CONTROLLER_EUSART_DRIVER_CONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>

// <h>SPI Controller EUSART Driver

// <o SL_EUSART_EXTFLASH_FREQUENCY> Frequency
// <i> Default: 6400000
#define SL_EUSART_EXTFLASH_FREQUENCY          6400000

// </h>

// <<< end of configuration section >>>

// <<< sl:start pin_tool >>>
// <eusart signal=TX,RX,SCLK,CS> SL_EUSART_EXTFLASH
// $[EUSART_SL_EUSART_EXTFLASH]
#ifndef SL_EUSART_EXTFLASH_PERIPHERAL           
#define SL_EUSART_EXTFLASH_PERIPHERAL            EUSART1
#endif
#ifndef SL_EUSART_EXTFLASH_PERIPHERAL_NO        
#define SL_EUSART_EXTFLASH_PERIPHERAL_NO         1
#endif

// EUSART1 TX on PC01
#ifndef SL_EUSART_EXTFLASH_TX_PORT              
#define SL_EUSART_EXTFLASH_TX_PORT               gpioPortC
#endif
#ifndef SL_EUSART_EXTFLASH_TX_PIN               
#define SL_EUSART_EXTFLASH_TX_PIN                1
#endif

// EUSART1 RX on PC02
#ifndef SL_EUSART_EXTFLASH_RX_PORT              
#define SL_EUSART_EXTFLASH_RX_PORT               gpioPortC
#endif
#ifndef SL_EUSART_EXTFLASH_RX_PIN               
#define SL_EUSART_EXTFLASH_RX_PIN                2
#endif

// EUSART1 SCLK on PC03
#ifndef SL_EUSART_EXTFLASH_SCLK_PORT            
#define SL_EUSART_EXTFLASH_SCLK_PORT             gpioPortC
#endif
#ifndef SL_EUSART_EXTFLASH_SCLK_PIN             
#define SL_EUSART_EXTFLASH_SCLK_PIN              3
#endif

// EUSART1 CS on PC04
#ifndef SL_EUSART_EXTFLASH_CS_PORT              
#define SL_EUSART_EXTFLASH_CS_PORT               gpioPortC
#endif
#ifndef SL_EUSART_EXTFLASH_CS_PIN               
#define SL_EUSART_EXTFLASH_CS_PIN                4
#endif
// [EUSART_SL_EUSART_EXTFLASH]$

// <gpio> SL_EXTFLASH_WP
// $[GPIO_SL_EXTFLASH_WP]

// [GPIO_SL_EXTFLASH_WP]$

// <gpio> SL_EXTFLASH_HOLD
// $[GPIO_SL_EXTFLASH_HOLD]

// [GPIO_SL_EXTFLASH_HOLD]$

// <<< sl:end pin_tool >>>

#endif // BTL_SPI_CONTROLLER_EUSART_DRIVER_CONFIG_H
