/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx1276-Hal.c
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
#include <stdint.h>
#include <stdbool.h> 

#include "platform.h"
#include "sx1276-Hal.h"
#include "main.h"

#if defined( USE_SX1276_RADIO )

//#include "ioe.h"
#include "spi.h"
#include "sx1276-Hal.h"
#include "stm32l1xx_hal_gpio.h"

//john remarked RESET¡¢NSS¡¢DIO0~DIO5 2017-10-09

/*!
 * SX1276 RESET I/O definitions
 */
#if defined( STM32F4XX ) || defined( STM32F2XX ) 
#define RESET_IOPORT                                GPIOG
#define RESET_PIN                                   GPIO_PIN_12
#elif defined( STM32F429_439xx )
#define RESET_IOPORT                                GPIOG
#define RESET_PIN                                   GPIO_PIN_12
#else
#define RESET_IOPORT                                SX1278_RESET_GPIO_Port
#define RESET_PIN                                   SX1278_RESET_Pin
#endif

/*!
 * SX1276 SPI NSS I/O definitions
 */
#if defined( STM32F4XX ) || defined( STM32F2XX )
#define NSS_IOPORT                                  GPIOA
#define NSS_PIN                                     GPIO_PIN_15
#elif defined( STM32F429_439xx )
#define NSS_IOPORT                                  GPIOA
#define NSS_PIN                                     GPIO_PIN_4
#else
#define NSS_IOPORT                                  SX1278_NSS_GPIO_Port
#define NSS_PIN                                     SX1278_NSS_Pin
#endif

/*!
 * SX1276 DIO pins  I/O definitions
 */
#if defined( STM32F4XX ) || defined( STM32F2XX ) 
#define DIO0_IOPORT                                 GPIOG
#define DIO0_PIN                                    GPIO_PIN_13
#elif defined( STM32F429_439xx )
#define DIO0_IOPORT                                 GPIOG
#define DIO0_PIN                                    GPIO_PIN_13
#else
#define DIO0_IOPORT                                 SX1278_DIO0_GPIO_Port
#define DIO0_PIN                                    SX1278_DIO0_Pin
#endif

#if defined( STM32F4XX ) || defined( STM32F2XX )
#define DIO1_IOPORT                                 GPIOB
#define DIO1_PIN                                    GPIO_PIN_8
#elif defined( STM32F429_439xx )
#define DIO1_IOPORT                                 GPIOB
#define DIO1_PIN                                    GPIO_PIN_7
#else
#define DIO1_IOPORT                                 SX1278_DIO1_GPIO_Port
#define DIO1_PIN                                    SX1278_DIO1_Pin
#endif

#if defined( STM32F4XX ) || defined( STM32F2XX ) 
#define DIO2_IOPORT                                 GPIOA
#define DIO2_PIN                                    GPIO_PIN_2
#elif defined( STM32F429_439xx )
#define DIO2_IOPORT                                 GPIOA
#define DIO2_PIN                                    GPIO_PIN_2
#else
#define DIO2_IOPORT                                 SX1278_DIO2_GPIO_Port
#define DIO2_PIN                                    SX1278_DIO2_Pin
#endif

#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32F429_439xx )
#define DIO3_IOPORT                                 
#define DIO3_PIN                                    RF_DIO3_PIN
#else
#define DIO3_IOPORT                                 SX1278_DIO3_GPIO_Port                                
#define DIO3_PIN                                    SX1278_DIO3_Pin
#endif

#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32F429_439xx )
#define DIO4_IOPORT                                 
#define DIO4_PIN                                    RF_DIO4_PIN
#else
#define DIO4_IOPORT                                 SX1278_DIO4_GPIO_Port                                
#define DIO4_PIN                                    SX1278_DIO4_Pin
#endif

#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32F429_439xx )
#define DIO5_IOPORT                                 
#define DIO5_PIN                                    RF_DIO5_PIN
#else
#define DIO5_IOPORT                                 SX1278_DIO5_GPIO_Port                                
#define DIO5_PIN                                    SX1278_DIO5_Pin
#endif

#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32F429_439xx )
#define RXTX_IOPORT                                 
#define RXTX_PIN                                    FEM_CTX_PIN
#else
#define RXTX_IOPORT                                 
#define RXTX_PIN                                    FEM_CTX_PIN
#endif


void SX1276InitIo( void )
{
    GPIO_InitTypeDef GPIO_InitStructure;

		/* GPIO Ports Clock Enable */
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
	
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

    // Configure NSS as output
    HAL_GPIO_WritePin( NSS_IOPORT, NSS_PIN, GPIO_PIN_SET );
    GPIO_InitStructure.Pin = NSS_PIN;
    HAL_GPIO_Init( NSS_IOPORT, &GPIO_InitStructure );

    // Configure radio DIO as inputs
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	
    GPIO_InitStructure.Pin =  DIO0_PIN;
    HAL_GPIO_Init( DIO0_IOPORT, &GPIO_InitStructure );
    
    // Configure DIO1
    GPIO_InitStructure.Pin =  DIO1_PIN;
    HAL_GPIO_Init( DIO1_IOPORT, &GPIO_InitStructure );
    
    // Configure DIO2
    GPIO_InitStructure.Pin =  DIO2_PIN;
    HAL_GPIO_Init( DIO2_IOPORT, &GPIO_InitStructure );
		
		// REAMARK: DIO3/4/5  john 2017-10-09
		
		 // Configure DIO3
    GPIO_InitStructure.Pin =  DIO3_PIN;
    HAL_GPIO_Init( DIO3_IOPORT, &GPIO_InitStructure );
		
		 // Configure DIO4
    GPIO_InitStructure.Pin =  DIO4_PIN;
    HAL_GPIO_Init( DIO4_IOPORT, &GPIO_InitStructure );
		
		 // Configure DIO5
    GPIO_InitStructure.Pin =  DIO5_PIN;
    HAL_GPIO_Init( DIO5_IOPORT, &GPIO_InitStructure );
    
    // REAMARK: DIO3/4/5 configured are connected to IO expander

    // Configure DIO3 as input
    
    // Configure DIO4 as input
    
    // Configure DIO5 as input
}

void SX1276SetReset( uint8_t state )
{
//    GPIO_InitTypeDef GPIO_InitStructure;

    if( state == RADIO_RESET_ON )
    {
        // Set RESET pin to 0
        HAL_GPIO_WritePin( RESET_IOPORT, RESET_PIN, GPIO_PIN_RESET );
    }
    else
    {
//#if FPGA == 0    
//        // Configure RESET as input
//#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32F429_439xx )
//        GPIO_InitStructure.Mode = GPIO_Mode_IN;
//#else
//        GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//				GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//#endif        
//        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
//        GPIO_InitStructure.Pin =  RESET_PIN;
//        HAL_GPIO_Init( RESET_IOPORT, &GPIO_InitStructure );
//#else
//        HAL_GPIO_WritePin( RESET_IOPORT, RESET_PIN, GPIO_PIN_SET );
//#endif
				HAL_GPIO_WritePin( RESET_IOPORT, RESET_PIN, GPIO_PIN_SET );
    }
}

void SX1276Write( uint8_t addr, uint8_t data )
{

    SX1276WriteBuffer( addr, &data, 1 );

}

void SX1276Read( uint8_t addr, uint8_t *data )
{

    SX1276ReadBuffer( addr, data, 1 );

}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    HAL_GPIO_WritePin( NSS_IOPORT, NSS_PIN, GPIO_PIN_RESET );

    SpiInOut( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        SpiInOut( buffer[i] );
    }

    //NSS = 1;
    HAL_GPIO_WritePin( NSS_IOPORT, NSS_PIN, GPIO_PIN_SET );
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    HAL_GPIO_WritePin( NSS_IOPORT, NSS_PIN, GPIO_PIN_RESET );

    SpiInOut( addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( 0 );
    }

    //NSS = 1;
    HAL_GPIO_WritePin( NSS_IOPORT, NSS_PIN, GPIO_PIN_SET );
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}

inline uint8_t SX1276ReadDio0( void )
{
    return HAL_GPIO_ReadPin( DIO0_IOPORT, DIO0_PIN );
}

inline uint8_t SX1276ReadDio1( void )
{
    return HAL_GPIO_ReadPin( DIO1_IOPORT, DIO1_PIN );
}

inline uint8_t SX1276ReadDio2( void )
{
    return HAL_GPIO_ReadPin( DIO2_IOPORT, DIO2_PIN );
}

inline uint8_t SX1276ReadDio3( void )
{
   // return IoePinGet( RF_DIO3_PIN );
		return HAL_GPIO_ReadPin( DIO3_IOPORT, DIO3_PIN );
}

inline uint8_t SX1276ReadDio4( void )
{
   // return IoePinGet( RF_DIO4_PIN );
	return HAL_GPIO_ReadPin( DIO4_IOPORT, DIO4_PIN );
}

inline uint8_t SX1276ReadDio5( void )
{
    //return IoePinGet( RF_DIO5_PIN );
	return 0;
}

inline void SX1276WriteRxTx( uint8_t txEnable )
{
//    if( txEnable != 0 )
//    {
//        IoePinOn( FEM_CTX_PIN );
//        IoePinOff( FEM_CPS_PIN );
//    }
//    else
//    {
//        IoePinOff( FEM_CTX_PIN );
//        IoePinOn( FEM_CPS_PIN );
//    }
	
}

#endif // USE_SX1276_RADIO
