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
 * \file       sx1276.c
 * \brief      SX1276 RF chip driver
 *
 * \version    2.0.0 
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
#include "platform.h"

# include <stdio.h>
# include <stdlib.h>

#include "sx1276.h"
#include "sx1276-Hal.h"
#include "sx1276-LoRa.h"
#include "usart.h"
#include "spi.h"


//typedef struct sSX1276
//{
//    uint8_t RegFifo;                                // 0x00
//    // Common settings
//    uint8_t RegOpMode;                              // 0x01
//    uint8_t RegBitrateMsb;                          // 0x02
//    uint8_t RegBitrateLsb;                          // 0x03
//    uint8_t RegFdevMsb;                             // 0x04
//    uint8_t RegFdevLsb;                             // 0x05
//    uint8_t RegFrfMsb;                              // 0x06
//    uint8_t RegFrfMid;                              // 0x07
//    uint8_t RegFrfLsb;                              // 0x08
//    // Tx settings
//    uint8_t RegPaConfig;                            // 0x09
//    uint8_t RegPaRamp;                              // 0x0A
//    uint8_t RegOcp;                                 // 0x0B
//    // Rx settings
//    uint8_t RegLna;                                 // 0x0C
//    uint8_t RegRxConfig;                            // 0x0D
//    uint8_t RegRssiConfig;                          // 0x0E
//    uint8_t RegRssiCollision;                       // 0x0F
//    uint8_t RegRssiThresh;                          // 0x10
//    uint8_t RegRssiValue;                           // 0x11
//    uint8_t RegRxBw;                                // 0x12
//    uint8_t RegAfcBw;                               // 0x13
//    uint8_t RegOokPeak;                             // 0x14
//    uint8_t RegOokFix;                              // 0x15
//    uint8_t RegOokAvg;                              // 0x16
//    uint8_t RegRes17;                               // 0x17
//    uint8_t RegRes18;                               // 0x18
//    uint8_t RegRes19;                               // 0x19
//    uint8_t RegAfcFei;                              // 0x1A
//    uint8_t RegAfcMsb;                              // 0x1B
//    uint8_t RegAfcLsb;                              // 0x1C
//    uint8_t RegFeiMsb;                              // 0x1D
//    uint8_t RegFeiLsb;                              // 0x1E
//    uint8_t RegPreambleDetect;                      // 0x1F
//    uint8_t RegRxTimeout1;                          // 0x20
//    uint8_t RegRxTimeout2;                          // 0x21
//    uint8_t RegRxTimeout3;                          // 0x22
//    uint8_t RegRxDelay;                             // 0x23
//    // Oscillator settings
//    uint8_t RegOsc;                                 // 0x24
//    // Packet handler settings
//    uint8_t RegPreambleMsb;                         // 0x25
//    uint8_t RegPreambleLsb;                         // 0x26
//    uint8_t RegSyncConfig;                          // 0x27
//    uint8_t RegSyncValue1;                          // 0x28
//    uint8_t RegSyncValue2;                          // 0x29
//    uint8_t RegSyncValue3;                          // 0x2A
//    uint8_t RegSyncValue4;                          // 0x2B
//    uint8_t RegSyncValue5;                          // 0x2C
//    uint8_t RegSyncValue6;                          // 0x2D
//    uint8_t RegSyncValue7;                          // 0x2E
//    uint8_t RegSyncValue8;                          // 0x2F
//    uint8_t RegPacketConfig1;                       // 0x30
//    uint8_t RegPacketConfig2;                       // 0x31
//    uint8_t RegPayloadLength;                       // 0x32
//    uint8_t RegNodeAdrs;                            // 0x33
//    uint8_t RegBroadcastAdrs;                       // 0x34
//    uint8_t RegFifoThresh;                          // 0x35
//    // Sequencer settings
//    uint8_t RegSeqConfig1;                          // 0x36
//    uint8_t RegSeqConfig2;                          // 0x37
//    uint8_t RegTimerResol;                          // 0x38
//    uint8_t RegTimer1Coef;                          // 0x39
//    uint8_t RegTimer2Coef;                          // 0x3A
//    // Service settings
//    uint8_t RegImageCal;                            // 0x3B
//    uint8_t RegTemp;                                // 0x3C
//    uint8_t RegLowBat;                              // 0x3D
//    // Status
//    uint8_t RegIrqFlags1;                           // 0x3E
//    uint8_t RegIrqFlags2;                           // 0x3F
//    // I/O settings
//    uint8_t RegDioMapping1;                         // 0x40
//    uint8_t RegDioMapping2;                         // 0x41
//    // Version
//    uint8_t RegVersion;                             // 0x42
//    // Additional settings
//    uint8_t RegAgcRef;                              // 0x43
//    uint8_t RegAgcThresh1;                          // 0x44
//    uint8_t RegAgcThresh2;                          // 0x45
//    uint8_t RegAgcThresh3;                          // 0x46
//    // Test
//    uint8_t RegTestReserved47[0x4B - 0x47];         // 0x47-0x4A
//    // Additional settings
//    uint8_t RegPllHop;                              // 0x4B
//    uint8_t RegTestReserved4C;                      // 0x4C
//    uint8_t RegPaDac;                               // 0x4D
//    // Test
//    uint8_t RegTestReserved4E[0x58-0x4E];           // 0x4E-0x57
//    // Additional settings
//    uint8_t RegTcxo;                                // 0x58
//    // Test
//    uint8_t RegTestReserved59;                      // 0x59
//    // Test
//    uint8_t RegTestReserved5B;                      // 0x5B
//    // Additional settings
//    uint8_t RegPll;                                 // 0x5C
//    // Test
//    uint8_t RegTestReserved5D;                      // 0x5D
//    // Additional settings
//    uint8_t RegPllLowPn;                            // 0x5E
//    // Test
//    uint8_t RegTestReserved5F[0x6C - 0x5F];         // 0x5F-0x6B
//    // Additional settings
//    uint8_t RegFormerTemp;                          // 0x6C
//    // Test
//    uint8_t RegTestReserved6D[0x70 - 0x6D];         // 0x6D-0x6F
//    // Additional settings
//    uint8_t RegBitrateFrac;                         // 0x70
//}tSX1276;

//extern tSX1276* SX1276;
/*!
 * SX1276 registers variable
 */
//tSX1276* SX1276;

/*!
 * SX1276 registers variable
 */
uint8_t SX1276Regs[0x70];

static bool LoRaOn = false;
static bool LoRaOnState = false;

void SX1276Init( void )
{
    // Initialize FSK and LoRa registers structure
  //  SX1276 = ( tSX1276* )SX1276Regs;
    SX1276LR = ( tSX1276LR* )SX1276Regs; 
		SX1276InitIo(); //初始化：NSS_PIN、DIO_PIN、
    SX1276Reset();//复位10ms延时1ms再操作寄存器
    LoRaOn = true;
	  SPI_Enable(&hspi2);
    SX1276SetLoRaOn( LoRaOn );
    // Initialize LoRa modem
    SX1276LoRaInit( );
}

void SX1276Reset( void )
{
    SX1276SetReset( RADIO_RESET_ON );
    
    // Wait 1ms
    uint32_t startTick = GET_TICK_COUNT( );
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 100 ) );    

    SX1276SetReset( RADIO_RESET_OFF );
    
    // Wait 6ms
    startTick = GET_TICK_COUNT( );
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 10 ) ); 

}

void SX1276SetLoRaOn( bool enable )
{
    if( LoRaOnState == enable )
    {
        return;
    }
		
    LoRaOnState = enable;
    LoRaOn = enable;
    if( LoRaOn == true )
    {
			RFDebug("file %s on function %s line %d\r\n", __FILE__,__FUNCTION__, __LINE__);
        SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );

        SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON;
        SX1276Write( REG_LR_OPMODE, SX1276LR->RegOpMode );
        
        SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
                                        // RxDone               RxTimeout                   FhssChangeChannel           CadDone
        SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                        // CadDetected          ModeReady
        SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

        SX1276ReadBuffer( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );
				RFDebug("REG_LR_OPMODE:");
				for(int i = 0 ; i < 0x70 ; i ++)
					RFDebug("0x%X ",SX1276Regs[i]);
				RFDebug("\r\n");
			
    }
    else
    {
        SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
        
        SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF;
        SX1276Write( REG_LR_OPMODE, SX1276LR->RegOpMode );
        
        SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
        
    //    SX1276ReadBuffer( REG_OPMODE, SX1276Regs + 1, 0x70 - 1 );
			  SX1276ReadBuffer( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );
    }
}

bool SX1276GetLoRaOn( void )
{
    return LoRaOn;
}

void SX1276SetOpMode( uint8_t opMode )
{
    if( LoRaOn == false )
    {
        ;  //  SX1276FskSetOpMode( opMode );
    }
    else
    {
        SX1276LoRaSetOpMode( opMode );
    }
}

uint8_t SX1276GetOpMode( void )
{
    if( LoRaOn == false )
    {
        return 0;  //  return SX1276FskGetOpMode( );
    }
    else
    {
        return SX1276LoRaGetOpMode( );
    }
}

double SX1276ReadRssi( void )
{
    if( LoRaOn == false )
    {
        return 0;  //  return SX1276FskReadRssi( );
    }
    else
    {
        return SX1276LoRaReadRssi( );
    }
}

uint8_t SX1276ReadRxGain( void )
{
    if( LoRaOn == false )
    {
        return 0;  //  return SX1276FskReadRxGain( );
    }
    else
    {
        return SX1276LoRaReadRxGain( );
    }
}

uint8_t SX1276GetPacketRxGain( void )
{
    if( LoRaOn == false )
    {
        return 0;  //  return SX1276FskGetPacketRxGain(  );
    }
    else
    {
        return SX1276LoRaGetPacketRxGain(  );
    }
}

int8_t SX1276GetPacketSnr( void )
{
    if( LoRaOn == false )
    {
         while( 1 )
         {
             // Useless in FSK mode
             // Block program here
         }
    }
    else
    {
        return SX1276LoRaGetPacketSnr(  );
    }
}

double SX1276GetPacketRssi( void )
{
    if( LoRaOn == false )
    {
        return 0;  //  return SX1276FskGetPacketRssi(  );
    }
    else
    {
        return SX1276LoRaGetPacketRssi( );
    }
}

uint32_t SX1276GetPacketAfc( void )
{
    if( LoRaOn == false )
    {
        return 0;  //  return SX1276FskGetPacketAfc(  );
    }
    else
    {
         while( 1 )
         {
             // Useless in LoRa mode
             // Block program here
         }
    }
}

void SX1276StartRx( void )
{
    if( LoRaOn == false )
    {
        ;  //  SX1276FskSetRFState( RF_STATE_RX_INIT );
    }
    else
    {
        SX1276LoRaSetRFState( RFLR_STATE_RX_INIT );
    }
}

void SX1276GetRxPacket( void *buffer, uint16_t *size )
{
    if( LoRaOn == false )
    {
        ;  //  SX1276FskGetRxPacket( buffer, size );
    }
    else
    {
        SX1276LoRaGetRxPacket( buffer, size );
    }
}

void SX1276SetTxPacket( const void *buffer, uint16_t size )
{
    if( LoRaOn == false )
    {
        ;  //  SX1276FskSetTxPacket( buffer, size );
    }
    else
    {
        SX1276LoRaSetTxPacket( buffer, size );
    }
}

uint8_t SX1276GetRFState( void )
{
    if( LoRaOn == false )
    {
        return 0;  //  eturn SX1276FskGetRFState( );
    }
    else
    {
        return SX1276LoRaGetRFState( );
    }
}

void SX1276SetRFState( uint8_t state )
{
    if( LoRaOn == false )
    {
        ;  //  SX1276FskSetRFState( state );
    }
    else
    {
        SX1276LoRaSetRFState( state );
    }
}

uint32_t SX1276Process( void )
{
    if( LoRaOn == false )
    {
        return 0;  //  return SX1276FskProcess( );
    }
    else
    {
        return SX1276LoRaProcess( );
    }
}


