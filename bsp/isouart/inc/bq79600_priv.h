/*
 * bq79600.h
 *
 *  Created on: Oct 17, 2025
 *      Author: rgeng
 */

#ifndef BSP_ISOUART_INC_BQ79600_PRIV_H_
#define BSP_ISOUART_INC_BQ79600_PRIV_H_
#define Single_Device_Rd            0x80    ///<Single Read
#define Stack_Rd                    0xA0    ///<Daisy Chain Read
#define Broadcast_Rd                0xC0    ///<Broadcast Read

#define Single_Device_Wr            0x90    ///<Single device write
#define Stack_Wr                    0xB0    ///<Stack Write
#define Broadcast_Wr                0xD0    ///<Broadcast write
#define Broadcast_Wr_Reverse        0xE0    ///<Broadcast write with return




#define Z_DIR0_ADDR             0x306
#define Z_DIR1_ADDR             0x307
#define Z_CONTROL1              0x309
#define Z_CONTROL2              0x30A
#define Z_DIAG_CTRL             0x2000
#define Z_DEV_CONF1             0x2001
#define Z_DEV_CONF2             0x2002
#define Z_TX_HOLD_OFF           0x2003
#define Z_SLP_TIMEOUT           0x2004
#define Z_COMM_TIMEOUT          0x2005
#define Z_SPI_FIFO_UNLOCK       0x2010
#define Z_FAULT_MSK             0x2020
#define Z_FAULT_RST             0x2030
#define Z_FAULT_SUMMARY         0x2100
#define Z_FAULT_REG             0x2101
#define Z_FAULT_SYS             0x2102
#define Z_FAULT_PWR             0x2103
#define Z_FAULT_COMM1           0x2104
#define Z_FAULT_COMM2           0x2105
#define Z_DEV_DIAG_STAT         0x2110
#define Z_PARTID                0x2120
#define Z_DIE_ID1               0x2121
#define Z_DIE_ID2               0x2122
#define Z_DIE_ID3               0x2123
#define Z_DIE_ID4               0x2124
#define Z_DIE_ID5               0x2125
#define Z_DIE_ID6               0x2126
#define Z_DIE_ID7               0x2127
#define Z_DIE_ID8               0x2128
#define Z_DIE_ID9               0x2129
#define Z_DEBUG_CTRL_UNLOCK     0x2200
#define Z_DEBUG_COMM_CTRL       0x2201
#define Z_DEBUG_COMM_STAT       0x2300
#define Z_DEBUG_SPI_PHY         0x2301
#define Z_DEBUG_SPI_FRAME       0x2302
#define Z_DEBUG_UART_FRAME      0x2303
#define Z_DEBUG_COMH_PHY        0x2304
#define Z_DEBUG_COMH_FRAME      0x2305
#define Z_DEBUG_COML_PHY        0x2306
#define Z_DEBUG_COML_FRAME      0x2307

#define DIS_SHORT_TIMEOUT   0x00///<Prohibit short comm timeout
#define DIS_LONG_TIMEOUT    0x00///<Prohibit long comm timeout
#define LONG2SHUTDOWN       0x01///<Long comm timeout, to shutdown mode
#define LONG2SLEEP          0x00///<Long comm timeout, to sleep mode



typedef struct _Bq79600_Register_Group
{
    union
    {
        struct
        {
            uint8 ADDRESS  : 6; //Follow steps in section Device Addressing?to config this register. A
            uint8 RSVD     : 2;
        } Bit;
        uint8 Byte;
    } DIR0_ADDR;

    union
    {
        struct
        {
            uint8 ADDRESS  : 6; //Follow steps in section Device Addressing?to config this register. A
            uint8 RSVD     : 2;
        } Bit;
        uint8 Byte;
    } DIR1_ADDR;

    union
    {
        struct
        {
            uint8 ADDR_WR          : 1; //Enable device to start auto-addressing
            uint8 SOFT_RESET       : 1; //Reset the digital to OTP default
            uint8 GOTO_SLEEP       : 1; //Transition device to SLEEP mode.
            uint8 GOTO_SHUTDOWN    : 1; //Transition device to SHUTDOWN mode
            uint8 SEND_SLPTOACT    : 1; //Send SLEEPtoWAKE tone up the stack
            uint8 SEND_WAKE        : 1; //Send WAKE tone up the stack
            uint8 SEND_SHUTDOWN    : 1; //Sends SHUTDOWN tone to next device up the stack
            uint8 DIR_SEL          : 1; //Select daisy chain comm direction. Not self-clear bit.
        } Bit;
        uint8 Byte;
    } CONTROL1;

    union
    {
        struct
        {
            uint8 SPARE            : 1;
            uint8 SEND_HW_RESET    : 1; //Send HW_RESET tone up the stack
            uint8 RSVD             : 6;
        } Bit;
        uint8 Byte;
    } CONTROL2;

    union
    {
        struct
        {
            uint8 INH_SET_GO           : 1; //This bits intentionally activates INH PMOS pull up, sets [INH] and [INH_STAT] to 1
            uint8 FLIP_TR_CRC          : 1; //Sends a purposely incorrect communication CRC by inverting all of the calculated CRC bits
            uint8 FLIP_FACT_CRC        : 1; //An enable bit to flip the factory CRC expected value. This is for factory CRC diagnostic.
            uint8 SPI_FIFO_DIAG_GO     : 1; //nitiate SPI FIFO diagnostic
            uint8 PWR_DIAG_GO          : 1; //Indicates a power supply BIST diagnostic is initiated, self-clear bit.
            uint8 CONF_MON_GO          : 1; //Resample 2 registers setting
            uint8 RSVD                 : 2;
        } Bit;
        uint8 Byte;
    } DIAG_CTRL;

    union
    {
        struct
        {
            uint8 HB_TX_EN             : 1; //Enable HEARTBEAT transmitter when device is in SLEEP mode
            uint8 SEL_TX_TONE          : 1; //Select daisy chain transmits bq79606 tones or non bq7606 tones
            uint8 NFAULT_EN            : 1; //Enables the NFAULT function
            uint8 TWO_STOP_EN          : 1; //Enables two stop bits for the UART in case of severe oscillator error in both the host and device
            uint8 FCOMM_EN             : 1; //Enable the fault state detection through communication in ACTIVE mode
            uint8 TONE_RX_EN           : 1; //Enable the Tone receiver, depends on [DIR_SEL],  in VALIDATE
            uint8 SNIFDET_DIS          : 1; //Enable the Sniff detector on COM* port, this bit is latched into AVAOREF domain,
            //once latched in, this bit is still effective in SHUTDOWN
            uint8 SNIFDET_EN           : 1; //Enable the Sniff detector on COM* port, this bit is latched into AVAOREF domain,
            //once latched in, this bit is still effective in SHUTDOWN
        } Bit;
        uint8 Byte;
    } DEV_CONF1;

    union
    {
        struct
        {
            uint8 INH_DIS       : 2; //Disable INH driver (PMOS pull up).
            uint8 RSVD          : 6;
        } Bit;
        uint8 Byte;
    } DEV_CONF2;

    uint8 TX_HOLD_OFF; //Set the number of bit period delay from 0 to 255, after receiving the STOP bit of a command frame
    //and before transmitting the 1st bit of response frame

    union
    {
        struct
        {
            uint8 SLP_TME        : 3; //This timer starts counting when device enters SLEEP or VALIDATE
            uint8 RSVD           : 5;
        } Bit;
        uint8 Byte;
    } SLP_TIMEOUT;

    union
    {
        struct
        {
            uint8 CTL_TIME       : 3; //Set the long communication timeout.
            uint8 CTL_ACT        : 1; //Configure the device action when long communication timeout timer expires
            uint8 CTS_TIME       : 3; //Set the short communication timeout. When this timer expires, the device set the FAULT_SYS [CTS] bit.
            //This can be used as an alert to the system to prevent a long communication timeout.
            uint8 RSVD           : 1;
        } Bit;
        uint8 Byte;
    } COMM_TIMEOUT;

    union
    {
        struct
        {
            /*
            *In UART mode - write has no impact and read always returns 0 In SPI mode - Write the unlock code 0x0A to SPI_FIFO_UNLOCK (MSB 4
            *bits are don't care, e.g. 0x2A would also unlock) and followed by writing [SPI_FIFO_DIAG_GO] = 1 to do FIFO diagnostic. For detailed
            *steps of FIFO diagnostic, refer to safety manual. After these bits are written, read cmd doesn't affect them while any write cmd clears them
            */

            uint8 CODE           : 4; //Set the long communication timeout.
            uint8 RSVD           : 4;
        } Bit;
        uint8 Byte;
    } SPI_FIFO_UNLOCK;


    union
    {
        struct
        {
            uint8 MSK_PWR           : 1;
            uint8 MSK_SYS           : 1;
            uint8 MSK_REG           : 1;
            uint8 MSK_HB            : 1;
            uint8 MSK_FTONE_DET     : 1;
            uint8 MSK_FCOMM_DET     : 1;
            uint8 MSK_UART_SPI      : 1;
            uint8 MSK_COML_H        : 1;
        } Bit;
        uint8 Byte;
    } FAULT_MSK;

    union
    {
        struct
        {
            uint8 RST_PWR           : 1; //Resets FAULT_SUMMARY [FAULT_PWR] to '0' and register FAULT_PWR
            uint8 RST_SYS           : 1; //This bit is self-clear to 0 after writing to 1.
            uint8 RST_REG           : 1; //Resets FAULT_SUMMARY [FAULT_REG] to '0', self-cleared bit.
            uint8 RST_HB            : 1; //Reset HB_FAIL and HB_FAST bit, self-cleared bit.
            uint8 RST_FTONE_DET     : 1; //Reset FTONE_DET bit, self-cleared bit.
            uint8 RST_FCOMM_DET     : 1; //Reset FCOMM_DET bit, self-cleared bit.
            uint8 RST_UART_SPI      : 1; //Reset STOP_DET, COMMCLR_DET, UART_FRAME, SPI_FRAME, SPI_PHY bits and DEBUG_UART_FRAME, DEBUG_SPI_PHY,
            //DEBUG_SPI_FRAME registers. self-cleared bit.
            uint8 RST_COML_H        : 1; //Reset bits: COML_FRAME, COMH_FRAME, COML_PHY, COMH_PHY and registers: DEBUG_COMH_PHY, DEBUG_COMH_FRAME,
            //DEBUG_COML_PHY, DEBUG_COML_FRAME. self-cleared bit.
        } Bit;
        uint8 Byte;
    } FAULT_RST;

    union
    {
        struct
        {
            uint8 FAULT_PWR         : 1; //Indicates a power supply fault is detected (any bits in register FAULT_PWR)
            uint8 FAULT_SYS         : 1; //Indicates system fault is detected (any bits in register FAULT_SYS).
            uint8 FAULT_REG         : 1; //Indicate registers related fault is detected
            uint8 FAULT_COMM        : 1; //Indicate communication related fault is detected (any bits in register FAULT_COMM1, FAULT_COMM2 is set)
            uint8 RSVD              : 4;
        } Bit;
        uint8 Byte;
    } FAULT_SUMMARY;

    union
    {
        struct
        {
            uint8 FACT_CRC           : 1; //Indicates a CRC error has occurred in the factory register space.
            uint8 FACTLDERR          : 1; //Indicates the factory NVM registers could not be loaded from OTP
            uint8 CONF_MON_ERR       : 1; //Indicates monitored 2 registers (DEV_CONF1, FAULT_MSK) have at least one bit flip.
            uint8 RSVD               : 5;
        } Bit;
        uint8 Byte;
    } FAULT_REG;

    union
    {
        struct
        {
            uint8 INH                : 1; //Indicates INH PMOS is enabled
            uint8 TSHUT              : 1; //Indicates the previous shutdown was due to thermal shutdown
            uint8 CTS                : 1; //Indicates a short communication timeout occurred.
            uint8 CTL                : 1; //Indicates a long communication timeout occurred
            uint8 DRST               : 1; //Indicates a digital reset has occurred.
            uint8 SHUTDOWN_REC       : 1; //Indicates the device was shut down using SHUTDOWN ping.
            //If this bit is set, the COML and COMH RX are both disabled at wake up as a way to isolate itself from the stack.
            uint8 LFO                : 1; //Indicated LFO frequency is out of the expect range
            uint8 VALIDATE_DET       : 1; //Indicates device transitioned to VALIDATE mode
        } Bit;
        uint8 Byte;
    } FAULT_SYS;

    union
    {
        struct
        {
            uint8 AVAO_SW_FAIL        : 1; //Indicates a fault is detected on the AVAO_REF switch.
            uint8 AVDDREF_OV          : 1; //Indicates an over voltage fault on the AVDD_REF internal supply
            uint8 DVDD_OV             : 1; //Indicates an over voltage fault on the DVDD pin
            uint8 CVDD_OV             : 1; //Indicates an over voltage fault on the CVDD pin
            uint8 CVDD_UV_DRST        : 1; //Indicates CVDDUV fault caused DRST. Shutdown/power up event shall not trigger this fault.
            uint8 RSVD                : 1;
        } Bit;
        uint8 Byte;
    } FAULT_PWR;

    union
    {
        struct
        {
            uint8 STOP_DET            : 1; //Indicates and unexpected STOP condition is received. Apply to UART mode
            uint8 COMMCLR_DET         : 1; //A UART/SPI communication clear signal is detected.
            uint8 UART_FRAME          : 1; //Indicates a UART FAULT detected when receiving command or transmitting response frames Further
            uint8 HB_FAST             : 1; //Indicates HEARTBEAT is received too frequently either from FAULT Port if work with bq79606
            uint8 HB_FAIL             : 1; //Indicates HB tone is detected too fast or too slow either from FAULT Port
            uint8 FTONE_DET           : 1; //Indicates the fault tone is detected, either from FAULT Port
            //If this bit is set, the COML and COMH RX are both disabled at wake up as a way to isolate itself from the stack.
            uint8 FCOMM_DET           : 1; //Indicates the fault status bit in comm frame is set by stack devices
            uint8 RSVD                : 1;
        } Bit;
        uint8 Byte;
    } FAULT_COMM1;

    union
    {
        struct
        {
            uint8 COMH_PHY            : 1; //Indicate a COMH bit level fault detected, which would cause at least a byte level (RC_RR) fault.
            uint8 COMH_FRAME          : 1; //Indicate a COMH byte level fault detected when receiving response frames.
            uint8 COML_PHY            : 1; //Indicate a COML bit level fault detected,
            uint8 COML_FRAME          : 1; //Indicate a COML byte level fault detected
            uint8 SPI_PHY             : 1; //Indicates a SPI bit level FAULT detected when receiving command or transmitting response frames
            uint8 SPI_FRAME           : 1; //Indicates a SPI frame level FAULT detected when receiving command or transmitting response frames
            uint8 RSVD                : 2;
        } Bit;
        uint8 Byte;
    } FAULT_COMM2;

    union
    {
        struct
        {
            uint8 INH_STAT             : 1; //Indicates INH Pin see VINH_DET, this bit reflects real time value of pin INH
            uint8 PWR_DIAG_RDY         : 1; //Indicates a power supply BIST test is done. It is cleared when [PWR_DIAG_GO] bit is set.
            uint8 RSVD                 : 6;
        } Bit;
        uint8 Byte;
    } DEV_DIAG_STAT;

    uint8 PARTID;  //Device revision 0x00 = A0, 0x01 = A1, 0x02 = A2, etc.
    uint8 DIE_ID[9];   //Digital reset value 0x00, after factory NVM loaded successfully, it is value in the NVM
    uint8 DEBUG_CTRL_UNLOCK;   //Write the unlock code (0xA5) to this register will activate the setting in the DEBUG_COMM_CTRL register


    union
    {
        struct
        {
            uint8 USER_DAISY_EN         : 1; //This bit enables the setting of [6:3] in this register
            uint8 USER_UART_EN          : 1; //This bit enables [UART_VIF_BAUD] setting
            uint8 UART_VIF_BAUD         : 1; //This bit changes the baud rate of UART and Daisy Chain (Also called VIF) to 250kb/s
            uint8 COMH_RX_EN            : 1; //Enable COMH receiver
            uint8 COMH_TX_EN            : 1; //Enable COMH transmitter
            uint8 COML_RX_EN            : 1; //Enable COML receiver
            uint8 COML_TX_EN            : 1; //Enable COML transmitter
            uint8 RSVD                  : 1;
        } Bit;
        uint8 Byte;
    } DEBUG_COMM_CTRL;

    union
    {
        struct
        {
            uint8 COMH_RX_ON             : 1; //Show the current COMH receiver status
            uint8 COMH_TX_ON             : 1; //Show the current COMH transmitter status
            uint8 COML_RX_ON             : 1; //Show the current COML receiver status
            uint8 COML_TX_ON             : 1; //Show the current COML transmitter status
            uint8 HW_DAISY_DRV           : 1; //Indicates the COML and COMH are controlled by the device itself or by MCU control
            uint8 RSVD                   : 3;
        } Bit;
        uint8 Byte;
    } DEBUG_COMM_STAT;

    union
    {
        struct
        {
            uint8 RXFIFO_OF               : 1; //While device is receiving command frame, more data is sent than device can accommodate in RX FIFO.
            uint8 TXFIFO_UF               : 1; //In Host Read mode, when both FIFOs are empty and the host continues to send clocks for more read data (send 0xFF through MOSI).
            uint8 TXFIFO_OF               : 1; //If SPI module receives data (from itself or daisy chain) when current FIFO is full and 2nd FIFO is not empty
            uint8 RXDATA_UNEXP            : 1; //Host sends data other than 0xFF during Device Read mode OR initiates SPI communication when SPI_RDY = 0
            uint8 TXDATA_UNEXP            : 1; //This fault occurs when:
            uint8 COMCLR_ERR              : 1; //More than 8 SCLK pulses are received during a comm clear.
            uint8 FMT_ERR                 : 1; //When not in Host Read mode (after sending a write cmd frame), indicates malformed cmd is received:
            uint8 RSVD                    : 1;
        } Bit;
        uint8 Byte;
    } DEBUG_SPI_PHY;

    union
    {
        struct
        {
            uint8 RC_CRC                  : 1; //Detects are CRC error in the received command frame from SPI
            uint8 RC_BYTE_ERR             : 1; //Device receives data from the VIF that needs to go out the SPI, while host send data into device through SPI
            uint8 RC_SOF                  : 1; //Detects a start-of-frame (SOF) error. Mostly, this is SPI COMM CLEAR is received on the SPI before the current frame is finished, e.g.
            //partial cmd frame is sent
            uint8 RC_IERR                 : 1; //Detects initialization byte error in the received command frame.
            uint8 TR_WAIT                 : 1; //Indicates that a SPI COMM CLEAR is received while there is no data in output FIFO regardless of SPI_RDY.
            uint8 TR_SOF                  : 1; //Indicates that a SPI COMM CLEAR is received while there is data in output FIFO
            uint8 RSVD                    : 2;
        } Bit;
        uint8 Byte;
    } DEBUG_SPI_FRAME;


    union
    {
        struct
        {
            uint8 RC_CRC                  : 1; //Detects are CRC error in the received command frame from UART
            uint8 RC_BYTE_ERR             : 1; //Other than INIT byte, if STOP error is detected in rest of byte in the frame or RX receives data while RX is using
            uint8 RC_SOF                  : 1; //Detects a start-of-frame (SOF) error
            uint8 RC_IERR                 : 1; //Detects initialization byte error in the received command frame
            uint8 TR_WAIT                 : 1; //The device is waiting for its turn to transfer a response out but the action is terminated because
            uint8 TR_SOF                  : 1; //Indicates that a UART COMM CLEAR is received while the device is still transmitting data to host.
            uint8 RSVD                    : 2;
        } Bit;
        uint8 Byte;
    } DEBUG_UART_FRAME;


    union
    {
        struct
        {
            uint8 BIT                      : 1; //The device has detected a data bit, however, the detection samples is not enough to guarantee a strong ??or ??
            uint8 SYNC1                    : 1; //If the demodulation of the preamble half-bit and the two full bits of synchronization data have errors
            uint8 SYNC2                    : 1; //Timing information extracted from the demodulation of the preamble half-bit and the two full bits
            uint8 BERR_TAG                 : 1; //Indicates BERR bit is set in at least one byte in received response frame from stack device
            uint8 PERR                     : 1; //Pertain to received response frame from stack device only
            uint8 RSVD                     : 3;
        } Bit;
        uint8 Byte;
    } DEBUG_COMH_PHY;

    union
    {
        struct
        {
            uint8 RR_CRC                   : 1; //Indicates one or more COMH response frames being discarded due to CRC error.
            uint8 RR_UNEXP                 : 1; //If [DIR_SEL] = 1, but device received response frame from COMH
            uint8 RR_BYTE_ERR              : 1; //Valid when [DIR_SEL] = 0. Detected any byte errors
            uint8 RR_SOF                   : 1; //Valid when [DIR_SEL] = 0. Detects a start-of-frame (SOF) error on COMH.
            uint8 RR_IERR                  : 1; //Detects initialization byte error in the received response frame
            uint8 RSVD                     : 3;
        } Bit;
        uint8 Byte;
    } DEBUG_COMH_FRAME;

    union
    {
        struct
        {
            uint8 BIT                      : 1; //The device has detected a data bit
            uint8 SYNC1                    : 1; //If the demodulation of the preamble half-bit and the two full bits of synchronization data have errors
            uint8 SYNC2                    : 1; //Timing information extracted from the demodulation of the preamble half-bit
            uint8 BERR_TAG                 : 1; //Indicates BERR bit is set in at least one byte in received response frame from stack device
            uint8 PERR                     : 1; //Pertain to received response frame from stack device only
            uint8 RSVD                     : 3;
        } Bit;
        uint8 Byte;
    } DEBUG_COML_PHY;

    union
    {
        struct
        {
            uint8 RR_CRC                   : 1; //Indicates one or more COML response frames being discarded due to CRC error.
            uint8 RR_UNEXP                 : 1; //If [DIR_SEL] = 0, but device received response frame from COML
            uint8 RR_BYTE_ERR              : 1; //Valid when [DIR_SEL] = 1. Detected any byte errors, other than the error in the initialization byte, in the received response frame
            uint8 RR_SOF                   : 1; //Valid when [DIR_SEL] = 1.
            uint8 RR_IERR                  : 1; //Detects initialization byte error in the received response frame.
            uint8 RSVD                     : 3;
        } Bit;
        uint8 Byte;
    } DEBUG_COML_FRAME;


} _Bq79600_RegisterGroup;
extern _Bq79600_RegisterGroup Bq79600_Reg;

#endif /* BSP_ISOUART_INC_BQ79600_PRIV_H_ */
