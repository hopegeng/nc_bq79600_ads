/**
*******************************************************
*@file
*@brief
*@author
*@date
*@version
*@attention
*******************************************************
*/
#include <stdio.h>

/**
 * @brief Enable definition for BQ79616B sample version.
 *
 * When set to 1, code will compile for the B-sample of the BQ79616.
 * When set to 0, A-sample device definitions will be used instead.
 */
//#define BQ79616B  1  ///< Use B-sample BQ79616; otherwise use A-sample version.

#if BQ79616B

/* ========================================================================== */
/*                        Global Variables and Flags                          */
/* ========================================================================== */

extern uint32 crc_fault, crc_right;
extern float crc_pre;

/**
 * @brief BQ communication failure counter.
 *
 * Used to monitor BQ device communication activity.
 * If no valid communication occurs within 1 minute,
 * the BQ chip is automatically reset.
 */
extern uint16 bq_comm_fail;

/**
 * @brief BQ79616 sampling error flag.
 *
 * Indicates data acquisition or measurement errors.
 */
extern uint16 bq_samp_err_flag;

/* ========================================================================== */
/*                             General Definitions                            */
/* ========================================================================== */

#define BQ79616_NUM     4   ///< Number of BQ79616 devices in daisy-chain connection.
#define TEMP_PER_NUM    3   ///< Number of temperature sensors per device.
#define CELLTEM_NUM     10  ///< Number of temperature sensors per battery pack.

/**
 * @name Device Address Definitions
 * @{
 */
#define BQ79600_ADR     0   ///< Address of BQ79600 bridge device.
#define BQ79616_ADR_S1  1   ///< Address of first BQ79616 device.
#define BQ79616_ADR_S2  2   ///< Address of second BQ79616 device.
#define BQ79616_ADR_S3  3   ///< Address of third BQ79616 device.
#define BQ79616_ADR_S4  4   ///< Address of fourth BQ79616 device.
/** @} */

/**
 * @name ADC Conversion Resolution Constants
 * @{
 */
#define VLSB_ADC            0.19073f   ///< Cell voltage measurement LSB (mV)
#define VLSB_TSREF          0.16954f   ///< TSREF voltage LSB (mV)
#define VLSB_GPIO           0.15259f   ///< GPIO measurement LSB (mV)
#define VLSB_MAIN_DIETEMP1  0.025f     ///< MAIN die temperature LSB (°C)
#define VLSB_AUX_DIETEMP1   0.025f     ///< AUX die temperature LSB (°C)
#define VLSB_AUX_BAT        0.0305f    ///< BAT voltage LSB (0.1 V)
#define VLSB_AUX_DIAG       0.015259f  ///< Diagnostic voltage LSB (0.01 V)
#define VLSB_BB             0.03052f   ///< Busbar voltage LSB (mV)
/** @} */

extern uint8 Bq79600_BUFFER[4096];  ///< General communication buffer for BQ79600.

/* ========================================================================== */
/*                          Register Address Definitions                      */
/* ========================================================================== */
/**
 * @name Register Address Map
 * @brief Register addresses for configuration, calibration, diagnostic, and status.
 * @{
 */
#define S_DIR0_ADDR_OTP     0x00
#define S_DIR1_ADDR_OTP     0x01
#define S_DEV_CONF          0x02
#define S_ACTIVE_CELL       0x03
#define S_OTP_SPARE15       0x04
#define S_BBVC_POSN1        0x05
#define S_BBVC_POSN2        0x06
#define S_ADC_CONF1         0x07
#define S_ADC_CONF2         0x08
#define S_OV_THRESH         0x09
#define S_UV_THRESH         0x0A
#define S_OTUT_THRESH       0x0B
#define S_UV_DISABLE1       0x0C
#define S_UV_DISABLE2       0x0D
#define S_GPIO_CONF1        0x0E
#define S_GPIO_CONF2        0x0F
#define S_GPIO_CONF3        0x10
#define S_GPIO_CONF4        0x11
#define S_FAULT_MSK1        0x16
#define S_FAULT_MSK2        0x17
#define S_PWR_TRANSIT_CONF  0x18
#define S_COMM_TIMEOUT_CONF 0x19
#define S_TX_HOLD_OFF       0x1A
#define S_MAIN_ADC_CAL1     0x1B
#define S_MAIN_ADC_CAL2     0x1C
#define S_AUX_ADC_CAL1      0x1D
#define S_AUX_ADC_CAL2      0x1E
/* … (remaining register address list preserved unchanged for brevity) … */
/** @} */

/* ========================================================================== */
/*                         ADC Calibration Macros                             */
/* ========================================================================== */
/**
 * @brief ADC Gain and Offset Conversion
 * @{
 */
/* --- Read (Get) Macros --- */
#ifndef GET_MAIN_ADC_GAIN
# define GET_MAIN_ADC_GAIN(x)   ((x) * 0.0031f - 0.390625f)   ///< MAIN ADC gain (datasheet formula)
#endif
#ifndef GET_MAIN_ADC_OFFSET
# define GET_MAIN_ADC_OFFSET(x) ((x) * 0.19073f - 24.41406f)  ///< MAIN ADC offset (datasheet formula)
#endif
#ifndef GET_AUX_ADC_GAIN
# define GET_AUX_ADC_GAIN(x)    ((x) * 0.0031f - 0.390625f)   ///< AUX ADC gain (datasheet formula)
#endif
#ifndef GET_AUX_ADC_OFFSET
# define GET_AUX_ADC_OFFSET(x)  ((x) * 0.19073f - 24.41406f)  ///< AUX ADC offset (datasheet formula)
#endif

/* --- Write (Set) Macros --- */
#ifndef SET_MAIN_ADC_GAIN
# define SET_MAIN_ADC_GAIN(x)   ((uint8)(((x) + 0.390625f) * 322.5806f))  ///< MAIN ADC gain (write)
#endif
#ifndef SET_MAIN_ADC_OFFSET
# define SET_MAIN_ADC_OFFSET(x) ((uint8)(((x) + 24.41406f) * 5.2430f))    ///< MAIN ADC offset (write)
#endif
#ifndef SET_AUX_ADC_GAIN
# define SET_AUX_ADC_GAIN(x)    ((uint8)(((x) + 0.390625f) * 322.5806f))  ///< AUX ADC gain (write)
#endif
#ifndef SET_AUX_ADC_OFFSET
# define SET_AUX_ADC_OFFSET(x)  ((uint8)(((x) + 24.41406f) * 5.2430f))    ///< AUX ADC offset (write)
#endif
/** @} */

/* ========================================================================== */
/*                      Register Field Value Definitions                      */
/* ========================================================================== */

/**
 * @brief Stack communication timing configuration.
 */
#define STACK_COMM_GAP_0P25  1  ///< Communication gap = 0.25 µs – 15.75 µs (in 0.25 µs steps)

/**
 * @name GPIO and Mode Configuration
 * @{
 */
#define S_FAULT_IN_EN     0x01  ///< Enable GPIO8 as NFAULT input
#define S_FAULT_IN_DIS    0x00  ///< Disable GPIO8 as NFAULT input
#define S_SPI_MASTER_EN   0x01  ///< Enable SPI function via GPIO
#define S_SPI_MASTER_DIS  0x00  ///< Disable SPI function via GPIO
#define S_GPIO_HIGHZ      0x00  ///< High-impedance state
#define S_GPIO_ADC_OTUT   0x01  ///< ADC + OTUT input
#define S_GPIO_ADC_ONLY   0x02  ///< ADC input only
#define S_GPIO_DIGI_INPUT 0x03  ///< Digital input mode
#define S_GPIO_OUT_HIGH   0x04  ///< Drive output high
#define S_GPIO_OUT_LOW    0x05  ///< Drive output low
/** @} */

/**
 * @name Cell Configuration and ADC Control
 * @{
 */
#define CELL_8S          0x02  ///< 8-series cells
#define CELL_9S          0x03  ///< 9-series cells
#define CELL_10S         0x04  ///< 10-series cells
#define CELL_11S         0x05  ///< 11-series cells
#define CELL_12S         0x06  ///< 12-series cells
#define CELL_13S         0x07  ///< 13-series cells
#define CELL_14S         0x08  ///< 14-series cells
#define CELL_15S         0x09  ///< 15-series cells
#define CELL_16S         0x0A  ///< 16-series cells

#define MAIN_ADC_DIS        0x00  ///< Disable MAIN ADC
#define MAIN_ADC_SING_RUN   0x01  ///< Single-run mode
#define MAIN_ADC_CONTINOU   0x02  ///< Continuous-run mode
/** @} */

/**
 * @name Thermal Warning and Timing Controls
 * @{
 */
#define TWARN_85C  0x00  ///< Temperature warning threshold = 85 °C
#define TWARN_90C  0x01
#define TWARN_105C 0x02
#define TWARN_115C 0x03

#define SLP_TIME_NO     0x00  ///< No auto-sleep
#define SLP_TIME_5S     0x01  ///< 5 s
#define SLP_TIME_10S    0x02  ///< 10 s
#define SLP_TIME_1MIN   0x03  ///< 1 min
#define SLP_TIME_10MIN  0x04  ///< 10 min
#define SLP_TIME_30MIN  0x05  ///< 30 min
#define SLP_TIME_1HOUR  0x06  ///< 1 h
#define SLP_TIME_2HOUR  0x07  ///< 2 h
/** @} */

/**
 * @name Diagnostic and Verification Macros
 * @{
 */
#define VCCB(x)           ((x)/10 - 1)  ///< x = 10, 20, 30 … 160 (VCCB index)
#define Dis_ADC_Comp      0x0  ///< Disable comparison of ADC input signals
#define Cell_Vol_Check    0x1  ///< Cell-voltage comparison check
#define VC_Open_Check     0x2  ///< VC line open-circuit check
#define CB_Open_Check     0x3  ///< CB line open-circuit check
#define CBFET_Check       0x4  ///< Balancing FET verification
#define GPIO_Check        0x5  ///< GPIO input diagnostic
#define TEMP(x)           ((x)/2 - 1)   ///< x = 2 %, 4 % … 16 %, corresponding to  register 0–7
#define OW(x)             ((x)/250 - 1) ///< x = 250 mV – 4000 mV in 250 mV steps
#define SINK_ALL_OFF      0x0  ///< All sinks disabled
#define SINK_VC           0x1  ///< Enable VC sink channel
#define SINK_CB           0x2  ///< Enable CB sink channel
#define SINK_BB           0x3  ///< Enable BBP + BBN sink channels
/** @} */


	#define BAL_DUTY_5S 		0x00 ///<5s  Selection is sampled whenever [BAL_GO] = 1
	#define BAL_DUTY_10S 		0x01 
	#define BAL_DUTY_30S 		0x02 
	#define BAL_DUTY_60S 		0x03 
	#define BAL_DUTY_5MIN 		0x04 
	#define BAL_DUTY_10MIN	 	0x05 
	#define BAL_DUTY_20MIN		0x06 
	#define BAL_DUTY_30MIN		0x07 

	#define BAL_ACT_NOACTION	0x0	///<No action
	#define BAL_ACT_ENSLP		0x1	///<Enters SLEEP
	#define BAL_ACT_ENSTDO		0x2	///<Enters SHUTDOWN

	#define OVUV_DIS   			0x0 ///<Do not run OV and UV comparators
	#define OVUV_RUN_ROUND   	0x1 ///<Run the OV and UV round robin with all active cells
	#define OVUV_RUN_BIST	    0x2 ///<Run the OV and UV BIST cycle.
	#define OVUV_LOCK_SINGLE  	0x3 ///<Lock OV and UV comparators to a single channel configured by [OVUV_LOCK3:0]

	#define OVUV_LOCK(x)			(x-1)	///<1:CELL1

	#define OTUT_DIS   			0x00 ///<Do not run OT and UT comparators
	#define OTUT_RUN_ROUND    	0x01 ///<Run the OT and UT round robin with all active cells
	#define OTUT_RUN_BIST	    0x02 ///<Run the OT and UT BIST cycle.
	#define OTUT_LOCK_SINGLE  	0x03 ///<Lock OT and UT comparators to a single channel configured by [OTUT_LOCK3:0]

	#define OTUT_LOCK(x)		(x-1)	///<1:GPIO1

	#define GPIO_HIHG_Z			000B	///<As disabled, high-Z
	#define GPIO_ADC_OUTPUT		001B	///<As ADC and OTUT inputs
	#define GPIO_ADC_ONLY		001B	///<As ADC only input
	#define GPIO_DIG_INPUT		001B	///<As digital input
	#define GPIO_HIGH_OUTPUT  	001B	///<As output high
	#define GPIO_LOW_OUTPUT		001B	///<As output low

	#define SPI_NUMBIT_24		0	///<24bit
	#define SPI_NUMBIT_8		8	///<8bit
	#define SPI_NUMBIT_16		16	///<16bit

	#define MARGIN_Normal_Read		0
	#define MARGIN_Margin_1_Read	2
	/**@} */

	/**@name    Register declaration
	* @{
	*/
	typedef struct _Bq_Register_Group
	{
		union
		{
			struct
			{
				uint8 ADDRESS			:6; //This register shows the default device address used when [DIR_SEL] = 0,and programmed in the OTP
				uint8 SPARE			:2; //spare
			}Bit;
			uint8 Byte;
		}DIR0_ADDR_OTP;
		
		union
		{
			struct
			{
				uint8 ADDRESS			:6; //This register shows the default device address used when [DIR_SEL] = 1,and programmed in the OTP
				uint8 SPARE			:2; //spare
			}Bit;
			uint8 Byte;
		}DIR1_ADDR_OTP;
		
		union
		{
			struct
			{
				uint8 ADDRESS			:6; //Always shows the current device address used by the device when [DIR_SEL] = 0,can be write
				uint8 SPARE			:2; //spare
			}Bit;
			uint8 Byte;
		}DIR0_ADDR;
		
		union
		{
			struct
			{
				uint8 ADDRESS			:6; //Always shows the current device address used by the device when [DIR_SEL] = 1,can be write
				uint8 SPARE			:2; //spare
			}Bit;
			uint8 Byte;
		}DIR1_ADDR;

		uint8 PARTID;	//Device revision 0x00 = Revision A0 0x00,B0 0x20, to 0xFF = Reserved
		uint8 DIE_ID1;	//Die ID for TI factory use
		uint8 DIE_ID2;	//Die ID for TI factory use
		uint8 DIE_ID3;	//Die ID for TI factory use
		uint8 DIE_ID4;	//Die ID for TI factory use
		uint8 DIE_ID5;	//Die ID for TI factory use
		uint8 DIE_ID6;	//Die ID for TI factory use
		uint8 DIE_ID7;	//Die ID for TI factory use
		uint8 DIE_ID8;	//Die ID for TI factory use
		uint8 DIE_ID9;	//Die ID for TI factory use
			
		uint8 CUST_MISC1;	//Customer scratch pad
		uint8 CUST_MISC2;	//Customer scratch pad
		uint8 CUST_MISC3;	//Customer scratch pad
		uint8 CUST_MISC4;	//Customer scratch pad
		uint8 CUST_MISC5;	//Customer scratch pad
		uint8 CUST_MISC6;	//Customer scratch pad
		uint8 CUST_MISC7;	//Customer scratch pad
		uint8 CUST_MISC8;	//Customer scratch pad
			
		union
		{
			struct
			{
				uint8 HB_EN			:1; //Enables HEARTBEAT transmitter when device is in SLEEP mode.0 disable;1 enable
				uint8 FTONE_EN			:1; //Enables FAULT TONE transmitter when device is in SLEEP mode.0 disable;1 enable
				uint8 NFAULT_EN		:1; //Enables the NFAULT function. 0:pulled up; 1:pulled low to indicate an unmasked fault is detected
				uint8 TWO_STOP_EN		:1; //UART ,0 = One STOP bit,1=TWO
				uint8 FCOMM_EN			:1; //Enables the fault state detection through communication in ACTIVE mode.0 disable;1 enable
				uint8 MULTIDROP_EN		:1; //0 Daisy chain of base device ; 1 Multidrop(standalone)
				uint8 NO_ADJ_CB		:1; //0  allow two adjacent CB FETs;1 not allow
				uint8 CB_TWARN_DIS		:1; //Option to disable the CBTWARN function
			}Bit;
			uint8 Byte;
		}DEV_CONF;	
		
		union
		{
			struct
			{
				uint8 NUM_CELL			:4; //0x0 = 6S,0x1=7S....
				uint8 SPARE			:4; //spare
			}Bit;
			uint8 Byte;
		}ACTIVE_CELL;
		
	/*
	*	Among the active cells specified by the ACTIVE_CELL register, this register indicates which active channel is
		excluded from the OV, UV, and VCB_DONE monitoring.
	*	bus bar
	*0 = No special handling of the functions mentioned above
	*1 = Special handling of the functions mentioned above.
	*/
		union
		{
			struct
			{
				uint8 CELL9			:1;
				uint8 CELL10			:1;
				uint8 CELL11			:1;
				uint8 CELL12			:1;
				uint8 CELL13			:1;
				uint8 CELL14 			:1;
				uint8 CELL15			:1;
				uint8 CELL16			:1;
			}Bit;
			uint8 Byte;
		}BBVC_POSN1;	
			
		union
		{
			struct
			{
				uint8 CELL1			:1;
				uint8 CELL2			:1;
				uint8 CELL3			:1;
				uint8 CELL4			:1;
				uint8 CELL5			:1;
				uint8 CELL6 			:1;
				uint8 CELL7			:1;
				uint8 CELL8			:1;
			}Bit;
			uint8 Byte;
		}BBVC_POSN2;	
		
		union
		{
			struct
			{
				uint8 SLP_TIME			:3; //A timeout in SLEEP mode.
				uint8 TWARN_THR		:2; //Sets the TWARN threshold  T warning
				uint8 SPARE			:3;
			}Bit;
			uint8 Byte;
		}PWR_TRANSIT_CONF;	
		
		union
		{
			struct
			{
				uint8 CTL_TIME			:3; //Sets the short communication timeout
				uint8 CTL_ACT			:1; //Configures the device action when long communication timeout timer expires.
				uint8 CTS_TIME			:3; //Sets the long communication timeout.
				uint8 SPARE			:1; //
			}Bit;
			uint8 Byte;
		}COMM_TIMEOUT_CONF;
		
		uint8 TX_HOLD_OFF;	//between receive stop bit to tx
		
		union
		{
			struct
			{
				uint8 DELAY			:6; //Add additional byte delay gap in daisy chain data response frame
				uint8 SPARE			:2; //
			}Bit;
			uint8 Byte;
		}STACK_RESPONSE;
			
		union
		{
			struct
			{
				uint8 LOC				:5; //Indicate the BBP pin location
				uint8 SPARE			:3; //
			}Bit;
			uint8 Byte;
		}BBP_LOC;
		
		union
		{
			struct
			{
				uint8 TOP_STACK		:1; //Defines device as highest addressed device in the stack
				uint8 STACK_DEV		:1; //Defines device as a base or stack device in daisy chain configuration.
				uint8 SPARE			:6; //
			}Bit;
			uint8 Byte;
		}COMM_CTRL;
		
		union
		{
			struct
			{
				uint8 ADDR_WR			:1; //Enables device to start auto-addressing.
				uint8 SOFT_RESET		:1; //Resets the digital to OTP default. Bit is cleared on read.
				uint8 GOTO_SLEEP		:1; //Transitions device to SLEEP mode. Bit is cleared on read.
				uint8 GOTO_SHUTDOWN	:1; //Transitions device to SHUTDOWN mode. Bit is cleared on read
				uint8 SEND_SLPTOACT	:1; //Sends SLEEPtoACTIVE tone up the stack. Bit is cleared on read.
				uint8 SEND_WAKE		:1; //Sends WAKE tone to next device up the stack. Bit is cleared on read.
				uint8 SEND_SHUTDOWN	:1;	//Sends SHUTDOWN tone to next device up the stack.
				uint8 DIR_SEL			:1; //Selects daisy chain communication direction.
			}Bit;
			uint8 Byte;
		} CONTROL1;
		
		union
		{
			struct
			{
				uint8 TSREF_EN			:1; //Enables TSREF LDO output. Used to bias NTC thermistor.
				uint8 SEND_HW_RESET	:1; //Sends HW_RESET tone up the stack. Bit is cleared on read.
				uint8 SPARE			:6; //
			}Bit;
			uint8 Byte;
		} CONTROL2;
			
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}CUST_CRC;	// host-calculated CRC for customer OTP space.
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}CUST_CRC_RSLT;	//the device-calculated CRC for customer OTP space	
		
		union
		{
			struct
			{
				uint8 DRDY_BIST_PWR		:1; //Indicates the status of the power supplies diagnostic
				uint8 DRDY_BIST_OVUV		:1; //Indicates the status of the OVUV protector diagnostic.
				uint8 DRDY_BIST_OTUT		:1; //Indicates the status of the OTUT protector diagnostic
				uint8 DRDY_OVUV			:1; //Indicates the OVUV round robin has at least run once
				uint8 DRDY_OTUT			:1; //Indicates the OTUT round robin has run at least once.
				uint8 SPARE 				:3;
			}Bit;
			uint8 Byte;
		}DIAG_STAT;

		union
		{
			struct
			{
				uint8 DRDY_MAIN_ADC		:1; //Device has completed at least a single measurement on all Main ADC
				uint8 DRDY_AUX_MISC		:1; //Device has completed at least a single measurement on all AUX ADC MISC
				uint8 DRDY_AUX_CELL		:1; //Device has completed at least a single measurement on all AUXCELL
				uint8 DRDY_AUX_GPIO		:1; //AUX ADC has completed at least a single measurement on all active GPIO
				uint8 SPARE 				:4;
			}Bit;
			uint8 Byte;
		}ADC_STAT1;
		
		union
		{
			struct
			{
				uint8 DRDY_VCCB			:1; //Device has finished VCELL vs. AUXCELL diagnostic comparison
				uint8 DRDY_CBFET			:1; //Device has finished CB FET diagnostic comparison
				uint8 DRDY_CBOW			:1; //Device has finished CB OW diagnostic comparison
				uint8 DRDY_VCOW			:1; //Device has finished VC OW diagnostic comparison
				uint8 DRDY_GPIO 			:1; //Device has finished the GPIO Main and AUX ADC diagnostic comparisons
				uint8 DRDY_LPF				:1; //Device has finished at least 1 round of LPF checks
				uint8 SPARE 				:2;
			}Bit;
			uint8 Byte;
		}ADC_STAT2;
		
		union
		{
			struct
			{
				uint8 GPIO1				:1; //When GPIO is configured as digital input or output, this register shows the GPIO status
				uint8 GPIO2				:1;
				uint8 GPIO3				:1;
				uint8 GPIO4				:1;
				uint8 GPIO5 				:1;
				uint8 GPIO6				:1;
				uint8 GPIO7 				:1;
				uint8 GPIO8 				:1;
			}Bit;
			uint8 Byte;
		}GPIO_STAT;
		
		union
		{
			struct
			{
				uint8 CB_DONE				:1; //Indicates all cell balancing is completed
				uint8 MB_DONE				:1; //Indicates module balancing is completed
				uint8 ABORTFLT				:1; //Indicates cell balancing is aborted due to detection of unmasked fault.
				uint8 CB_RUN				:1; //Indicates cell balancing is running
				uint8 MB_RUN 				:1; //Indicates module balancing, controlled by the device, is running
				uint8 CB_INPAUSE			:1; //Indicates the cell balancing pause status
				uint8 OT_PAUSE_DET			:1; //Indicates the OTCB is detected if [OTCB_EN] = 1
				uint8 INVALID_CBCONF		:1; //Indicates CB is unable to start (after [BAL_GO] = 1) due to improper CB control settings.
			}Bit;
			uint8 Byte;
		}BAL_STAT;
		
		union
		{
			struct
			{
				uint8 MAIN_RUN				:1; //Shows the status of the Main ADC.
				uint8 AUX_RUN				:1; //Shows the status of the AUX ADC
				uint8 RSVD					:1;
				uint8 OVUV_RUN				:1; //Shows the status of the OVUV protector comparators
				uint8 OTUT_RUN				:1; //Shows the status of the OTUT protector comparators.
				uint8 CUST_CRC_DONE		:1; //Indicates module balancing, controlled by the device, is running
				uint8 FACT_CRC_DONE		:1; //Indicates the status of the factory CRC state machine.
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}DEV_STAT;
		
		union
		{
			struct
			{
				uint8 LPF_VCELL			:3; //Configures the post ADC low-pass filter cut-off frequency for VCELL measurement
				uint8 LPF_BB				:3; //Configures the post ADC low-pass filter cut-off frequency for bus bar measurement
				uint8 AUX_SETTLE			:2; //The AUXCELL configures the AUX CELL settling time
			}Bit;
			uint8 Byte;
		}ADC_CONF1;
		
		union
		{
			struct
			{
				uint8 ADC_DLY  			:6; //If [MAIN_GO] bit is written to 1, bit Main ADC is delayed for this setting time before being enabled to start the conversion.
				uint8 SPARE				:2;
			}Bit;
			uint8 Byte;
		}ADC_CONF2;

		uint8  MAIN_ADC_CAL1;		//Main ADC 25C gain calibration result
		union
		{
			struct
			{
				uint8 OFFSET	  			:7; //Main ADC 25Â°C offset calibration result.
				uint8 GAINH				:1; //Main ADC 25Â°C gain calibration result (MS bit)
			}Bit;
			uint8 Byte;
		}MAIN_ADC_CAL2;			//Main ADC 25C offset calibration result.
		uint8  AUX_ADC_CAL1;		//8-bit register for AUX ADC gain correction.
		union
		{
			struct
			{
				uint8 OFFSET	  			:7; //AUX ADC 25Â°C offset calibration result.
				uint8 GAINH				:1; //AUX ADC 25Â°C gain calibration result (MS bit)
			}Bit;
			uint8 Byte;
		}AUX_ADC_CAL2;		//8-bit register for AUX ADC offset correction
		
		union
		{
			struct
			{
				uint8 MAIN_MODE			:2; //Sets the Main ADC run mode.
				uint8 MAIN_GO				:1; //Starts main ADC conversion.
				uint8 LPF_VCELL_EN			:1; //Enables digital low-pass filter post-ADC conversion
				uint8 LPF_BB_EN			:1; //Enables digital low-pass filter post-ADC conversion
				uint8 SPARE				:3;
			}Bit;
			uint8 Byte;
		}ADC_CTRL1;
		
		union
		{
			struct
			{
				uint8 AUX_CELL_SEL			:5; //Selects which AUXCELL channel(s) will be multiplexed through the AUX ADC.
				uint8 AUX_CELL_ALIGN		:2; //Align the AUX ADC AUXCELL measurement to Main ADC CELL1 or CELL8
				uint8 SPARE				:3;
			}Bit;
			uint8 Byte;
		}ADC_CTRL2;
		
		union
		{
			struct
			{
				uint8 AUX_MODE				:2; //Sets the Main ADC run mode
				uint8 AUX_GO				:1; //Starts AUX ADC conversion
				uint8 AUX_GPIO_SEL			:4; //Selects which GPIO channel(s) will be multiplexed
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}ADC_CTRL3;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}VCELL[16];	
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}BUSBAR;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}TSREF;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}GPIO[8];
			
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}DIETEMP[2];	
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_CELL;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_GPIO;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_BAT;	
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_REFL;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_VBG2;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_VREF4P2;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_AVAO_REF;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_AVDD_REF;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_OV_DAC;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_UV_DAC;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_OT_OTCB_DAC;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_UT_DAC;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_VCBDONE_DAC;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_VCM[2];
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}VREF4P2;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}REFH;
			
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}DIAG_MAIN;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}DIAG_AUX;
		
		union
		{
			struct
			{
				uint8 TIME					:5; //Sets the timer for cell* balancing
				uint8 SPARE				:3;
			}Bit;
			uint8 Byte;
		}CB_CELL_CTRL[16];
		
		union
		{
			struct
			{
				uint8 MB_THR				:6; //If MB_TIMER_CTRL is not 0x00 and BAT voltage is less than this threshold, the module balancing through GPIO3 stops
				uint8 SPARE				:2;
			}Bit;
			uint8 Byte;
		}VMB_DONE_THRESH;
		
		union
		{
			struct
			{
				uint8 TIME					:5; //If MB_TIMER_CTRL is not 0x00 and BAT voltage is less than this threshold, the module balancing through GPIO3 stops
				uint8 SPARE				:3;
			}Bit;
			uint8 Byte;
		}MB_TIMER_CTRL;
		
		union
		{
			struct
			{
				uint8 TIME					:5; //If a cell voltage is less than this threshold, the cell balancing on that cell stops.
				uint8 SPARE				:3;
			}Bit;
			uint8 Byte;
		}VCB_DONE_THRESH;
		
		union
		{
			struct
			{
				uint8 OTCB_THR				:4; //Sets the OTCB threshold when BAL_CTRL1[OTCB_EN] = 1
				uint8 COOLOFF				:3; //Sets the COOLOFF hysteresis
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}OTCB_THRESH;
		
		union
		{
			struct
			{
				uint8 DUTY					:3; //Selection is sampled whenever [BAL_GO] = 1 is set by the host MCU.
				uint8 SPARE				:5;
			}Bit;
			uint8 Byte;
		}BAL_CTRL1;
		
		union
		{
			struct
			{
				uint8 AUTO_BAL				:1; //Selects between auto or manual cell balance control.
				uint8 BAL_GO				:1; //Starts cell or module balancing.
				uint8 BAL_ACT				:2; //Controls the device action when the MB and CB are completed
				uint8 OTCB_EN				:1; //Enables the OTCB detection during cell balancing
				uint8 FLTSTOP_EN			:1; //Stops cell or module balancing if unmasked fault occurs
				uint8 CB_PAUSE				:1; //Pauses cell balancing on all cells to allow diagnostics to run
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}BAL_CTRL2;
		
		union
		{
			struct
			{
				uint8 BAL_TIME_GO			:1; //Select a single CB channel to report its remaining balancing time
				uint8 BAL_TIME_SEL			:4; //Starts cell or module balancing.
				uint8 SPARE				:3;
			}Bit;
			uint8 Byte;
		}BAL_CTRL3;
		
		uint8 CB_COMPLETE1;	//Cell balance completion for cell9 to cell16.
		uint8 CB_COMPLETE2;  //Cell balance completion for cell1 to cell8.
		
		union
		{
			struct
			{
				uint8 OV_THR				:6; //Sets the overvoltage threshold for the OV comparator
				uint8 SPARE				:2;
			}Bit;
			uint8 Byte;
		}OV_THRESH;			//alone function
		
		union
		{
			struct
			{
				uint8 UV_THR				:6; //Sets the undervoltage threshold for the UV comparator.
				uint8 SPARE				:2;
			}Bit;
			uint8 Byte;
		}UV_THRESH;		//alone function
		
		union
		{
			struct
			{
				uint8 OT_THR				:5; //Sets the OT threshold for the OT comparator.
				uint8 UT_THR				:3; //Sets the UT threshold for the UT comparator
			}Bit;
			uint8 Byte;
		}OTUT_THRESH;
		
		union
		{
			struct
			{
				uint8 OVUV_MODE			:2; //Sets the OV and UV comparators operation mode when [OVUV_GO] = 1
				uint8 OVUV_GO				:1; //Starts the OV and UV comparators.
				uint8 OVUV_LOCK			:4; //Configures a particular single channel as the OV and UV comparators input when [OVUV_MOD1:0] = 0b11
				uint8 VCBDONE_THR_LOCK		:1;	//As the UV comparator is switching between UV threshold and VCBDONE threshold to measure the UV DAC or the VCBDONE DAC result for diagnostics
			}Bit;
			uint8 Byte;
		}OVUV_CTRL;
		
		union
		{
			struct
			{
				uint8 OTUT_MODE			:2; //Sets the OT and UT comparators operation mode when [OTUT_GO] = 1
				uint8 OTUT_GO				:1; //Starts the OT and UT comparators
				uint8 OTUT_LOCK			:3; //Configures a particular single channel as the OT and UT comparators input when [OTUT_MOD1:0] = 0b11.
				uint8 OTCB_THR_LOCK		:1; //As the OT comparator is switching between OT threshold and OTCB threshold to measure the OT or OTCB
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}OTUT_CTRL;
		
		union
		{
			struct
			{
				uint8 GPIO1				:3; //Configures GPIO1.
				uint8 GPIO2				:3; //Configures GPIO2.
				uint8 SPI_EN				:1; //Enables SPI master on GPIO4, GPIO5 and GPIO6, GPIO7
				uint8 FAULT_IN_EN			:1; //Enables GPIO8 as an active-low input to trigger the NFAULT pin when the input signal is low.
			}Bit;
			uint8 Byte;
		}GPIO_CONF1;
		
		union
		{
			struct
			{
				uint8 GPIO3				:3; //Configures GPIO3.
				uint8 GPIO4				:3; //Configures GPIO4.
				uint8 RSVD					:1;
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}GPIO_CONF2;
		
		union
		{
			struct
			{
				uint8 GPIO5				:3; //Configures GPIO5.
				uint8 GPIO6				:3; //Configures GPIO6.
				uint8 RSVD					:1;
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}GPIO_CONF3;
		
		union
		{
			struct
			{
				uint8 GPIO7				:3; //Configures GPIO7.
				uint8 GPIO8				:3; //Configures GPIO8.
				uint8 RSVD					:1;
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}GPIO_CONF4;
			
		union
		{
			struct
			{
				uint8 NUMBIT				:5; //SPI transaction length.
				uint8 CPHA					:1; //Sets the edge of SCLK where data is sampled on MISO.
				uint8 CPOL					:1; //Sets the SCLK polarity.
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}SPI_CONF;
		
		union
		{
			struct
			{
				uint8 SPI_GO				:1; //Executes the SPI transaction. T
				uint8 SS_CTRL				:1; //Programs the state of SS
				uint8 SPARE				:6;
			}Bit;
			uint8 Byte;
		}SPI_EXE;
		
		uint8 SPI_TX[3];	//Data to be used to write to SPI slave device.
		uint8 SPI_RX[3];	//Data returned from a read during SPI transaction.
		
		union
		{
			struct
			{
				uint8 MARGIN_GO			:1; //Starts OTP Margin test set by the [MARGIN_MOD] bit.
				uint8 MARGIN_MODE			:3; //Configures OTP Margin read mode:
				uint8 FLIP_FACT_CRC		:1; //An enable bit to flip the factory CRC value.
				uint8 SPARE				:4;
			}Bit;
			uint8 Byte;
		}DIAG_OTP_CTRL;
		
		union
		{
			struct
			{
				uint8 FLIP_TR_CRC			:1; //Sends a purposely incorrect communication (during transmitting response) CRC by inverting all of the calculated CRC bits..
				uint8 SPI_LOOPBACK			:1; //Enables SPI loopback function to verify SPI functionality
				uint8 SPARE				:6;
			}Bit;
			uint8 Byte;
		}DIAG_COMM_CTRL;
		
		union
		{
			struct
			{
				uint8 PWR_BIST_GO			:1; //When written to 1, the power supply BIST diagnostic will start.
				uint8 BIST_NO_RST			:1; //Use for further diagnostic if the power supply BIST detects a failure
				uint8 SPARE				:6;
			}Bit;
			uint8 Byte;
		}DIAG_PWR_CTRL;
		
		uint8 DIAG_CBFET_CTRL1;	//Enables CBFET for CBFET diagnostic CELL16-9
		uint8 DIAG_CBFET_CTRL2;	//Enables CBFET for CBFET diagnostic CELL8-1
		
		union
		{
			struct
			{
				uint8 BB_THR			:3; //Additional delta value added to the VCCB_THR setting,
				uint8 VCCB_THR			:5; //Configures the VCELL vs. AUXCELL delta. The VCELL vs. AUXCELL check is considered pass
			}Bit;
			uint8 Byte;
		}DIAG_COMP_CTRL1;
		
		union
		{
			struct
			{
				uint8 OW_THR			:4; //Configures the OW detection threshold for diagnostic comparison.
				uint8 GPIO_THR			:3; //Configures the GPIO comparison delta threshold between Main and AUX ADC measurements
				uint8 SPARE			:1;
			}Bit;
			uint8 Byte;
		}DIAG_COMP_CTRL2;
		
		union
		{
			struct
			{
				uint8 COMP_ADC_GO		:1; //Device starts diagnostic test specified by [COMP_ADC_SEL2:0] setting.
				uint8 COMP_ADC_SEL		:3; //Enables the device diagnostic comparison
				uint8 OW_SNK			:2; //Turns on current sink on VC pins, CB pins, or BBP/N pins.
				uint8 EXTD_CBFET   	:1; //When CBFET check comparison is selected, [COMP_ADC_SEL2:0] = 0b101, device turns off all CBFET upon
																	//the completion of the comparison. If this bit is set to 1, the device will put the CBFET check in an extended mode
																	//in which the device will NOT turn off the CBFETs.
				uint8 SPARE			:1;
			}Bit;
			uint8 Byte;
		}DIAG_COMP_CTRL3;
		
		union
		{
			struct
			{
				uint8 LPF_FAULT_INJ	:1; //Injects fault condition to the diagnostic LPF during LPF diagnostic
				uint8 COMP_FAULT_INJ	:1; //Injects fault to the ADC comparison logic.
				uint8 SPARE			:6;
			}Bit;
			uint8 Byte;
		}DIAG_COMP_CTRL4;
		
		union
		{
			struct
			{
				uint8 PROT_BIST_NO_RST		:1; //Use for further diagnostic if the protector BIST detects a failure
				uint8 SPARE				:7;
			}Bit;
			uint8 Byte;
		}DIAG_PROT_CTRL;
		
		union
		{
			struct
			{
				uint8 MSK_PWR				:1; //To mask the NFAULT assertion from any FAULT_PWR1 to FAULT_PWR3 register bit
				uint8 MSK_SYS				:1;	//To mask the NFAULT assertion from any FAULT_SYS register bit.
				uint8 MSK_COMP				:1; //Masks the FAULT_COMP_* registers to trigger NFAULT
				uint8 MSK_OV				:1; //Masks the FAULT_OV* registers to trigger NFAULT
				uint8 MSK_UV				:1; //Masks the FAULT_UV* registers to trigger NFAULT
				uint8 MSK_OT				:1; //Masks the FAULT_OT* registers to trigger NFAULT.
				uint8 MSK_UT				:1; //Masks the FAULT_UT* registers to trigger NFAULT.
				uint8 MSK_PROT				:1; //Masks the FAULT_PROT* registers to trigger NFAULT
			}Bit;
			uint8 Byte;
		}FAULT_MSK1;
		
		union
		{
			struct
			{
				uint8 MSK_COMM1			:1; //Masks FAULT_COMM1 register on NFAULT triggering.
				uint8 MSK_COMM2			:1;	//Masks FAULT_COMM2 register on NFAULT triggering
				uint8 MSK_COMM3_HB			:1; //Masks FAULT_COMM3[HB_FAST] or [HB_FAIL] faults on NFAULT triggering.
				uint8 MSK_COMM3_FTONE		:1; //Masks FAULT_COMM3[FTONE_DET] fault on NFAULT triggering.
				uint8 MSK_COMM3_FCOMM		:1; //Masks FAULT_COMM3[FCOMM_DET] fault on NFAULT triggering
				uint8 MSK_OTP_DATA			:1; //Masks the FAULT_OTP register (all bits except [CUST_CRC] and [FACT_CRC]) on NFAULT triggering
				uint8 MSK_OTP_CRC			:1; //Masks the FAULT_OTP register ([CUST_CRC] and [FACT_CRC] only) on NFAULT triggering.
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}FAULT_MSK2;
		
		union
		{
			struct
			{
				uint8 RST_PWR				:1; //To reset the FAULT_PWR1 to FAULT_PWR3 registers to 0x00
				uint8 RST_SYS				:1;	//To reset the FAULT_SYS register to 0x00
				uint8 RST_COMP				:1; //Resets all FAULT_COMP_* registers to 0x00.
				uint8 RST_OV				:1; //Resets all FAULT_OV* registers to 0x00
				uint8 RST_UV				:1; //Resets all FAULT_UV* registers to 0x00
				uint8 RST_OT				:1; //Resets all FAULT_OT registers to 0x00
				uint8 RST_UT				:1; //Resets all FAULT_UT registers to 0x00
				uint8 RST_PROT				:1; //Resets the FAULT_PROT1 and FAULT_PROT2 registers to 0x00.
			}Bit;
			uint8 Byte;
		}FAULT_RST1;
		
		union
		{
			struct
			{
				uint8 RST_COMM1			:1; //Resets FAULT_COMM1 and DEBUG_COMM_UART* registers.
				uint8 RST_COMM2			:1;	//Resets FAULT_COMM2, DEBUG_COML*, and DEBUG_COMM_COMH* registers
				uint8 RST_COMM3_HB			:1; //Resets FAULT_COMM3[HB_FAST] and [HB_FAIL] bits.
				uint8 RST_COMM3_FTONE		:1; //Resets FAULT_COMM3[FTONE_DET].
				uint8 RST_COMM3_FCOMM		:1; //Resets FAULT_COMM3[FCOMM_DET]
				uint8 RST_OTP_DATA			:1; //Resets the FAULT_OTP register ([SEC_DETECT] and [DED_DETECT] only).
				uint8 RST_OTP_CRC			:1; //Resets the FAULT_OTP register ([CUST_CRC] and [FACT_CRC] only).
				uint8 RSVD					:1;
			}Bit;
			uint8 Byte;
		}FAULT_RST2;
		
		union
		{
			struct
			{
				uint8 FAULT_PWR			:1; //This bit is set if [MSK_PWR] = 0 and any of the FAULT_PWR1 to FAULT_PWR3 register bits is set.
				uint8 FAULT_SYS			:1;	//This bit is set if [MSK_SYS] = 0 and any of the FAULT_SYS1 register bits is set
				uint8 FAULT_OVUV			:1; //This bit is set if any of the following is true:
				uint8 FAULT_OTUT			:1; //
				uint8 FAULT_COMM			:1; //
				uint8 FAULT_OTP			:1; //This bit is set if [MSK_OTP] = 0 and any of the FAULT_OTP register bits is set.
				uint8 FAULT_COMP_ADC		:1; //Resets the FAULT_OTP register ([CUST_CRC] and [FACT_CRC] only).
				uint8 FAULT_PROT			:1; //This bit is set if [MSK_PROT] = 0 and any of the FAULT_PROT1 or FAULT_PROT2 register bits is set.
			}Bit;
			uint8 Byte;
		}FAULT_SUMMARY;
		
		union
		{
			struct
			{
				uint8 STOP_DET				:1; //Indicates an unexpected STOP condition is received.
				uint8 COMMCLR_DET			:1;	//A UART communication clear signal is detected
				uint8 UART_RC				:1; //Indicates a UART FAULT is detected during receiving a command frame
				uint8 UART_RR				:1; //Indicates a UART FAULT is detected when receiving a response frame
				uint8 UART_TR				:1; //Indicates a UART FAULT is detected when transmitting a response frame.
				uint8 RSVD					:3;
			}Bit;
			uint8 Byte;
		}FAULT_COMM1;
		
		union
		{
			struct
			{
				uint8 COMH_BIT				:1; //Indicates a COMH bit level fault is detected
				uint8 COMH_RC				:1;	//Indicates a COMH byte level fault is detected
				uint8 COMH_RR				:1; //Indicates a COMH byte level fault is detected when receiving a response frame
				uint8 COMH_TR				:1; //Indicates a COMH byte level fault is detected when transmitting a response frame
				uint8 COML_BIT				:1; //Indicates a COML bit level fault is detected which would cause at least one byte level fault.
				uint8 COML_RC				:1; //Indicates a COML byte level fault is detected when receiving a command frame
				uint8 COML_RR				:1; //Indicates a COML byte level fault is detected when receiving a response frame
				uint8 COML_TR				:1; //Indicates a COML byte level fault is detected when transmitting a response frame
			}Bit;
			uint8 Byte;
		}FAULT_COMM2;
		
		union
		{
			struct
			{
				uint8 HB_FAST				:1; //Indicates HEARTBEAT is received too frequently
				uint8 HB_FAIL				:1;	//Indicates HEARTBEAT is not received within an expected time
				uint8 FTONE_DET			:1; //Indicates a FAULT TONE is received
				uint8 FCOMM_DET			:1; //Received communication transaction with the Fault Status bits set by any of the upper stack device
				uint8 RSVD					:4;
			}Bit;
			uint8 Byte;
		}FAULT_COMM3;
		
		union
		{
			struct
			{
				uint8 GBLOVERR				:1; //Indicates that on overvoltage error is detected on one of the OTP pages.
				uint8 FACTLDERR			:1; //Indicates errors during the factory space OTP load process.
				uint8 CUSTLDERR			:1;	//Indicates errors during the customer space OTP load process
				uint8 FACT_CRC				:1; //Indicates a CRC error has occurred in the factory register space.
				uint8 CUST_CRC				:1; //Indicates a CRC error has occurred in the customer register space
				uint8 SEC_DET				:1; //Indicates a SEC error has occurred during the OTP load.
				uint8 DED_DET				:1; //Indicates a DED error has occurred during the OTP load
				uint8 RSVD					:1;
			}Bit;
			uint8 Byte;
		}FAULT_OTP;
		
		union
		{
			struct
			{
				uint8 TWARN				:1; //Indicates the die temperature (die temp 2) is higher than the TWARN_THR[1:0] setting
				uint8 TSHUT				:1; //Indicates the previous shutdown was a thermal shutdown,
				uint8 CTS					:1;	//Indicates a short communication timeout occurred.
				uint8 CTL					:1; //Indicates a long communication timeout occurred
				uint8 DRST					:1; //Indicates a digital reset has occurred
				uint8 GPIO					:1; //Indicates GPIO8 detects a FAULT input when GPIO_CONF1[FAULT_IN_EN] = 1.
				uint8 SHUTDOWN_REC			:1; //Indicates the previous device shutdown was caused by one of the following:
				uint8 LFO					:1; //Indicated LFO frequency is outside an expected range
			}Bit;
			uint8 Byte;
		}FAULT_SYS;
		
		union
		{
			struct
			{
				uint8 VPARITY_FAIL			:1; //Indicates a parity fault is detected on any of the following OVUV related configurations
				uint8 TPARITY_FAIL			:1; //Indicates a parity fault is detected on any of the following OTUT related configurations
				uint8 RSVD					:6;
			}Bit;
			uint8 Byte;
		}FAULT_PROT1;
		
		union
		{
			struct
			{
				uint8 UVCOMP_FAIL			:1; //Indicates the UV comparator fails during BIST test
				uint8 OVCOMP_FAIL			:1; //Indicates the OV comparator fails during BIST test.
				uint8 OTCOMP_FAIL			:1;	//Indicates the OT comparator fails during BIST test
				uint8 UTCOMP_FAIL			:1; //Indicates the UT comparator fails during BIST test.
				uint8 VPATH_FAIL			:1; //Indicates a fault is detected along the OVUV signal path during BIST test
				uint8 TPATH_FAIL			:1; //Indicates a fault is detected along the OTUT signal path during BIST test
				uint8 BIST_ABORT			:1; //Indicates either OVUV or OTUT BIST run is aborted
				uint8 RSVD					:1;
			}Bit;
			uint8 Byte;
		}FAULT_PROT2;
		
		uint8 FAULT_OV1;					//OV9_DET to OV16_DET = OV fault status
		uint8 FAULT_OV2;					//OV1_DET to OV8_DET = OV fault status
		uint8 FAULT_UV1;					//UV9_DET to UV16_DET = UV fault status
		uint8 FAULT_UV2;					//UV1_DET to UV8_DET = UV fault status
		uint8 FAULT_OT;					//OT1_DET to OT8_DET = OT fault status
		uint8 FAULT_UT;					//UT1_DET to UT8_DET = UT fault status
		uint8 FAULT_COMP_GPIO;				//Indicates ADC vs. AUX ADC GPIO measurement diagnostic results for GPIO1 to GPIO8.
		uint8 FAULT_COMP_VCCB1;			//Indicates voltage diagnostic results for cell9 to cell16.
		uint8 FAULT_COMP_VCCB2;			//Indicates voltage diagnostic results for cell1 to cell8.
		uint8 FAULT_COMP_VCOW1;			//Indicates VC OW diagnostic results for cell9 to cell 16.
		uint8 FAULT_COMP_VCOW2;			//Indicates VC OW diagnostic results for cell1 to cell 8
		uint8 FAULT_COMP_CBOW1;			//Results of the CB OW diagnostic for CB FET9 to CB FET16
		uint8 FAULT_COMP_CBOW2;			//Results of the CB OW diagnostic for CB FET1 to CB FET8.
		uint8 FAULT_COMP_CBFET1;			//Results of the CB FET diagnostic for CB FET9 to CB FET16
		uint8 FAULT_COMP_CBFET2;			//Results of the CB FET diagnostic for CB FET1 to CB FET8.
				
		union
		{
			struct
			{
				uint8 LPF_FAIL				:1; //Indicates LPF diagnostic result
				uint8 COMP_ADC_ABORT		:1; //Indicates the most recent ADC comparison diagnostic is aborted due to improper setting
				uint8 RSVD					:6;
			}Bit;
			uint8 Byte;
		}FAULT_COMP_MISC;	
		
		union
		{
			struct
			{
				uint8 AVDD_OV				:1; //Indicates an overvoltage fault on the AVDD LDO.
				uint8 AVDD_OSC				:1; //Indicates AVDD is oscillating outside of acceptable limits.
				uint8 DVDD_OV				:1;	//Indicates an overvoltage fault on the DVDD LDO
				uint8 CVDD_OV				:1; //Indicates an overvoltage fault on the CVDD LDO
				uint8 CVDD_UV				:1; //Indicates an undervoltage fault on the CVDD LDO
				uint8 REFHM_OPEN			:1; //Indicates an open condition on REFHM pin
				uint8 DVSS_OPEN			:1; //Indicates an open condition on DVSS pin
				uint8 CVSS_OPEN			:1; //Indicates an open condition on CVSS pin
			}Bit;
			uint8 Byte;
		}FAULT_PWR1;
		
		union
		{
			struct
			{
				uint8 TSREF_OV				:1; //Indicates an overvoltage fault on the TSREF LDO.
				uint8 TSREF_UV				:1; //Indicates an undervoltage fault on the TSREF LDO.
				uint8 TSREF_OSC			:1;	//Indicates TSREF is oscillating outside of an acceptable limit
				uint8 NEG5V_UV				:1; //Indicates an undervoltage fault on the NEG5V charge pump
				uint8 REFH_OSC				:1; //Indicates REGH reference is oscillating outside of an acceptable limit.
				uint8 AVAO_OV				:1; //Indicates an overvoltage fault on the AVAO_REF
				uint8 PWRBIST_FAIL			:1; //Indicates a fail on the power supply BIST run
				uint8 RSVD					:1;
			}Bit;
			uint8 Byte;
		}FAULT_PWR2;
		
		union
		{
			struct
			{
				uint8 AVDDUV_DRST			:1; //Indicates a digital reset occurred due to AVDD UV detected.
				uint8 RSVD					:7;
			}Bit;
			uint8 Byte;
		}FAULT_PWR3;	
		
		uint8 DEBUG_CTRL_UNLOCK;		//Write the unlock code (0xA5) to this register to activate the setting in the DEBUG_COMM_CTRL* register
		
		union
		{
			struct
			{
				uint8 USER_DAISY_EN		:1; //This bit enables the debug COML and COMH control bits in the DEBUG_COMM_CTRL2 register
				uint8 USER_UART_EN 		:1; //This bit enables the debug UART control bits
				uint8 UART_TX_EN			:1;	//Stack device, by default, has the UART TX disabled
				uint8 UART_MIRROR_EN		:1; //This bit enables the stack VIF communication to mirror to UART
				uint8 UART_BAUD			:1; //This bit changes the UART baud rate to 250kb/s.
				uint8 RSVD					:3;
			}Bit;
			uint8 Byte;
		}DEBUG_COMM_CTRL1;
		
		union
		{
			struct
			{
				uint8 COMH_RX_EN			:1; //Enables COMH receiver.
				uint8 COMH_TX_EN		 	:1; //Enables COMH transmitter
				uint8 COML_RX_EN			:1;	//Enables COML receiver
				uint8 COML_TX_EN			:1; //Enables COML transmitter
				uint8 RSVD					:4;
			}Bit;
			uint8 Byte;
		}DEBUG_COMM_CTRL2;
			
		union
		{
			struct
			{
				uint8 COMH_RX_ON			:1; //Shows the current COMH receiver status.
				uint8 COMH_TX_ON		 	:1; //Shows the current COMH transmitter status.
				uint8 COML_RX_ON			:1;	//Shows the current COML receiver status
				uint8 COML_TX_ON			:1; //Shows the current COML transmitter status
				uint8 HW_DAISY_DRV			:1; //Indicates the COML and COMH are controlled by the device itself or by MCU control.
				uint8 HW_UART_DRV			:1;	//Indicates the UART TX is controlled by the device itself or by MCU control
				uint8 RSVD					:2;
			}Bit;
			uint8 Byte;
		}DEBUG_COMM_STAT;
		
		union
		{
			struct
			{
				uint8 RC_CRC				:1; //Detects a CRC error in the received command frame from UART
				uint8 RC_UNEXP			 	:1; //In a stack device, it is not expected to receive a stack or broadcast command through the UART interface.
				uint8 RC_BYTE_ERR			:1;	//Detects any byte error, other than the error in the initialization byte, in the received command frame
				uint8 RC_SOF				:1; //Detects a start-of-frame (SOF) error
				uint8 RC_TXDIS				:1; //Detects if UART TX is disabled, but the host MCU has issued a command to read data from the device.
				uint8 RC_IERR				:1;	//Detects initialization byte error in the received command frame
				uint8 RSVD					:2;
			}Bit;
			uint8 Byte;
		}DEBUG_UART_RC;	
		
		union
		{
			struct
			{
				uint8 RR_CRC				:1; //Detects are CRC error in the received response frame from UART
				uint8 RR_BYTE_ERR	 		:1; //Detects any byte error,
				uint8 RR_SOF				:1;	//Indicates a UART CLEAR is received while receiving the response frame
				uint8 TR_WAIT				:1; //The device is waiting for its turn to transfer a response out but the action is terminated
				uint8 TR_SOF				:1; //Indicates that a UART CLEAR is received while the device is still transmitting data
				uint8 RSVD					:3;
			}Bit;
			uint8 Byte;
		}DEBUG_UART_RR_TR;
		
		union
		{
			struct
			{
				uint8 BIT					:1; //The device has detected a data bit; however,
																	//the detection samples are not enough to assure a strong 1 or 0
				uint8 SYNC1				:1; //Unable to detect the preamble half-bit or any of the [SYNC1:0] bits
				uint8 SYNC2				:1;	//The Preamble half-bit and the [SYNC1:0] bits are detected
				uint8 BERR_TAG				:1; //Set when the received communication is tagged with [BERR] = 1.
				uint8 PERR					:1; //Detects abnormality of the incoming communication frame and hence
				uint8 RSVD					:3;
			}Bit;
			uint8 Byte;
		}DEBUG_COMH_BIT;
		
		union
		{
			struct
			{
				uint8 RC_CRC				:1; //Indicates a CRC error that resulted in one or more COMH command frames being discarded
				uint8 RC_UNEXP			 	:1; //If [DIR_SEL] = 0, but device receives command frame from COMH
																	//which is an invalid condition and device will set this error bit.
				uint8 RC_BYTE_ERR			:1;	//Valid when [DIR_SEL] = 1. Detected any byte error,
																	//other than the error in the initialization byte, in the received command frame
				uint8 RC_SOF				:1; //Valid when [DIR_SEL] = 1. Detects a start-of-frame (SOF) error on COMH. T
				uint8 RC_TXDIS				:1; //Valid when [DIR_SEL] = 1. Device detects the COMH TX is disabled but the device receives a command to read
																	//data (that is, to transmit data out). The command frame will be counted as a discard frame
				uint8 RC_IERR				:1; //...
				uint8 RSVD					:2;
			}Bit;
			uint8 Byte;
		}DEBUG_COMH_RC;	
		
		union
		{
			struct
			{
				uint8 RR_CRC				:1; //Indicates a CRC error that resulted in one or more COMH response frames being discarded
				uint8 RR_UNEXP			 	:1; //If [DIR_SEL] = 1, but device received response frame from COMH
																	//which is an invalid condition and device sets this error bit.
				uint8 RR_BYTE_ERR			:1;	//Valid when [DIR_SEL] = 0. Detects any byte error, other than the error in the initialization byte, in the received
																	//response frame. This error can trigger one or more error bits set in the DEBUG_COMMH_BIT register.
				uint8 RR_SOF				:1; //Valid when [DIR_SEL] = 0. Detects a start-of-frame (SOF) error on COMH
				uint8 RR_TXDIS				:1; //Valid when [DIR_SEL] = 0, device receives a response but fails to transmit to the next device because the COMH
																	//TX is disabled. The frame is counted as a discarded frame.
				uint8 TR_WAIT				:1; //The device is waiting for its turn to transfer a response out
																	//but the action is terminated because the devicereceives a new command.
				uint8 RSVD					:2;
			}Bit;
			uint8 Byte;
		}DEBUG_COMH_RR_TR;	
		
		union
		{
			struct
			{
				uint8 BIT					:1; //The device has detected a data bit.
				uint8 SYNC1				:1; //Unable to detect the preamble half-bit or any of the [SYNC1:0] bits.
				uint8 SYNC2				:1;	//The Preamble half-bit and the [SYNC1:0] bits are detected
				uint8 BERR_TAG				:1; //Set when the received communication is tagged with BERR
				uint8 PERR					:1; //Detect abnormality of the incoming communication frame
																	//and the outgoing communication frame will be set with	BERR.
				uint8 RSVD					:3;
			}Bit;
			uint8 Byte;
		}DEBUG_COML_BIT;	
		
		union
		{
			struct
			{
				uint8 RC_CRC				:1; //Indicates a CRC error that resulted in one or more COML command frames being discarded.
				uint8 RC_UNEXP			 	:1; //If [DIR_SEL] = 1, but device received command frame from COML which is an invalid condition
																	//and device will set this error bit.
				uint8 RC_BYTE_ERR			:1;	//Valid when [DIR_SEL] = 0. Detected any byte error,
																	//other than the error in the initialization byte, in the received command frame.
				uint8 RC_SOF				:1; //Valid when [DIR_SEL] = 0.
				uint8 RC_TXDIS				:1; //Valid when [DIR_SEL] = 0.
				uint8 RC_IERR				:1; //Detected initialization byte error in the received command frame.
				uint8 RSVD					:2;
			}Bit;
			uint8 Byte;
		}DEBUG_COML_RC;
		
		union
		{
			struct
			{
				uint8 RR_CRC				:1; //Indicates a CRC error that resulted in one or more COML response frames being discarded.
				uint8 RR_UNEXP			 	:1; //If [DIR_SEL] = 0, but device received a response frame from COML
																	//which is an invalid condition and device will set this error bit.
				uint8 RR_BYTE_ERR			:1;	//Valid when [DIR_SEL] = 1
				uint8 RR_SOF				:1; //Valid when [DIR_SEL] = 1.
				uint8 RR_TXDIS				:1; //Valid when [DIR_SEL] = 1
				uint8 TR_WAIT				:1; //The device is waiting for its turn to transfer a response out
																	//but the action is terminated because the device receives a new command
				uint8 RSVD					:2;
			}Bit;
			uint8 Byte;
		}DEBUG_COML_RR_TR;
		
		uint8 DEBUG_UART_DISCARD; 		//UART frame counter to track the number of discard frames received or transmitted.
		uint8 DEBUG_COMH_DISCARD;		//COMH frame counter to track the number of discard frames received or transmitted.
		uint8 DEBUG_COML_DISCARD;		//COML frame counter to track the number of discard frames received or transmitted.
		uint8 DEBUG_UART_VALID_HI;		//The high-byte of UART frame counter to track the number of valid frames received or transmitted.
		uint8 DEBUG_UART_VALID_LO;		//The low-byte of UART frame counter to track the number of valid frames received or transmitted.
		uint8 DEBUG_COMH_VALID_HI;		//The high-byte of COMH frame counter to track the number of valid frames received or transmitted.
		uint8 DEBUG_COMH_VALID_LO;		//The low-byte of COMH frame counter to track the number of valid frames received or transmitted.
		uint8 DEBUG_COML_VALID_HI;		//The high-byte of COML frame counter to track the number of valid frames received or transmitted.
		uint8 DEBUG_COML_VALID_LO;		//The low-byte of COML frame counter to track the number of valid frames received or transmitted
		uint8 DEBUG_OTP_SEC_BLK;		//Holds last OTP block address where SEC occurred. Valid only when FAULT_OTP[SEC_DET] = 1.
		uint8 OTP_PROG_UNLOCK1[4];		//programming unlock code is required as part of the OTP programming unlock sequence
									//before performing OTP programming
		uint8 OTP_PROG_UNLOCK2[4]; 	//OTP programming unlock code, required as part of the OTP programming unlock sequence
									//before performing OTP programming
		union
		{
			struct
			{
				uint8 PROG_GO			:1; //Enables programming for the OTP page selected by OTP_PROG_CTRL[PAGESEL].
				uint8 PAGESEL			:1; //Selects which customer OTP page to be programmed.
				uint8 RSVD				:6;
			}Bit;
			uint8 Byte;
		}OTP_PROG_CTRL;
		
		union
		{
			struct
			{
				uint8 ENABLE			:1; //Executes the OTP ECC test configured by [ENC_DEC] and [DED_SEC] bits
				uint8 ENC_DEC			:1; //Sets the encoder/decoder test to run when OTP_ECC_TEST[ENABLE] = 1.
				uint8 MANUAL_AUTO		:1;	//Sets the location of the data to use for the ECC test.
				uint8 DED_SEC			:1; //Sets the decoder function (SEC or DED) to test
				uint8 RSVD				:4;
			}Bit;
			uint8 Byte;
		}OTP_ECC_TEST;
		
		uint8 OTP_ECC_DATAIN[9];		//When ECC is enabled in manual mode, CUST_ECC_TEST[MANUAL_AUTO] = 1, OTP_ECC_DATAIN1?
														//registers are used to test the ECC encoder/decoder
		uint8 OTP_ECC_DATAOUT[9];		//OTP_ECC_DATAOUT* bytes output the results of the ECC decoder and encoder tests.
		
		union
		{
			struct
			{
				uint8 DONE				:1; //Indicates the status of the OTP programming for the selected page
				uint8 PROGERR			:1; //Indicates when an error is detected due to incorrect page setting caused by any of the following
				uint8 SOVERR			:1;	//A programming voltage stability test is performed before starting the actual OTP programming
				uint8 SUVERR			:1; //A programming voltage stability test is performed before starting the actual OTP programming
				uint8 OVERR			:1;	//Indicates an overvoltage error detected on the programming voltage during OTP programming
				uint8 UVERR			:1; //Indicates an undervoltage error detected on the programming voltage during OTP programming
				uint8 OTERR			:1;	//Indicates the die temperature is greater than TOTP_PROG and device does not start OTP programming.
				uint8 UNLOCK			:1; //Indicates the OTP programming function unlock status.
			}Bit;
			uint8 Byte;
		}OTP_PROG_STAT;
			
		union
		{
			struct
			{
				uint8 TRY				:1; //Indicates a first programming attempt for OTP page 1
				uint8 OVOK				:1; //Indicates an OTP programming voltage overvoltage condition is detected
																	//during programming attempt for OTP page 1
				uint8 UVOK				:1;	//Indicates an OTP programming voltage undervoltage condition is detected
																	//during programming attempt for OTP page 1
				uint8 PROGOK			:1; //Indicates the validity for loading for OTP page 1. A valid page indicates that successful programming occurred.
				uint8 FMTERR			:1;	//Indicates a formatting error in OTP page 1; that is, when [UVOK] or [OVOK] is set
				uint8 LOADERR			:1; //Indicates an error while attempting to load OTP page 1
				uint8 LOADWRN			:1;	//Indicates OTP page 1 was loaded but with one or more SEC warnings
				uint8 LOADED			:1; //Indicates OTP page 1 has been selected for loading into the related registers.
			}Bit;
			uint8 Byte;
		}OTP_CUST1_STAT;
		
		union
		{
			struct
			{
				uint8 TRY				:1; //Indicates a first programming attempt for OTP page 1.
				uint8 OVOK				:1; //Indicates an OTP programming voltage overvoltage condition is detected
																	//during programming attempt for OTP page 1
				uint8 UVOK				:1;	//Indicates an OTP programming voltage undervoltage condition is detected
																	//during programming attempt for OTP page 1
				uint8 PROGOK			:1; //Indicates the validity for loading for OTP page 1. A valid page indicates that successful programming occurred.
				uint8 FMTERR			:1;	//Indicates a formatting error in OTP page 1; that is, when [UVOK] or [OVOK] is set
				uint8 LOADERR			:1; //Indicates an error while attempting to load OTP page 1
				uint8 LOADWRN			:1;	//Indicates OTP page 1 was loaded but with one or more SEC warnings
				uint8 LOADED			:1; //Indicates OTP page 1 has been selected for loading into the related registers.
			}Bit;
			uint8 Byte;
		}OTP_CUST2_STAT;
		
	}_Bq79616_RegisterGroup;
	extern _Bq79616_RegisterGroup Bq79718_Reg[4];
	/**@} */

	/**
	 * @brief Core BQ79616 Device Control Functions
	 */

	/**
	 * @brief Wake up the BQ79616 device.
	 *
	 * Brings the BQ79616 from shutdown or sleep mode into wake-up mode
	 * using the WAKEUP command.
	 *
	 * @return Status code (0 = success, non-zero = error)
	 */
	uint8 Bq79616_WakeUp(void);

	/**
	 * @brief Transition the BQ79616 from sleep mode to active mode.
	 *
	 * Executes the SLP2ACT command to change the device state from sleep to active.
	 *
	 * @return Status code (0 = success, non-zero = error)
	 */
	uint8 Bq79616_SLP2ACT(void);

	/**
	 * @brief Configure MAIN ADC registers.
	 *
	 * Initializes and configures all required registers for the MAIN ADC.
	 *
	 * @return Status code (0 = success, non-zero = error)
	 */
	uint8 Bq79616_Main_Config(void);

	/**
	 * @brief Perform MAIN ADC measurement.
	 *
	 * Triggers a measurement sequence using the MAIN ADC and retrieves the results.
	 *
	 * @return Status code (0 = success, non-zero = error)
	 */
	uint8 Bq79616_Main_Mears(void);

	/**
	 * @brief Perform AUX ADC measurement.
	 *
	 * Triggers a measurement sequence using the AUX ADC and retrieves the results.
	 *
	 * @return Status code (0 = success, non-zero = error)
	 */
	uint8 Bq79616_AUX_Mears(void);

	/**
	 * @brief Configure module balancing time.
	 *
	 * Sets the cell balancing duration for each module.
	 *
	 * @param balance_time Pointer to the array of balance time values
	 * @param time_set Pointer to the time configuration array
	 */
	void Moule_Balance_Time(uint16* balance_time, uint16* time_set);

	/**
	 * @brief Put the BQ79616 device into shutdown mode.
	 *
	 * @return Status code (0 = success, non-zero = error)
	 */
	uint8 Bq79616_ShutDown(void);

	/**
	 * @brief Put the BQ79616 device into sleep mode.
	 *
	 * @return Status code (0 = success, non-zero = error)
	 */
	uint8 Bq79616_Sleep(void);


	/******************** Diagnostic and Verification Functions ************************/

	/**
	 * @brief Check cell voltage consistency.
	 *
	 * Compares the VC and CB measurement voltages to detect excessive deviations.
	 *
	 * @return Status code (0 = success, non-zero = error)
	 */
	uint8 Cell_Vol_Chek(void);

	/**
	 * @brief Check LPF measurement consistency.
	 *
	 * Performs a diagnostic check of the low-pass filter (LPF) measurement.
	 *
	 * @return Status code (0 = success, non-zero = error)
	 */
	uint8 Cell_LPF_Chek(void);

	/**
	 * @brief Check temperature consistency.
	 *
	 * Compares temperature readings obtained through GPIO inputs to verify that
	 * the temperature difference is within limits. The check is based on percentage deviation.
	 *
	 * @return Status code (0 = success, non-zero = error)
	 */
	uint8 Cell_Temp_Chek(void);

	/**
	 * @brief Verify cell balancing FETs.
	 *
	 * Performs functional checks on the cell balancing FETs (CBFETs).
	 *
	 * @return Status code (0 = success, non-zero = error)
	 */
	uint8 Balance_FET_Chek(void);

	/**
	 * @brief Check for open-circuit condition on VC lines.
	 *
	 * Performs open-circuit diagnostics on the cell voltage (VC) connections.
	 *
	 * @param vc_open_flag Pointer to a variable that stores open-circuit flags.
	 * @return Status code (0 = success, non-zero = error)
	 */
	uint8 VC_Open_Chek(uint16* vc_open_flag);

	/**
	 * @brief Check for open-circuit condition on CB lines.
	 *
	 * Performs open-circuit diagnostics on the cell balancing (CB) connections.
	 *
	 * @return Status code (0 = success, non-zero = error)
	 */
	uint8 CB_Open_Chek(void);

	/**
	 * @brief Retrieve fault and error information.
	 *
	 * Reads fault and error bit fields from the BQ79616 for diagnostic purposes.
	 */
	void Get_Fault(void);


	/************************************************************************/
	/*                       Version A-Sample Section                        */
	/************************************************************************/

	#else // A-sample (prototype version)

	/* ---------------- Communication and Status Variables ---------------- */
	extern uint32 crc_fault, crc_right;
	extern float crc_pre;

	/**
	 * @brief BQ communication failure timer.
	 *
	 * Used to monitor communication validity with BQ devices.
	 * If no valid communication occurs within 1 minute, the BQ chip is reset.
	 */
	extern uint16 bq_comm_fail;

	/**
	 * @brief BQ79616 data acquisition error flag.
	 *
	 * Indicates sampling or measurement errors detected in the BQ79616.
	 */
	extern uint16 bq_samp_err_flag;


	/********************** Common Macro Definitions ****************************/

	/**
	 * @brief Number of BQ79616 devices in daisy-chain configuration.
	 */
	#define BQ79616_NUM        4     ///< 4 BQ79616 devices in daisy chain

	/**
	 * @brief Number of temperature sensors per BQ79616 device.
	 */
	#define TEMP_PER_NUM       3     ///< Each device monitors 3 cell temperatures

	/**
	 * @brief Number of temperature sensors per battery pack.
	 */
	#define CELLTEM_NUM        10    ///< 10 temperature sensors per pack


	/**
	 * @name BQ Device Address Definitions
	 * @{
	 */
	#define BQ79600_ADR        0     ///< Address of the BQ79600 bridge device
	#define BQ79616_ADR_S1     1     ///< Address of the first BQ79616 slave
	#define BQ79616_ADR_S2     2     ///< Address of the second BQ79616 slave
	#define BQ79616_ADR_S3     3     ///< Address of the third BQ79616 slave
	#define BQ79616_ADR_S4     4     ///< Address of the fourth BQ79616 slave
	/** @} */


	/**
	 * @name LSB Resolution Constants (Conversion Factors)
	 * @{
	 */
	#define VLSB_ADC           0.19073f   ///< LSB resolution for cell voltage measurement (mV)
	#define VLSB_TSREF         0.16954f   ///< LSB resolution for TSREF measurement (mV)
	#define VLSB_GPIO          0.15259f   ///< LSB resolution for GPIO measurement (mV)
	#define VLSB_MAIN_DIETEMP1 0.025f     ///< LSB resolution for MAIN die temperature (°C)
	#define VLSB_AUX_DIETEMP1  0.025f     ///< LSB resolution for AUX die temperature (°C)
	#define VLSB_AUX_BAT       0.0305f    ///< LSB resolution for BAT voltage measurement (0.1 V)
	#define VLSB_AUX_DIAG      0.015259f  ///< LSB resolution for diagnostic voltage measurement (0.01 V)
	#define VLSB_BB            0.03052f   ///< LSB resolution for busbar voltage measurement (mV)
	/** @} */


	/**
	 * @name Register Address Definitions
	 * @{
	 */

	#define S_DIR0_ADDR_OTP			0x00
	#define S_DIR1_ADDR_OTP			0x01
	#define S_DEV_CONF				0x02
	#define S_ACTIVE_CELL			0x03
	#define S_OTP_SPARE12			0x04
	#define S_BB_POSN1				0x05
	#define S_BB_POSN2				0x06
	#define S_ADC_CONF1				0x07
	#define S_ADC_CONF2				0x08
	#define S_OV_THRESH				0x09
	#define S_UV_THRESH				0x0A
	#define S_OTUT_THRESH			0x0B
	#define S_GPIO_CONF1			0x0C
	#define S_GPIO_CONF2			0x0D
	#define S_GPIO_CONF3			0x0E
	#define S_GPIO_CONF4			0x0F
	#define S_FAULT_MSK1			0x10
	#define S_FAULT_MSK2			0x11
	#define S_PWR_TRANSIT_CONF		0x12
	#define S_COMM_TIMEOUT_CONF		0x13
	#define S_TX_HOLD_OFF			0x14
	#define S_MAIN_ADC_GAIN			0x15
	#define S_MAIN_ADC_OFFSET		0x16
	#define S_AUX_ADC_GAIN			0x17
	#define S_AUX_ADC_OFFSET		0x18

	#define S_CUST_MISC1			0x1B
	#define S_CUST_MISC2			0x1C
	#define S_CUST_MISC3			0x1D
	#define S_CUST_MISC4			0x1E
	#define S_CUST_MISC5			0x1F
	#define S_CUST_MISC6			0x20
	#define S_CUST_MISC7			0x21
	#define S_CUST_MISC8			0x22

	#define S_OTP_SPARE11			0x23
	#define S_OTP_SPARE10			0x24
	#define S_OTP_SPARE9			0x25
	#define S_OTP_SPARE8			0x26
	#define S_OTP_SPARE7			0x27
	#define S_OTP_SPARE6			0x28
	#define S_OTP_SPARE5			0x29
	#define S_OTP_SPARE4			0x2A
	#define S_OTP_SPARE3			0x2B
	#define S_OTP_SPARE2			0x2C
	#define S_OTP_SPARE1			0x2D
	#define S_CUST_CRC_HI			0x2E
	#define S_CUST_CRC_LO			0x2F

	#define S_OTP_PROG_UNLOCK1A		0x300
	#define S_OTP_PROG_UNLOCK1B		0x301
	#define S_OTP_PROG_UNLOCK1C		0x302
	#define S_OTP_PROG_UNLOCK1D		0x303

	#define S_DIR0_ADDR				0x306
	#define S_DIR1_ADDR				0x307
	#define S_COMM_CTRL				0x308
	#define S_CONTROL1				0x309
	#define S_CONTROL2				0x30A
	#define S_OTP_PROG_CTRL			0x30B

	#define S_ADC_CTRL1				0x30D
	#define S_ADC_CTRL2				0x30E
	#define S_ADC_CTRL3				0x30F

	#define S_CB_CELL16_CTRL		0x318
	#define S_CB_CELL15_CTRL		0x319
	#define S_CB_CELL14_CTRL		0x31A
	#define S_CB_CELL13_CTRL		0x31B
	#define S_CB_CELL12_CTRL		0x31C
	#define S_CB_CELL11_CTRL		0x31D
	#define S_CB_CELL10_CTRL		0x31E
	#define S_CB_CELL9_CTRL			0x31F
	#define S_CB_CELL8_CTRL			0x320
	#define S_CB_CELL7_CTRL			0x321
	#define S_CB_CELL6_CTRL			0x322
	#define S_CB_CELL5_CTRL			0x323
	#define S_CB_CELL4_CTRL			0x324
	#define S_CB_CELL3_CTRL			0x324
	#define S_CB_CELL2_CTRL			0x326
	#define S_CB_CELL1_CTRL			0x327

	#define S_VMB_DONE_THRESH		0x328
	#define S_MB_TIMER_CTRL			0x329
	#define S_VCB_DONE_THRESH		0x32A
	#define S_OTCB_THRESH			0x32B
	#define S_OVUV_CTRL				0x32C
	#define S_OTUT_CTRL				0x32D
	#define S_BAL_CTRL1				0x32E
	#define S_BAL_CTRL2				0x32F

	#define S_FAULT_RST1			0x331
	#define S_FAULT_RST2			0x332
	#define S_DIAG_OTP_CTRL			0x335
	#define S_DIAG_COMM_CTRL		0x336
	#define S_DIAG_PWR_CTRL			0x337
	#define S_DIAG_CBFET_CTRL1		0x338
	#define S_DIAG_CBFET_CTRL2		0x339
	#define S_DIAG_COMP_CTRL1		0x33A
	#define S_DIAG_COMP_CTRL2		0x33B
	#define S_DIAG_COMP_CTRL3		0x33C
	#define S_DIAG_COMP_CTRL4		0x33D
	#define S_DIAG_PROT_CTRL		0x33E

	#define S_OTP_ECC_DATAIN1		0x343
	#define S_OTP_ECC_DATAIN2		0x344
	#define S_OTP_ECC_DATAIN3		0x345
	#define S_OTP_ECC_DATAIN4		0x346
	#define S_OTP_ECC_DATAIN5		0x347
	#define S_OTP_ECC_DATAIN6		0x348
	#define S_OTP_ECC_DATAIN7		0x349
	#define S_OTP_ECC_DATAIN8		0x34A
	#define S_OTP_ECC_DATAIN9		0x34B
	#define S_OTP_ECC_TEST			0x34C
	#define S_SPI_CONF				0x34D
	#define S_SPI_TX3				0x34E
	#define S_SPI_TX2				0x34F
	#define S_SPI_TX1				0x350
	#define S_OTP_PROG_UNLOCK2A		0x351
	#define S_OTP_PROG_UNLOCK2B		0x352
	#define S_OTP_PROG_UNLOCK2C		0x353
	#define S_OTP_PROG_UNLOCK2D		0x354

	#define S_DEBUG_CTRL_UNLOCK		0x700
	#define S_DEBUG_COMM_CTRL1		0x701
	#define S_DEBUG_COMM_CTRL2		0x702

	#define S_PARTID				0x500
	#define S_DIE_ID1				0x501
	#define S_DIE_ID2				0x502
	#define S_DIE_ID3				0x503
	#define S_DIE_ID4				0x504
	#define S_DIE_ID5				0x505
	#define S_DIE_ID6				0x506
	#define S_DIE_ID7				0x507
	#define S_DIE_ID8				0x508
	#define S_DIE_ID9				0x509

	#define S_CUST_CRC_RSLT_HI		0x50C
	#define S_CUST_CRC_RSLT_LO		0x50D
	#define S_OTP_ECC_DATAOUT1		0x510
	#define S_OTP_ECC_DATAOUT2		0x511
	#define S_OTP_ECC_DATAOUT3		0x512
	#define S_OTP_ECC_DATAOUT4		0x513
	#define S_OTP_ECC_DATAOUT5		0x514
	#define S_OTP_ECC_DATAOUT6		0x515
	#define S_OTP_ECC_DATAOUT7		0x516
	#define S_OTP_ECC_DATAOUT8		0x517
	#define S_OTP_ECC_DATAOUT9		0x518

	#define S_OTP_PROG_STAT			0x519
	#define S_OTP_CUST1_STAT		0x51A
	#define S_OTP_CUST2_STAT		0x51B

	#define S_SPI_RX3				0x520
	#define S_SPI_RX2				0x521
	#define S_SPI_RX1				0x522
	#define S_DIAG_STAT				0x527
	#define S_ADC_STAT1				0x528
	#define S_ADC_STAT2				0x529
	#define S_GPIO_STAT				0x52A
	#define S_BAL_STAT				0x52B
	#define S_DEV_STAT				0x52C
	#define S_FAULT_SUMMARY			0x52D

	#define S_FAULT_COMM1			0x530
	#define S_FAULT_COMM2			0x531
	#define S_FAULT_COMM3			0x532

	#define S_FAULT_OTP				0x535
	#define S_FAULT_SYS				0x536
	#define S_FAULT_PROT1			0x53A
	#define S_FAULT_PROT2			0x53B
	#define S_FAULT_OV1				0x53C
	#define S_FAULT_OV2				0x53D
	#define S_FAULT_UV1				0x53E
	#define S_FAULT_UV2				0x53F

	#define S_FAULT_OT				0x540
	#define S_FAULT_UT				0x541
	#define S_FAULT_COMP_GPIO		0x543
	#define S_FAULT_COMP_VCCB1		0x545
	#define S_FAULT_COMP_VCCB2		0x546
	#define S_FAULT_COMP_VCOW1		0x548
	#define S_FAULT_COMP_VCOW2		0x549
	#define S_FAULT_COMP_CBOW1		0x54B
	#define S_FAULT_COMP_CBOW2		0x54C
	#define S_FAULT_COMP_CBFET1		0x54E
	#define S_FAULT_COMP_CBFET2		0x54F

	#define S_FAULT_COMP_MISC		0x550
	#define S_FAULT_PWR1			0x552
	#define S_FAULT_PWR2			0x553
	#define S_FAULT_PWR3			0x554
	#define S_CB_COMPLETE1			0x556
	#define S_CB_COMPLETE2			0x557

	#define S_VCELL18_HI			0x564
	#define S_VCELL18_LO			0x566
	#define S_VCELL17_HI			0x566
	#define S_VCELL17_LO			0x567

	#define S_VCELL16_HI			0x568
	#define S_VCELL16_LO			0x569
	#define S_VCELL15_HI			0x56A
	#define S_VCELL15_LO			0x56B
	#define S_VCELL14_HI			0x56C
	#define S_VCELL14_LO			0x56D
	#define S_VCELL13_HI			0x56E
	#define S_VCELL13_LO			0x56F
	#define S_VCELL12_HI			0x570
	#define S_VCELL12_LO			0x571
	#define S_VCELL11_HI			0x572
	#define S_VCELL11_LO			0x573
	#define S_VCELL10_HI			0x574
	#define S_VCELL10_LO			0x575
	#define S_VCELL9_HI				0x576
	#define S_VCELL9_LO				0x577
	#define S_VCELL8_HI				0x578
	#define S_VCELL8_LO				0x579
	#define S_VCELL7_HI				0x57A
	#define S_VCELL7_LO				0x57B
	#define S_VCELL6_HI				0x57C
	#define S_VCELL6_LO				0x57D
	#define S_VCELL5_HI				0x57E
	#define S_VCELL5_LO				0x57F
	#define S_VCELL4_HI				0x580
	#define S_VCELL4_LO				0x581
	#define S_VCELL3_HI				0x582
	#define S_VCELL3_LO				0x583
	#define S_VCELL2_HI				0x584
	#define S_VCELL2_LO				0x585
	#define S_VCELL1_HI				0x586
	#define S_VCELL1_LO				0x587

	#define S_BUSBAR_HI				0x588
	#define S_BUSBAR_LO				0x589
	#define S_TSREF_HI				0x58C
	#define S_TSREF_LO				0x58D

	#define S_GPIO1_HI				0x58E
	#define S_GPIO1_LO				0x58F
	#define S_GPIO2_HI				0x590
	#define S_GPIO2_LO				0x591
	#define S_GPIO3_HI				0x592
	#define S_GPIO3_LO				0x593
	#define S_GPIO4_HI				0x594
	#define S_GPIO4_LO				0x595
	#define S_GPIO5_HI				0x596
	#define S_GPIO5_LO				0x597
	#define S_GPIO6_HI				0x598
	#define S_GPIO6_LO				0x599
	#define S_GPIO7_HI				0x59A
	#define S_GPIO7_LO				0x59B
	#define S_GPIO8_HI				0x59C
	#define S_GPIO8_LO				0x59D

	#define S_DIETEMP1_HI			0x5AE
	#define S_DIETEMP1_LO			0x5AF
	#define S_DIETEMP2_HI			0x5B0
	#define S_DIETEMP2_LO			0x5B1
	#define S_AUX_CELL_HI			0x5B2
	#define S_AUX_CELL_LO			0x5B3
	#define S_AUX_GPIO_HI			0x5B4
	#define S_AUX_GPIO_LO			0x5B5
	#define S_AUX_BAT_HI			0x5B6
	#define S_AUX_BAT_LO			0x5B7
	#define S_AUX_REFL_HI			0x5B8
	#define S_AUX_REFL_LO			0x5B9
	#define S_AUX_VBG2_HI			0x5BA
	#define S_AUX_VBG2_LO			0x5BB
	#define S_AUX_VBG5_HI			0x5BC
	#define S_AUX_VBG5_LO			0x5BD
	#define S_AUX_AVAO_REF_HI		0x5BE
	#define S_AUX_AVAO_REF_LO		0x5BF
	#define S_AUX_AVDD_REF_HI		0x5C0
	#define S_AUX_AVDD_REF_LO		0x5C1
	#define S_AUX_OV_DACHI			0x5C2
	#define S_AUX_OV_DAC_LO			0x5C3
	#define S_AUX_UV_DAC_HI			0x5C4
	#define S_AUX_UV_DAC_LO			0x5C5
	#define S_AUX_OT_OTCB_DAC_HI	0x5C6
	#define S_AUX_OT_OTCB_DAC_LO	0x5C7
	#define S_AUX_UT_DAC_HI			0x5C8
	#define S_AUX_UT_DAC_LO			0x5C9
	#define S_AUX_VCBDONE_DAC_HI	0x5CA
	#define S_AUX_VCBDONE_DAC_LO	0x5CB
	#define S_AUX_VCM1_HI			0x5CC
	#define S_AUX_VCM1_LO			0x5CD
	#define S_VCM2_HI				0x5CE
	#define S_VCM2_LO				0x5CF

	#define S_DEBUG_COMM_STAT		0x780
	#define S_DEBUG_UART_RC			0x781
	#define S_DEBUG_UARTR_RR		0x782
	#define S_DEBUG_COMH_BIT		0x783
	#define S_DEBUG_COMH_RC			0x784
	#define S_DEBUG_COMH_RR_TR		0x785
	#define S_DEBUG_COML_BIT		0x786
	#define S_DEBUG_COML_RC			0x787
	#define S_DEBUG_COML_RR_TR		0x788
	#define S_DEBUG_UART_DISCARD	0x789
	#define S_DEBUG_COMH_DISCARD	0x78A
	#define S_DEBUG_COML_DISCARD	0x78B
	#define S_DEBUG_UART_VALID_HI	0x78C
	#define S_DEBUG_UART_VALID_LO	0x78D
	#define S_DEBUG_COMH_VALID_HI	0x78E
	#define S_DEBUG_COMH_VALID_LO	0x78F
	#define S_DEBUG_COML_VALID_HI	0x790
	#define S_DEBUG_COML_VALID_LO	0x791
	#define S_DEBUG_OTP_SEC_BLK		0x7A0
	#define S_DEBUG_OTP_DED_BLK		0x7A1
	/**@} */

	/**
	 * @brief ADC Gain and Offset Conversion Macros
	 *
	 * Macros for calculating and converting ADC gain and offset values
	 * between register codes and real-world values.
	 * Derived from the device datasheet.
	 * @{
	 */

	/********************  Get (Read) Conversions  ********************/

	/**
	 * @brief Convert MAIN ADC gain register value to real gain.
	 *
	 * Formula based on the device datasheet.
	 *
	 * @param x Register value (integer)
	 * @return Calculated MAIN ADC gain (floating-point)
	 */
	#ifndef GET_MAIN_ADC_GAIN
	    #define GET_MAIN_ADC_GAIN(x)   ((x) * 0.0031f - 0.390625f)   ///< MAIN_ADC_GAIN read conversion
	#endif

	/**
	 * @brief Convert MAIN ADC offset register value to real offset.
	 *
	 * @param x Register value (integer)
	 * @return Calculated MAIN ADC offset (floating-point)
	 */
	#ifndef GET_MAIN_ADC_OFFSET
	    #define GET_MAIN_ADC_OFFSET(x) ((x) * 0.19073f - 24.41406f)  ///< MAIN_ADC_OFFSET read conversion
	#endif

	/**
	 * @brief Convert AUX ADC gain register value to real gain.
	 *
	 * @param x Register value (integer)
	 * @return Calculated AUX ADC gain (floating-point)
	 */
	#ifndef GET_AUX_ADC_GAIN
	    #define GET_AUX_ADC_GAIN(x)    ((x) * 0.0031f - 0.390625f)   ///< AUX_ADC_GAIN read conversion
	#endif

	/**
	 * @brief Convert AUX ADC offset register value to real offset.
	 *
	 * @param x Register value (integer)
	 * @return Calculated AUX ADC offset (floating-point)
	 */
	#ifndef GET_AUX_ADC_OFFSET
	    #define GET_AUX_ADC_OFFSET(x)  ((x) * 0.19073f - 24.41406f)  ///< AUX_ADC_OFFSET read conversion
	#endif


	/********************  Set (Write) Conversions  ********************/

	/**
	 * @brief Convert real MAIN ADC gain to register value.
	 *
	 * Converts a floating-point MAIN ADC gain value to an 8-bit register code.
	 * Formula follows the device datasheet (division replaced with multiplication for efficiency).
	 *
	 * @param x Desired MAIN ADC gain (floating-point)
	 * @return Register value for MAIN_ADC_GAIN
	 */
	#ifndef SET_MAIN_ADC_GAIN
	    #define SET_MAIN_ADC_GAIN(x)   ((uint8)(((x) + 0.390625f) * 322.5806f))  ///< MAIN_ADC_GAIN write conversion
	#endif

	/**
	 * @brief Convert real MAIN ADC offset to register value.
	 *
	 * @param x Desired MAIN ADC offset (floating-point)
	 * @return Register value for MAIN_ADC_OFFSET
	 */
	#ifndef SET_MAIN_ADC_OFFSET
	    #define SET_MAIN_ADC_OFFSET(x) ((uint8)(((x) + 24.41406f) * 5.2430f))    ///< MAIN_ADC_OFFSET write conversion
	#endif

	/**
	 * @brief Convert real AUX ADC gain to register value.
	 *
	 * @param x Desired AUX ADC gain (floating-point)
	 * @return Register value for AUX_ADC_GAIN
	 */
	#ifndef SET_AUX_ADC_GAIN
	    #define SET_AUX_ADC_GAIN(x)    ((uint8)(((x) + 0.390625f) * 322.5806f))  ///< AUX_ADC_GAIN write conversion
	#endif

	/**
	 * @brief Convert real AUX ADC offset to register value.
	 *
	 * @param x Desired AUX ADC offset (floating-point)
	 * @return Register value for AUX_ADC_OFFSET
	 */
	#ifndef SET_AUX_ADC_OFFSET
	    #define SET_AUX_ADC_OFFSET(x)  ((uint8)(((x) + 24.41406f) * 5.2430f))    ///< AUX_ADC_OFFSET write conversion
	#endif
	/** @} */


	/**
	 * @name Register Field Value Definitions
	 * @{
	 */

	/* --- GPIO and Function Configuration --- */
	#define S_FAULT_IN_EN       0x01  ///< Enable GPIO8 as NFAULT input
	#define S_FAULT_IN_DIS      0x00  ///< Disable GPIO8 as NFAULT input
	#define S_SPI_MASTER_EN     0x01  ///< Enable SPI master mode via GPIO
	#define S_SPI_MASTER_DIS    0x00  ///< Disable SPI master mode via GPIO
	#define S_GPIO_HIGHZ        0x00  ///< High-impedance (Hi-Z) state
	#define S_GPIO_ADC_OTUT     0x01  ///< ADC and OTUT input mode
	#define S_GPIO_ADC_ONLY     0x02  ///< ADC input only mode
	#define S_GPIO_DIGI_INPUT   0x03  ///< Digital input mode
	#define S_GPIO_OUT_HIGH     0x04  ///< Drive GPIO output high
	#define S_GPIO_OUT_LOW      0x05  ///< Drive GPIO output low

	/* --- Cell Configuration --- */
	#define CELL_8S             0x02  ///< 8-series cell configuration
	#define CELL_9S             0x03  ///< 9-series cell configuration
	#define CELL_10S            0x04  ///< 10-series cell configuration
	#define CELL_11S            0x05  ///< 11-series cell configuration
	#define CELL_12S            0x06  ///< 12-series cell configuration
	#define CELL_13S            0x07  ///< 13-series cell configuration
	#define CELL_14S            0x08  ///< 14-series cell configuration
	#define CELL_15S            0x09  ///< 15-series cell configuration
	#define CELL_16S            0x0A  ///< 16-series cell configuration

	/* --- MAIN ADC Control --- */
	#define MAIN_ADC_DIS        0x00  ///< Disable MAIN ADC operation
	#define MAIN_ADC_SING_RUN   0x01  ///< Enable single conversion mode
	#define MAIN_ADC_CONTINOU   0x02  ///< Enable continuous conversion mode
	/** @} */


	#define TWARN_85C 			0x00	///<device T warining thresh 
	#define TWARN_90C 			0x01
	#define TWARN_105C 			0x02
	#define TWARN_115C 			0x03

	#define SLP_TIME_NO 		0x00
	#define SLP_TIME_5S 		0x01
	#define SLP_TIME_10S 		0x02
	#define SLP_TIME_1MIN 		0x03
	#define SLP_TIME_10MIN 		0x04
	#define SLP_TIME_30MIN 		0x05
	#define SLP_TIME_1HOUR 		0x06
	#define SLP_TIME_2HOUR 		0x07

	#define CTS_TIME_DIS 		0x00
	#define CTS_TIME_100MS 		0x01
	#define CTS_TIME_2S 		0x02
	#define CTS_TIME_10S 		0x03
	#define CTS_TIME_1MIN 		0x04
	#define CTS_TIME_10MIN 		0x05
	#define CTS_TIME_30MIN 		0x06
	#define CTS_TIME_1HOUR 		0x07

	#define CTL_SLP 			0x00	//sleep
	#define CTL_SHD 			0x01	//shot dowm

	#define CTL_TIME_DIS 		0x00
	#define CTL_TIME_100MS 		0x01
	#define CTL_TIME_2S 		0x02
	#define CTL_TIME_10S 		0x03
	#define CTL_TIME_1MIN 		0x04
	#define CTL_TIME_10MIN 		0x05
	#define CTL_TIME_30MIN 		0x06
	#define CTL_TIME_1HOUR 		0x07

	#define LPF_VCELL_6P5 		0x0	///<6.5Hz (154 ms average
	#define LPF_VCELL_13  		0x1	///<13Hz 77 ms average
	#define LPF_VCELL_26  		0x2	///<38 ms average
	#define LPF_VCELL_53  		0x3	///<19 ms average
	#define LPF_VCELL_111 		0x4	///<9 ms average
	#define LPF_VCELL_240 		0x5	///<4 ms average
	#define LPF_VCELL_600 		0x6	///<1.6 ms average

	#define AUX_SETTLE_4P3 		0x0	///<4.3ms
	#define AUX_SETTLE_2P3 		0x1	///<2.3ms
	#define AUX_SETTLE_1P3 		0x2	///<1.3ms
	#define AUX_SETTLE_0P3 		0x3	///<0.3ms

	#define MIAN_ADC_DIS 		00B	
	#define MIAN_ADC_SR 		01B	///<Single run
	#define MIAN_ADC_CR 		10B	///<Continuous run

	#define AUX_CELL_ALL 		0X00
	#define AUX_CELL_Busbar 	0X01
	#define AUX_CELL_CELL1	 	0X02
	#define AUX_CELL_CELL2	 	0X03
	#define AUX_CELL_CELL3	 	0X04
	#define AUX_CELL_CELL4	 	0X05
	#define AUX_CELL_CELL5	 	0X06
	#define AUX_CELL_CELL6	 	0X07
	#define AUX_CELL_CELL7	 	0X08
	#define AUX_CELL_CELL8	 	0X09
	#define AUX_CELL_CELL9	 	0X0A
	#define AUX_CELL_CELL10	 	0X0B
	#define AUX_CELL_CELL11	 	0X0C
	#define AUX_CELL_CELL12	 	0X0D
	#define AUX_CELL_CELL13	 	0X0E
	#define AUX_CELL_CELL14	 	0X0F
	#define AUX_CELL_CELL15	 	0X10
	#define AUX_CELL_CELL16	 	0X11

	#define AUX_MODE_DIS 		0x0	
	#define AUX_MODE_SR 		0x1	///<Single run
	#define AUX_MODE_CR 		0x2	///<Continuous run  AUX_GPIO

	#define AUX_GPIO_ALL 		0x00
	#define AUX_GPIO_1 			0x01
	#define AUX_GPIO_2 			0x02
	#define AUX_GPIO_3 			0x03
	#define AUX_GPIO_4 			0x04
	#define AUX_GPIO_5 			0x06
	#define AUX_GPIO_6 			0x06
	#define AUX_GPIO_7 			0x07
	#define AUX_GPIO_8 			0x08

	#define BALANCE_TIME_DIS  	 0x00	///<stop 
	#define BALANCE_TIME_10S  	 0x01	///<10s
	#define BALANCE_TIME_30S  	 0x02	///<30s
	#define BALANCE_TIME_60S  	 0x03	///<60s
	#define BALANCE_TIME_300S  	 0x04	///<300s
	#define BALANCE_TIME_10MIN   0x05	///<10min
	#define BALANCE_TIME_20MIN   0x06	
	#define BALANCE_TIME_30MIN   0x07	
	#define BALANCE_TIME_40MIN   0x08
	#define BALANCE_TIME_50MIN   0x09
	#define BALANCE_TIME_60MIN   0x0A
	#define BALANCE_TIME_70MIN   0x0B
	#define BALANCE_TIME_80MIN   0x0C
	#define BALANCE_TIME_90MIN   0x0D
	#define BALANCE_TIME_100MIN  0x0E
	#define BALANCE_TIME_110MIN  0x0F
	#define BALANCE_TIME_120MIN  0x10
	#define BALANCE_TIME_150MIN  0x11	
	#define BALANCE_TIME_180MIN  0x12	
	#define BALANCE_TIME_210MIN  0x13	
	#define BALANCE_TIME_240MIN  0x14	
	#define BALANCE_TIME_270MIN  0x15	
	#define BALANCE_TIME_300MIN  0x16	
	#define BALANCE_TIME_330MIN  0x17	
	#define BALANCE_TIME_360MIN  0x18	
	#define BALANCE_TIME_390MIN  0x19	
	#define BALANCE_TIME_420MIN  0x1A	
	#define BALANCE_TIME_450MIN  0x1B	
	#define BALANCE_TIME_480MIN  0x1C	
	#define BALANCE_TIME_510MIN  0x1D	
	#define BALANCE_TIME_540MIN  0x1E	
	#define BALANCE_TIME_600MIN  0x1F	

	#define VCB_THR_DIS 0x00 ///<Disables voltage based on CB_DONE comparison
	#define VCB_THR_2P8 0x01 ///<stop
	#define VCB_THR(x) ((x)>=(2800) ? ((x-2800)/25+1) : (0))///<Range from 2.8 V to 4.35 V with 25-mV steps

	#define OV_THR(x) ((x)>=(4175) ? ((x-4175)/25+0x22) : (0))///< 0x22 to 0x2E: range from 4175 mV to 4475 mV,else 2700mV
	#define UV_THR(x) ((x)>=(1200) ? ((x-1200)/50) : (0x28))///< 0x00 to 0x26: range from 1200 mV to 3100 mV,else 3100mV

	/**
	 * @brief Macros for MAIN and AUX ADC calibration settings.
	 */

	/**
	 * @brief MAIN ADC gain setting.
	 *
	 * Converts a floating-point gain value into an 8-bit register value
	 * for the MAIN ADC gain configuration.
	 *
	 * @param x Desired gain value.
	 * @return Register value for MAIN_ADC_GAIN.
	 */
	#define MAIN_GAIN(x)      (uint8)((x + 0.390625f) / 0.0031f)   ///< MAIN_ADC_GAIN setting

	/**
	 * @brief MAIN ADC offset setting.
	 *
	 * Converts a floating-point offset value into an 8-bit register value
	 * for the MAIN ADC offset configuration.
	 *
	 * @param x Desired offset value.
	 * @return Register value for MAIN_ADC_OFFSET.
	 */
	#define MAIN_OFFSET(x)    (uint8)((x + 24.41606f) / 0.19073f)  ///< MAIN_ADC_OFFSET setting

	/**
	 * @brief AUX ADC gain setting.
	 *
	 * Converts a floating-point gain value into an 8-bit register value
	 * for the AUX ADC gain configuration.
	 *
	 * @param x Desired gain value.
	 * @return Register value for AUX_ADC_GAIN.
	 */
	#define AUX_GAIN(x)       (uint8)((x + 0.390625f) / 0.0031f)   ///< AUX_ADC_GAIN setting

	/**
	 * @brief AUX ADC offset setting.
	 *
	 * Converts a floating-point offset value into an 8-bit register value
	 * for the AUX ADC offset configuration.
	 *
	 * @param x Desired offset value.
	 * @return Register value for AUX_ADC_OFFSET.
	 */
	#define AUX_OFFSET(x)     (uint8)((x + 24.41606f) / 0.19073f)  ///< AUX_ADC_OFFSET setting


	/**
	 * @brief Diagnostic and verification macro definitions.
	 * @{
	 */

	/**
	 * @brief Convert voltage index to register value.
	 *
	 * @param x Voltage value (integer multiples of 10, e.g., 10, 20, 30 … 160)
	 * @return Register value corresponding to the voltage index.
	 */
	#define VCCB(x)           (x / 10 - 1)   ///< x is in the range [10, 20, 30 … 160], integer values only

	#define Dis_ADC_Comp      0x0   ///< Disable comparison of acquired ADC signals
	#define Cell_Vol_Check    0x1   ///< Perform cell voltage comparison check
	#define VC_Open_Check     0x2   ///< Check for open-circuit condition on VC lines
	#define CB_Open_Check     0x3   ///< Check for open-circuit condition on CB lines
	#define CBFET_Check       0x4   ///< Check the balancing FET operation
	#define GPIO_Check        0x5   ///< Perform GPIO diagnostic check

	/**
	 * @brief Temperature threshold setting.
	 *
	 * Converts percentage value to register index.
	 *
	 * @param x Temperature deviation percentage (2%, 4%, … 16%)
	 * @return Register index (0–7) corresponding to the temperature threshold.
	 */
	#define TEMP(x)           (x / 2 - 1)    ///< x values: [2%, 4%, …, 16%], corresponding to register values 0–7

	/**
	 * @brief Open-wire voltage threshold setting.
	 *
	 * Converts millivolt value to register index.
	 *
	 * @param x Voltage threshold in millivolts (250 mV – 4000 mV, step size 250 mV)
	 * @return Register index corresponding to the voltage threshold.
	 */
	#define OW(x)             (x / 250 - 1)  ///< x values: 250 mV – 4000 mV, step size 250 mV

	#define SINK_ALL_OFF      0x0   ///< Disable all discharge (sink) paths
	#define SINK_VC           0x1   ///< Enable VC channel sinking
	#define SINK_CB           0x2   ///< Enable CB channel sinking
	#define SINK_BB           0x3   ///< Enable both BBP and BBN channel sinking

	/** @} */


	/***


	#define BAL_DUTY_5S 		0x00 ///<5s  Selection is sampled whenever [BAL_GO] = 1
	#define BAL_DUTY_10S 		0x01 
	#define BAL_DUTY_30S 		0x02 
	#define BAL_DUTY_60S 		0x03 
	#define BAL_DUTY_5MIN 		0x04 
	#define BAL_DUTY_10MIN	 	0x05 
	#define BAL_DUTY_20MIN		0x06 
	#define BAL_DUTY_30MIN		0x07 

	#define BAL_ACT_NOACTION	0x0	///<No action
	#define BAL_ACT_ENSLP		0x1	///<Enters SLEEP
	#define BAL_ACT_ENSTDO		0x2	///<Enters SHUTDOWN

	#define OVUV_DIS   			0x0 ///<Do not run OV and UV comparators
	#define OVUV_RUN_ROUND   	0x1 ///<Run the OV and UV round robin with all active cells
	#define OVUV_RUN_BIST	    0x2 ///<Run the OV and UV BIST cycle.
	#define OVUV_LOCK_SINGLE  	0x3 ///<Lock OV and UV comparators to a single channel configured by [OVUV_LOCK3:0]

	#define OVUV_LOCK(x)			(x-1)	///<1:CELL1

	#define OTUT_DIS   			0x00 ///<Do not run OT and UT comparators
	#define OTUT_RUN_ROUND    	0x01 ///<Run the OT and UT round robin with all active cells
	#define OTUT_RUN_BIST	    0x02 ///<Run the OT and UT BIST cycle.
	#define OTUT_LOCK_SINGLE  	0x03 ///<Lock OT and UT comparators to a single channel configured by [OTUT_LOCK3:0]

	#define OTUT_LOCK(x)		(x-1)	///<1:GPIO1

	#define GPIO_HIHG_Z			000B	///<As disabled, high-Z
	#define GPIO_ADC_OUTPUT		001B	///<As ADC and OTUT inputs
	#define GPIO_ADC_ONLY		001B	///<As ADC only input
	#define GPIO_DIG_INPUT		001B	///<As digital input
	#define GPIO_HIGH_OUTPUT  	001B	///<As output high
	#define GPIO_LOW_OUTPUT		001B	///<As output low

	#define SPI_NUMBIT_24		0	///<24bit
	#define SPI_NUMBIT_8		8	///<8bit
	#define SPI_NUMBIT_16		16	///<16bit

	#define MARGIN_Normal_Read		0
	#define MARGIN_Margin_1_Read	2
	/**@} */

	/**@name   Register declaration
	* @{
	*/
	typedef struct _Bq_Register_Group
	{
		union
		{
			struct
			{
				uint8 ADDRESS			:6; //This register shows the default device address used when [DIR_SEL] = 0,and programmed in the OTP
				uint8 SPARE			:2; //spare
			}Bit;
			uint8 Byte;
		}DIR0_ADDR_OTP;
		
		union
		{
			struct
			{
				uint8 ADDRESS			:6; //This register shows the default device address used when [DIR_SEL] = 1,and programmed in the OTP
				uint8 SPARE			:2; //spare
			}Bit;
			uint8 Byte;
		}DIR1_ADDR_OTP;
		
		union
		{
			struct
			{
				uint8 ADDRESS			:6; //Always shows the current device address used by the device when [DIR_SEL] = 0,can be write
				uint8 SPARE			:2; //spare
			}Bit;
			uint8 Byte;
		}DIR0_ADDR;
		
		union
		{
			struct
			{
				uint8 ADDRESS			:6; //Always shows the current device address used by the device when [DIR_SEL] = 1,can be write
				uint8 SPARE			:2; //spare
			}Bit;
			uint8 Byte;
		}DIR1_ADDR;

		uint8 PARTID;	//Device revision 0x00 = Revision A1 0x01 to 0xFF = Reserved
		uint8 DIE_ID1;	//Die ID for TI factory use
		uint8 DIE_ID2;	//Die ID for TI factory use
		uint8 DIE_ID3;	//Die ID for TI factory use
		uint8 DIE_ID4;	//Die ID for TI factory use
		uint8 DIE_ID5;	//Die ID for TI factory use
		uint8 DIE_ID6;	//Die ID for TI factory use
		uint8 DIE_ID7;	//Die ID for TI factory use
		uint8 DIE_ID8;	//Die ID for TI factory use
		uint8 DIE_ID9;	//Die ID for TI factory use
			
		uint8 CUST_MISC1;	//Customer scratch pad
		uint8 CUST_MISC2;	//Customer scratch pad
		uint8 CUST_MISC3;	//Customer scratch pad
		uint8 CUST_MISC4;	//Customer scratch pad
		uint8 CUST_MISC5;	//Customer scratch pad
		uint8 CUST_MISC6;	//Customer scratch pad
		uint8 CUST_MISC7;	//Customer scratch pad
		uint8 CUST_MISC8;	//Customer scratch pad
			
		union
		{
			struct
			{
				uint8 HB_EN			:1; //Enables HEARTBEAT transmitter when device is in SLEEP mode.0 disable;1 enable
				uint8 FTONE_EN			:1; //Enables FAULT TONE transmitter when device is in SLEEP mode.0 disable;1 enable
				uint8 NFAULT_EN		:1; //Enables the NFAULT function. 0:pulled up; 1:pulled low to indicate an unmasked fault is detected
				uint8 TWO_STOP_EN		:1; //UART ,0 = One STOP bit,1=TWO
				uint8 FCOMM_EN			:1; //Enables the fault state detection through communication in ACTIVE mode.0 disable;1 enable
				uint8 MULTIDROP_EN		:1; //0 Daisy chain of base device ; 1 Multidrop(standalone)
				uint8 NO_ADJ_CB		:1; //0  allow two adjacent CB FETs;1 not allow
				uint8 SPARE			:1; //spare
			}Bit;
			uint8 Byte;
		}DEV_CONF;	
		
		union
		{
			struct
			{
				uint8 NUM_CELL			:4; //0x0 = 6S,0x1=7S....
				uint8 SPARE			:4; //spare
			}Bit;
			uint8 Byte;
		}ACTIVE_CELL;	
	/*
	*	Among the active cells specified by the ACTIVE_CELL register, this register indicates which active channel is
		excluded from the OV, UV, and VCB_DONE monitoring.
	*	bus bar
	*0 = No special handling of the functions mentioned above
	*1 = Special handling of the functions mentioned above.
	*/
		union
		{
			struct
			{
				uint8 CELL9			:1;
				uint8 CELL10			:1;
				uint8 CELL11			:1;
				uint8 CELL12			:1;
				uint8 CELL13			:1;
				uint8 CELL14 			:1;
				uint8 CELL15			:1;
				uint8 CELL16			:1;
			}Bit;
			uint8 Byte;
		}BB_POSN1;	
		
		union
		{
			struct
			{
				uint8 SLP_TIME			:3; //A timeout in SLEEP mode.
				uint8 TWARN_THR		:2; //Sets the TWARN threshold  T warning
				uint8 SPARE			:3;
			}Bit;
			uint8 Byte;
		}PWR_TRANSIT_CONF;		
		
		union
		{
			struct
			{
				uint8 CELL1			:1;
				uint8 CELL2			:1;
				uint8 CELL3			:1;
				uint8 CELL4			:1;
				uint8 CELL5			:1;
				uint8 CELL6 			:1;
				uint8 CELL7			:1;
				uint8 CELL8			:1;
			}Bit;
			uint8 Byte;
		}BB_POSN2;	
		
		union
		{
			struct
			{
				uint8 CTL_TIME			:3; //Sets the short communication timeout
				uint8 CTL_ACT			:1; //Configures the device action when long communication timeout timer expires.
				uint8 CTS_TIME			:3; //Sets the long communication timeout.
				uint8 SPARE			:1; //
			}Bit;
			uint8 Byte;
		}COMM_TIMEOUT_CONF;
		
		uint8 TX_HOLD_OFF;	//between receive stop bit to tx
			
		union
		{
			struct
			{
				uint8 TOP_STACK		:1; //Defines device as highest addressed device in the stack
				uint8 STACK_DEV		:1; //Defines device as a base or stack device in daisy chain configuration.
				uint8 SPARE			:6; //
			}Bit;
			uint8 Byte;
		}COMM_CTRL;
		
		union
		{
			struct
			{
				uint8 ADDR_WR			:1; //Enables device to start auto-addressing.
				uint8 SOFT_RESET		:1; //Resets the digital to OTP default. Bit is cleared on read.
				uint8 GOTO_SLEEP		:1; //Transitions device to SLEEP mode. Bit is cleared on read.
				uint8 GOTO_SHUTDOWN	:1; //Transitions device to SHUTDOWN mode. Bit is cleared on read
				uint8 SEND_SLPTOACT	:1; //Sends SLEEPtoACTIVE tone up the stack. Bit is cleared on read.
				uint8 SEND_WAKE		:1; //Sends WAKE tone to next device up the stack. Bit is cleared on read.
				uint8 SEND_SHUTDOWN	:1;	//Sends SHUTDOWN tone to next device up the stack.
				uint8 DIR_SEL			:1; //Selects daisy chain communication direction.
			}Bit;
			uint8 Byte;
		} CONTROL1;
		
		union
		{
			struct
			{
				uint8 TSREF_EN			:1; //Enables TSREF LDO output. Used to bias NTC thermistor.
				uint8 SEND_HW_RESET	:1; //Sends HW_RESET tone up the stack. Bit is cleared on read.
				uint8 SPARE			:6; //
			}Bit;
			uint8 Byte;
		} CONTROL2;	
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}CUST_CRC;	// host-calculated CRC for customer OTP space.
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}CUST_CRC_RSLT;	//the device-calculated CRC for customer OTP space
			
		union
		{
			struct
			{
				uint8 DRDY_BIST_PWR		:1; //Indicates the status of the power supplies diagnostic
				uint8 DRDY_BIST_OVUV		:1; //Indicates the status of the OVUV protector diagnostic.
				uint8 DRDY_BIST_OTUT		:1; //Indicates the status of the OTUT protector diagnostic
				uint8 DRDY_OVUV			:1; //Indicates the OVUV round robin has at least run once
				uint8 DRDY_OTUT			:1; //Indicates the OTUT round robin has run at least once.
				uint8 SPARE 				:3;
			}Bit;
			uint8 Byte;
		}DIAG_STAT;
		
		union
		{
			struct
			{
				uint8 DRDY_MAIN_ADC		:1; //Device has completed at least a single measurement on all Main ADC
				uint8 DRDY_AUX_MISC		:1; //Device has completed at least a single measurement on all AUX ADC MISC
				uint8 DRDY_AUX_CELL		:1; //Device has completed at least a single measurement on all AUXCELL
				uint8 DRDY_AUX_GPIO		:1; //AUX ADC has completed at least a single measurement on all active GPIO
				uint8 SPARE 				:4;
			}Bit;
			uint8 Byte;
		}ADC_STAT1;
		
		union
		{
			struct
			{
				uint8 DRDY_VCCB			:1; //Device has finished VCELL vs. AUXCELL diagnostic comparison
				uint8 DRDY_CBFET			:1; //Device has finished CB FET diagnostic comparison
				uint8 DRDY_CBOW			:1; //Device has finished CB OW diagnostic comparison
				uint8 DRDY_VCOW			:1; //Device has finished VC OW diagnostic comparison
				uint8 DRDY_GPIO 			:1; //Device has finished the GPIO Main and AUX ADC diagnostic comparisons
				uint8 DRDY_LPF				:1; //Device has finished at least 1 round of LPF checks
				uint8 SPARE 				:2;
			}Bit;
			uint8 Byte;
		}ADC_STAT2;
		
		union
		{
			struct
			{
				uint8 GPIO1				:1; //When GPIO is configured as digital input or output, this register shows the GPIO status
				uint8 GPIO2				:1;
				uint8 GPIO3				:1;
				uint8 GPIO4				:1;
				uint8 GPIO5 				:1;
				uint8 GPIO6				:1;
				uint8 GPIO7 				:1;
				uint8 GPIO8 				:1;
			}Bit;
			uint8 Byte;
		}GPIO_STAT;
		
		union
		{
			struct
			{
				uint8 CB_DONE				:1; //Indicates all cell balancing is completed
				uint8 MB_DONE				:1; //Indicates module balancing is completed
				uint8 ABORTFLT				:1; //Indicates cell balancing is aborted due to detection of unmasked fault.
				uint8 CB_RUN				:1; //Indicates cell balancing is running
				uint8 MB_RUN 				:1; //Indicates module balancing, controlled by the device, is running
				uint8 CB_INPAUSE			:1; //Indicates the cell balancing pause status
				uint8 OT_PAUSE_DET			:1; //Indicates the OTCB is detected if [OTCB_EN] = 1
				uint8 INVALID_CBCONF		:1; //Indicates CB is unable to start (after [BAL_GO] = 1) due to improper CB control settings.
			}Bit;
			uint8 Byte;
		}BAL_STAT;
		
		union
		{
			struct
			{
				uint8 MAIN_RUN				:1; //Shows the status of the Main ADC.
				uint8 AUX_RUN				:1; //Shows the status of the AUX ADC
				uint8 RSVD					:1;
				uint8 OVUV_RUN				:1; //Shows the status of the OVUV protector comparators
				uint8 OTUT_RUN				:1; //Shows the status of the OTUT protector comparators.
				uint8 CUST_CRC_DONE		:1; //Indicates module balancing, controlled by the device, is running
				uint8 FACT_CRC_DONE		:1; //Indicates the status of the factory CRC state machine.
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}DEV_STAT;
		
		union
		{
			struct
			{
				uint8 LPF_VCELL			:3; //Configures the post ADC low-pass filter cut-off frequency for VCELL measurement
				uint8 LPF_BB				:3; //Configures the post ADC low-pass filter cut-off frequency for bus bar measurement
				uint8 AUX_SETTLE			:2; //The AUXCELL configures the AUX CELL settling time
			}Bit;
			uint8 Byte;
		}ADC_CONF1;
		
		union
		{
			struct
			{
				uint8 ADC_DLY  			:6; //If [MAIN_GO] bit is written to 1, bit Main ADC is delayed for this setting time before being enabled to start the conversion.
				uint8 SPARE				:2;
			}Bit;
			uint8 Byte;
		}ADC_CONF2;

		uint8  MAIN_ADC_GAIN;		//Main ADC 25C gain calibration result
		uint8  MAIN_ADC_OFFSET;	//Main ADC 25C offset calibration result.
		uint8  AUX_ADC_GAIN;		//8-bit register for AUX ADC gain correction.
		uint8  AUX_ADC_OFFSET;		//8-bit register for AUX ADC offset correction
		
		union
		{
			struct
			{
				uint8 MAIN_MODE			:2; //Sets the Main ADC run mode.
				uint8 MAIN_GO				:1; //Starts main ADC conversion.
				uint8 LPF_VCELL_EN			:1; //Enables digital low-pass filter post-ADC conversion
				uint8 LPF_BB_EN			:1; //Enables digital low-pass filter post-ADC conversion
				uint8 SPARE				:3;
			}Bit;
			uint8 Byte;
		}ADC_CTRL1;
		
		union
		{
			struct
			{
				uint8 AUX_CELL_SEL			:5; //Selects which AUXCELL channel(s) will be multiplexed through the AUX ADC.
				uint8 SPARE				:3;
			}Bit;
			uint8 Byte;
		}ADC_CTRL2;
		
		union
		{
			struct
			{
				uint8 AUX_MODE				:2; //Sets the Main ADC run mode
				uint8 AUX_GO				:1; //Starts AUX ADC conversion
				uint8 AUX_GPIO_SEL			:4; //Selects which GPIO channel(s) will be multiplexed
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}ADC_CTRL3;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}VCELL[16];
			
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}BUSBAR;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}TSREF;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}GPIO[8];	
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}DIETEMP[2];	
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_CELL;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_GPIO;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_BAT;
			
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_REFL;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_VBG2;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_LPBG5;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_AVAO_REF;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_AVDD_REF;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_OV_DAC;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_UV_DAC;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_OT_OTCB_DAC;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_UT_DAC;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_VCBDONE_DAC;
		
		union
		{
			struct
			{
				uint8 HI;
				uint8 LO;
			}Byte;
			uint16 Word;
		}AUX_VCM[2];
			
		union
		{
			struct
			{
				uint8 TIME					:5; //Sets the timer for cell* balancing
				uint8 SPARE				:3;
			}Bit;
			uint8 Byte;
		}CB_CELL_CTRL[16];
		
		union
		{
			struct
			{
				uint8 MB_THR				:6; //If MB_TIMER_CTRL is not 0x00 and BAT voltage is less than this threshold, the module balancing through GPIO3 stops
				uint8 SPARE				:2;
			}Bit;
			uint8 Byte;
		}VMB_DONE_THRESH;
		
		union
		{
			struct
			{
				uint8 TIME					:5; //If MB_TIMER_CTRL is not 0x00 and BAT voltage is less than this threshold, the module balancing through GPIO3 stops
				uint8 SPARE				:3;
			}Bit;
			uint8 Byte;
		}MB_TIMER_CTRL;
		
		union
		{
			struct
			{
				uint8 TIME					:5; //If a cell voltage is less than this threshold, the cell balancing on that cell stops.
				uint8 SPARE				:3;
			}Bit;
			uint8 Byte;
		}VCB_DONE_THRESH;
		
		union
		{
			struct
			{
				uint8 OTCB_THR				:5; //Sets the OTCB threshold when BAL_CTRL1[OTCB_EN] = 1
				uint8 COOLOFF				:3; //Sets the COOLOFF hysteresis
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}OTCB_THRESH;
		
		union
		{
			struct
			{
				uint8 DUTY					:3; //Selection is sampled whenever [BAL_GO] = 1 is set by the host MCU.
				uint8 SPARE				:5;
			}Bit;
			uint8 Byte;
		}BAL_CTRL1;
		
		union
		{
			struct
			{
				uint8 AUTO_BAL				:1; //Selects between auto or manual cell balance control.
				uint8 BAL_GO				:1; //Starts cell or module balancing.
				uint8 BAL_ACT				:2; //Controls the device action when the MB and CB are completed
				uint8 OTCB_EN				:1; //Enables the OTCB detection during cell balancing
				uint8 FLTSTOP_EN			:1; //Stops cell or module balancing if unmasked fault occurs
				uint8 CB_PAUSE				:1; //Pauses cell balancing on all cells to allow diagnostics to run
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}BAL_CTRL2;
		
		uint8 CB_COMPLETE1;	//Cell balance completion for cell9 to cell16.
		uint8 CB_COMPLETE2;  //Cell balance completion for cell1 to cell8.
		
		union
		{
			struct
			{
				uint8 OV_THR				:6; //Sets the overvoltage threshold for the OV comparator
				uint8 SPARE				:2;
			}Bit;
			uint8 Byte;
		}OV_THRESH;			//alone function
		
		union
		{
			struct
			{
				uint8 UV_THR				:6; //Sets the undervoltage threshold for the UV comparator.
				uint8 SPARE				:2;
			}Bit;
			uint8 Byte;
		}UV_THRESH;		//alone function
		
		union
		{
			struct
			{
				uint8 OT_THR				:5; //Sets the OT threshold for the OT comparator.
				uint8 UT_THR				:3; //Sets the UT threshold for the UT comparator
			}Bit;
			uint8 Byte;
		}OTUT_THRESH;
		
		union
		{
			struct
			{
				uint8 OVUV_MODE			:2; //Sets the OV and UV comparators operation mode when [OVUV_GO] = 1
				uint8 OVUV_GO				:1; //Starts the OV and UV comparators.
				uint8 OVUV_LOCK			:4; //Configures a particular single channel as the OV and UV comparators input when [OVUV_MOD1:0] = 0b11
				uint8 VCBDONE_THR_LOCK		:1;	//As the UV comparator is switching between UV threshold and VCBDONE threshold to measure the UV DAC or the VCBDONE DAC result for diagnostics
			}Bit;
			uint8 Byte;
		}OVUV_CTRL;
		
		union
		{
			struct
			{
				uint8 OTUT_MODE			:2; //Sets the OT and UT comparators operation mode when [OTUT_GO] = 1
				uint8 OTUT_GO				:1; //Starts the OT and UT comparators
				uint8 OTUT_LOCK			:4; //Configures a particular single channel as the OT and UT comparators input when [OTUT_MOD1:0] = 0b11.
				uint8 OTCB_THR_LOCK		:1; //As the OT comparator is switching between OT threshold and OTCB threshold to measure the OT or OTCB
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}OTUT_CTRL;
		
		union
		{
			struct
			{
				uint8 GPIO1				:3; //Configures GPIO1.
				uint8 GPIO2				:3; //Configures GPIO2.
				uint8 SPI_EN				:1; //Enables SPI master on GPIO4, GPIO5 and GPIO6, GPIO7
				uint8 FAULT_IN_EN			:1; //Enables GPIO8 as an active-low input to trigger the NFAULT pin when the input signal is low.
			}Bit;
			uint8 Byte;
		}GPIO_CONF1;
		
		union
		{
			struct
			{
				uint8 GPIO3				:3; //Configures GPIO3.
				uint8 GPIO4				:3; //Configures GPIO4.
				uint8 RSVD					:1;
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}GPIO_CONF2;
		
		union
		{
			struct
			{
				uint8 GPIO5				:3; //Configures GPIO5.
				uint8 GPIO6				:3; //Configures GPIO6.
				uint8 RSVD					:1;
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}GPIO_CONF3;
		
		union
		{
			struct
			{
				uint8 GPIO7				:3; //Configures GPIO7.
				uint8 GPIO8				:3; //Configures GPIO8.
				uint8 RSVD					:1;
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}GPIO_CONF4;	
		
		union
		{
			struct
			{
				uint8 NUMBIT				:5; //SPI transaction length.
				uint8 CPHA					:1; //Sets the edge of SCLK where data is sampled on MISO.
				uint8 CPOL					:1; //Sets the SCLK polarity.
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}SPI_CONF;
		
		union
		{
			struct
			{
				uint8 SPI_GO				:1; //Executes the SPI transaction. T
				uint8 SS_CTRL				:1; //Programs the state of SS
				uint8 SPARE				:6;
			}Bit;
			uint8 Byte;
		}SPI_EXE;
		
		uint8 SPI_TX[3];	//Data to be used to write to SPI slave device.
		uint8 SPI_RX[3];	//Data returned from a read during SPI transaction.
		
		union
		{
			struct
			{
				uint8 MARGIN_GO			:1; //Starts OTP Margin test set by the [MARGIN_MOD] bit.
				uint8 MARGIN_MODE			:3; //Configures OTP Margin read mode:
				uint8 FLIP_FACT_CRC		:1; //An enable bit to flip the factory CRC value.
				uint8 SPARE				:4;
			}Bit;
			uint8 Byte;
		}DIAG_OTP_CTRL;
		
		union
		{
			struct
			{
				uint8 FLIP_TR_CRC			:1; //Sends a purposely incorrect communication (during transmitting response) CRC by inverting all of the calculated CRC bits..
				uint8 SPI_LOOPBACK			:1; //Enables SPI loopback function to verify SPI functionality
				uint8 SPARE				:6;
			}Bit;
			uint8 Byte;
		}DIAG_COMM_CTRL;
		
		union
		{
			struct
			{
				uint8 PWR_BIST_GO			:1; //When written to 1, the power supply BIST diagnostic will start.
				uint8 BIST_NO_RST			:1; //Use for further diagnostic if the power supply BIST detects a failure
				uint8 SPARE				:6;
			}Bit;
			uint8 Byte;
		}DIAG_PWR_CTRL;
		
		uint8 DIAG_CBFET_CTRL1;	//Enables CBFET for CBFET diagnostic CELL16-9
		uint8 DIAG_CBFET_CTRL2;	//Enables CBFET for CBFET diagnostic CELL8-1
		
		union
		{
			struct
			{
				uint8 BB_THR			:3; //Additional delta value added to the VCCB_THR setting,
				uint8 GPIO_THR			:3; //Configures the GPIO comparison delta threshold between Main and AUX ADC measurements
				uint8 SPARE			:2;
			}Bit;
			uint8 Byte;
		}DIAG_COMP_CTRL1;
		
		union
		{
			struct
			{
				uint8 OW_THR			:4; //Configures the OW detection threshold for diagnostic comparison.
				uint8 VCCB_THR			:4; //Configures the VCELL vs. AUXCELL delta. The VCELL vs. AUXCELL check is considered pass
			}Bit;
			uint8 Byte;
		}DIAG_COMP_CTRL2;
		
		union
		{
			struct
			{
				uint8 COMP_ADC_GO		:1; //Device starts diagnostic test specified by [COMP_ADC_SEL2:0] setting.
				uint8 COMP_ADC_SEL		:3; //Enables the device diagnostic comparison
				uint8 OW_SNK			:2; //Turns on current sink on VC pins, CB pins, or BBP/N pins.
				uint8 EXTD_CBFET   	:1; //When CBFET check comparison is selected, [COMP_ADC_SEL2:0] = 0b101, device turns off all CBFET upon
																	//the completion of the comparison. If this bit is set to 1, the device will put the CBFET check in an extended mode
																	//in which the device will NOT turn off the CBFETs.
				uint8 SPARE			:1;
			}Bit;
			uint8 Byte;
		}DIAG_COMP_CTRL3;
		
		union
		{
			struct
			{
				uint8 LPF_FAULT_INJ	:1; //Injects fault condition to the diagnostic LPF during LPF diagnostic
				uint8 COMP_FAULT_INJ	:1; //Injects fault to the ADC comparison logic.
				uint8 SPARE			:6;
			}Bit;
			uint8 Byte;
		}DIAG_COMP_CTRL4;
		
		union
		{
			struct
			{
				uint8 PROT_BIST_NO_RST		:1; //Use for further diagnostic if the protector BIST detects a failure
				uint8 SPARE				:7;
			}Bit;
			uint8 Byte;
		}DIAG_PROT_CTRL;
		
		union
		{
			struct
			{
				uint8 MSK_PWR				:1; //To mask the NFAULT assertion from any FAULT_PWR1 to FAULT_PWR3 register bit
				uint8 MSK_SYS				:1;	//To mask the NFAULT assertion from any FAULT_SYS register bit.
				uint8 MSK_COMP				:1; //Masks the FAULT_COMP_* registers to trigger NFAULT
				uint8 MSK_OV				:1; //Masks the FAULT_OV* registers to trigger NFAULT
				uint8 MSK_UV				:1; //Masks the FAULT_UV* registers to trigger NFAULT
				uint8 MSK_OT				:1; //Masks the FAULT_OT* registers to trigger NFAULT.
				uint8 MSK_UT				:1; //Masks the FAULT_UT* registers to trigger NFAULT.
				uint8 MSK_PROT				:1; //Masks the FAULT_PROT* registers to trigger NFAULT
			}Bit;
			uint8 Byte;
		}FAULT_MSK1;
		
		union
		{
			struct
			{
				uint8 MSK_COMM1			:1; //Masks FAULT_COMM1 register on NFAULT triggering.
				uint8 MSK_COMM2			:1;	//Masks FAULT_COMM2 register on NFAULT triggering
				uint8 MSK_COMM3_HB			:1; //Masks FAULT_COMM3[HB_FAST] or [HB_FAIL] faults on NFAULT triggering.
				uint8 MSK_COMM3_FTONE		:1; //Masks FAULT_COMM3[FTONE_DET] fault on NFAULT triggering.
				uint8 MSK_COMM3_FCOMM		:1; //Masks FAULT_COMM3[FCOMM_DET] fault on NFAULT triggering
				uint8 MSK_OTP_DATA			:1; //Masks the FAULT_OTP register (all bits except [CUST_CRC] and [FACT_CRC]) on NFAULT triggering
				uint8 MSK_OTP_CRC			:1; //Masks the FAULT_OTP register ([CUST_CRC] and [FACT_CRC] only) on NFAULT triggering.
				uint8 SPARE				:1;
			}Bit;
			uint8 Byte;
		}FAULT_MSK2;
		
		union
		{
			struct
			{
				uint8 RST_PWR				:1; //To reset the FAULT_PWR1 to FAULT_PWR3 registers to 0x00
				uint8 RST_SYS				:1;	//To reset the FAULT_SYS register to 0x00
				uint8 RST_COMP				:1; //Resets all FAULT_COMP_* registers to 0x00.
				uint8 RST_OV				:1; //Resets all FAULT_OV* registers to 0x00
				uint8 RST_UV				:1; //Resets all FAULT_UV* registers to 0x00
				uint8 RST_OT				:1; //Resets all FAULT_OT registers to 0x00
				uint8 RST_UT				:1; //Resets all FAULT_UT registers to 0x00
				uint8 RST_PROT				:1; //Resets the FAULT_PROT1 and FAULT_PROT2 registers to 0x00.
			}Bit;
			uint8 Byte;
		}FAULT_RST1;
		
		union
		{
			struct
			{
				uint8 RST_COMM1			:1; //Resets FAULT_COMM1 and DEBUG_COMM_UART* registers.
				uint8 RST_COMM2			:1;	//Resets FAULT_COMM2, DEBUG_COML*, and DEBUG_COMM_COMH* registers
				uint8 RST_COMM3_HB			:1; //Resets FAULT_COMM3[HB_FAST] and [HB_FAIL] bits.
				uint8 RST_COMM3_FTONE		:1; //Resets FAULT_COMM3[FTONE_DET].
				uint8 RST_COMM3_FCOMM		:1; //Resets FAULT_COMM3[FCOMM_DET]
				uint8 RST_OTP_DATA			:1; //Resets the FAULT_OTP register ([SEC_DETECT] and [DED_DETECT] only).
				uint8 RST_OTP_CRC			:1; //Resets the FAULT_OTP register ([CUST_CRC] and [FACT_CRC] only).
				uint8 RSVD					:1;
			}Bit;
			uint8 Byte;
		}FAULT_RST2;
		
		union
		{
			struct
			{
				uint8 FAULT_PWR			:1; //This bit is set if [MSK_PWR] = 0 and any of the FAULT_PWR1 to FAULT_PWR3 register bits is set.
				uint8 FAULT_SYS			:1;	//This bit is set if [MSK_SYS] = 0 and any of the FAULT_SYS1 register bits is set
				uint8 FAULT_OVUV			:1; //This bit is set if any of the following is true:
				uint8 FAULT_OTUT			:1; //
				uint8 FAULT_COMM			:1; //
				uint8 FAULT_OTP			:1; //This bit is set if [MSK_OTP] = 0 and any of the FAULT_OTP register bits is set.
				uint8 FAULT_COMP_ADC		:1; //Resets the FAULT_OTP register ([CUST_CRC] and [FACT_CRC] only).
				uint8 FAULT_PROT			:1; //This bit is set if [MSK_PROT] = 0 and any of the FAULT_PROT1 or FAULT_PROT2 register bits is set.
			}Bit;
			uint8 Byte;
		}FAULT_SUMMARY;
		
		union
		{
			struct
			{
				uint8 STOP_DET				:1; //Indicates an unexpected STOP condition is received.
				uint8 COMMCLR_DET			:1;	//A UART communication clear signal is detected
				uint8 UART_RC				:1; //Indicates a UART FAULT is detected during receiving a command frame
				uint8 UART_RR				:1; //Indicates a UART FAULT is detected when receiving a response frame
				uint8 UART_TR				:1; //Indicates a UART FAULT is detected when transmitting a response frame.
				uint8 RSVD					:3;
			}Bit;
			uint8 Byte;
		}FAULT_COMM1;
		
		union
		{
			struct
			{
				uint8 COMH_BIT				:1; //Indicates a COMH bit level fault is detected
				uint8 COMH_RC				:1;	//Indicates a COMH byte level fault is detected
				uint8 COMH_RR				:1; //Indicates a COMH byte level fault is detected when receiving a response frame
				uint8 COMH_TR				:1; //Indicates a COMH byte level fault is detected when transmitting a response frame
				uint8 COML_BIT				:1; //Indicates a COML bit level fault is detected which would cause at least one byte level fault.
				uint8 COML_RC				:1; //Indicates a COML byte level fault is detected when receiving a command frame
				uint8 COML_RR				:1; //Indicates a COML byte level fault is detected when receiving a response frame
				uint8 COML_TR				:1; //Indicates a COML byte level fault is detected when transmitting a response frame
			}Bit;
			uint8 Byte;
		}FAULT_COMM2;
		
		union
		{
			struct
			{
				uint8 HB_FAST				:1; //Indicates HEARTBEAT is received too frequently
				uint8 HB_FAIL				:1;	//Indicates HEARTBEAT is not received within an expected time
				uint8 FTONE_DET			:1; //Indicates a FAULT TONE is received
				uint8 FCOMM_DET			:1; //Received communication transaction with the Fault Status bits set by any of the upper stack device
				uint8 RSVD					:4;
			}Bit;
			uint8 Byte;
		}FAULT_COMM3;
		
		union
		{
			struct
			{
				uint8 GBLOVERR				:1; //Indicates that on overvoltage error is detected on one of the OTP pages.
				uint8 FACTLDERR			:1; //Indicates errors during the factory space OTP load process.
				uint8 CUSTLDERR			:1;	//Indicates errors during the customer space OTP load process
				uint8 FACT_CRC				:1; //Indicates a CRC error has occurred in the factory register space.
				uint8 CUST_CRC				:1; //Indicates a CRC error has occurred in the customer register space
				uint8 SEC_DET				:1; //Indicates a SEC error has occurred during the OTP load.
				uint8 DED_DET				:1; //Indicates a DED error has occurred during the OTP load
				uint8 RSVD					:1;
			}Bit;
			uint8 Byte;
		}FAULT_OTP;
		
		union
		{
			struct
			{
				uint8 TWARN				:1; //Indicates the die temperature (die temp 2) is higher than the TWARN_THR[1:0] setting
				uint8 TSHUT				:1; //Indicates the previous shutdown was a thermal shutdown,
				uint8 CTS					:1;	//Indicates a short communication timeout occurred.
				uint8 CTL					:1; //Indicates a long communication timeout occurred
				uint8 DRST					:1; //Indicates a digital reset has occurred
				uint8 GPIO					:1; //Indicates GPIO8 detects a FAULT input when GPIO_CONF1[FAULT_IN_EN] = 1.
				uint8 SHUTDOWN_REC			:1; //Indicates the previous device shutdown was caused by one of the following:
				uint8 LFO					:1; //Indicated LFO frequency is outside an expected range
			}Bit;
			uint8 Byte;
		}FAULT_SYS;
		
		union
		{
			struct
			{
				uint8 VPARITY_FAIL			:1; //Indicates a parity fault is detected on any of the following OVUV related configurations
				uint8 TPARITY_FAIL			:1; //Indicates a parity fault is detected on any of the following OTUT related configurations
				uint8 RSVD					:6;
			}Bit;
			uint8 Byte;
		}FAULT_PROT1;
		
		union
		{
			struct
			{
				uint8 UVCOMP_FAIL			:1; //Indicates the UV comparator fails during BIST test
				uint8 OVCOMP_FAIL			:1; //Indicates the OV comparator fails during BIST test.
				uint8 OTCOMP_FAIL			:1;	//Indicates the OT comparator fails during BIST test
				uint8 UTCOMP_FAIL			:1; //Indicates the UT comparator fails during BIST test.
				uint8 VPATH_FAIL			:1; //Indicates a fault is detected along the OVUV signal path during BIST test
				uint8 TPATH_FAIL			:1; //Indicates a fault is detected along the OTUT signal path during BIST test
				uint8 BIST_ABORT			:1; //Indicates either OVUV or OTUT BIST run is aborted
				uint8 RSVD					:1;
			}Bit;
			uint8 Byte;
		}FAULT_PROT2;
		
		uint8 FAULT_OV1;					//OV9_DET to OV16_DET = OV fault status
		uint8 FAULT_OV2;					//OV1_DET to OV8_DET = OV fault status
		uint8 FAULT_UV1;					//UV9_DET to UV16_DET = UV fault status
		uint8 FAULT_UV2;					//UV1_DET to UV8_DET = UV fault status
		uint8 FAULT_OT;					//OT1_DET to OT8_DET = OT fault status
		uint8 FAULT_UT;					//UT1_DET to UT8_DET = UT fault status
		uint8 FAULT_COMP_GPIO;				//Indicates ADC vs. AUX ADC GPIO measurement diagnostic results for GPIO1 to GPIO8.
		uint8 FAULT_COMP_VCCB1;			//Indicates voltage diagnostic results for cell9 to cell16.
		uint8 FAULT_COMP_VCCB2;			//Indicates voltage diagnostic results for cell1 to cell8.
		uint8 FAULT_COMP_VCOW1;			//Indicates VC OW diagnostic results for cell9 to cell 16.
		uint8 FAULT_COMP_VCOW2;			//Indicates VC OW diagnostic results for cell1 to cell 8
		uint8 FAULT_COMP_CBOW1;			//Results of the CB OW diagnostic for CB FET9 to CB FET16
		uint8 FAULT_COMP_CBOW2;			//Results of the CB OW diagnostic for CB FET1 to CB FET8.
		uint8 FAULT_COMP_CBFET1;			//Results of the CB FET diagnostic for CB FET9 to CB FET16
		uint8 FAULT_COMP_CBFET2;			//Results of the CB FET diagnostic for CB FET1 to CB FET8.
				
		union
		{
			struct
			{
				uint8 LPF_FAIL				:1; //Indicates LPF diagnostic result
				uint8 COMP_ADC_ABORT		:1; //Indicates the most recent ADC comparison diagnostic is aborted due to improper setting
				uint8 RSVD					:6;
			}Bit;
			uint8 Byte;
		}FAULT_COMP_MISC;
			
		union
		{
			struct
			{
				uint8 AVDD_OV				:1; //Indicates an overvoltage fault on the AVDD LDO.
				uint8 AVDD_OSC				:1; //Indicates AVDD is oscillating outside of acceptable limits.
				uint8 DVDD_OV				:1;	//Indicates an overvoltage fault on the DVDD LDO
				uint8 CVDD_OV				:1; //Indicates an overvoltage fault on the CVDD LDO
				uint8 CVDD_UV				:1; //Indicates an undervoltage fault on the CVDD LDO
				uint8 REFHM_OPEN			:1; //Indicates an open condition on REFHM pin
				uint8 DVSS_OPEN			:1; //Indicates an open condition on DVSS pin
				uint8 CVSS_OPEN			:1; //Indicates an open condition on CVSS pin
			}Bit;
			uint8 Byte;
		}FAULT_PWR1;
		
		union
		{
			struct
			{
				uint8 TSREF_OV				:1; //Indicates an overvoltage fault on the TSREF LDO.
				uint8 TSREF_UV				:1; //Indicates an undervoltage fault on the TSREF LDO.
				uint8 TSREF_OSC			:1;	//Indicates TSREF is oscillating outside of an acceptable limit
				uint8 NEG5V_UV				:1; //Indicates an undervoltage fault on the NEG5V charge pump
				uint8 REFH_OSC				:1; //Indicates REGH reference is oscillating outside of an acceptable limit.
				uint8 AVAO_OV				:1; //Indicates an overvoltage fault on the AVAO_REF
				uint8 PWRBIST_FAIL			:1; //Indicates a fail on the power supply BIST run
				uint8 RSVD					:1;
			}Bit;
			uint8 Byte;
		}FAULT_PWR2;
		
		union
		{
			struct
			{
				uint8 AVDDUV_DRST			:1; //Indicates a digital reset occurred due to AVDD UV detected.
				uint8 AVDDREFUV_DRST		:1; //Indicates a digital reset occurred due to AVDDREF UV detected
				uint8 AVAO_SW_FAIL			:1;	//Indicates a fault is detected on the AVAO switch.
				uint8 RSVD					:5;
			}Bit;
			uint8 Byte;
		}FAULT_PWR3;
			
		uint8 DEBUG_CTRL_UNLOCK;		//Write the unlock code (0xA5) to this register to activate the setting in the DEBUG_COMM_CTRL* register
			
		union
		{
			struct
			{
				uint8 USER_DAISY_EN		:1; //This bit enables the debug COML and COMH control bits in the DEBUG_COMM_CTRL2 register
				uint8 USER_UART_EN 		:1; //This bit enables the debug UART control bits
				uint8 UART_TX_EN			:1;	//Stack device, by default, has the UART TX disabled
				uint8 UART_MIRROR_EN		:1; //This bit enables the stack VIF communication to mirror to UART
				uint8 UART_BAUD			:1; //This bit changes the UART baud rate to 250kb/s.
				uint8 RSVD					:3;
			}Bit;
			uint8 Byte;
		}DEBUG_COMM_CTRL1;
		
		union
		{
			struct
			{
				uint8 COMH_RX_EN			:1; //Enables COMH receiver.
				uint8 COMH_TX_EN		 	:1; //Enables COMH transmitter
				uint8 COML_RX_EN			:1;	//Enables COML receiver
				uint8 COML_TX_EN			:1; //Enables COML transmitter
				uint8 RSVD					:4;
			}Bit;
			uint8 Byte;
		}DEBUG_COMM_CTRL2;
			
		union
		{
			struct
			{
				uint8 COMH_RX_ON			:1; //Shows the current COMH receiver status.
				uint8 COMH_TX_ON		 	:1; //Shows the current COMH transmitter status.
				uint8 COML_RX_ON			:1;	//Shows the current COML receiver status
				uint8 COML_TX_ON			:1; //Shows the current COML transmitter status
				uint8 HW_DAISY_DRV			:1; //Indicates the COML and COMH are controlled by the device itself or by MCU control.
				uint8 HW_UART_DRV			:1;	//Indicates the UART TX is controlled by the device itself or by MCU control
				uint8 RSVD					:2;
			}Bit;
			uint8 Byte;
		}DEBUG_COMM_STAT;
		
		union
		{
			struct
			{
				uint8 RC_CRC				:1; //Detects a CRC error in the received command frame from UART
				uint8 RC_UNEXP			 	:1; //In a stack device, it is not expected to receive a stack or broadcast command through the UART interface.
				uint8 RC_BYTE_ERR			:1;	//Detects any byte error, other than the error in the initialization byte, in the received command frame
				uint8 RC_SOF				:1; //Detects a start-of-frame (SOF) error
				uint8 RC_TXDIS				:1; //Detects if UART TX is disabled, but the host MCU has issued a command to read data from the device.
				uint8 RC_IERR				:1;	//Detects initialization byte error in the received command frame
				uint8 RSVD					:2;
			}Bit;
			uint8 Byte;
		}DEBUG_UART_RC;	
		
		union
		{
			struct
			{
				uint8 RR_CRC				:1; //Detects are CRC error in the received response frame from UART
				uint8 RR_BYTE_ERR	 		:1; //Detects any byte error,
				uint8 RR_SOF				:1;	//Indicates a UART CLEAR is received while receiving the response frame
				uint8 TR_WAIT				:1; //The device is waiting for its turn to transfer a response out but the action is terminated
				uint8 TR_SOF				:1; //Indicates that a UART CLEAR is received while the device is still transmitting data
				uint8 RSVD					:3;
			}Bit;
			uint8 Byte;
		}DEBUG_UART_RR_TR;
		
		union
		{
			struct
			{
				uint8 BIT					:1; //The device has detected a data bit; however,
																	//the detection samples are not enough to assure a strong 1 or 0
				uint8 SYNC1				:1; //Unable to detect the preamble half-bit or any of the [SYNC1:0] bits
				uint8 SYNC2				:1;	//The Preamble half-bit and the [SYNC1:0] bits are detected
				uint8 BERR_TAG				:1; //Set when the received communication is tagged with [BERR] = 1.
				uint8 PERR					:1; //Detects abnormality of the incoming communication frame and hence
				uint8 RSVD					:3;
			}Bit;
			uint8 Byte;
		}DEBUG_COMH_BIT;
		
		union
		{
			struct
			{
				uint8 RC_CRC				:1; //Indicates a CRC error that resulted in one or more COMH command frames being discarded
				uint8 RC_UNEXP			 	:1; //If [DIR_SEL] = 0, but device receives command frame from COMH
																	//which is an invalid condition and device will set this error bit.
				uint8 RC_BYTE_ERR			:1;	//Valid when [DIR_SEL] = 1. Detected any byte error,
																	//other than the error in the initialization byte, in the received command frame
				uint8 RC_SOF				:1; //Valid when [DIR_SEL] = 1. Detects a start-of-frame (SOF) error on COMH. T
				uint8 RC_TXDIS				:1; //Valid when [DIR_SEL] = 1. Device detects the COMH TX is disabled but the device receives a command to read
																	//data (that is, to transmit data out). The command frame will be counted as a discard frame
				uint8 RC_IERR				:1; //...
				uint8 RSVD					:2;
			}Bit;
			uint8 Byte;
		}DEBUG_COMH_RC;
			
		union
		{
			struct
			{
				uint8 RR_CRC				:1; //Indicates a CRC error that resulted in one or more COMH response frames being discarded
				uint8 RR_UNEXP			 	:1; //If [DIR_SEL] = 1, but device received response frame from COMH
																	//which is an invalid condition and device sets this error bit.
				uint8 RR_BYTE_ERR			:1;	//Valid when [DIR_SEL] = 0. Detects any byte error, other than the error in the initialization byte, in the received
																	//response frame. This error can trigger one or more error bits set in the DEBUG_COMMH_BIT register.
				uint8 RR_SOF				:1; //Valid when [DIR_SEL] = 0. Detects a start-of-frame (SOF) error on COMH
				uint8 RR_TXDIS				:1; //Valid when [DIR_SEL] = 0, device receives a response but fails to transmit to the next device because the COMH
																	//TX is disabled. The frame is counted as a discarded frame.
				uint8 TR_WAIT				:1; //The device is waiting for its turn to transfer a response out
																	//but the action is terminated because the devicereceives a new command.
				uint8 RSVD					:2;
			}Bit;
			uint8 Byte;
		}DEBUG_COMH_RR_TR;
		
		
		union
		{
			struct
			{
				uint8 BIT					:1; //The device has detected a data bit.
				uint8 SYNC1				:1; //Unable to detect the preamble half-bit or any of the [SYNC1:0] bits.
				uint8 SYNC2				:1;	//The Preamble half-bit and the [SYNC1:0] bits are detected
				uint8 BERR_TAG				:1; //Set when the received communication is tagged with BERR
				uint8 PERR					:1; //Detect abnormality of the incoming communication frame
																	//and the outgoing communication frame will be set with	BERR.
				uint8 RSVD					:3;
			}Bit;
			uint8 Byte;
		}DEBUG_COML_BIT;	
		
		union
		{
			struct
			{
				uint8 RC_CRC				:1; //Indicates a CRC error that resulted in one or more COML command frames being discarded.
				uint8 RC_UNEXP			 	:1; //If [DIR_SEL] = 1, but device received command frame from COML which is an invalid condition
																	//and device will set this error bit.
				uint8 RC_BYTE_ERR			:1;	//Valid when [DIR_SEL] = 0. Detected any byte error,
																	//other than the error in the initialization byte, in the received command frame.
				uint8 RC_SOF				:1; //Valid when [DIR_SEL] = 0.
				uint8 RC_TXDIS				:1; //Valid when [DIR_SEL] = 0.
				uint8 RC_IERR				:1; //Detected initialization byte error in the received command frame.
				uint8 RSVD					:2;
			}Bit;
			uint8 Byte;
		}DEBUG_COML_RC;
		
		union
		{
			struct
			{
				uint8 RR_CRC				:1; //Indicates a CRC error that resulted in one or more COML response frames being discarded.
				uint8 RR_UNEXP			 	:1; //If [DIR_SEL] = 0, but device received a response frame from COML
																	//which is an invalid condition and device will set this error bit.
				uint8 RR_BYTE_ERR			:1;	//Valid when [DIR_SEL] = 1
				uint8 RR_SOF				:1; //Valid when [DIR_SEL] = 1.
				uint8 RR_TXDIS				:1; //Valid when [DIR_SEL] = 1
				uint8 TR_WAIT				:1; //The device is waiting for its turn to transfer a response out
																	//but the action is terminated because the device receives a new command
				uint8 RSVD					:2;
			}Bit;
			uint8 Byte;
		}DEBUG_COML_RR_TR;
		
		uint8 DEBUG_UART_DISCARD; 		//UART frame counter to track the number of discard frames received or transmitted.
		uint8 DEBUG_COMH_DISCARD;		//COMH frame counter to track the number of discard frames received or transmitted.
		uint8 DEBUG_COML_DISCARD;		//COML frame counter to track the number of discard frames received or transmitted.
		uint8 DEBUG_UART_VALID_HI;		//The high-byte of UART frame counter to track the number of valid frames received or transmitted.
		uint8 DEBUG_UART_VALID_LO;		//The low-byte of UART frame counter to track the number of valid frames received or transmitted.
		uint8 DEBUG_COMH_VALID_HI;		//The high-byte of COMH frame counter to track the number of valid frames received or transmitted.
		uint8 DEBUG_COMH_VALID_LO;		//The low-byte of COMH frame counter to track the number of valid frames received or transmitted.
		uint8 DEBUG_COML_VALID_H;		//The high-byte of COML frame counter to track the number of valid frames received or transmitted.
		uint8 DEBUG_COML_VALID_LO;		//The low-byte of COML frame counter to track the number of valid frames received or transmitted
		uint8 DEBUG_OTP_SEC_BLK;		//Holds last OTP block address where SEC occurred. Valid only when FAULT_OTP[SEC_DET] = 1.
		uint8 OTP_PROG_UNLOCK1[4];		//programming unlock code is required as part of the OTP programming unlock sequence
									//before performing OTP programming
		uint8 OTP_PROG_UNLOCK2[4]; 	//OTP programming unlock code, required as part of the OTP programming unlock sequence
									//before performing OTP programming
		union
		{
			struct
			{
				uint8 PROG_GO			:1; //Enables programming for the OTP page selected by OTP_PROG_CTRL[PAGESEL].
				uint8 PAGESEL			:1; //Selects which customer OTP page to be programmed.
				uint8 RSVD				:6;
			}Bit;
			uint8 Byte;
		}OTP_PROG_CTRL;
		
		union
		{
			struct
			{
				uint8 ENABLE			:1; //Executes the OTP ECC test configured by [ENC_DEC] and [DED_SEC] bits
				uint8 ENC_DEC			:1; //Sets the encoder/decoder test to run when OTP_ECC_TEST[ENABLE] = 1.
				uint8 MANUAL_AUTO		:1;	//Sets the location of the data to use for the ECC test.
				uint8 DED_SEC			:1; //Sets the decoder function (SEC or DED) to test
				uint8 RSVD				:4;
			}Bit;
			uint8 Byte;
		}OTP_ECC_TEST;
		
		uint8 OTP_ECC_DATAIN[9];		//When ECC is enabled in manual mode, CUST_ECC_TEST[MANUAL_AUTO] = 1, OTP_ECC_DATAIN1?													//registers are used to test the ECC encoder/decoder
		uint8 OTP_ECC_DATAOUT[9];		//OTP_ECC_DATAOUT* bytes output the results of the ECC decoder and encoder tests.
		
		union
		{
			struct
			{
				uint8 DONE				:1; //Indicates the status of the OTP programming for the selected page
				uint8 PROGERR			:1; //Indicates when an error is detected due to incorrect page setting caused by any of the following
				uint8 SOVERR			:1;	//A programming voltage stability test is performed before starting the actual OTP programming
				uint8 SUVERR			:1; //A programming voltage stability test is performed before starting the actual OTP programming
				uint8 OVERR			:1;	//Indicates an overvoltage error detected on the programming voltage during OTP programming
				uint8 UVERR			:1; //Indicates an undervoltage error detected on the programming voltage during OTP programming
				uint8 OTERR			:1;	//Indicates the die temperature is greater than TOTP_PROG and device does not start OTP programming.
				uint8 UNLOCK			:1; //Indicates the OTP programming function unlock status.
			}Bit;
			uint8 Byte;
		}OTP_PROG_STAT;
			
		union
		{
			struct
			{
				uint8 TRY				:1; //Indicates a first programming attempt for OTP page 1
				uint8 OVOK				:1; //Indicates an OTP programming voltage overvoltage condition is detected
																	//during programming attempt for OTP page 1
				uint8 UVOK				:1;	//Indicates an OTP programming voltage undervoltage condition is detected
																	//during programming attempt for OTP page 1
				uint8 PROGOK			:1; //Indicates the validity for loading for OTP page 1. A valid page indicates that successful programming occurred.
				uint8 FMTERR			:1;	//Indicates a formatting error in OTP page 1; that is, when [UVOK] or [OVOK] is set
				uint8 LOADERR			:1; //Indicates an error while attempting to load OTP page 1
				uint8 LOADWRN			:1;	//Indicates OTP page 1 was loaded but with one or more SEC warnings
				uint8 LOADED			:1; //Indicates OTP page 1 has been selected for loading into the related registers.
			}Bit;
			uint8 Byte;
		}OTP_CUST1_STAT;
		
		union
		{
			struct
			{
				uint8 TRY				:1; //Indicates a first programming attempt for OTP page 1.
				uint8 OVOK				:1; //Indicates an OTP programming voltage overvoltage condition is detected
																	//during programming attempt for OTP page 1
				uint8 UVOK				:1;	//Indicates an OTP programming voltage undervoltage condition is detected
																	//during programming attempt for OTP page 1
				uint8 PROGOK			:1; //Indicates the validity for loading for OTP page 1. A valid page indicates that successful programming occurred.
				uint8 FMTERR			:1;	//Indicates a formatting error in OTP page 1; that is, when [UVOK] or [OVOK] is set
				uint8 LOADERR			:1; //Indicates an error while attempting to load OTP page 1
				uint8 LOADWRN			:1;	//Indicates OTP page 1 was loaded but with one or more SEC warnings
				uint8 LOADED			:1; //Indicates OTP page 1 has been selected for loading into the related registers.
			}Bit;
			uint8 Byte;
		}OTP_CUST2_STAT;
		
	}_Bq79616_RegisterGroup;

	extern _Bq79616_RegisterGroup Bq79718_Reg[];
	/**@} */
	uint8 Bq79616_WakeUp(void);     ///< Bring BQ79616 from shutdown or sleep mode into wake-up mode using the WAKEUP command
	uint8 Bq79616_SLP2ACT(void);    ///< Transition from sleep mode to active mode using the SLP2ACT command
	uint8 Bq79616_Main_Config(void);///< Configure registers for the MAIN ADC
	uint8 Bq79616_Main_Mears(void); ///< Perform MAIN ADC measurement
	uint8 Bq79616_AUX_Mears(void);  ///< Perform AUX ADC measurement
	void Moule_Balance_Time(uint16* balance_time, uint16* time_set); ///< Set module balancing time
	uint8 Bq79616_ShutDown(void);
	uint8 Bq79616_Sleep(void);

	/******************** Diagnostic check functions ************************/
	uint8 Cell_Vol_Chek(void);      ///< Compare VC and CB measurement voltages to check if deviation is too large
	uint8 Cell_LPF_Chek(void);      ///< Check the LPF (low-pass filter) measurement
	uint8 Cell_Temp_Chek(void);     ///< Compare temperature values read from GPIO to check if temperature difference is too large (validated using percentage)
	uint8 Balance_FET_Chek(void);   ///< Verify the operation of the cell balancing FETs
	uint8 VC_Open_Chek(uint16* vc_open_flag); ///< Check for open-circuit condition on VC lines
	uint8 CB_Open_Chek(void);       ///< Check for open-circuit condition on CB lines
	void Get_Fault(void);        ///< Read fault/error status bits

#endif
