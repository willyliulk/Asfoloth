#ifndef SX127x_REG_H
#define SX127x_REG_H

/* Common registers */
#define SX127x_Fifo 0x00
#define SX127x_OpMode 0x01
#define SX127x_OpMode_Mode 0x07            //?
#define SX127x_OpMode_LoRaMode 0x80        //?
#define SX127x_FrMsb 0x06
#define SX127x_FrMid 0x07
#define SX127x_FrLsb 0x08
#define SX127x_PaConfig 0x09
#define SX127x_PaRamp 0x0A
#define SX127x_Ocp 0x0B
#define SX127x_Lna 0x0C
#define SX127x_DioMapping1 0x40
#define SX127x_DioMapping2 0x41
#define SX127x_Version 0x42
#define SX127x_Tcxo 0x4B
#define SX127x_PaDac 0x4D
#define SX127x_FormerTemp 0x5B
#define SX127x_AgcRef 0x61
#define SX127x_AgcThresh1 0x62
#define SX127x_AgcThresh2 0x63
#define SX127x_AgcThresh3 0x64
#define SX127x_PLLBW 0x70

/* LoRa registers */
#define SX127x_LoRa_FifoAddrPtr 0x0D
#define SX127x_LoRa_FifoTxBaseAddr 0x0E
#define SX127x_LoRa_FifoRxBaseAddr 0x0F
#define SX127x_LoRa_FifoRxCurrentAddr 0x10
#define SX127x_LoRa_IrqFlagsMask 0x11
#define SX127x_LoRa_IrqFlags 0x12
#define SX127x_LoRa_RxNbBytes 0x13
#define SX127x_LoRa_RxHeaderCntValueMsb 0x14
#define SX127x_LoRa_RxHeaderCntValueLsb 0x15
#define SX127x_LoRa_RxPacketCntValueMsb 0x16
#define SX127x_LoRa_RxPacketCntValueLsb 0x17
#define SX127x_LoRa_ModemStat 0x18
#define SX127x_LoRa_PktSnrValue 0x19
#define SX127x_LoRa_PktRssiValue 0x1A
#define SX127x_LoRa_RssiValue 0x1B
#define SX127x_LoRa_HopChannel 0x1C
#define SX127x_LoRa_ModemConfig 0x1D
#define SX127x_LoRa_ModemConfig_Bw 0xF0
#define SX127x_LoRa_ModemConfig2 0x1E
#define SX127x_LoRa_SymbTimeoutLsb 0x1F
#define SX127x_LoRa_PreambleMsb 0x20
#define SX127x_LoRa_PreambleLsb 0x21
#define SX127x_LoRa_PayloadLength 0x22
#define SX127x_LoRa_MaxPayloadLength 0x23
#define SX127x_LoRa_HopPeriod 0x24
#define SX127x_LoRa_FifoRxByteAddr 0x25
#define SX127x_LoRa_ModemConfig3 0x26
#define SX127x_LoRa_FeiMsb 0x28
#define SX127x_LoRa_FeiMid 0x29
#define SX127x_LoRa_FeiLsb 0x2A
#define SX127x_LoRa_RssiWideband 0x2C
#define SX127x_LoRa_DetectOptimize 0x31
#define SX127x_LoRa_InvertIQ 0x33
#define SX127x_LoRa_DetectionThreshold 0x37
#define SX127x_LoRa_SyncWord 0x39
#define SX127x_LoRa_HighBwOptimize2 0x3A
#define SX127x_LoRa_InvertIQ2 0x3B

/* FSK/OOK registers */
#define SX127x_FSK_BitrateMsb 0x02
#define SX127x_FSK_BitrateLsb 0x03
#define SX127x_FSK_FdevMsb 0x04
#define SX127x_FSK_FdevLsb 0x05
#define SX127x_FSK_RxConfig 0x0D
#define SX127x_FSK_RssiConfig 0x0E
#define SX127x_FSK_RssiCollision 0x0F
#define SX127x_FSK_RssiThresh 0x10
#define SX127x_FSK_RssiValue 0x11
#define SX127x_FSK_RxBw 0x12
#define SX127x_FSK_AfcBw 0x13
#define SX127x_FSK_OokPeak 0x14
#define SX127x_FSK_OokFix 0x15
#define SX127x_FSK_OokAvg 0x16
#define SX127x_FSK_Reserved17 0x17
#define SX127x_FSK_Reserved18 0x18
#define SX127x_FSK_Reserved19 0x19
#define SX127x_FSK_AfcFei 0x1A
#define SX127x_FSK_AfcMsb 0x1B
#define SX127x_FSK_AfcLsb 0x1C
#define SX127x_FSK_FeiMsb 0x1D
#define SX127x_FSK_FeiLsb 0x1E
#define SX127x_FSK_PreambleDetect 0x1F
#define SX127x_FSK_RxTimeout1 0x20
#define SX127x_FSK_RxTimeout2 0x21
#define SX127x_FSK_RxTimeout3 0x22
#define SX127x_FSK_RxDelay 0x23
#define SX127x_FSK_Osc 0x24
#define SX127x_FSK_PreambleMsb 0x25
#define SX127x_FSK_PreambleLsb 0x26
#define SX127x_FSK_SyncConfig 0x27
#define SX127x_FSK_SyncValue1 0x28
#define SX127x_FSK_SyncValue2 0x29
#define SX127x_FSK_SyncValue3 0x2A
#define SX127x_FSK_SyncValue4 0x2B
#define SX127x_FSK_SyncValue5 0x2C
#define SX127x_FSK_SyncValue6 0x2D
#define SX127x_FSK_SyncValue7 0x2E
#define SX127x_FSK_SyncValue8 0x2F
#define SX127x_FSK_PacketConfig1 0x30
#define SX127x_FSK_PacketConfig2 0x31
#define SX127x_FSK_PayloadLength 0x32
#define SX127x_FSK_NodeAdrs 0x33
#define SX127x_FSK_BroadcastAdrs 0x34
#define SX127x_FSK_FifoThresh 0x35
#define SX127x_FSK_SeqConfig1 0x36
#define SX127x_FSK_SeqConfig2 0x37
#define SX127x_FSK_TimerResol 0x38
#define SX127x_FSK_Timer1Coef 0x39
#define SX127x_FSK_Timer2Coef 0x3A
#define SX127x_FSK_ImageCal 0x3B
#define SX127x_FSK_Temp 0x3C
#define SX127x_FSK_LowBat 0x3D
#define SX127x_FSK_IrqFlags1 0x3E
#define SX127x_FSK_IrqFlags2 0x3F
#define SX127x_FSK_PllHop 0x44
#define SX127x_FSK_BitRateFrac 0x5D

/*default register value */
#define SX127x_Regfifo_dft 0x00
#define SX127x_RegOpMode_dft 0x01
#define SX127x_RegBitrateMsb_dft 0x1A
#define SX127x_RegBitrateLsb_dft 0x0B
#define SX127x_RegFdevMsb_dft 0x00
#define SX127x_RegFdevLsb_dft 0x52
#define SX127x_RegFrfMsb_dft 0x6C
#define SX127x_RegFrfMid_dft 0x80
#define SX127x_RegFrfLsb_dft 0x00
#define SX127x_RegPaConfig_dft 0x4F
#define SX127x_RegPaRamp_dft 0x09
#define SX127x_RegOcp_dft 0x2B
#define SX127x_RegLna_dft 0x20
#define SX127x_RegRxConfig_dft 0x0E
#define SX127x_RegRssiConfig_dft 0x02
#define SX127x_RegRssiCollision_dft 0x0A
#define SX127x_RegRssiThresh_dft 0xFF
#define SX127x_RegRxBw_dft 0x15
#define SX127x_RegAfcBw_dft 0x0B
#define SX127x_RegOokPeak_dft 0x28
#define SX127x_RegOokFix_dft 0x0C
#define SX127x_RegOokAvg_dft 0x12
#define SX127x_RegAfcFei_dft 0x00
#define SX127x_RegPreambleDetect_dft 0xAA
#define SX127x_RegRxTimeout1_dft 0x00
#define SX127x_RegRxTimeout2_dft 0x00
#define SX127x_RegRxTimeout3_dft 0x00
#define SX127x_RegRxDelay_dft 0x00
#define SX127x_RegOsc_dft 0x07
#define SX127x_RegPreambleMsb_dft 0x00
#define SX127x_RegPreambleLsb_dft 0x03
#define SX127x_RegSyncConfig_dft 0x93
#define SX127x_FSK_SyncValue1_dft 0x01
#define SX127x_FSK_SyncValue2_dft 0x01
#define SX127x_FSK_SyncValue3_dft 0x01
#define SX127x_FSK_SyncValue4_dft 0x01
#define SX127x_FSK_SyncValue5_dft 0x01
#define SX127x_FSK_SyncValue6_dft 0x01
#define SX127x_FSK_SyncValue7_dft 0x01
#define SX127x_FSK_SyncValue8_dft 0x01
#define SX127x_FSK_PacketConfig1_dft 0x90
#define SX127x_FSK_PacketConfig2_dft 0x40
#define SX127x_FSK_PayloadLength_dft 0x40
#define SX127x_FSK_NodeAdrs_dft 0x00
#define SX127x_FSK_BroadcastAdrs_dft 0x00
#define SX127x_FSK_FifoThresh_dft 0x1F
#define SX127x_FSK_SeqConfig1_dft 0x00
#define SX127x_FSK_SeqConfig2_dft 0x00
#define SX127x_FSK_TimerResol_dft 0x00
#define SX127x_FSK_Timer1Coef_dft 0x12
#define SX127x_FSK_Timer2Coef_dft 0x20
#define SX127x_FSK_ImageCal_dft 0x02
#define SX127x_FSK_LowBat_dft 0x02
#define SX127x_FSK_IrqFlags1_dft 0x80
#define SX127x_FSK_IrqFlags2_dft 0x40
#define SX127x_DioMapping1_dft 0x00
#define SX127x_DioMapping2_dft 0x00
#define SX127x_Version_dft 0x12
#define SX127x_FSK_PllHop_dft 0x2D
#define SX127x_Tcxo_dft 0x09
#define SX127x_PaDac_dft 0x84
#define SX127x_FSK_BitRateFrac_dft 0x00
#define SX127x_AgcRef_dft 0x13
#define SX127x_AgcThresh1_dft 0x0E
#define SX127x_AgcThresh2_dft 0x5B
#define SX127x_AgcThresh3_dft 0xDB
#define SX127x_FSK_PllBW_dft 0xD0

/* MODE */
#define SX127x_ModeSetting_FSK_OOK 0x00
#define SX127x_ModeSetting_LoRa 0x01

//--------- Frequency MODES ---------//
#define HF 0
#define LF 1

//--------- MODES ---------//
#define SLEEP_MODE				    0
#define	STANDBY_MODE			    1
#define TRANSMIT_MODE			    3
#define RXCONTIN_MODE			    5
#define RXSINGLE_MODE			    6
//#define SLEEP_MODE_hex                 0x00
//#define STANDBY_MODE_hex               0x01
//#define TRANSMIT_MODE_hex              0x03
//#define RXCONTIN_MODE_hex              0x05
//#define RXSINGLE_MODE_hex              0x06



//------- BANDWIDTH -------//
#define BW_7_8KHz					0
#define BW_10_4KHz				    1
#define BW_15_6KHz				    2
#define BW_20_8KHz				    3
#define BW_31_25KHz				    4
#define BW_41_7KHz				    5
#define BW_62_5KHz			    	6
#define BW_125KHz					7
#define BW_250KHz					8
#define BW_500KHz					9

//------ CODING RATE ------//
#define CR_4_5						1
#define CR_4_6						2
#define CR_4_7						3
#define CR_4_8						4

//--- SPREADING FACTORS ---//
#define SF_6                        6
#define SF_7						7
#define SF_8						8
#define SF_9						9
#define SF_10						10
#define SF_11  						11
#define SF_12						12

//------ IMPLICIT HEADER MODE ------//
#define EXPLICIT               0
#define IMPLICIT               1

//------ PA SELECT ------//
#define RFO                    0
#define PA_BOOST               1

//------ POWER GAIN ------//
#define POWER_11db				0xF6
#define POWER_14db				0xF9
#define POWER_17db				0xFC
#define POWER_20db				0xFF

//------ LORA STATUS ------//
#define LORA_OK							200
#define LORA_NOT_FOUND			404
#define LORA_LARGE_PAYLOAD	413
#define LORA_UNAVAILABLE		503

#endif
