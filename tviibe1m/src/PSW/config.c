typedef enum
{	
  MO0_TSC1_E_TX       = 0x0C00000B,
  MO1_TSC1_ER_TX      = 0x0C000F0B,
  MO2_TSC1_AD_TX      = 0x0C00100B,
  MO3_TSC1_EXR_TX     = 0x0C00290B,
  MO4_EBC1_TX         = 0x18F0010B,

  MO5_EBC2_TX         = 0x18FEBF0B,
  MO6_EBC3_TX         = 0x18FEAD0B,
  MO7_EBC4_TX         = 0x18FEAC0B,
  MO8_EBC5_TX         = 0x18FDC40B,
  MO9_VDC1_TX         = 0x18FE4F0B,
    
  MO10_VDC2_TX        = 0x18F0090B,
  MO11_TCO1_TX        = 0x18FE6C0B,
  MO12_HRW_TX         = 0x08FE6E0B,
  MO13_VW_TX          = 0x08FEEA0B,
  MO14_SASCAL_TX      = 0x18EFE40B,

  MO17_DM1_TX         = 0x18FECA0B,
  MO18_DM1_BAM_TX     = 0x18ECFF0B,
  MO19_DM1_DT_TX      = 0x18EBFF0B,
  MO20_UDS_RES_TX     = 0x18DAFA0B,
    
  MO21_UDS_PHY_RX     = 0x18DA0BFA,
  MO22_UDS_FUNC_RX    = 0x18DBFFFA,
  MO23_EEC1_RX        = 0x0CF00400,
  MO24_EEC2_RX        = 0x0CF00300,
  MO25_EEC3_RX        = 0x18FEDF00,
  MO26_ETC2_RX        = 0x18F00503,
  MO27_ETC7_RX        = 0x18FE4A03,
  MO28_AIR1_RX        = 0x18FEAE21,
  MO29_SAS_RX         = 0x18F01DE4,
  MO30_TCO1_RX        = 0x0CFE6CEE,
  MO31_MSF_RX         = 0x18FF6017,
  MO32_CCVS_RX        = 0x18FEF100,
    
  MO33_XBR_ACC_RX     = 0x0C040B2A,
  MO34_XBR_AEBS_RX    = 0x0C040BA0,
  MO35_XBR_EPB_RX     = 0x0C040B64,
  MO36_ERC1_RX        = 0x18F0000F,
	
  MO37_ESCM_SIGNAL_RX = 0x18FF1549,
  MO38_ESCM_SIGNAL_RX = 0x18FF1649,
  MO39_ESCM_Calib_RX  = 0x18EF490B,
} Can0_msgID;

typedef enum
{  
  MO33_ESCM_SIGNAL_RX        = 0x18FF1549,
  MO34_ESCM_SIGNAL_RX        = 0x18FF1649,
  MO35_SAS_ESCM_CALI_REQ_RX  = 0x728,	
  MO36_ENGINE_REACTION727_RX = 0x727,
  MO37_VAL_REACTION_0x7DF_RX = 0x7DF,
  MO38_SAS_SIGNAL_RX         = 0x18F01DE4,
} Can1_msgID;

//����CAN����msgID�б�
const Can0_msgID can0_rcvMsg_Rx[] = 
{ 
    MO21_UDS_PHY_RX,   MO22_UDS_FUNC_RX,   MO23_EEC1_RX,    MO24_EEC2_RX,     MO25_EEC3_RX,	
    MO26_ETC2_RX,      MO27_ETC7_RX,       MO28_AIR1_RX,    MO29_SAS_RX,      MO30_TCO1_RX, 
    MO31_MSF_RX,       MO32_CCVS_RX,       MO33_XBR_ACC_RX, MO34_XBR_AEBS_RX, MO35_XBR_EPB_RX, 
    MO36_ERC1_RX,      MO39_ESCM_Calib_RX 
};
uint8_t can0_rcvID_cnt = sizeof(can0_rcvMsg_Rx) / sizeof(can0_rcvMsg_Rx[0]);

//����CAN����msgID�б�
const Can0_msgID can0_sendMsg_Tx[] = 
{ 
    MO0_TSC1_E_TX,     MO1_TSC1_ER_TX,     MO2_TSC1_AD_TX, MO3_TSC1_EXR_TX,   MO4_EBC1_TX, 
    MO5_EBC2_TX,       MO6_EBC3_TX,        MO7_EBC4_TX,    MO8_EBC5_TX,       MO9_VDC1_TX, 
    MO10_VDC2_TX,      MO11_TCO1_TX,       MO12_HRW_TX,    MO13_VW_TX,        MO14_SASCAL_TX, 
    MO17_DM1_TX,       MO18_DM1_BAM_TX,    MO19_DM1_DT_TX, MO20_UDS_RES_TX 
};
uint8_t can0_sendID_cnt = sizeof(can0_sendMsg_Tx) / sizeof(can0_sendMsg_Tx[0]);

const Can1_msgID can1_rcvMsg_Rx[] = 
{
    MO33_ESCM_SIGNAL_RX,         MO34_ESCM_SIGNAL_RX,          MO35_SAS_ESCM_CALI_REQ_RX,
    MO36_ENGINE_REACTION727_RX,  MO37_VAL_REACTION_0x7DF_RX,   MO38_SAS_SIGNAL_RX
};
uint8_t can1_rcvID_cnt = sizeof(can1_rcvMsg_Rx) / sizeof(can1_rcvMsg_Rx[0]);