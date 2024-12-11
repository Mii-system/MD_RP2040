//--------------------
//  GPIO
//--------------------
// MD_RP2040 V0.1
//  IO  Port
//  GP00  -
//  GP01  -
#define E_PA_PIN      (2)       //  GP02  E_PA
#define E_PB_PIN      (3)       //  GP03  E_PB
#define SDA2_PIN      (4)       //  GP04  SDA1
#define SCL2_PIN      (5)       //  GP05  SCL1
#define E_SA_PIN      (6)       //  GP06  E_SA
#define E_SB_PIN      (7)       //  GP07  E_SB
#define E_MA_PIN      (8)       //  GP08  E_MA
#define E_MB_PIN      (9)       //  GP09  E_MB
#define PWMB_PIN      (10)      //  GP10  PWMB
#define PWMA_PIN      (11)      //  GP11  PWMA
#define SV1_PIN       (12)      //  GP12  SV1
#define SV2_PIN       (13)      //  GP13  SV2
#define SDA0_PIN      (14)      //  GP14  SDA0
#define SCL0_PIN      (15)      //  GP15  SCL0

#define POT1_PIN      (26)      //  GP26  POT1
#define POT2_PIN      (27)      //  GP27  POT2
#define SD1_PIN       (28)      //  GP28  SD1
//  GP29  -

#define SW_ON         (1)
#define SW_OFF        (0)
#define LED_ON        (0)
#define LED_OFF       (1)
#define OUT_ON        (1)
#define OUT_OFF       (0)
#define INP_ON        (0)
#define INP_OFF       (1)
#define DIP_ON        (1)
#define DIP_OFF       (0)

//--  Modbus Registers
enum {
  H_sMode = 0,
  H_sPwmM,
  H_sCurM,
  H_sVelM,
  H_sPosM,
  H_sFrqM,
  H_pCurMP,
  H_pCurMI,
  H_pVelMP,
  H_pVelMI,
  H_pCurMax,
  H_pVelMax,
  H_pAccTim,
  H_pEPM,
  H_pEPP,
  H_pEPS
};

enum {
  I_posM = 0,
  I_velM,
  I_accM,
  I_posP,
  I_velP,
  I_accP,
  I_posS,
  I_velS,
  I_accS,
  I_dCurM,
  I_dVolM,
  I_dINP,
  I_dPot1,
  I_dPot2,
  I_dPosZ0,
  I_dPosZ1
};
