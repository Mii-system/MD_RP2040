//--------------------
//  Private Variable
//--------------------
//--  Tick
int32_t gTick1 = 0;
int16_t gTmr1 = 0;

//--  Encoder
PioEncoder encM(E_MA_PIN);
PioEncoder encP(E_PA_PIN);
PioEncoder encS(E_SA_PIN);

#define ENC_BUF_SIZE    (16)
int16_t pEP = 0;                        // Pointer
int16_t bufEPM[ENC_BUF_SIZE];
int16_t bufEPP[ENC_BUF_SIZE];
int16_t bufEPS[ENC_BUF_SIZE];

// Encoder pulse/r
  int16_t pEPM = 68;
  int16_t pEPP = 1600;
  int16_t pEPS = 1600;

// Pulse to rpm(1rpm:256)  256*60000ms=15360000
  int16_t pK_PV_EPM;
  int16_t pK_PV_EPP;
  int16_t pK_PV_EPS;
// Pulse to accAng(1deg/s:256)  256*1000ms*360*=92160000
  int16_t pK_PA_EPM;
  int16_t pK_PA_EPP;
  int16_t pK_PA_EPS;

int32_t velEPMq = 0;                    // VelQ
int32_t velEPPq = 0;
int32_t velEPSq = 0;

int32_t sumEPMb = 0;                    // Last Sum
int32_t sumEPPb = 0;
int32_t sumEPSb = 0;

int32_t accEPMq = 0;                    // AccQ
int32_t accEPPq = 0;
int32_t accEPSq = 0;

//--  Motor
#define		M_FRE	(0)
#define		M_BRK	(1)
#define		M_PWM	(2)
#define		M_CUR	(3)
#define		M_VEL	(4)
#define		M_POS	(5)

int16_t   sModeM = M_FRE;
int16_t   sFrqM = 1000;                 // 1Hz/bit
int16_t		sPwmM = 0;										// Pwm.req.
float		  sCurM = 0;										// Cur.req.
int32_t		sVelM = 0;  									// Vel.req.
int32_t		sPosM = 0;										// Pos.req.

//--  Control parameters
//  Motor
float pK_I2P = (10000 / (5* 1000));     // 100%/5000mA
float pK_V2I = (0.2);                   // 1000mA/2000rpm
int16_t pVelLimit = 18000;              // MAX rpm
int16_t pCurLimit = (8) * 1000;         // MAX Amp

// APR
int16_t pVelMax = 600;
int16_t pAccTim = 100;
float   pAccM;
int16_t pDecPos;

// ASR
float pVelMP = 1.0;
float pVelMI = 0.1;

// ACR
int16_t pCurMax = (3) * 1000;
float pCurMP = 1.0;
float pCurMI = 0.1;

// PWM
int32_t pPwmMAX = 10000;

//--  Current
float dCurMq = 0;

//--  PI Control
float sVelMq = 0;
float sVelMp = 0;
float sVelMi = 0;
float pCurM = 0;
float iCurM = 0;

int wPos;

//--------------------
//  Timer Interrupt
//--------------------
bool onTimer1(repeating_timer *t) {
  gTick1++;
  gTmr1++;

  //--  Encoder read
  int32_t wPM,wPP,wPS;
  wPM = -encM.getCount();
  wPP = encP.getCount();
  wPS = encS.getCount();

  //--  Buffer set
  bufEPM[pEP] = (wPM - posM);
  bufEPP[pEP] = (wPP - posP);
  bufEPS[pEP] = (wPS - posS);
  pEP++;
  if(pEP >= ENC_BUF_SIZE){ pEP = 0x0000;  }       // limit

  //--  Sum
  int32_t sumEPM = 0;                         // SUM Buffer
  int32_t sumEPP = 0;
  int32_t sumEPS = 0;
  for(int i = 0;i < ENC_BUF_SIZE;i ++){
    sumEPM += bufEPM[i];
    sumEPP += bufEPP[i];
    sumEPS += bufEPS[i];
  }

  //--  Vel Cal
  velEPMq += ((((sumEPM * pK_PV_EPM) << 8) -velEPMq) >> 2);
  velEPPq += ((((sumEPP * pK_PV_EPP) << 8) -velEPPq) >> 2);
  velEPSq += ((((sumEPS * pK_PV_EPS) << 8) -velEPSq) >> 2);

  //--  Acc Cal
  accEPMq += (((((sumEPM - sumEPMb) * pK_PA_EPM) << 8) - accEPMq) >> 4);
  accEPPq += (((((sumEPP - sumEPPb) * pK_PA_EPP) << 8) - accEPPq) >> 4);
  accEPSq += (((((sumEPS - sumEPSb) * pK_PA_EPS) << 8) - accEPSq) >> 4);
  
  sumEPMb = sumEPM;                               // Last update
  sumEPPb = sumEPP;
  sumEPSb = sumEPS;
  
  //--  update
  spin_lock_unsafe_blocking(spinLock);
    posM = wPM;
    posP = wPP;
    posS = wPS;
    velM = velEPMq >> 8;
    velP = velEPPq >> 8;
    velS = velEPSq >> 8;
    accM = accEPMq >> 8;
    accP = accEPPq >> 8;
    accS = accEPSq >> 8;
  spin_unlock_unsafe(spinLock);

  //--  Current cal
  float w1,w2;
  w1 = INA.getBusVoltage();
  w2 = INA.getCurrent_mA();
  if(sPwmM < 0){  w2 = -w2; }
  dCurMq += (w2 - dCurMq) * 0.1;

  spin_lock_unsafe_blocking(spinLock);
    dVolM = w1;
    dCurM = dCurMq;
  spin_unlock_unsafe(spinLock);

//--  Pos Control
  if(sModeM >= M_POS){                            // Pos.Cont.
    int32_t wDat = sPosM - posM;                 // Diff
    if(wDat < 0){  wDat = -wDat; }
    sVelMq += pAccM;
    if(wDat <= pDecPos){                         // Deceleration Zone
      int32_t wDat2 = wDat * pAccM;
      sVelMq = constrain(sVelMq,0,wDat2);
    }
    sVelMq = constrain(sVelMq,0,pVelMax);
    sVelM = sVelMq;
    if(sPosM < posM){ sVelM = 0 - sVelM;  }
  }

//--  Vel Control
  if(sModeM >= M_VEL){                            // Vel.Cont.
    float wDat = sVelM - (velM >> 8);             // Diff
    if(sVelM == 0){ sVelMi = 0; }                 // I CLR
    sVelMp = wDat * pVelMP;                       // P
    sVelMi += (wDat * pVelMI);                    // I
    sVelMi = constrain(sVelMi,-pVelLimit,pVelLimit);  // I Limit
    wDat = sVelMp + sVelMi;                       // P+I
    // wDat = constrain(wDat,-pVelMax,pVelMax);      // Limit
    sCurM = wDat * pK_V2I;                        // Vel to Cur
  }else{
    sVelMi = 0;                                   // I CLR
  }

//--  Current Control
  if(sModeM >= M_CUR){                            // Cur.Cont.
    float wDat = sCurM - dCurM;                   // Diff
    if(sCurM == 0){ iCurM = 0;  }                 // I CLR
    pCurM = wDat * pCurMP;                        // P
    iCurM += (wDat * pCurMI);                     // I
    iCurM = constrain(iCurM,-pCurLimit,pCurLimit);  // I Limit
    wDat = pCurM + iCurM;                         // P+I
    wDat = constrain(wDat,-pCurMax,pCurMax);      // Limit
    wDat = wDat * pK_I2P;                         // I to PWM
    sPwmM = constrain(wDat,-pPwmMAX,pPwmMAX);     // Limit
  }else{
    iCurM = 0;                                    // I CLR
  }

//----  PWM Control
  if(sModeM >= M_PWM){                            // PWM.Cont.
    if(sPwmM >= 0){
      digitalWriteFast(SD1_PIN, 0);
      analogWrite(PWMA_PIN, 0);
      analogWrite(PWMB_PIN, sPwmM);
    }else{
      digitalWriteFast(SD1_PIN, 0);
      analogWrite(PWMA_PIN, -sPwmM);
      analogWrite(PWMB_PIN, 0);
    }
  }else{                                          // Brake
    if(sModeM == M_BRK){
      digitalWriteFast(SD1_PIN, 0);
      digitalWriteFast(PWMA_PIN,1);
      digitalWriteFast(PWMB_PIN,1);
    }else{
      digitalWriteFast(SD1_PIN, 1);               // Free
    }
  }

  return(true);
}

//----  Timer setup
void setup_onTimer1() {
  add_repeating_timer_us(-T_TIM1, &onTimer1, NULL, &timer1);
}

//----  Setup
void setup_core1(){
  //--  Encoder
  encM.begin();
  encP.begin();
  encS.begin();

  for(int i = 0;i < ENC_BUF_SIZE;i ++){
    bufEPM[i] = 0;
    bufEPP[i] = 0;
    bufEPS[i] = 0;
  }

  //--  CUR
  pinMode(SDA2_PIN, INPUT_PULLUP);
  pinMode(SCL2_PIN, INPUT_PULLUP);
  Wire.setSDA(SDA2_PIN);
  Wire.setSCL(SCL2_PIN);
  Wire.begin();
  Wire.setClock(1000000);

  if (!INA.begin()){
    Serial.println("Failed to begin ina226");
  }else{
    INA.setMaxCurrentShunt(2.0, 0.032);                         // 20240730 OK
    INA.setAverage(INA226_16_SAMPLES);
    INA.setBusVoltageConversionTime(INA226_140_us);
    INA.setModeShuntContinuous();
  }

  //--  PWM
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(SD1_PIN, OUTPUT);
  analogWriteFreq(sFrqM);
  analogWriteRange(pPwmMAX);

  //--  Parameter
  update_Control();

  //--  Timer start
  setup_onTimer1();
}

//----  Loop
void loop_Core1(){
  funcState();
  if(gTick1 > 10){
    gTick1 = 0;
    // Serial.printf("Pos= %d  Vel= %d  Acc= %d \n",posM,velM >> 8,accM);
    // Serial.printf("Pos= %d  Vel= %d  Acc= %d \n",posP,velP >> 0,accP >> 0);
  }
}

//--  StateMachine
void funcState(){
  if(dMode != dModeb){
    gTmr1 = 0;
    dModeb = dMode;
  }
  switch(dMode){
    case 0:
      digitalWriteFast(SD1_PIN, 1);
      sModeM = M_FRE;
      if(dINP[SW2] == SW_ON){
        dMode = 1;
      }
      break;
    case 1:
      dLED[LD1] = LED_ON;
      sModeM = M_VEL;
      pCurMax = 1 *1000;
      sVelM = -400;
      if(velM < ((sVelM / 8) * 256)){
        gTmr1 = 0;
      }
      if(gTmr1 > 250){
        dPosZ0 = posM;
        dMode = 2;
      }
      break;
    case 2:
      dLED[LD1] = LED_ON;
      sModeM = M_VEL;
      pCurMax = 1 *1000;
      sVelM = 0;
      if(gTmr1 > 100){
        dMode = 3;
      }
      break;
    case 3:
      dLED[LD1] = LED_ON;
      sModeM = M_VEL;
      pCurMax = 1 *1000;
      sVelM = 400;
      if(velM > ((sVelM / 8) * 256)){
        gTmr1 = 0;
      }
      if(gTmr1 > 250){
        dPosZ1 = posM;
        dMode = 4;
      }
      break;
    case 4:
      dLED[LD1] = LED_ON;
      sModeM = M_VEL;
      pCurMax = 1 *1000;
      sVelM = 0;
      if(gTmr1 > 100){
        dMode = 5;
      }
      break;
    case 5:
      dLED[LD1] = LED_ON;
      pCurMax = 2 * 1000;
      sModeM = M_POS;
      sPosM = dPosZ0 + 50;
      wPos = posM - sPosM;
      wPos = abs(wPos);
      if(wPos > 10){
        gTmr1 = 0;
      }
      if(gTmr1 > 100){
        dMode = 10;
      }
      break;
    case 10:
      dLED[LD1] = LED_OFF;
      if(dINP[SW1] == SW_ON){
        dMode = 19;
      }
      if(dINP[SW2] == SW_ON){
        dMode = 11;
      }
      break;
    case 11:
      dLED[LD1] = LED_ON;

      sFrqM = 4000;
      pVelMax = dPot1 * 4;
      pAccTim = dPot2;
      update_Control();
      sModeM = M_POS;
      sPosM = dPosZ1 - 50;

      wPos = sPosM - posM;
      if(wPos > 10){
        gTmr1 = 0;
      }
      if(gTmr1 > 20){
        dMode = 12;
      }
      break;
    case 12:
      dLED[LD1] = LED_ON;

      sFrqM = 4000;
      pVelMax = dPot1 * 4;
      pAccTim = dPot2;
      update_Control();
      sModeM = M_POS;
      sPosM = dPosZ0 + 50;

      wPos = posM - sPosM;
      if(wPos > 10){
        gTmr1 = 0;
      }
      if(gTmr1 > 20){
        dMode = 10;
      }
      break;
    case 19:
      sModeM = M_FRE;
      dLED[LD1] = LED_OFF;
      if(dINP[SW1] == SW_OFF){
        dMode = 20;
      }
      break;
    case 20:
      dLED[LD2] = LED_ON;
      if(dINP[SW1] == SW_ON){
        dMode = 29;
      }
      if(((posM - dPosZ0) < 30) || ((dPosZ1 - posM) < 30)){
        sModeM = M_BRK;
      }else{
        sModeM = mb.Hreg(H_sMode);
        sPosM = mb.Hreg(H_sPosM) + dPosZ0;
        sFrqM = mb.Hreg(H_sFrqM);
        pVelMax = mb.Hreg(H_pVelMax);
        pAccTim = mb.Hreg(H_pAccTim);
        update_Control();
      }
      break;
    case 29:
      dLED[LD2] = LED_OFF;
      if(dINP[SW1] == SW_OFF){
        dMode = 10;
      }
      break;
    default:
      dMode = 0;
      break;
  }
}

//----  Parameter
void  update_Control(){
// Pulse to rpm(1rpm:256)  256*60000ms=15360000
  pK_PV_EPM = (((256 * 60000) / ENC_BUF_SIZE) / pEPM);
  pK_PV_EPP = (((256 * 60000) / ENC_BUF_SIZE) / pEPP);
  pK_PV_EPS = (((256 * 60000) / ENC_BUF_SIZE) / pEPS);
// Pulse to accFai(1deg/s2:256)  256*1000ms*360*=92160000
  pK_PA_EPM = ((256 * 1000 * 360) / pEPM);
  pK_PA_EPP = ((256 * 1000 * 360) / pEPP);
  pK_PA_EPS = ((256 * 1000 * 360) / pEPS);

  pAccM = (float)pVelMax / (((float)pAccTim * 1000) / T_TIM1);                // 加速度[rpm/tick] : 最高速 / 加減速時間 / Tick[ms]
  pDecPos = (int16_t)((float)(pVelMax * pEPM * pAccTim) / (1000 * 60 * 2)) + 1;
  analogWriteFreq(sFrqM);
  analogWriteRange(pPwmMAX);
}