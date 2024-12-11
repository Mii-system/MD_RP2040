//--------------------
//  Private Variable
//--------------------
//--  Tick
int32_t gTick0 = 0;
int16_t gTmr0 = 0;

//--  private

//--------------------
//  Timer Interrupt
//--------------------
bool onTimer0(repeating_timer *t) {
  gTick0++;
  gTmr0++;
  return(true);
}

//----  Timer setup
void setup_onTimer0() {
  add_repeating_timer_us(-T_TIM0, &onTimer0, NULL, &timer0);
}

//----  Setup
void setup_core0(){
  //--  MultiCore
  spinLock = spin_lock_instance(spin_lock_claim_unused(true));

  //--  Modbus
  setup_Modbus();

  //--  GPIO
  pinMode(POT1_PIN, INPUT);
  pinMode(POT2_PIN, INPUT);

  // PWM出力ピンの初期化
  servo1.attach(SV1_PIN,SV_MIN,SV_MAX);
  servo2.attach(SV2_PIN,SV_MIN,SV_MAX);

  //--  i2c
  Wire1.setSDA(SDA0_PIN);
  Wire1.setSCL(SCL0_PIN);
  Wire1.begin();

  PCA9554A_init();
  delay(100);
  for(int i = 0;i < 8;i++){
    update_PCA9554A();
    delay(10);
  }

  //--  LCD
  lcd.init();                                       // 表示器初期化
  canvas.setTextWrap(false);                        // 右端到達時のカーソル折り返しを禁止(true)で許可
  canvas.createSprite(lcd.width(), lcd.height());   // スプライト用の仮想画面を画面幅を取得して準備
  show_wait();

  // Variables

  //--  Timer start
  setup_onTimer0();
}

//----  Loop
void loop_Core0(){
//--  Get State
  update_PCA9554A();
  dPot1 = analogRead(POT1_PIN);
  dPot2 = analogRead(POT2_PIN);
// Serial.printf("1:%d   2:%d  \n",dPot1,dPot2);

  int16_t angle1 = map(dPot1, 0, 1023, 0, 180);
  int16_t angle2 = map(dPot2, 0, 1023, 0, 180);
// Serial.printf("1:%d   2:%d  \n",angle1,angle2);

  update_Modbus();

 //--  disp
  show_wait();
}