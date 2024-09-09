#include <Wire.h>
#include <Servo.h>
#include <LovyanGFX.hpp>  // lovyanGFXのヘッダを準備
#include <INA226.h>
#include <ModbusRTU.h>

#include "pio_encoder.h"
#include "setting.h"

//--------------------
//  Const / Define
//--------------------

//--------------------
//  Multi Core
//--------------------
spin_lock_t *spinLock;

//--------------------
//  Timer
//--------------------
//----  Timer
const int T_TIM0 = (1 * 1000);                  // [us] Timer
struct repeating_timer timer0;
const int T_TIM1 = (1 * 1000);                  // [us] Timer
struct repeating_timer timer1;

//--------------------
//  Global Variable
//--------------------
//--  Modbus
ModbusRTU mb;

//--  Encoder
volatile uint32_t tSystickNow = 0;
volatile uint32_t tSystickLast = 0;
int32_t posM = 0;         // Pulse
int32_t posP = 0;
int32_t posS = 0;
int32_t velM = 0;         // rpm
int32_t velP = 0;
int32_t velS = 0;
int32_t accM = 0;         // rpm/s
int32_t accP = 0;
int32_t accS = 0;

//--  DI
uint8_t dINP[4] = {DIP_OFF,DIP_OFF,DIP_OFF,DIP_OFF};
  #define SW1     (0)
  #define SW2     (1)
  #define DIP1    (2)
  #define DIP2    (3)

uint8_t dLED[2] = {LED_OFF,LED_OFF};
  #define LD1     (0)
  #define LD2     (1)

//--  AI
uint16_t dPot1;
uint16_t dPot2;

//--  CUR
#define ADR_INA226  0x40
INA226 INA(ADR_INA226);
float dVolM = 0.0;
float dCurM = 0.0;

//--  PWM
#define SV_MIN  (500)
#define SV_MAX  (2500)
Servo servo1;
Servo servo2;

//--  LovyanGFX
class LGFX_SSD1306 : public lgfx::LGFX_Device {
  lgfx::Panel_SSD1306 _panel_instance;    // SSD1306を使用する場合
  lgfx::Bus_I2C _bus_instance;            // I2Cバスのインスタンス (ESP32のみ)

public:                                       // Constructor
  LGFX_SSD1306() {                            // コンストラクタ名はクラス名に合わせてLGFXからLGFX_SSD1306に変更してます。（クラス名と同じにする）
    {                                         // バス制御の設定を行います。
      auto cfg = _bus_instance.config();      // I2Cバス設定用の構造体を取得します。
      cfg.i2c_port    = 1;                    // 使用するI2Cポートを選択 (0 or 1)
      cfg.freq_write  = 400000;               // 送信時のクロック
      cfg.freq_read   = 400000;               // 受信時のクロック
      cfg.pin_sda     = SDA0_PIN;             // SDAを接続しているピン番号
      cfg.pin_scl     = SCL0_PIN;             // SCLを接続しているピン番号
      cfg.i2c_addr    = 0x3C;                 // I2Cデバイスのアドレス

      _bus_instance.config(cfg);              // 設定値をバスに反映します。
      _panel_instance.setBus(&_bus_instance); // バスをパネルにセットします。
    }
    {  // 表示パネル制御の設定を行います。
      // 以下の設定値はパネル毎に一般的な初期値が設定されていますので、不明な項目はコメントアウトして試してみてください。
      auto cfg = _panel_instance.config();    // 表示パネル設定用の構造体を取得します。
      cfg.panel_width     = 128;              // 実際に表示可能な幅
      cfg.panel_height    = 64;               // 実際に表示可能な高さ
      cfg.offset_rotation = 2;                // 回転方向の値のオフセット 0~7 (4~7は上下反転)
      // cfg.invert          = true;             // パネルの明暗が反転してしまう場合 trueに設定
      _panel_instance.config(cfg);            // 設定をパネルに反映
    }
    setPanel(&_panel_instance);               // 使用するパネルをセットします。
  }
};
static LGFX_SSD1306 lcd;                      // LGFX_SSD1306のインスタンス（クラスLGFX_SSD1306を使ってlcdでいろいろできるようにする）を作成
static LGFX_Sprite canvas(&lcd);              // スプライトを使うためのLGFX_Spriteのインスタンスを作成

//--  Private
int32_t dMode = 0;
int32_t dModeb = -1;
int32_t dPosZ0 = 0;
int32_t dPosZ1 = 0;

//----------------------------------------
//-- SetUP
void setup() {
  setup_core0();
}

void setup1() {
  setup_core1();
}

//----------------------------------------
//--  loop
void loop(){
  loop_Core0();
}

void loop1(){
  loop_Core1();
}
