//----  Dipp-SW
//--  PCA9554A registers
#define REG_INPUT     (0x00)
#define REG_OUTPUT    (0x01)
#define REG_INVERSION (0x02)
#define REG_CONFIG    (0x03)

#define ADDR0         (0x20)                // DIPP-SW
#define IOMODE0       (0x0F)                // b7-0:DIP8-1
#define INV_DEF0      (0x00)                // Inversion OFF
#define IO_DEF0       (0x00)                // 

uint8_t wInp[4] = { 0,0,0,0 };

//--  Init
void PCA9554A_init(){
  Wire1.beginTransmission(ADDR0);
  Wire1.write(REG_OUTPUT);
  Wire1.write(IO_DEF0);
  Wire1.endTransmission();
  Wire1.beginTransmission(ADDR0);
  Wire1.write(REG_INVERSION);
  Wire1.write(INV_DEF0);
  Wire1.endTransmission();
  Wire1.beginTransmission(ADDR0);
  Wire1.write(REG_CONFIG);
  Wire1.write(IOMODE0);
  Wire1.endTransmission();
}

//--  update
void update_PCA9554A(){
  uint8_t w = 0;

  Wire1.beginTransmission(ADDR0);
  Wire1.write(REG_OUTPUT);
  bitWrite(w,4,dLED[LD1]);
  bitWrite(w,5,dLED[LD2]);
  Wire1.write(w);
  Wire1.endTransmission();

  Wire1.beginTransmission(ADDR0);
  Wire1.write(REG_INPUT); 
  if(Wire1.endTransmission() == 0){
    Wire1.requestFrom(ADDR0, 1);                  // Read 1 byte
    while(Wire1.available() < 1);
    w = Wire1.read();
  }
  
  wInp[0] = wInp[0] * 2 + bitRead(w, 0);
  wInp[1] = wInp[1] * 2 + bitRead(w, 1);
  wInp[2] = wInp[2] * 2 + bitRead(w, 2);
  wInp[3] = wInp[3] * 2 + bitRead(w, 3);

  for (int i = 0; i < 4; i++) {                               // 8bit
    wInp[i] &= 0x03;
    if (wInp[i] == 0x03)  { dINP[i] = SW_OFF;  }              // ALL OFF
    if (wInp[i] == 0x00)  { dINP[i] = SW_ON;   }              // ALL ON
  }
}
