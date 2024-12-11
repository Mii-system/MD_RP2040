const int SLAVE_ID = 1; // スレーブID

void setup_Modbus(){
  //--  Serial Setting
  // Serial.begin(115200);

  mb.begin(&Serial); // Modbusの初期化
  mb.setBaudrate(115200);
  mb.slave(SLAVE_ID); // スレーブIDの設定
  mb.begin(&Serial); // Modbusの初期化

  // ホールディングレジスタと入力レジスタを追加
  for (uint16_t i = 0; i < 16; i++) {
    mb.addHreg(i);
    mb.addIreg(i);
  }
}

void  update_Modbus(){
  mb.Ireg(I_posM,   (int16_t)(posM));
  mb.Ireg(I_velM,   (int16_t)(velM >> 8));
  mb.Ireg(I_accM,   (int16_t)(accM >> 8));
  mb.Ireg(I_posP,   (int16_t)(posP));
  mb.Ireg(I_velP,   (int16_t)(velP >> 8));
  mb.Ireg(I_accP,   (int16_t)(accP >> 8));
  mb.Ireg(I_posS,   (int16_t)(posS));
  mb.Ireg(I_velS,   (int16_t)(posS >> 8));
  mb.Ireg(I_accS,   (int16_t)(accS >> 8));
  mb.Ireg(I_dCurM,  (int16_t)(dCurM));
  mb.Ireg(I_dVolM,  (int16_t)(dVolM * 256));
  mb.Ireg(I_dINP,   (int16_t)(dINP[3] << 3 + dINP[2] << 2 + dINP[1] << 1 + dINP[0]));
  mb.Ireg(I_dPot1,  (int16_t)(dPot1));
  mb.Ireg(I_dPot2,  (int16_t)(dPot2));
  mb.Ireg(I_dPosZ0, (int16_t)(dPosZ0));
  mb.Ireg(I_dPosZ1, (int16_t)(dPosZ1));

  mb.task();

}

