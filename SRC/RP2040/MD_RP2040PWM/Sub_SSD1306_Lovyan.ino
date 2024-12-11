//--  Wait
void show_wait() {
  canvas.fillScreen(TFT_BLACK);    // 背景塗り潰し
  canvas.setTextColor(TFT_WHITE);  // 文字色と背景を指定（文字色, 背景）

  canvas.setFont(&fonts::Font0);   // 6x8 固定長
  canvas.setCursor( 0, 0); canvas.printf("RP2040%6.0fmA %3.1fV",dCurM,dVolM);
  canvas.setCursor( 0, 8); canvas.printf("%6d %6d %6d",(uint16_t)posM,(uint16_t)posP,(uint16_t)posS);
  canvas.setCursor( 0,16); canvas.printf("%6.1f %6.1f %6.1f",(float)velM / 256,(float)velP / 256,(float)velS / 256);
  // canvas.setCursor( 0,24); canvas.printf("%6.1f %5d",sCurM,sPwmM);
  // canvas.setCursor( 0,32); canvas.printf("%6.1f %6.1f",pCurM,iCurM);
  canvas.setCursor( 0,24); canvas.printf("%6d %6.1f",sVelM,sCurM);
  canvas.setCursor( 0,32); canvas.printf("%6.1f %6.1f",sVelMp,sVelMi);
  canvas.setCursor( 0,40); canvas.printf("H:%4X %4X %4X %4X",mb.Hreg(0),mb.Hreg(1),mb.Hreg(2),mb.Hreg(3));
  canvas.setCursor( 0,48); canvas.printf("I:%4X %4X %4X %4X",mb.Ireg(0),mb.Ireg(1),mb.Ireg(2),mb.Ireg(3));
  canvas.setCursor( 0,56); canvas.printf("%1d%1d%1d%1d  %3d %3d %2d",dINP[0],dINP[1],dINP[2],dINP[3],dPot1,dPot2,dMode);
  canvas.pushSprite(0, 0);  // メモリ内に描画したcanvasを座標を指定して表示する
}
