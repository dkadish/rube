elapsedMillis printTimer = 0;

float pos_out = 0.0, enc_pos = 0.0, pos_setpt = 0.0, spd_out = 0.0, spd_est = 0.0;

void print_loop(){
      if(abs(((int)enc_pos*100.0) - ((int)pos_setpt*100.0)) < 1.0){
        pos_setpt = random(-10000,10000)/100.0;
      }

      spd_out = 0.01 * (pos_setpt - pos_out);
      spd_est = 0.01 * (pos_setpt - pos_out) + random(-150,150)/100.0;
      pos_out += 0.01 * (pos_setpt - pos_out);
      enc_pos = pos_out + random(random(-100,0),random(0,100))/100.0;
        
      if(printTimer > 1000){
        Serial.printf("PID > Position: %i (%i/%i), Speed: %i (%i/%i)\n",
                      (int)(pos_out*100.0), (int)(enc_pos*100.0), (int)(pos_setpt*100.0),
                      (int)(spd_out*100.0), (int)(spd_est*100.0), (int)(pos_out*100.0));
        /*Serial.printf("Estimation: Pos: %i, PosErr: %i, SpdInt: %i, Spd: %i\n",
                      (int)(pos_est*100.0), (int)((enc_pos-pos_est)*100.0), (int)(spd_int*100.0), (int)(spd_est*100.0)
        );*/
        Serial.printf("Position > x: %i\n", (int)(enc_pos*100));
        printTimer=0;
    }

    delay(100);
}

