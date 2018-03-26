
uint8_t tbuf_pos;
char t_line[32];

void t_add(char c)
{
  t_line[tbuf_pos] = c;
  tbuf_pos++;
}

void t_angle()
{
  tbuf_pos  = sprintf( t_line,"%d",          (int)angle[0]);
  t_add(' ');
  tbuf_pos += sprintf(&t_line[tbuf_pos],"%d",(int)angle[1]);
  t_add(13);
  t_add(10);
  esp_now_send(NULL, (u8*)t_line, tbuf_pos);
}

void t_gyro()
{
  tbuf_pos  = sprintf(t_line,"%d",           gyroADC[0]); 
  t_add(' ');
  tbuf_pos += sprintf(&t_line[tbuf_pos],"%d",gyroADC[1]); 
  t_add(' ');
  tbuf_pos += sprintf(&t_line[tbuf_pos],"%d",gyroADC[2]); 
 t_add(13);
  t_add(10);
  esp_now_send(NULL, (u8*)t_line, tbuf_pos);
}

void t_acc()
{
  tbuf_pos  = sprintf(t_line,"%d",           accADC[0]);
  t_add(' ');
  tbuf_pos += sprintf(&t_line[tbuf_pos],"%d",accADC[1]);
  t_add(' ');
  tbuf_pos += sprintf(&t_line[tbuf_pos],"%d",accADC[2]);
  t_add(13);
  t_add(10);
  esp_now_send(NULL, (u8*)t_line, tbuf_pos);
}



