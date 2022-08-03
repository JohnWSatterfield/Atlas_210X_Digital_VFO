#define DT    4   // DT pin, connected to ESP32 pin 4 (GPIO0)
#define CLK   25   // CLK pin, connected to ESP32 pin 17 (GPIO4)

volatile long  quad = 0;
volatile long previous_data;
 
void ICACHE_RAM_ATTR enc_read();

void setupencoder() {
  pinMode(DT,  INPUT_PULLUP);
  pinMode(CLK, INPUT_PULLUP);
  
 
  // enable interrupts for rotary encoder pins
  attachInterrupt(DT,  enc_read, CHANGE);
  attachInterrupt(CLK, enc_read, CHANGE);
  previous_data = digitalRead(DT) << 1 | digitalRead(CLK);
  
}


void  enc_read()
{
  long current_data = digitalRead(DT) << 1 | digitalRead(CLK);
  if( current_data == previous_data ) return;
   if( bitRead(current_data, 0) == bitRead(previous_data, 1) )
    quad -= 1;
  else
    quad += 1;
  previous_data = current_data;
  
}
