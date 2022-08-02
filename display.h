/* 
 * File:   display.h
 * Author: JF3HZB / T.UEBO
 *
 * Created on 2019/02/10, 22:13
 */
 /*
  * for 128x160 display change yW below to 160 and
  * change line 137 of display.cpp SPI.write(0x9F);  0x9F is 159 0x7F is 127
  * for 128x128 display change Yw below to 128 and 
  * change Line 137 of display.cpp SPI.write(0x7F);  0x7F is 127 0x7F is 127
 */

#ifndef _display_
#define _display_

/*----------------------------------------
 Select Display controller
   ST7735 : LCD  (1.8inch 128x160 TFT)
   SEPS525: OLED ( NHD-1.69-160128UGC3 ) 
-----------------------------------------*/

//#define SEPS525
#define ST7735

/*---------------------------------------*/


#define Xw 128
#define Yw 160   //128 for 128x128 display and 160 for 128x160 display

#define Nx Yw
#define Ny Xw

void display_init(void);
void Transfer_Image(void);
void trans65k(void);
void d_Command(uint8_t d);

#endif
