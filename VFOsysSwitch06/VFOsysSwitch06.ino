

/*------------------------------------------------------------------

   VFO System for ESP-WROOM-32

      Designed to work on Atlas 210X
      
        by J. Satterfield / KI5IDZ

          July 1, 2022
          
    based on work by JF3HZB / T.UEBO
--------------------------------------------------------------------*/
/*The following were contributions by others for software and hardware design:
 * dial.cpp, display.cpp, graph.cpp software Author: JF3HZB / T.UEBO
 * 
 * si5351.cpp - Si5351 library for Arduino
 * Copyright (C) 2015 - 2016 Jason Milldrum <milldrum@gmail.com>
 *                           Dana H. Myers <k6jq@comcast.net>
 *
 * Some tuning algorithms derived from clk-si5351.c in the Linux kernel.
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 * Rabeeh Khoury <rabeeh@solid-run.com>
 * 
 * changes needed to implement RTC and give si5351 I2C better stability
 * Use this software with the original Atlas Crystal Filter 5.645 MHz

ESP32 Arduino
"FireBeetle ESP32"
*/

/*------Hard ware Configuration ---------------------
<<ESP32-Pico Kit>>
pin No.  Connection
  25 :  Rotary Encoder Interrupt Pin CLK  
   4 :  Rotary Encoder Interrupt Pin DT
   2 :  si5351A SCL dedicated 
  32 :  si5351A SDA dedicated
  21 :  i2c SDA for the rest
  22 :  i2c SCL for the rest
  18 :  SCK    / ST7735,SEPS525(128x160 display)  *
  23 :  MOSI   / ST7735,SEPS525(128x160 display)
   5 :  CS     / ST7735,SEPS525(128x160 display)  *
  15 :  DC(RS or AO) / ST7735,SEPS525(128x160 display)  *
  -1 :  RESET - to RST pin / ST7735,SEPS525(128x160 display)  *
  26 :  Output for future relay
  33 :  PinButton button1 to turn VFO output off (not in use)
  p0 :  PCF8574 p0 80m band
  p1 :  PCF8574 p1 40m band
  p2 :  PCF8574 p2 20m band
  p3 :  PCF8574 p3 15m band
  p4 :  PCF8574 p4 10m band
  p5 :  PCF8574 p5 button5 for frequency steps (1k, 500, 100, 10)
  p6 :  PCF8574 p6 button6 for frequency memory positions
  p7 :  PCF8574 p7 button7 scanning 
  27 :  PCF8574 Interrupt Pin
  39 :  slideSwitch1 input
  34 :  cw (3.3v input)
  19 :  PinButton button2 to calibrate si5351
  
<<si5351A>>
CLK0 : Car Signal (I)
CLK1 : 5646 mHz
CLK2 : off
------------------------------------------------*/

/*-------------------------------------------------------
   Frequency settings
--------------------------------------------------------*/

#define IF 5645000               // IF amplifier frequency 

/*----------------------------------------------------------------------------------
    Control flags
-----------------------------------------------------------------------------------*/

uint8_t f_dchange;  // if need to renew display, set this flag to 1



/*--------------------------------------------------------
   pin assign
----------------------------------------------------------*/


#define  EncoderStep    20   //number of encoder pulses per step 
#define  VFO_DRIVE SI5351_DRIVE_8MA
#define  VFO_CLOCK SI5351_CLK0
#define  IF_CLOCK  SI5351_CLK1

//------------------------------------------------------------------------------
#define NAME "VFO System"
#define VERSION "Ver. 2.06"
#define ID "by KI5IDZ"

#define CORRECTION 95700ULL  //tuned on 20m at 14.250 - 8605000 when tuned warm
//Correction 70780ULL //tuned on 20m at 14.250 - 8605000 when tuned Switch06 
//Correction 95700ULL //tuned on 20m at 14.250 - 8605000 when tuned Proto board (177.3)


#include "display.h"          //https://github.com/tjlab-jf3hzb/Digital_VFO_with_analog_dial
#include "graph.h"            //https://github.com/tjlab-jf3hzb/Digital_VFO_with_analog_dial
#include "dial.h"             //https://github.com/tjlab-jf3hzb/Digital_VFO_with_analog_dial
#include "si5351.h"           //
#include "encodersetup.h"     //use when encoder is connected to i2c encoder shield
#include "pcf8574inputpins.h" //custom starup
#include <RTClib.h>           //https://github.com/adafruit/RTClib
#include <Wire.h>

/*-----------------------------------------------------------------------------
 *      Discrete I/O
-----------------------------------------------------------------------------*/
//const int relay_On_Off = 26; //on_off output relay
const int VFO = 33;            //Turn VFO on_off
const int slideSwitch1 = 39;   //Norm side band position
const int cw = 34;             //mode slector in cw position
//const int button2 = 19;        //button switch to do si5351 calibration
/*-----------------------------------------------------------------------------
 *       Global
-----------------------------------------------------------------------------*/

long frq=8973500;              //
extern char f_rev;
extern uint32_t cl_BG;

Si5351 si5351;
RTC_DS3231 rtc;


long freqa[5] = {3848000,7185000,14235000,21285000,28385000};
long freqb[5] = {3916000,7275000,14250000,21300000,28450000};
long freqc[5] = {3987500,7279000,14300000,21320000,28785000};
const long begofBand[5] = {3800000,7175000,14225000,21275000,28300000};
const long endofBand[5] = {4000000,7300000,14350000,21450000,29000000};
long freq = 14235000;
long ifFreq = 5645000;
long oppFreq = 5648300;
long cwFreq = 5646200;
int32_t calibA=0;
long fstep = 1000;
long interfreq = 0;
long encodercount = 0;
long encodercountlast = 0;
long diff = 0;
unsigned long currentMillis = 0;
unsigned long currentMillis1 = 0;
unsigned long previousMillis = 0;
unsigned long previousMillis1 = 0;
int mem_count=1;
int recall_count=1;
int recall[11] = {1,1,1,1,1,1,1,1,1,1,1};

int scan_no = 1;
int delaytime1 = 2000;
int delaytime2 = 2000;
int ActiveBand = 3;
int OldBand = 3;
int HoldBand = 0;
bool slideSwitch = LOW;
bool slideSwitchLast = HIGH;
bool cwpos = LOW;
bool cwposLast = HIGH;
bool VFOState = HIGH;  //VFO variable set high uses pin 33
bool VFOStateLast = HIGH;
bool on_off_flag=HIGH;
bool calibAON = HIGH;

char mem_str[4];
char band_str[4];
char step_str[5];
char time_str[5];
byte encoder = 1;
byte stp = 1; 
byte n = 1;
byte count, x, xo;
bool scanning = false;
bool memScanning = false;
bool sts = 0;
unsigned int period1 = 100, pinStateA6 = 0, pinStateA7 = 0;
unsigned long time_now = 0;


//------------------------------------------------------------------------
void setup() {
//------------------------------------------------------------------------
    Serial.begin(115200);

    log_d("Total heap: %d", ESP.getHeapSize());
    log_d("Free heap: %d", ESP.getFreeHeap());
    log_d("Total PSRAM: %d", ESP.getPsramSize());
    log_d("Free PSRAM: %d", ESP.getFreePsram());
    
    char str[64];
    bool i2c_found;
    //Startup display
    display_init();    
    GRAM_clr();  
    sprintf(str,  NAME  ); disp_str16(str,8, 90, 0x00ffff);
    sprintf(str, VERSION); disp_str12(str,30, 50, 0x00ffff);   
    sprintf(str,    ID  ); disp_str8(str,40, 20, 0x00ffff);
    trans65k();
    //f_redraw=1;
    Transfer_Image();
    delay(1000); 
    init_Dial(); //initiate dial program
    GRAM_clr();  //clear screen

    // Start serial and initialize the Si5351
    
    delay (100);  
    Wire.begin();
    delay(500);
    pinSetup();                // Set the pinModes 
    delay(500);
    setupencoder();         // Set up the encoder
    delay(500);
    
    //Set up the si5351
    i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, CORRECTION);
    if (i2c_found) {
      Serial.println("si5351 OK");
    } else {
      Serial.println("si5351 not OK");
    }
    si5351.drive_strength( SI5351_CLK0, SI5351_DRIVE_8MA );
    set_frequency();
    delay(1000);  //to give the si5351 chip time to initialize
    
    si5351.set_freq((ifFreq) * 100ULL, IF_CLOCK);  // set up second frequency output to the IF frequency Norm Position
    
    // Set up and start real time clock (rtc)
    if (rtc.begin()) {
      Serial.println("RTClock Started");
    } else { Serial.println("Could not find RTC");}

    //rtc.adjust(DateTime(2022, 5, 19, 22, 54, 10));
    DateTime now = rtc.now();
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.println(now.minute(), DEC);
    
    f_dchange = 1;           //set display renew flag to true (1)
              
    pinMode(VFO,INPUT);
    pinMode(cw, INPUT);
    pinMode(slideSwitch1,INPUT_PULLUP);
    //pinMode(button2,INPUT_PULLUP);
    stp = 1;                 /*sets the encoder step frequency to 5kHz*/
    setstep();               /*loads 1kHz into the first encoder step*/    
    readSetPins();
    if (buttonChange[0] == LOW) ActiveBand = 1; //80m
    if (buttonChange[1] == LOW) ActiveBand = 2; //40m
    if (buttonChange[2] == LOW) ActiveBand = 3; //20m
    if (buttonChange[3] == LOW) ActiveBand = 4; //15m
    if (buttonChange[4] == LOW) ActiveBand = 5; //10m
    OldBand = ActiveBand;
    HoldBand = OldBand;
    count = ActiveBand;      /*Sets the band (frequency) when started up*/
    bandpresets();           /*initializes the frequency for each band.*/
    bandsave();              /*sets frequency into the startup band*/
    
}

void serialprintcount() { // Encoder routine
  if(on_off_flag) {
  if (quad >= EncoderStep || quad <= -EncoderStep){
    if (quad > EncoderStep) quad = EncoderStep;
    if (quad <-EncoderStep) quad = -EncoderStep;
    //put code here for asigning encoder values like freq = freq + long((encodercount * fstep)/EncoderStep); 
    //communicate to the rest of the program the encoder change like: f_dchange=1 and f_fchange=1
    //check for max an min values
    encodercount += quad/EncoderStep;
    freq = freq + long((quad * fstep)/EncoderStep);
    quad = 0;
  } 
  if (encodercount != encodercountlast) {
  Serial.print(freq); Serial.print("  ");
  Serial.println(encodercount);
  set_frequency();
  f_dchange=1;
  encodercountlast = 0; encodercount =0;
  }
}
}

//-----------------------------------------------------------------------------------------------
void loop() {    // begin main loop here
//-----------------------------------------------------------------------------------------------
// initialize pushbutton flags
     //last state of button1 to permit latching of input
    // start 1 minute RTC update clock
    currentMillis = millis();             //start currentMillis counter value
    
    if (currentMillis - previousMillis >= 60000) {      //look for difference since last update
      display_time();
      f_dchange=1;                                      //set display change flat to true (1)
      previousMillis = currentMillis;                   //restart timer value 
    }

    VFOState = digitalRead(VFO);   //read the input of VFO (state of pin 33)    
    //if (VFOState == HIGH) then plug in radio
    //if (VFOState == LOW) then external VFO plug in radio 
    //Serial.print("button1State = "); Serial.println(VFOState);
    //delay(1);
    if(VFOState){
      if (!on_off_flag) {
          on_off_flag=1;                          // make status flag HIGH
          si5351.output_enable( SI5351_CLK0, 1);  // turn on VFO
          Serial.println("VFO is on");
          //digitalWrite(relay_On_Off, LOW);       // turn relay off
          f_dchange = 1;
       }
    } else {
      if (on_off_flag) {                   // and the status flag is HIGH
          on_off_flag=0;                          // make status flag LOW
          si5351.output_enable( SI5351_CLK0, 0);  // turn off VFO
          Serial.println("VFO is off");
          //digitalWrite(relay_On_Off, HIGH);        // turn relay on
          f_dchange = 1;
      } 
    }     

  
    cwpos = digitalRead(cw);                         //check if mode switch is in CW Position
    if ((cwpos) && (cwposLast)) {   // if mode switch in on cw first time around
      si5351.set_freq((cwFreq) * 100ULL, IF_CLOCK);  // set up second frequency output to the IF frequency Norm Position
      cwposLast = LOW;                             // set cwposLast to High for a single pass through if statement
      Serial.println("mode switch in CW");
      slideSwitchLast = !slideSwitchLast;
    } 
    
    if (cwpos == LOW)   {                              // if mode switch in not in CW Position
    cwposLast = HIGH;                                  // set cwposLast to Low 
    slideSwitch = digitalRead(slideSwitch1);           // read slideSwitch1 pin 16
    if (slideSwitch != slideSwitchLast) {              // if slideSwitch not on Norm first time around
      Serial.println("switch changed");                // diagnostic for slideswitch1
      if (slideSwitch == LOW) {                        // slideSwitch P16 on set low
      si5351.set_freq((ifFreq) * 100ULL, IF_CLOCK);    // set up second frequency output to the IF frequency Norm Position
      slideSwitchLast = slideSwitch;                   // set slideSwitchLast to Low for one pass through if statement
      Serial.println("switch not in CW");
      Serial.println("switch in norm");                // diagnostic for slideswitch1 norm position
      }
    else {                                             // alternative for opp position of slideswitch
        si5351.set_freq((oppFreq) * 100ULL, IF_CLOCK); // set up second frequency output to the IF frequency opp Position
        slideSwitchLast = slideSwitch;                 // set slideSwitchLast to Low for one pass through if statement
        Serial.println("switch not in CW");
        Serial.println("switch in opp");               // diagnostic for slideswitch1 opp position
        }
      }
    }
    
    updatePins();                          //update status of pcf8574 pins
    serialprintcount();                    //check for changes from the encoder for frequency change
    display_write();                       //routine to write the display
    readSetPins();                         //read the band position 
        //----------------- Code for buttons, switches and relays ------------------  
    if (buttonChange[0] ==LOW) {           //if the band switch is on the 80m position
     // Serial.println("Pin0 is pressed ");//diagnostic for seeing if pin0 is pressed
      ActiveBand = 1;                      //set variable for band selection 80m
      OldBand = 1;                         //set variable to detect of a band change has been made
    } 
    if (buttonChange[1] ==LOW) {           //if the band switch is on the 40m position
    //  Serial.println("Pin1 is pressed ");
      ActiveBand = 2; 
      OldBand = 2;
     } 
    if (buttonChange[2] ==LOW) {           //if the band switch is on the 20m position
    //  Serial.println("Pin2 is pressed ");
      ActiveBand = 3; 
      OldBand = 3;
     } 
    if (buttonChange[3] ==LOW) {           //if the band switch is on the 15m position
     // Serial.println("Pin3 is pressed ");
      ActiveBand = 4; 
      OldBand = 4;
     } 
    if (buttonChange[4] ==LOW) {           //if the band switch is on the 10m position
    //  Serial.println("Pin4 is pressed ");
      ActiveBand = 5; 
      OldBand = 5;
     } 
     if (HoldBand != OldBand) {           //check if switch position has changed
      band_change();                      //make a band change
      HoldBand = OldBand;                 //reset the band change detection variable
      display_write();                    //write the band change to the display
     }
     
      
          
     if (buttonChange[5] ==LOW) {         //check if push button5 has been pressed
    //  Serial.print("Pin5 is pressed - stp=");
   //   Serial.println(stp);
      setstep1();                         //make a change in the encoder step rate
      f_dchange=1;                        //set variable for the desplay to write
      buttonChange[5] = HIGH;             //reset the button5 to high state
     }
     if (buttonChange[6] ==LOW) {        //check if push button7 has been pressed
     // Serial.println("Pin6 is pressed ");
      freqrecall();                        //go to next memory (A,B or C)    
      buttonChange[6] = HIGH;
     }
     if (buttonChange[7] ==LOW) {         //check if push button6 has been pressed
     // Serial.println("Pin7 is pressed ");
     // Serial.println(scan_no);
      switch (scan_no) {                  //start the scan/memory routine
          case 1 : scanning = true; scan_no = 2; break; //scan one khz per second
          case 2 : scanning = false; memScanning = true; scan_no = 3; break; //set scan off set memory scan true
          case 3 : memScanning = false; scan_no = 1; break;  //set memory scan off
         }
      buttonChange[7] = HIGH;            //reset the button6 to high state
     }
     
     
    
    
    
    while (scanning) {        // start 1 seconds between frequency while scanning 
      currentMillis1 = millis();                            //start currentMillis counter value
      if (currentMillis1 - previousMillis1 >= delaytime1) { //look for difference since last update
        scan();
        previousMillis1 = currentMillis1;                   //restart timer value 
      }
      if (keyPressed) updatePins();
      if (buttonChange[7]==LOW) {
        scanning = false; 
        memScanning = true;
        buttonChange[7] = HIGH;
      }
     }
     while (memScanning) {     // start 5 seconds between frequency while scanning 
      currentMillis1 = millis();                            //start currentMillis counter value
      if (currentMillis1 - previousMillis1 >= delaytime2) { //look for difference since last update
        freqrecall();
        previousMillis1 = currentMillis1;                   //restart timer value 
      }
      if (keyPressed) updatePins();
      if (buttonChange[7]==LOW) {
        scanning = false; 
        memScanning = false;
        buttonChange[7] = HIGH;
      }
     }
}

void serialprintcalib() { // Encoder routine
  if(on_off_flag) {
  if (quad >= EncoderStep || quad <= -EncoderStep){
    if (quad > EncoderStep) quad = EncoderStep;
    if (quad <-EncoderStep) quad = -EncoderStep;
    //put code here for asigning encoder values like freq = freq + long((encodercount * fstep)/EncoderStep); 
    //communicate to the rest of the program the encoder change like: f_dchange=1 and f_fchange=1
    //check for max an min values
    encodercount += quad/EncoderStep;
    freq = freq + long((quad * fstep)/EncoderStep);
    quad = 0;
  } 
  if (encodercount != encodercountlast) {
  Serial.print(freq); Serial.print("  ");
  Serial.println(encodercount);
  //set_frequency();
  f_dchange=1;
  Transfer_Image();
  display_write();
  encodercountlast = 0; encodercount =0;
  }
}
}


void setstep() {                                /*procedure to set (advance) the step frequency*/

  switch (stp) {  //used by encoder switch if present
    case 1: stp = 2; fstep = 1000; break;
    case 2: stp = 3; fstep = 500; break;
    case 3: stp = 1; fstep = 100; break;
  }
}
void setstep1() {                                /*procedure to set (advance) the step frequency*/
  switch (stp) {  //used by pushbutton to change frequency step
    case 1: stp = 2; fstep = 1000; break;
    case 2: stp = 3; fstep = 500; break;
    case 3: stp = 4; fstep = 100; break;
    case 4: stp = 1; fstep = 10; break;
  }
}

void band_change() {                               /*procedure to set (advance) the frequency band*/
  bandsave();                                     /*procedure to save current band frequency to varriable freqa[array]*/
  count = ActiveBand;                                        /*increase the band count variable*/
  bandpresets();                                  /*load the last current band frequency variable freqa[array] into DDS*/
  f_dchange=1;  
  mem_count = 1;
  delay(5);                                       /*5 ms delay*/
}


void scan() {                                             //procedure to scan frequency
    freq = freq + 1000;                                   //change frequency up 1kHz
    if (freq >= endofBand[count-1]) freq = begofBand[count-1];
    set_frequency();
    f_dchange=1;
    Transfer_Image();
    display_write();
}

void freqrecall(){                               //code for recalling the saved frequencies 
  switch (recall_count) {                        //code for saving existing frequency
    case 1: freqa[count-1] = freq;recall_count=2;break;
    case 2: freqb[count-1] = freq;recall_count=3;break;
    case 3: freqc[count-1] = freq;recall_count=1;break;
  }
  switch (recall_count) {                        //code for recalling saved frequencies
    case 1: freq = freqa[count-1];break;
    case 2: freq = freqb[count-1];break;
    case 3: freq = freqc[count-1];break;
  }
  set_frequency();
  f_dchange=1;
  Transfer_Image();
  display_write();
  delay(5);
}

void display_step() {                            //code to display step frequency on screen
  switch (fstep) {
    case 10: sprintf(step_str,  "10 Hz");break;    
    case 100: sprintf(step_str, "100Hz");break;
    case 500: sprintf(step_str, "500Hz");break;
    case 1000: sprintf(step_str,"1 kHz");break;

  } 
}

void display_mem_count() {                      //code to display the current memory A, B, or C
  switch (recall_count) {
    case 1: sprintf(mem_str, "M:A");break;
    case 2: sprintf(mem_str, "M:B");break;
    case 3: sprintf(mem_str, "M:C");break;
  } 
}

void display_time() {                          //code to display the current UTC time on screen
  DateTime now = rtc.now();
  delay(5);
  char buf2[] = "hh:mm";
  now.toString(buf2);
  Serial.println(now.toString(buf2));        //print to console for debugging
  sprintf(time_str,buf2);
}

void display_band() {                           //code to display the current selected band
  switch(count) {
    case 1: sprintf(band_str, " 80m"); break;
    case 2: sprintf(band_str, " 40m"); break;
    case 3: sprintf(band_str, " 20m"); break;  //set to 20M position USB
    case 4: sprintf(band_str, " 15m"); break;
    case 5: sprintf(band_str, " 10m"); break;

  } 
}

void bandlist() {                //procedure for displaying the correct band frequency & calibration of each band
  if (count == 1) interfreq =  IF - 0;       //count = 1 (80m) on the display screen
  if (count == 2) interfreq =  IF - 0;       //count = 2 (40M) IF frequency for LSB (7274.97 subtract additional 30)
 /* if (count == 3) interfreq= IF - 0;       //count = 3 (WWV on 40m) + or - as needed to calibrate radio */
  if (count == 3) interfreq = -IF + 0;       //count = 4 (20m) IF Frequency for USB starting with 20m (8605000)
  if (count == 4) interfreq = -IF + 0;       //count = 5 (15) on the display screen
  if (count == 5) interfreq = -IF + 0;       //count = 6 (10m) on the display screen
  frq = freq + interfreq;  //in this case for 20m or 14250000 - IF to calibrate radio (8605190)
}


void set_frequency() {                          /*procedure for setting output frequency*/
    bandlist();
    if (on_off_flag) si5351.set_freq((frq) * 100ULL, VFO_CLOCK);
}

void bandpresets() {                            /*procedure for setting current frequency variable freq */
  freq = freqa[count-1];
  recall_count = recall[count-1];
  set_frequency();
  delay(5);
  stp = 1;
  setstep();
}

void bandsave() {                 /*procedure to save the current frequency into frequency A variable freqa[array]*/
  freqa[count-1] = freq;
  recall[count-1] = recall_count;
}

void display_write() {
  char str[64];    
    if(f_dchange==1){     //check if display change variable is true then enter this section of code
        f_dchange=0;        
        //GRAM_clr();
        boxfill(0,0,Nx-1,Ny-1,cl_BG);     
        //---------------- Display Dial ------------------------------------------
        Dial(freq);
        box(7,105,153,84, 0x00FE00);
        box(6,106,154,85, 0x00FE00);
        //-------- Display Digital Frquency and other information -----------------
        sprintf(str, "%3d,%03d,%03d",  freq/1000000, (freq/1000)%1000, (freq)%1000 );        
        disp_str16(str,8, 88, 0x00FFFF);              
        if (on_off_flag) sprintf(str, "MHz" ); else sprintf(str, "OFF" );
        disp_str12(str,120, 89, 0x00FFFF);
        display_band();
        disp_str8(band_str, 0, 73, 0x00FE00);
        display_step();
        disp_str8(step_str, 38, 73, 0x00FE00);
        display_time();
        disp_str8(time_str, 88, 73, 0x00FE00);
        display_mem_count();
        disp_str8(mem_str, 136, 73, 0x00FE00);
        
        trans65k();
        Transfer_Image();
        //f_redraw=1;
         
    }
}
