/* Zach Simon Francis Ge (zbsimon, fge3)
 * CSE 466 lab 8
 */
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "ILI9341_t3.h"

static int sample_rate = 250; // in Hz

static int ecg_input_pin = 13;
#define TFT_DC 9
#define TFT_CS 10

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

const int max_width = 320;
const int max_height = 240;

int PDB_CONFIG = 0;

void setup() {

  tft.begin(); // Initialize the display
  tft.setRotation(1); //  0 or 2) width = width, 1 or 3) width = height, swapped etc.
  SIM_SCGC6 |= SIM_SCGC6_PDB;
  PDB0_MOD = 312; // 1 / (48Mhz / 128 / 10) * X = 0.00833333333 s
  PDB0_IDLY = 0;
  PDB_CONFIG = PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE | PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1);
  PDB0_SC = PDB_CONFIG;
  PDB0_SC |= PDB_SC_SWTRIG;
  PDB0_SC |= PDB_SC_LDOK;
  NVIC_ENABLE_IRQ(IRQ_PDB);
  // register  timer to fire every sample rate
  // in interupt handler, copy ecg into adc buffer
  // when adc finished interupt fires, copy data into "userspace" buff
  // in loop, just look at this "userspace" buff each time through loop
  // and update lcd if it's changed (prolly just wanna set flag each time
  // we copy in new data to buff and when we display it out so we only write
  // it once per update.)

  // setup lcd

  // setup adc

  // setup timer

  // setup ecg input
}

void loop() {

}