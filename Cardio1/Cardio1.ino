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

void setup() {

  tft.begin(); // Initialize the display
  tft.setRotation(1); //  0 or 2) width = width, 1 or 3) width = height, swapped etc.
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
  Serial.begin(9600);
  while (!Serial); // wait until the serial debug window is opened

  adcInit();
  pdbInit();
  dmaInit();
}

void loop() {

}


/*
	ADC_CFG1_ADIV(2)         Divide ratio = 4 (F_BUS = 48 MHz => ADCK = 12 MHz)
	ADC_CFG1_MODE(2)         Single ended 10 bit mode
	ADC_CFG1_ADLSMP          Long sample time
*/
#define ADC_CONFIG1 (ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(2) | ADC_CFG1_ADLSMP)

/*
	ADC_CFG2_MUXSEL          Select channels ADxxb
	ADC_CFG2_ADLSTS(3)       Shortest long sample time
*/
#define ADC_CONFIG2 (ADC_CFG2_MUXSEL | ADC_CFG2_ADLSTS(3))

void adcInit() {
	ADC0_CFG1 = ADC_CONFIG1;
	ADC0_CFG2 = ADC_CONFIG2;
	// Voltage ref vcc, hardware trigger, DMA
	ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG | ADC_SC2_DMAEN;

	// Enable averaging, 4 samples
	ADC0_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(0);

	adcCalibrate();
	Serial.println("calibrated");

	// Enable ADC interrupt, configure pin
	ADC0_SC1A = ADC_SC1_AIEN | channel2sc1a[3];
	NVIC_ENABLE_IRQ(IRQ_ADC0);
}

void adcCalibrate() {
	uint16_t sum;

	// Begin calibration
	ADC0_SC3 = ADC_SC3_CAL;
	// Wait for calibration
	while (ADC0_SC3 & ADC_SC3_CAL);

	// Plus side gain
	sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
	sum = (sum / 2) | 0x8000;
	ADC0_PG = sum;

	// Minus side gain (not used in single-ended mode)
	sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
	sum = (sum / 2) | 0x8000;
	ADC0_MG = sum;
}

/*
	PDB_SC_TRGSEL(15)        Select software trigger
	PDB_SC_PDBEN             PDB enable
	PDB_SC_PDBIE             Interrupt enable
	PDB_SC_CONT              Continuous mode
	PDB_SC_PRESCALER(7)      Prescaler = 128
	PDB_SC_MULT(1)           Prescaler multiplication factor = 10
*/
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
	| PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1))

// 48 MHz / 128 / 10 / 1 Hz = 37500
#define PDB_PERIOD (F_BUS / 128 / 10 / 1)

void pdbInit() {
	pinMode(13, OUTPUT);

	// Enable PDB clock
	SIM_SCGC6 |= SIM_SCGC6_PDB;
	// Timer period
	PDB0_MOD = PDB_PERIOD;
	// Interrupt delay
	PDB0_IDLY = 0;
	// Enable pre-trigger
	PDB0_CH0C1 = PDB_CH0C1_TOS | PDB_CH0C1_EN;
	// PDB0_CH0DLY0 = 0;
	PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
	// Software trigger (reset and restart counter)
	PDB0_SC |= PDB_SC_SWTRIG;
	// Enable interrupt request
	NVIC_ENABLE_IRQ(IRQ_PDB);
}

void dmaInit() {
	// Enable DMA, DMAMUX clocks
	SIM_SCGC7 |= SIM_SCGC7_DMA;
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

	// Use default configuration
	DMA_CR = 0;

	// Source address
	DMA_TCD1_SADDR = &ADC0_RA;
	// Don't change source address
	DMA_TCD1_SOFF = 0;
	DMA_TCD1_SLAST = 0;
	// Destination address
	DMA_TCD1_DADDR = samples;
	// Destination offset (2 byte)
	DMA_TCD1_DOFF = 2;
	// Restore destination address after major loop
	DMA_TCD1_DLASTSGA = -sizeof(samples);
	// Source and destination size 16 bit
	DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
	// Number of bytes to transfer (in each service request)
	DMA_TCD1_NBYTES_MLNO = 2;
	// Set loop counts
	DMA_TCD1_CITER_ELINKNO = sizeof(samples) / 2;
	DMA_TCD1_BITER_ELINKNO = sizeof(samples) / 2;
	// Enable interrupt (end-of-major loop)
	DMA_TCD1_CSR = DMA_TCD_CSR_INTMAJOR;

	// Set ADC as source (CH 1), enable DMA MUX
	DMAMUX0_CHCFG1 = DMAMUX_DISABLE;
	DMAMUX0_CHCFG1 = DMAMUX_SOURCE_ADC0 | DMAMUX_ENABLE;

	// Enable request input signal for channel 1
	DMA_SERQ = 1;

	// Enable interrupt request
	NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
}


void adc0_isr() {
	Serial.print("adc isr: ");
	Serial.println(millis());
	for (uint16_t i = 0; i < 16; i++) {
		if (i != 0) Serial.print(", ");
		Serial.print(samples[i]);
	}
	Serial.println(" ");
}

void pdb_isr() {
	Serial.print("pdb isr: ");
	Serial.println(millis());
	digitalWrite(13, (ledOn = !ledOn));
	// Clear interrupt flag
	PDB0_SC &= ~PDB_SC_PDBIF;
}

void dma_ch1_isr() {
	Serial.print("dma isr: ");
	Serial.println(millis());
	// Clear interrupt request for channel 1
	DMA_CINT = 1;
}