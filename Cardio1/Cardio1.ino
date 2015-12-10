/* Zach Simon Francis Ge (zbsimon, fge3)
 * CSE 466 lab 8
 */

#include <Wire.h>
#include <Time.h>
#include <SPI.h>
#include <SdFat.h> // enable reading SD card on TFT
#include "ILI9341_t3.h"

#define SD_CS 4

// samle rate in hz
#define SAMPLE_RATE 250

static int ecg_input_pin = 17;

static const uint8_t channel2sc1a[] = {
	5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
	0, 19, 3, 21, 26, 22
};

#define TFT_DC 9
#define TFT_CS 10

#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01

char *filename_template = "ecg_log_data_%d.csv";
char *header_template = "zsfg:%d at %d Hx\n";
char *log_line_template = "%d, %d, %d, %d, %d, %d, %d, %d\n";
char *eof = "EOF";

// We choke when it's this large, but I think we need this much
// space!
#define NUM_DMA_SAMPLES 16
#define NUM_SAMPLES (30 * SAMPLE_RATE)

uint16_t dma_buffer[NUM_DMA_SAMPLES]; // Buffer used to store ADC data
uint16_t samples[NUM_SAMPLES]; // Buffer that stores 30 seconds of data
uint16_t* buff_front = samples; // Pointer to where we are writing
uint16_t* baseRead = samples; // Pointer to the beginning of the chunk we are drawing
uint16_t* readIndex = samples; // Pointer to the part of the chunk we are drawing

const int SAMPLES_TO_DRAW = 320;
const int BASELINE = 2000;
const int MAX = 4095;
const int HALF_MAX = 2048;
const int MIN = 0;
const int GRID_SIZE = 20;
const int SCREEN_HEIGHT = 240;
const int SCREEN_WIDTH = 320;
const int NUM_HLINES = SCREEN_WIDTH / GRID_SIZE;
int HLineHeights[NUM_HLINES];
bool VLinePositions[SCREEN_WIDTH];
bool stabilized = false;
bool writeToSDCard = false;

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

SdFat sd;  // fs obj

const int max_width = 320;
const int max_height = 240;

bool dma_occurred = false;  // flag to know in loop when we recieved fresh data

void setup() {
  // Initialize Pins
  pinMode(ecg_input_pin, INPUT);

  // Initialize Serial and wait for Serial
  Serial.begin(9600);
  // while (!Serial);

  // Initialize PDB/ADC/DMA
  pdbInit();
  adcInit();
  dmaInit();

  // Intialize LCD and Graph
  reset_data();
  initialize_lcd_graph();
  display_stabilization_screen();

  // Initialize SD Card
  initialize_sd_card();
}

void loop() {
  if (stabilized)
    drawColumn();

  // need to wait for data to stabilize?
  // neet to wait for button to be pressed?
  if (dma_occurred) {
    dma_occurred = false;
    beginWriteToSDCard();
  }
}

// TODO
void initialize_sd_card() {
  if (!sd.begin(SD_CS, SPI_HALF_SPEED)) sd.initErrorHalt();
}

void beginWriteToSDCard() {
    // If we've collected 30 seconds of data, write to sd card
    // note that we don't even need to be collecting data at this point
    if (writeToSDCard) {
      writeToSDCard = false;
      Serial.println("Writing to SD CArd");
      NVIC_DISABLE_IRQ(IRQ_DMA_CH1);
      char header[30];
      char filename[30];
      sprintf(header, header_template, hourFormat12(), SAMPLE_RATE);
      sprintf(filename, filename_template, hourFormat12());
      Serial.println(header);
      Serial.println(filename);
      SdFile data_file;  // log file
      if (!data_file.open(filename, O_WRITE | O_READ | O_CREAT)) {
        sd.errorHalt("opening log file failed");
        return;
      }
      data_file.write(header);
      // since there's no guarantee that the last line will have exactly
      // 8 samples, we do the last line outside of the loop.
      uint16_t *sample;
      for (sample = samples;
           sample < samples + NUM_SAMPLES - 8;
           sample +=8) {
        char log_line[100];
        sprintf(log_line, log_line_template, sample, sample + 1, sample + 2,
                sample + 3, sample + 4, sample + 5, sample + 6, sample + 7);
        data_file.write(log_line);
      }
      while (sample < samples + NUM_SAMPLES - 1) {
        char * templ = "%d, ";
        char sample_text[10];
        sprintf(sample_text, templ, sample);
        Serial.println(sample_text);
        data_file.write(sample_text);
        sample++;
      }
      char * templ = "%d\n";
      char sample_text[10];
      sprintf(sample_text, templ, sample);
      data_file.write(sample_text);
      Serial.println(sample_text);
      data_file.write(eof);
      data_file.close();
      NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
    }
}

// Perform initial setup
void initialize_lcd_graph() {
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(2);

  // Calculate Line positions for horizontal lines
  for (int i = 1; i < NUM_HLINES - 1; i++) {
    HLineHeights[i] = i * GRID_SIZE;
  }

  // Calculate Line positions for vertical lines
  for (int i = 1; i < 320; i++) {
    VLinePositions[i] = (i % GRID_SIZE) == 0;
  }
}

void reset_data() {
  for (int i = 0; i < NUM_DMA_SAMPLES; i++) {
    samples[i] = 0;
  }

  buff_front = samples;
  baseRead = buff_front;
  readIndex = baseRead;
  stabilized = false;
}

void display_stabilization_screen() {
  tft.fillScreen(ILI9341_WHITE);
  tft.setRotation(3);
  tft.setTextSize(3);
  tft.setCursor(40, 100);
  tft.setTextColor(ILI9341_BLACK);
  tft.println("Stabilizing...");
  tft.setRotation(2);
}

void drawColumn() {
  // If the buff_front causes readIndex to be behind the front of the buffer,
  // wrap it around from the end of the buffer
  if (readIndex < samples) {
    readIndex = (samples + SAMPLES_TO_DRAW) - (samples - readIndex);
  }

  // Get the sample
  int sample = *readIndex;
  // Get the difference from the baseline
  int differenceFromBase = sample - BASELINE;
  // Turn this into pixels
  int normalizedDiff = abs(differenceFromBase) / (float)HALF_MAX * 120;
  // Subtract or add it onto the pixel level of the baseline
  int height = 120;
  if (differenceFromBase < 0) height -= normalizedDiff;
  else height += normalizedDiff;

  // The current number of samples we've read since the baseRead
  int samplesRead = SAMPLES_TO_DRAW - ((baseRead + SAMPLES_TO_DRAW) - readIndex);

  // Draw the grid
  if (VLinePositions[samplesRead])
    tft.fillRect(0, samplesRead, 240, 1, ILI9341_RED);
  else
    tft.fillRect(0, samplesRead, 240, 1, ILI9341_WHITE);
  for (int i = 0; i < NUM_HLINES; i++) {
    tft.drawPixel(HLineHeights[i], samplesRead, ILI9341_RED);
  }
  // Draw the graph
  tft.fillRect(height, samplesRead, 3, 3, ILI9341_BLACK);

  // Advance the readIndex
  readIndex++;
  // If we'd read X amount past the base, advance the base
  // Basically so we're reading a chunk of data at a time
  if (readIndex >= baseRead + SAMPLES_TO_DRAW) {
    baseRead = buff_front - SAMPLES_TO_DRAW;
    readIndex = baseRead;
  }

  // delay(4);
}

/*
	ADC_CFG1_ADIV(2)         Divide ratio = 4 (F_BUS = 48 MHz => ADCK = 12 MHz)
	ADC_CFG1_MODE()         Single ended 12 bit mode
	ADC_CFG1_ADLSMP          Long sample time
*/
#define ADC_CONFIG1 (ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP)

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
	// ADC0_SC1A = ADC_SC1_AIEN | ecg_input_pin;
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
#define PDB_PRESCALE 7
#define PDB_MULT_VAL 1
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
	| PDB_SC_CONT | PDB_SC_PRESCALER(PDB_PRESCALE) | PDB_SC_MULT(PDB_MULT_VAL))



// we already have the frequency we want stored in sample_rate
// which is 250.
#define PDB_PERIOD (F_BUS / 128 / 10 / SAMPLE_RATE)

void pdbInit() {

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
	DMA_TCD1_DADDR = dma_buffer;
	// Destination offset (2 byte)
	DMA_TCD1_DOFF = 2;
	// Restore destination address after major loop
	DMA_TCD1_DLASTSGA = -sizeof(dma_buffer);
	// Source and destination size 16 bit
	DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
	// Number of bytes to transfer (in each service request)
	DMA_TCD1_NBYTES_MLNO = 2;
	// Set loop counts
	DMA_TCD1_CITER_ELINKNO = sizeof(dma_buffer) / 2;
	DMA_TCD1_BITER_ELINKNO = sizeof(dma_buffer) / 2;
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
}

void pdb_isr() {
  // Clear interrupt flag
  PDB0_SC &= ~PDB_SC_PDBIF;
}

void dma_ch1_isr() {
  NVIC_DISABLE_IRQ(IRQ_DMA_CH1);

  for (int i = 0; i < NUM_DMA_SAMPLES; i++) {
    *buff_front = dma_buffer[i];
    buff_front++;
    // Stabilize after getting 320 samples
    if (buff_front > (samples + 640)) stabilized = true;
    // Use the buffer as a circular buffer
    if (buff_front > (samples + NUM_SAMPLES)) {
      buff_front = samples;
      writeToSDCard = true;
    }
  }
  // Clear interrupt request for channel 1
  DMA_CINT = 1;
  dma_occurred = true;

  NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
}
