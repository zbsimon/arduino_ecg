/* Zach Simon Francis Ge (zbsimon, fge3)
 * CSE 466 Final Project
 */

#include <Wire.h>
#include <Time.h>
#include <SPI.h>
#include <Bounce.h>
#include <SdFat.h> // enable reading SD card on TFT
#include "ILI9341_t3.h"

#define SD_CS 4  // sd card chip select pin
#define SAMPLE_RATE 250  // samle rate in hz
#define TFT_DC 9  // lcd dc pin
#define TFT_CS 10 // lcd cd pin
#define PDB_CH0C1_TOS 0x0100  // timer tos mask
#define PDB_CH0C1_EN 0x01  // timer timr enable mask
#define NUM_DMA_SAMPLES 16  // buffer size that dma places adc's output in
#define NUM_SAMPLES (30 * SAMPLE_RATE)  // buffer size for 30 secs of data
#define Y_PIN 22  // Pin for y-axis data from joystick
#define B_PIN 21  // Pin for button clicks from joystick

// states for different modes of operation.
#define MENU 1
#define RECORD 2
#define REPLAY 3


static const uint8_t channel2sc1a[] = {
	5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
	0, 19, 3, 21, 26, 22
};
static const int ecg_input_pin = 17;  // our analog input


// constants for graphics calculations
static const int SAMPLES_TO_DRAW = 320;  // screen width
static const int BASELINE = 2000;
static const int MAX = 4095;
static const int HALF_MAX = 2048;
static const int MIN = 0;
static const int GRID_SIZE = 20;
static const int SCREEN_HEIGHT = 240;
static const int SCREEN_WIDTH = 320;
static const int NUM_HLINES = SCREEN_WIDTH / GRID_SIZE;
static const int SAMPLES = 5;
static const int max_width = 320;
static const int max_height = 240;


// data logging format strings
static const char *filename_template = "ecg_log_data_%d.csv";
static const char *header_template = "zsfg:%d at %d Hz\n";
static const char *log_line_template = "%d, %d, %d, %d, %d, %d, %d, %d\n";
static const char *eof = "EOF";
static const char *num_files_filename = "index";
static char newline_delim[] = "\n";
static char subline_delim[] = ", \n";


// Big Blob o Globals:
int HLineHeights[NUM_HLINES];  // locations of horizontal lines for drawing grid
bool VLinePositions[SCREEN_WIDTH];  // locations of vertical lines for drawing grid
bool stabilized = false; // whether we have enough data to start filtering
bool actualy_stabilized = true; // whether heart rate has stabilized
bool writeToSDCard = false; // whether we've collected enough data to write to sd card (30 seconds worth)
int old_drawX = 0;  // last loop's X val
int old_height = 0;  // last loop's height
uint16_t peak_data[SAMPLES_TO_DRAW];  // locations of QRS peaks.
float BPM = 0;  // our heart rate estimate.
volatile int current_line = 0;  // which menu line cursor is on
volatile int current_state = MENU;  // which mode of operation we are in
boolean first_display = true;  // if this is the first time we are drawing screen
int old_scroll_state = 0;  // what the previous scroll value was.

// NOTE: not all of these need to be volatile, but we were experimenting with
// accessing them in ISRs so we want to ensure that they aren't cached in regs

volatile bool dma_occurred = false;  // flag to know in loop when we recieved fresh data

// ********************************************
// *HERE IS WHERE OUR DATA IS ACTUALLY STORED!*
// ********************************************
volatile uint16_t dma_buffer[NUM_DMA_SAMPLES]; // Buffer used to store ADC data
volatile uint16_t samples[NUM_SAMPLES]; // Buffer that stores 30 seconds of raw data
uint16_t samples_processed[NUM_SAMPLES]; // Buffer that stores 30 seconds of processed data
volatile uint16_t* buff_front = samples; // Pointer to where we are writing
uint16_t* sd_front = samples_processed;
volatile uint16_t* baseRead = samples_processed; // Pointer to the beginning of the chunk we are drawing
volatile uint16_t* readIndex = samples_processed; // Pointer to the part of the chunk we are drawing


// Our lcd display object
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

SdFat sd;  // our sd fs interface

// debounce button pressed on the joystick
static const long debounce_delay = 50;
Bounce joystick_button = Bounce(B_PIN, debounce_delay);

void setup() {
  // Initialize Pins
  pinMode(ecg_input_pin, INPUT);

  // Initialize Serial
  Serial.begin(9600);

  // Initialize PDB/ADC/DMA (i.e. start collecting/processing data)
  pdbInit();
  adcInit();
  dmaInit();

  // Intialize LCD and Graph
  reset_data();
  initialize_lcd_graph();
  display_stabilization_screen();

  // Initialize SD Card
  initialize_sd_card();

  // uncomment this when running with startup menu / filebrowser
  // to initialize state
  //change_state(MENU);
}


void loop() {

  record_ekg_data();

  /*
  // here's our code for when we include a menu and file browswer

  if (current_state == MENU) {
    display_options();
  } else if (current_state == REPLAY) {
    replay_ekg_data(current_line);
  } else {
    record_ekg_data();
  }
  */


}


// Start up our SdFat library so we can talk to the sd card
// on our lcd display over spi. block on failure.
void initialize_sd_card() {
  if (!sd.begin(SD_CS, SPI_HALF_SPEED)) sd.initErrorHalt();
}


// write data to the SD card in 30-second increments, iff we have 30 secs of
// samples to write
void beginWriteToSDCard() {
    // If we've collected 30 seconds of data, write to sd card
    if (writeToSDCard) {
      // reset flag
      writeToSDCard = false;

      // let people know why there might be a pause.
      Serial.println("Writing to SD CArd");

      // make sure our samples aren't overwritten
      NVIC_DISABLE_IRQ(IRQ_DMA_CH1);


      // create and open log file
      char header[30];
      char filename[30];
            int new_uid = increment_number_of_files();
      sprintf(header, header_template, new_uid, SAMPLE_RATE);
      sprintf(filename, filename_template, new_uid);
      SdFile data_file;  // log file object
      if (!data_file.open(filename, O_WRITE | O_READ | O_CREAT)) {
        sd.errorHalt("opening log file failed");
        return;
      }

      // Now, we write to the file in the format described in the spec:
      // header line
      // lines of at most 8 samples
      // eof line


      // write header
      data_file.write(header);

      // since there's no guarantee that the last line will have exactly
      // 8 samples, we do the last line outside of the for loop to account
      // for the last NUM_SAMPLES % 8 samples.

      volatile uint16_t *sample;
      // while we have at least 8 samples to write, write 8 samples.
      for (sample = samples;
           sample < samples + NUM_SAMPLES - 8;
           sample +=8) {
        char log_line[100];
        sprintf(log_line, log_line_template, sample, sample + 1, sample + 2,
                sample + 3, sample + 4, sample + 5, sample + 6, sample + 7);
        data_file.write(log_line);
      }

      // write the remaining NUM_SAMPLES % 8 samples to the last line of data.
      while (sample < samples + NUM_SAMPLES - 1) {
        char * templ = "%d, ";
        char sample_text[10];
        sprintf(sample_text, templ, sample);
        Serial.println(sample_text);
        data_file.write(sample_text);
        sample++;
      }


      // write eof and clean up / close fds
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


// clear our data stores, reset state
void reset_data() {
  // zero out sample buffers.
  for (int i = 0; i < NUM_DMA_SAMPLES; i++) {
    samples[i] = 0;
    samples_processed[i] = 0;
  }

  buff_front = samples;
  sd_front = samples_processed;
  baseRead = buff_front;
  readIndex = baseRead;
  stabilized = false;
}


// display a "stabilizing" message while we wait for our samples
// to stabilize
void display_stabilization_screen() {
  tft.fillScreen(ILI9341_WHITE);
  tft.setRotation(3);
  tft.setTextSize(3);
  tft.setCursor(40, 100);
  tft.setTextColor(ILI9341_BLACK);
  tft.println("Stabilizing...");
  tft.setRotation(2);
}


// draws an individual column of data to the lcd. subsequent call will refresh
// column to the right.
void drawColumn() {

  // The current number of samples we've read since the baseRead
  int samplesRead = SAMPLES_TO_DRAW - ((baseRead + SAMPLES_TO_DRAW) - readIndex);

  // Get the sample
  int sample = *(samples_processed + samplesRead);

  // Get the difference from the baseline
  int differenceFromBase = sample - BASELINE;
  // Turn this into pixels
  int normalizedDiff = abs(differenceFromBase) / (float)HALF_MAX * 120;
  // Subtract or add it onto the pixel level of the baseline
  int height = 120;
  if (differenceFromBase < 0) height -= normalizedDiff;
  else height += normalizedDiff;

  // Draw the grid
  if (actualy_stabilized) {
    if (VLinePositions[samplesRead])
      tft.fillRect(0, samplesRead, 240, 1, ILI9341_RED);
    else
      tft.fillRect(0, samplesRead, 240, 1, ILI9341_WHITE);
    for (int i = 0; i < NUM_HLINES; i++) {
      tft.drawPixel(HLineHeights[i], samplesRead, ILI9341_RED);
    }
    // Draw the graph
    tft.drawLine(old_height, old_drawX, height, samplesRead, ILI9341_BLACK);
    old_drawX = samplesRead;
    old_height = height;
    // Draw the heartbeat
    if (peak_data[samplesRead] == 1) {
      tft.fillRect(0, samplesRead, 240, 1, ILI9341_BLUE);
    }
  }

  // Advance the readIndex, but only if we're not viewing a log file.
  if (current_state != REPLAY) {
    readIndex++;
  }

  // If we'd read X amount past the base, advance the base
  // Basically so we're reading a chunk of data at a time
  if (readIndex >= baseRead + SAMPLES_TO_DRAW) {
    baseRead = buff_front - SAMPLES_TO_DRAW;
    readIndex = baseRead;
    old_drawX = 0;
    old_height = 0;
    // If the buff_front causes readIndex to be behind the front of the buffer,
    // wrap it around from the end of the buffer
    if (readIndex < samples) {
      readIndex = (samples + NUM_SAMPLES) - (samples - readIndex);
    }

    uint16_t temp_buffer[SAMPLES_TO_DRAW];
    uint16_t temp_buffer2[SAMPLES_TO_DRAW];
    copySamples(temp_buffer2);

    float blurFilter[] = {0.3333, 0.3333, 0.3333};
    float laplacian[] = {1, -2, 1};
    // Smooth out the data
    medianFilter(temp_buffer2, temp_buffer, SAMPLES_TO_DRAW);
    convolution(blurFilter, 3, temp_buffer, temp_buffer2, SAMPLES_TO_DRAW, 0);
    convolution(blurFilter, 3, temp_buffer2, temp_buffer, SAMPLES_TO_DRAW, 0);
    // Calculate Acceleration
    convolution(laplacian, 3, temp_buffer, temp_buffer2, SAMPLES_TO_DRAW, BASELINE);
    // Find Peaks
    findPeaks(temp_buffer2, peak_data, SAMPLES_TO_DRAW);

    memcpy(sd_front, temp_buffer, SAMPLES_TO_DRAW * sizeof(uint16_t));
    // sd_front += SAMPLES_TO_DRAW;

    calculateBPM(SAMPLES_TO_DRAW);
  } else if (readIndex >= samples + NUM_SAMPLES) {
    // Wrap around the end of the array
    readIndex = samples;
  }

}


// copy data from raw sample input into a new location so we can process it without
// overwriting raw data for subsequent processing.
void copySamples(uint16_t* dst) {
  // Copy raw data into processed_array
  // Check if our read_head is in the negative
  NVIC_DISABLE_IRQ(IRQ_DMA_CH1);
  if (baseRead < samples) {
    // Handle wrap around case
    // (cast from volatile since interrupts disabled)
    volatile uint16_t* startCpy = (samples + NUM_SAMPLES) - (samples - baseRead);
    memcpy((uint16_t *) dst, (uint16_t *) startCpy, (samples - baseRead) * sizeof(uint16_t));
    memcpy((uint16_t *) dst, (uint16_t *) samples, (baseRead - samples) * sizeof(uint16_t));
  } else {
    memcpy((uint16_t *)dst, (uint16_t *)baseRead, SAMPLES_TO_DRAW * sizeof(uint16_t));
  }
  NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
}

// 1D Convolution Operation - reads from src, and puts into dst. len represents number of values to read and write.
// Filter should be of odd size, e.g. [1/5, 1/5, 1/5, 1/5, 1/5, 1/5]
void convolution(float* filter, int filterSize, uint16_t* src, uint16_t* dst, int len, int hello_world_1234) {
  // How many values in the filter
  int filterSize_half = filterSize / 2;
  uint16_t* _index;
  uint16_t* _index2;

  for (_index = src; _index < src + len; _index++) {
    int newReading = 0;
    int filterIndex = 0;
    // Get the 'order' number of values centered at i
    for (_index2 = _index - filterSize_half; _index2 <= _index + filterSize_half; _index2++) {
      uint16_t* temp_i = _index2;
      // Handle boundary conditions
      if (temp_i < src) temp_i = src;
      if (temp_i >= src + len) temp_i = src + len - 1;
      newReading += *temp_i * filter[filterIndex];
      filterIndex++;
    }
    *dst = newReading + hello_world_1234;
    dst++;
  }
}


// Calculate the beats per minutes based on moving average distance between peaks
void calculateBPM(int len) {
  int prevPeakIndex = -1;
  for (int i = 0; i < len - 1; i++) {
    if (peak_data[i] == 1) {
      if (prevPeakIndex == -1) {
        prevPeakIndex = i;
      } else {
        // Difference in terms of how far apart the samples are
        float difference = i - prevPeakIndex;
        float timePerSample = 1 / 250.0f;
        float deltaT = difference * timePerSample;
        float freqMinute = 60 / deltaT;
        // Compute the moving average of the BPM over 2 beats
        BPM = (BPM * 2 + freqMinute - BPM) / 2;
        prevPeakIndex = i;
        Serial.println(BPM);
        if (BPM < 60)
          Serial.println("Bradycardia");
        else if (BPM > 100)
          Serial.println("Tachycardia");
        else {
          actualy_stabilized = true;
        }
      }
    }
  }
}


// filter to replace each sample with the median of the values in
// the range of length len centered at that sample. filters data
// from src into dst.
void medianFilter(uint16_t* src, uint16_t* dst, int len) {
  // How many values in the filter
  int filterSize = 3;
  int filterSize_half = filterSize / 2;
  uint16_t* _index;
  uint16_t* _index2;

  for (_index = src; _index < src + len; _index++) {
    int newReading = 0;
    int filterIndex = 0;
    int values[3];
    // Get the 'order' number of values centered at i
    for (_index2 = _index - filterSize_half; _index2 <= _index + filterSize_half; _index2++) {
      uint16_t* temp_i = _index2;
      // Handle boundary conditions
      if (temp_i < src) temp_i = src;
      if (temp_i >= src + len) temp_i = src + len - 1;
      values[filterIndex] = *temp_i;
      filterIndex++;
    }

    // Sort the values
    int x = values[0];
    int y = values[1];
    int z = values[2];

    *dst = values[1];
    dst++;
  }
}



// Find peaks in the graph based on the second derivative
// src - samples to read from
// dst - place to store data in 1 or 0 peak or not
// len - number of samples
void findPeaks(uint16_t* src, uint16_t *dst, int len) {
  float avgAccel = BASELINE;
  float threshHold = 40; // threshold for spike detection

  // We look at the second derivative + BASELINE_OFFSET values
  // A peak is detected as a negative spike in these spikes
  for (int i = 0; i < len; i++) {
    float accelReading = src[i];
    // Compute a moving average as the baseline to compare against
    avgAccel = (avgAccel * SAMPLES + accelReading - avgAccel) / SAMPLES;
    // Compare the reading with the averaged baseline readings
    float differential = (avgAccel - accelReading);
    // A peak is detected if this difference is larger than the threshold.
    dst[i] = differential > threshHold ? 1 : 0;
  }

  // Pack the peak data
  for (int i = 0; i < len - 1; i++) {
    if ((dst[i + 1] == 1) && (dst[i] == 1)) {
      dst[i] = 0;
    }
  }
}


// ADC / PDB / DMA setup code

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


// Our interrupt service handlers:

void adc0_isr() {
  // nothing to do here
}

void pdb_isr() {
  // Clear interrupt flag
  PDB0_SC &= ~PDB_SC_PDBIF;
}

void dma_ch1_isr() {
  NVIC_DISABLE_IRQ(IRQ_DMA_CH1);


  // copy each new sample into "userspace" buffer.
  for (int i = 0; i < NUM_DMA_SAMPLES; i++) {
    *buff_front = dma_buffer[i];
    buff_front++;
    // Stabilize after getting 320 samples
    if (buff_front > (samples + 320)) stabilized = true;
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



// FS code

// increment the nubmer of files stored on our sd card, since
// this little bit of metadata is stored as well. returns the new
// number of files.
int increment_number_of_files() {
  SdFile metadata;
  if (!metadata.open(num_files_filename, O_WRITE | O_READ | O_CREAT)) {
    sd.errorHalt("error opening metadatafile");
  }

  char metadata_contents[10];
  int res = metadata.fgets(metadata_contents, 10);
  if (res <1) {
    sd.errorHalt("error reading metadata file");
  }
  int count = atoi(metadata_contents) + 1;
  char new_contents[20];
  Serial.printf("count: %d\n", count);
  sprintf(new_contents, "%d", count);
  metadata.rewind();
  metadata.write(new_contents, strlen(new_contents));
  metadata.sync();
  metadata.close();
  return count;
}


// display a menu which allows users to begin an ekg reading or view
// previously recorded sample data using the joystick.
void display_options() {
  int move_cursor = scroll_direction();
  int num_logs = get_number_of_files();

  // update current line if user scrolled.
  if (move_cursor == -1 && current_line < num_logs) {
    current_line++;
  } else if (move_cursor == 1 && current_line > 0)  {
    current_line--;
  }


  // select current line's state as the new state if
  // they pressed the button.
  if (button_pressed()) {
    stabilized = false;
    tft.fillScreen(ILI9341_WHITE);
    if (current_line == 0) {
      change_state(RECORD);
    } else {
      change_state(REPLAY);
    }
    tft.setRotation(2);
    return;
  }

  // draw the menu.
  if (move_cursor || first_display) {
    first_display = false;
    tft.setRotation(3);
    tft.setCursor(5, 0);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK);
    tft.fillScreen(ILI9341_WHITE);
    if (current_line == 0){
      tft.setTextColor(ILI9341_RED);
      tft.println("record ekg data");
      tft.setTextColor(ILI9341_BLACK);
    } else {
      tft.println("record ekg data");
    }

    for (int i = 1; i <= num_logs; i++) {
      if (current_line == i) {
        tft.setTextColor(ILI9341_RED);
        tft.printf("view ecg_log_data_%d.csv\n", i);
        tft.setTextColor(ILI9341_BLACK);
      } else {
        tft.printf("view ecg_log_data_%d.csv\n", i);
      }

    }
    tft.setRotation(2);
  }
}


// returns current number of files stored in
int get_number_of_files() {
  SdFile metadata;
  if (!metadata.open(num_files_filename, O_WRITE | O_READ | O_CREAT)) {
    sd.errorHalt("error opening metadatafile");
  }
  char metadata_contents[10];
  int res = metadata.fgets(metadata_contents, 10);
  if (res <1) {
    sd.errorHalt("error reading metadata file");
  }
  int count = atoi(metadata_contents);

  metadata.close();
  return count;
}


// button / joystick code


// returns a value indicating whether the joystick is being pressed moved.
// -1 = down, 0 = no scroll, 1 = up.
int scroll_direction() {

  // get raw  value.
  int val = analogRead(Y_PIN);
  int new_state = 0;

  // check if it meets the cut-offs.
  if (val < 500) {
    new_state = 1;
  } else if (val > 3300) {
    new_state = -1;
  }

  // see if it has changed.
  int reading = 0;
  if (new_state == 1 && old_scroll_state != 1) {
    reading = 1;
  } else if (new_state == -1 && old_scroll_state != -1) {
    reading = -1;
  } else {
    reading = 0;
  }
  old_scroll_state = new_state;
  return reading;
}


// returns true iff button has been pressed during last loop.
boolean button_pressed() {
  if (joystick_button.update() && joystick_button.fallingEdge()) {
    return true;
  } else {
    return false;
  }
}


// view a previously recorded file on the lcd
void replay_ekg_data(int file) {

  // We can re-use almost all of the code for this from the "record"
  // state, because we just need to fill the sample buffer we are drawing
  // from with the data we want to replay, and then update where we are drawing
  // based on user input.

  if (first_display) {
    copy_file_to_sample_buff(current_line);
    first_display = false;
    baseRead = samples;
    readIndex = samples;
  }

  // draw one line of the currently visible data.
  drawColumn();

  // if the button is pressed, go back to the menu.
  if (button_pressed()) {
    reset_data();
    change_state(MENU);
  }

  // update the page of data that we are viewing
  int scroll_val = scroll_direction();
  if (scroll_val) {
    baseRead += (SCREEN_WIDTH * scroll_val) % NUM_SAMPLES;
    readIndex = baseRead;
  }
}


// updates the state to change the mode of operation
// for the next loop.
void change_state(int new_state) {
  if (new_state == MENU) {
    NVIC_DISABLE_IRQ(IRQ_DMA_CH1);
    first_display = true;
    current_state = MENU;
  } else if (new_state == RECORD) {
    reset_data();
    display_stabilization_screen();
    NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
    current_state = RECORD;
  } else if (new_state == REPLAY) {
    NVIC_DISABLE_IRQ(IRQ_DMA_CH1);
    first_display = true;
    current_state = REPLAY;
  }
}


// fill up the buffer we draw from with data from a previous run
// stored on sd card
void copy_file_to_sample_buff(int current_line) {
  char filename[50];
  sprintf(filename, filename_template, current_line);
  SdFile current_file;
  if (current_file.open(filename, O_WRITE | O_READ | O_CREAT)) {
    sd.errorHalt("error opening log file");
  }
  uint16_t * current_sample = samples_processed;
  char line_buff[100];
  char *next_token;
  char *tmp;
  int res = current_file.fgets(line_buff, 100, newline_delim);

  // parse lines and place samples in array.
  for (int i = 0; i < NUM_SAMPLES / 8 + 1; i++) {
    int res = current_file.fgets(line_buff, 100, newline_delim);
    next_token = strtok_r(line_buff, subline_delim, &tmp);
    while (next_token != NULL) {
      *current_sample = atoi(next_token);
      current_sample++;
      next_token = strtok_r(NULL, subline_delim, &tmp);
    }
  }
  current_file.close();
}



// perform one iteration of the steps needed to sample, filter and draw
// live data to the lcd and log it to the sd card when 30 secs have been
// collected.
void record_ekg_data() {
  // if we've collected enough data to start processing,
  if (stabilized)
    // refresh the next column of the graph on the lcd
    drawColumn();

  // If we have fresh data in our buffer
  if (dma_occurred) {
    // reset flag
    dma_occurred = false;
    // write data to sd card if our buffer is full
    beginWriteToSDCard();
  }
}
