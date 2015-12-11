/* Zach Simon Francis Ge (zbsimon, fge3)
 * CSE 466 Final Project
 */

#include <Wire.h>
#include <Time.h>
#include <SPI.h>
#include <Bounce.h>
#include <string.h>
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
char *header_template = "zsfg: %d at %d Hz\n";
char *log_line_template = "%hu, %hu, %hu, %hu, %hu, %hu, %hu, %hu\n";
char *eof = "EOF";

#define NUM_DMA_SAMPLES 16
#define NUM_SAMPLES (30 * SAMPLE_RATE)

#define SAMPLES_TO_AVERAGE 3

uint16_t dma_buffer[NUM_DMA_SAMPLES]; // Buffer used to store ADC data
uint16_t samples[NUM_SAMPLES]; // Buffer that stores 30 seconds of data
uint16_t sd_data[NUM_SAMPLES]; // to store data read from sd card
uint16_t* buff_front = samples; // Pointer to where we are writing in raw data
uint16_t* sd_buff_front = sd_data; // Pointer to where we are writing in sd/avg buff
uint16_t* baseRead = sd_data; // Pointer to the beginning of the chunk we are drawing
uint16_t* readIndex = sd_data; // Pointer to the part of the chunk we are drawing

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

// the endpoint of the line segment we're currently plotting
int last_height = 0;

// our buffer that stores the approximate 2nd deriv wrt t
uint16_t sample_acceleration[SAMPLES_TO_DRAW];


// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

SdFat sd;  // fs obj
SdFile root;

const int max_width = 320;
const int max_height = 240;

bool dma_occurred = false;  // flag to know in loop when we recieved fresh data

// joystick setup
#define Y_PIN 22
#define B_PIN 21

boolean last_button_state = false;
volatile uint16_t y_val = 0;
static long debounce_delay = 50;

Bounce joystick_button = Bounce(B_PIN, debounce_delay);


#define MENU 1
#define RECORD 2
#define REPLAY 3

volatile int current_line = 0;
volatile int current_state = MENU;
boolean first_display = true;


void setup() {
  // Initialize Pins
  pinMode(ecg_input_pin, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(B_PIN, INPUT_PULLUP);

  // Initialize Serial and wait for Serial
  Serial.begin(9600);
  while (!Serial);

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
  //change_state(MENU);
  change_state(RECORD);
}



void loop () {
  //Serial.printf("current_state: %d\n", current_state);
  // record_ekg_data();
  if (current_state == MENU) {
    display_options();
  } else if (current_state == REPLAY) {
    replay_ekg_data(current_line);
  } else {
    record_ekg_data();
  }
}

void replay_ekg_data(int file) {
    copy_file_to_sample_buff(current_line);
    Serial.print("done copying sd data in");
}

void change_state(int new_state) {
  if (new_state == MENU) {
    //NVIC_DISABLE_IRQ(IRQ_DMA_CH1);
    first_display = true;
    current_state = MENU;
  } else if (new_state == RECORD) {
    reset_data();
    display_stabilization_screen();
    //NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
    current_state = RECORD;
  } else if (new_state == REPLAY) {
    //NVIC_DISABLE_IRQ(IRQ_DMA_CH1);
    current_state = REPLAY;
  }
}

static char newline_delim[] = "\n";
static char subline_delim[] = ", \n";

void copy_file_to_sample_buff(int current_line) {
  char filename[50];
  sprintf(filename, filename_template, current_line);
  SdFile current_file;
  if (current_file.open(filename, O_WRITE | O_READ | O_CREAT)) {
    sd.errorHalt("error opening log file");
  }
  uint16_t * current_sample = sd_data;
  char line_buff[100];
  char *next_token;
  char *tmp;
  int res = current_file.fgets(line_buff, 100, newline_delim);
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



void record_ekg_data() {
  if (stabilized) {
    filterData();
    drawColumn();
  }

  // need to wait for data to stabilize?
  // neet to wait for button to be pressed?
  if (dma_occurred) {
    dma_occurred = false;
    beginWriteToSDCard();
  }
}

uint16_t filter_aux_buff[SAMPLES_TO_DRAW];

void filterData() {

  /*

    ALSO NEED TO INCREMENT SD_BUFF_FRONT like readindex is in drawCOl:

  // Advance the readIndex
  readIndex++;
  // If we'd read X amount past the base, advance the base
  // Basically so we're reading a chunk of data at a time
  if (readIndex >= baseRead + SAMPLES_TO_DRAW) {
    baseRead = buff_front - SAMPLES_TO_DRAW;
    readIndex = baseRead;
  }

   */


  // ensure we have 320 samples ...
  if (stabilized && buff_front - samples > SAMPLES_TO_DRAW) {
    // I don't want to disable interrupts this whole time but...
    NVIC_DISABLE_IRQ(IRQ_DMA_CH1);

    memcpy(filter_aux_buff, buff_front - SAMPLES_TO_DRAW, SAMPLES_TO_DRAW);
    NVIC_ENABLE_IRQ(IRQ_DMA_CH1);

    //memcpy(sd_buff_front - SAMPLES_TO_DRAW, filter_aux_buff, SAMPLES_TO_DRAW);
    blur_filter(filter_aux_buff, sd_buff_front, SAMPLES_TO_DRAW);
    calc_sample_accel_est(sd_buff_front - SAMPLES_TO_DRAW,
                          sample_acceleration, SAMPLES_TO_DRAW);
    update_hr_estimate(SAMPLES_TO_DRAW);
  }

}


void blur_filter(uint16_t *src, uint16_t *dst, int length) {
  // 1d convolution with [.333, .333, .333]
  double filter[3] = {0.333, 0.333, 0.333};
  int order = 3;
  int half = order / 2;
  for (int i = 0; i < length; i++) {
    double new_reading = 0;
    int filter_index = 0;
    for (int j = i - half; j <= i + half; j++) {
      // handle boundary conditionss
      int index = j;
      if (index < 0) {
        index = 0;
      } else if (index > length) {
        index = length - 1;
      }
      new_reading += src[index] * filter[filter_index];
      filter_index++;
    }
    dst[i] = new_reading;
  }
}

boolean local_maxes[SCREEN_WIDTH];
uint16_t avg_accel = BASELINE;
uint16_t samples_to_avg = 5;
uint16_t accel_threshhold = 40;

void calc_sample_accel_est(uint16_t * src, uint16_t * dst, int length) {
  // 1d convolution with [1, -2, 1]
  double filter[3] = {1, -2, 1};
  int order = 3;
  int half = order / 2;
  for (int i = 0; i < length; i++) {
    double new_reading = 0;
    int filter_index = 0;
    for (int j = i - half; j <= i + half; j++) {
      // handle boundary conditions
      int index = j;
      if (index < 0) {
        index = 0;
      } else if (index > length) {
        index = length - 1;
      }
      new_reading += src[index] * filter[filter_index];
      filter_index++;
    }
    dst[i] = new_reading;
  }

  for (int i = 0; i < length; i++) {
    dst[i] += BASELINE;
  }

  // now, detect "peak", or s waves.
  for (int i = 0; i < length; i++) {
    uint16_t accel_reading = src[i];
    avg_accel = (avg_accel * samples_to_avg + accel_reading - avg_accel) / samples_to_avg;
    uint16_t differential = avg_accel - accel_reading;
    local_maxes[i] = differential > accel_threshhold;
  }
  for (int i = 0; i < length - 1; i++) {
    if (local_maxes[i + 1] && local_maxes[i]) {
      local_maxes[i] = false;
    }
  }

  /* for (int i = 0; i < length; i ++) { */
  /*   if (local_maxes[i]) { */
  /*     tft.drawLine(i, 0, i, SCREEN_HEIGHT, ILI9341_BLUE); */
  /*   } */
  /* } */
}


double heartrate = 0; // our running average
void update_hr_estimate(int length) {
  uint16_t prev_peak_index = -1;
  for (int i = 0; i < length - 1; i++) {
    if (local_maxes[i]) {
      if (prev_peak_index == -1) {
        prev_peak_index = i;
      } else {
        double difference = i - prev_peak_index;
        double time_per_sample = 1 / 250;
        double delta_t = difference * time_per_sample;
        double freq_min = 60 / delta_t;
        Serial.printf("delta_t: %d\n", delta_t);
        heartrate = (heartrate * 2 + freq_min - heartrate) / 2;
        prev_peak_index = i;
      }
    }
  }
  Serial.printf("HR est: %f bpm\n", heartrate);
}



void initialize_sd_card() {
  if (!sd.begin(SD_CS, SPI_HALF_SPEED)) sd.initErrorHalt();
  sd.vwd()->rewind();
}

void beginWriteToSDCard() {
    if (writeToSDCard && current_state == RECORD) {
      writeToSDCard = false;
      Serial.println("Writing to SD CArd");
      NVIC_DISABLE_IRQ(IRQ_DMA_CH1);
      char header[30];
      char filename[30];
      Serial.println("there");
      int new_uid = increment_number_of_files();
      Serial.println("here");
      sprintf(header, header_template, new_uid, SAMPLE_RATE);
      sprintf(filename, filename_template, new_uid);
      SdFile data_file;  // log file
      if (!data_file.open(filename, O_WRITE | O_READ | O_CREAT)) {
        NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
        sd.errorHalt("opening log file failed");
        return;
      }
      data_file.write(header);
      // since there's no guarantee that the last line will have exactly
      // 8 samples, we do the last line outside of the loop.
      uint16_t *sample;
      for (sample = sd_data;
           sample < samples + NUM_SAMPLES - 8;
           sample +=8) {
        char log_line[100];
        Serial.println("here");
        sprintf(log_line, log_line_template, sd_data[0], sd_data[1], sd_data[2],
                sd_data[3], sd_data[4], sd_data[5], sd_data[6], sd_data[7]);
        data_file.write(log_line);
      }
      Serial.println("here");
      while (sample < sd_data + NUM_SAMPLES - 1) {
        char * templ = "%hu, ";
        char sample_text[10];
        sprintf(sample_text, templ, *sample);
        data_file.write(sample_text);
        sample++;
      }
      char * templ = "%hu\n";
      char sample_text[10];
      sprintf(sample_text, templ, *sample);
      data_file.write(sample_text);
      data_file.write(eof);
      data_file.sync();
      data_file.close();
      data_file.sync();
      // change_state(MENU);
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
  baseRead = sd_data;
  readIndex = baseRead;
  stabilized = false;
}

void display_stabilization_screen() {
  Serial.println("***stabliization screen ******");
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
  if (readIndex < sd_data) {
    readIndex = (sd_data + SAMPLES_TO_DRAW) - (sd_data - readIndex);
  }

  Serial.printf("readIndex: %d\n", readIndex - sd_data);

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


  // plot the sample on the graph.
  //tft.fillScreen(ILI9341_BLACK);
  tft.fillRect(height, samplesRead, 3, 3, ILI9341_BLACK);
  // NOTE WE REALLY WANT TO DRAW FROM sd_data, SO...
  //   sample/readIndex should reference sd_data, not samples.
  //if (readIndex = (samples + SAMPLES_TO_DRAW) - (samples - readIndex)) {

  /* if (samplesRead % (2 * SAMPLES_TO_AVERAGE) == 0) { */

  /* } */


  /* if (samplesRead > SAMPLES_TO_DRAW - 1) { */
  /*   tft.fillRect(height, samplesRead, 5, 5, ILI9341_GREEN); */
  /* } else if (samplesRead > 0) { */
  /*   tft.drawLine(last_height, samplesRead - 1, height, samplesRead, ILI9341_BLACK); */
  /* } */
  /* last_height = height; */

  // Advance the readIndex
  readIndex++;
  // If we'd read X amount past the base, advance the base
  // Basically so we're reading a chunk of data at a time
  if (readIndex >= baseRead + SAMPLES_TO_DRAW) {
    baseRead = sd_buff_front - SAMPLES_TO_DRAW;
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
    if (buff_front > (samples + 640)) {
      stabilized = true;
    }
    // Use the buffer as a circular buffer
    if (buff_front > (samples + NUM_SAMPLES)) {
      buff_front = samples;
      writeToSDCard = true;
    }
  }
  sd_buff_front += NUM_DMA_SAMPLES;
  // Clear interrupt request for channel 1
  DMA_CINT = 1;
  dma_occurred = true;

  NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
}


// FS code
static char *num_files_filename = "index";

int increment_number_of_files() {
  SdFile metadata;
  Serial.println("opening metadatafile");



  if (!metadata.open(num_files_filename, O_WRITE | O_READ | O_CREAT)) {
    sd.errorHalt("error opening metadatafile");
  }
  char metadata_contents[10];
  Serial.println("callling fgets");
  int res = metadata.fgets(metadata_contents, 10);
  Serial.println("done calling fgets");
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
  metadata.sync();
  return count;
}



void display_options() {
  int move_cursor = scroll_direction();
  int num_logs = get_number_of_files();
  if (move_cursor == -1 && current_line < num_logs) {
    current_line++;
  } else if (move_cursor == 1 && current_line > 0)  {
    current_line--;
  }

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
int old_scroll_state = 0;
// -1 = down, 0 = no scroll, 1 = up
int scroll_direction() {
  int val = analogRead(Y_PIN);
  int new_state = 0;
  if (val < 500) {
    new_state = 1;
  } else if (val > 3300) {
    new_state = -1;
  }
  /* Serial.printf("New state: %d \n", new_state); */
  /* Serial.printf("Old scroll state; %d \n", old_scroll_state); */
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

boolean button_pressed() {
  if (joystick_button.update() && joystick_button.fallingEdge()) {
    return true;
  } else {
    return false;
  }
}
