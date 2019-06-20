#include <SPI.h>
#include "SdFat.h"

const int CS_PIN=9;
File testFile;

SdFat SD;

File tempFile;

volatile bool writeToSd = false;

volatile uint8_t reading;
bool isRecording = false;
volatile bool stopRecording = true;

int mic = A1;
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
#define BUFF_SIZE 512

bool isLEDOn = false;

static volatile uint8_t buffA[BUFF_SIZE];
static volatile uint8_t buffB[BUFF_SIZE];
static volatile uint16_t count = 0; 

volatile bool activeBuffer = false;

uint8_t totalBuff = 100;
uint32_t totalCount = 40000;

uint8_t numBuff = 0;

uint8_t waveHeader[44] = {
  'R', 'I', 'F', 'F', 36, 44, 1, 0, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ',
  16, 0, 0, 0, 1, 0, 1, 0, 64, 31, 0, 0, 64, 31, 0, 0, 1, 0, 8, 0, 
  'd', 'a', 't', 'a', 0, 44, 1, 0};  

uint32_t sampleCount = 0; 

long startTime;
long endTime; 

uint8_t filteredReading = 0;
#define FILTER_ALPHA 0.04

void setup() {
  init_adc();
  init_dac();
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  //while (!Serial) {};

  Serial.println(SD.begin(CS_PIN, SD_SCK_MHZ(10)));

  NVIC_ClearPendingIRQ(SERCOM3_IRQn);
  NVIC_SetPriority(SERCOM3_IRQn, 2);
  NVIC_EnableIRQ(SERCOM3_IRQn);

  pinMode(5, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(9, OUTPUT);
   
 //analogReadResolution(8);
 //pinMode(mic, INPUT); 

}

void startRecording() {
  SD.remove("test.wav");
  SD.remove("data.tmp");
  tempFile = SD.open("data.tmp" , O_WRITE | O_CREAT);
  stopRecording = false;
  isRecording = true;
  digitalWrite(13, HIGH);
  Serial.println("Recording started");

  count = 0;
  sampleCount = 0;
  numBuff = 0;
  for (int i=0; i<512; i++) {
    buffA[i] = 0;
    buffB[i] = 0;
  }
  activeBuffer = false;
  
  startTime = millis();
  startTimer(8000);
}

void loop() {
  // put your main code here, to run repeatedly:
  //int value = analogRead(A1);
  //float voltage = value * (5.0 / 1023.0);
//Serial.println(voltage);

  if (!isRecording && !digitalRead(5)) {
    startRecording();
  } else if (isRecording && digitalRead(5)) {
    stopRecording = true;
  }
 
 if (writeToSd) {
   if (activeBuffer) {
      tempFile.write((char *) buffB, BUFF_SIZE);
   } else {
      tempFile.write((char *) buffA, BUFF_SIZE);
   }
   writeToSd = false;
 }
}

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  Serial.println(TC->COUNT.reg);
  Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_SetPriority(TC3_IRQn, 0);
  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void finishRecording() {
tempFile.close();
  
  uint16_t freq = (sampleCount*1000) / ((endTime - startTime));

  Serial.println(endTime - startTime);
  Serial.println(sampleCount);
  Serial.println(freq);

  uint32_t chunkSize = sampleCount + 36;
  waveHeader[4] = chunkSize & 0xFF;
  waveHeader[5] = (chunkSize >> 8) & 0xFF;
  waveHeader[6] = (chunkSize >> 16) & 0xFF;
  waveHeader[7] = (chunkSize >> 24) & 0xFF;

  waveHeader[24] = freq & 0xFF;
  waveHeader[25] = (freq >> 8) & 0xFF;
  waveHeader[26] = (freq >> 16) & 0xFF;
  waveHeader[27] = (freq >> 24) & 0xFF;

  waveHeader[28] = freq & 0xFF;
  waveHeader[29] = (freq >> 8) & 0xFF;
  waveHeader[30] = (freq >> 16) & 0xFF;
  waveHeader[31] = (freq >> 24) & 0xFF;


  waveHeader[40] = sampleCount & 0xFF;
  waveHeader[41] = (sampleCount >> 8) & 0xFF;
  waveHeader[42] = (sampleCount >> 16) & 0xFF;
  waveHeader[43] = (sampleCount >> 24) & 0xFF;

  testFile = SD.open("test.wav" , FILE_WRITE);
  Serial.println("The first part is working");
  testFile.write(waveHeader, 44);
  //testFile.flush();
  Serial.println("It's working");

  tempFile = SD.open("data.tmp",FILE_READ);
  //Serial.println(tempFile);

  uint32_t i = 0;
  Serial.println(numBuff);
  while (++i < numBuff) {
   
    tempFile.read((char *) buffA, BUFF_SIZE);
    testFile.write((char *) buffA, BUFF_SIZE);  

}

   Serial.println(i);

  tempFile.close();
  testFile.close();

  isRecording = false;
  stopRecording = true;
  digitalWrite(13, LOW);
  }

  

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    // Write callback here!!!
    //filteredReading = analogRead(A1) * FILTER_ALPHA + (filteredReading * (1.0-FILTER_ALPHA));
    if (!stopRecording) {
      if (activeBuffer) {
        buffA[count] = reading;
      } else {
        buffB[count] = reading;
      }
      //if (writeToSd) return;
      //tempFile.write(reading);
      count++;
      sampleCount++;
      if (count == BUFF_SIZE) {
        if (!writeToSd) {
          count = 0;
          activeBuffer = !activeBuffer;
          numBuff++;
          writeToSd = true;
        } else {
          Serial.println("Overflow");
          count--;
          sampleCount--;
        }
      }
    } else {
      endTime = millis();
      TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;   
      Serial.println("Recording stopped");
      finishRecording();
    }
  }
}

void init_adc()
{
  PM->APBCMASK.reg |= PM_APBCMASK_ADC;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(ADC_GCLK_ID) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  ADC->CTRLA.reg = ADC_CTRLA_SWRST;
  while (ADC->CTRLA.reg & ADC_CTRLA_SWRST);

  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1 | ADC_REFCTRL_REFCOMP;
  ADC->CTRLB.reg = ADC_CTRLB_RESSEL_8BIT | ADC_CTRLB_PRESCALER_DIV512 | ADC_CTRLB_FREERUN;
  //ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS_PIN2 | ADC_INPUTCTRL_MUXNEG_GND |
      ADC_INPUTCTRL_GAIN_DIV2;
  //ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(0x3F);
  ADC->CALIB.reg = 0x388;

  ADC->CTRLA.reg = ADC_CTRLA_ENABLE;
  ADC->SWTRIG.reg = ADC_SWTRIG_START;

  while (!(ADC->INTFLAG.bit.RESRDY));
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
  ADC->INTENSET.reg = ADC_INTENSET_RESRDY;

  //ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(NVM_READ_CAL(ADC->CALIB.bit.BIAS_CAL)) |
  //    ADC_CALIB_LINEARITY_CAL(NVM_READ_CAL(ADC_LINEARITY));

  NVIC_SetPriority(ADC_IRQn, 0);
  NVIC_EnableIRQ(ADC_IRQn);
}

void init_dac()
{
  DAC->CTRLB.reg = (1<<6) | 1;
  DAC->CTRLA.reg = (1<<1);
  DAC->DATA.reg = 512;

  
  }

void ADC_Handler()
{
  reading = ADC->RESULT.reg;
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
}
