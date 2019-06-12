/*
 * This sketch configures Timer 1 to trigger the ADC at a rate of
 * 256Hz. This sample rate gives a frequency bin resolution of 1Hz
 * per bin. The Fast Hartley Transform (FHT) library performs a 
 * fourier transform on the input buffer containing the sampled 
 * EEG signal data. The FHT library is used to process the data and
 * output the weights of each frequency in their respective bin.
 * Helper functions are used to find the dominant frequency within
 * a specified range and to map a frequency weight to an LED value.
 * 
 * Author: Parker Lloyd
 */

#define LOG_OUT 1 // use the log output function
#define FHT_N 256 // set to 256 point fht (input buffer size)

#include <FHT.h> // include the library
const int ledPin = 11;
const byte adcPin = 0;  // A0
volatile int sampleCount;  // current number of samples in buffer


// ADC complete ISR
ISR (ADC_vect){
  if (sampleCount < FHT_N) {
    byte m = ADCL; // fetch adc data
    byte j = ADCH;
    int k = (j << 8) | m; // form into an int
    k -= 0x0200; // form into a signed int
    k <<= 6; // form into a 16b signed int
    fht_input[sampleCount++] = k; // put real data into bins
  }
}
  
EMPTY_INTERRUPT (TIMER1_COMPB_vect);
 
void setup () {
  Serial.begin (115200);
  pinMode(ledPin, OUTPUT);
  
  // initialize timer1 
  cli();      // disable interrupts
  TCCR1A = 0;   // reset timer configuration registers
  TCCR1B = 0;   //
  TCNT1  = 0;   //reset Timer 1 count
  
  OCR1A = 243;              // compare match register 16MHz/256/256Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt

  // initialize ADC
  ADCSRA  = bit (ADEN)  | bit (ADIE) | bit (ADIF);  //enable ADC, enable interrupts, set interrupt flag
  ADCSRA |= bit (ADPS2) | bit (ADPS1);              // prescaler of 64
  ADMUX   = bit (REFS0);                            // set reference voltage of 5V
  ADCSRB  = bit (ADTS2) | bit (ADTS0);              // Timer/Counter1 Compare Match B
  ADCSRA |= bit (ADATE);                            // turn on automatic triggering
  sei();                                            // enable interrupts
}

void loop () { 
  while(sampleCount < FHT_N) {}   // wait for input buffer to fill

  // stop ADC and disable interrupts
  ADCSRA &= ~ (1 << ADEN);
  cli();
  
  fht_window(); // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the fht
  fht_run(); // process the data in the fht
  fht_mag_log(); // take the output of the fht

  // Send data to serial to be read by Processing script (FHT_128_channel_analyser)
//  Serial.write(255); // send a start byte
//  Serial.write(fht_log_out, FHT_N/2); // send out the data

//  Serial.print(fht_log_out[8], DEC);
//  Serial.print(" ");
//  Serial.print(fht_log_out[9], DEC);
//  Serial.print(" ");
//  Serial.print(fht_log_out[10], DEC);
//  Serial.print(" ");
//  Serial.print(fht_log_out[11], DEC);
//  Serial.print(" ");
//  Serial.print(fht_log_out[12], DEC);
//  Serial.println();


  //int sum = (int) (fht_log_out[8] + fht_log_out[9] + fht_log_out[10] + fht_log_out[11] + fht_log_out[12]);
  //float avg = (float) sum / 5;
  //Serial.println(findMax(fht_log_out, 8, 12));
  // Determine the dominant alpha frequency
  int maxVal = (int) fht_log_out[findMax(fht_log_out, 8, 12)];
  int avg = (int) average(fht_log_out, 8, 12);
  setLED(avg); 
  //Serial.println(avg);

  // reset count and ADC, enable interrupts
  sampleCount = 0;
  sei();
  ADCSRA = 0xEC;
}

/*
 * Average a range of frequencies
 */
float average (uint8_t vals[], int lower, int upper) {
  float average = 0;
  for (int i = lower; i <= upper; i++) {
    average += (float) vals[i];
  }
  average /= (upper - lower + 1);
  return average;
}

int findMax (uint8_t vals[], int lower, int upper) {
  int maxIndex = lower;
  for (int i = lower; i <= upper; i++) {
    if (vals[i] > vals[maxIndex])
      maxIndex = i;
  }
  return maxIndex;
}

void setLED (int val) {
  double binMin = pow(80, 4);
  double binMax = pow(155, 4);
  double percentage = (pow(val, 4) - binMin) / (binMax - binMin);
  double brightness = percentage * 255;
  if (brightness < 0)
    brightness = 0;
  analogWrite(ledPin, (int) brightness);
  Serial.println((int) brightness);
}
