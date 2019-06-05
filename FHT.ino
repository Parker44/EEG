#define LOG_OUT 1 // use the log output function
#define FHT_N 256 // set to 256 point fht (try FHT_N=2048 and ADCSRA = 0xFF)
 
#include <FHT.h> // include the library
void setup() {
  Serial.begin(115200); // use the serial port
  pinMode(11, OUTPUT);
  TIMSK0 = 0; // turn off timer0 for lower jitter
  ADCSRA = 0xEF; // set the adc to free running mode
  ADMUX = 0x40; // use adc0
  DIDR0 = 0x01; // turn off the digital input for adc0
  delay(2000);
}
 
void loop() {
  while(1) { // reduces jitter
    cli();  // UDRE interrupt slows this way down on arduino1.0
    for (int i = 0 ; i < FHT_N ; i++) { // save 256 samples
      while(!(ADCSRA & 0x10)); // wait for adc to be ready (interrupt flag)

      int avg_samples = 96;
      for (int t = 0; t < avg_samples ; t++) {
        ADCSRA = 0xFF; // restart adc 9600Hz sample rate
        byte m = ADCL; // fetch adc data
        byte j = ADCH;
        int k = (j << 8) | m; // form into an int
        k -= 0x0200; // form into a signed int
        k <<= 6; // form into a 16b signed int
        fht_input[i] += k; // put real data into bins      
      }
    fht_input[i] /= avg_samples;
    }

    // 128 bins (0 - 100Hz)
    // Alpha waves: 8 - 12Hz
    // Bins 10 - 16
    //LED_conversion(findMax(fht_input[10:16]));
    
    fht_window(); // window the data for better frequency response
    fht_reorder(); // reorder the data before doing the fht
    fht_run(); // process the data in the fht
    fht_mag_log(); // take the output of the fht
    sei();
    Serial.write(255); // send a start byte
    Serial.write(fht_log_out, FHT_N/2); // send out the data
  }
}
void LED_conversion (int val) {
  // Determine maximum likely bin value for alpha wave strength
  // Map bin value to 0 - 255
  int binMax = 0;
  int brightness = val * (255/binMax);
  analogWrite(11, brightness);
}

int findMax (int vals[]) {
  int maxIndex = 0;
  for (int i = 0 ; i < sizeof(vals) ; i++) {
    if (vals[i] > vals[maxIndex]) {
      maxIndex = i;
    }
  }
  return maxIndex;
}
