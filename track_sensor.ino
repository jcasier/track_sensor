
// CT Sensor test
// Using UNO built-in ADC to read the sensor
//////////////////////////////////////////////////////////////////

#define VERSION "1.006"
#define SYS_ID "CT Sensor Test - Direct to UNO ADC"
const int numpins = 3;
const int adcpins[numpins] = {0, 1, 2};
const int outpins[numpins] = {22, 23, 24};

// Sampling Parameters
const unsigned long sampleTime = 2000UL; 
const unsigned long numSamples = 100UL; 
const unsigned long sampleInterval = sampleTime/numSamples; 

#define SENSITIVITY 5000
#define DETECTION_MULTIPLIER 1.3
#define CALIBRATION_READS 300

// variables to hold sensor quiescent readings
float aqv[numpins];  // Average Quiescent Voltage; e.g. ADC Zero
float aqc[numpins];  // Average Quiescent Current; 

void setup() {
  Serial.begin(9600);
  //Serial.println(String(F(SYS_ID)) + String(F(" - SW:")) + String(F(VERSION)));
  for (int i = 0; i < numpins; i++) {
    Serial.print("\nCalibrating the sensor at pin ");
    Serial.println(adcpins[i]);
    aqv[i] = determineVQ(adcpins[i]); 
    Serial.print("AQV: ");
    Serial.print(aqv[i] * 1000, 4);
    Serial.print(" mV\t");
    aqc[i] = determineCQ(adcpins[i], aqv[i]);
    Serial.print("AQC: ");
    Serial.print(aqc[i] * 1000, 4);
    Serial.print(" mA\t");
    float sense = (aqc[i] * DETECTION_MULTIPLIER) - aqc[i];
    Serial.print("Detection Sensitivity: ");
    Serial.print(sense * 1000, 3);
    Serial.println(" mA\n");
    pinMode(outpins[i], OUTPUT);
    digitalWrite(outpins[i], LOW);
  }
  delay(7500);
  
}

void loop() {
  for (int i = 0; i < numpins; i++) {
    float current = readCurrent(adcpins[i], aqv[i]);
    float delta = abs(aqc[i] - current);
    bool occupied = delta > ((aqc[i] * DETECTION_MULTIPLIER) - aqc[i]);
    Serial.print("Segment");
    Serial.print(adcpins[i]);
    Serial.print("\t");
    Serial.print("Current Sensed: ");
    Serial.print(current * 1000,3);
    Serial.print(" mA\t");  
    if(occupied){
      Serial.println("Occupied");
      digitalWrite(outpins[i], HIGH);
    } else {
      Serial.println("Not occupied");
      digitalWrite(outpins[i], LOW);
    }
  }
  delay(3000);
}

//////////////////////////////////////////
// Current Sensor Functions
//////////////////////////////////////////
float readCurrent(int pin, float adc_zero) {
  float currentAcc = 0;
  unsigned int count = 0;
  unsigned long prevMicros = micros() - sampleInterval ;
  while (count < numSamples)
  {
    if (micros() - prevMicros >= sampleInterval)
    {
      float adc_raw = (float) analogRead(pin) - adc_zero; // sensor reading in volts
      adc_raw /= SENSITIVITY; // convert to amperes
      currentAcc += (adc_raw * adc_raw); // sum the squares
      count++;
      prevMicros += sampleInterval;
    }
  }
  //https://en.wikipedia.org/wiki/Root_mean_square
  float rms = sqrt((float)currentAcc / (float)numSamples);
  return rms;
}

//////////////////////////////////////////
// Calibration
// Track Power must be OFF during calibration
//////////////////////////////////////////

float determineVQ(int pin) {
  float VQ = 0;
  //read a large number of samples to stabilize value
  for (int i = 0; i < CALIBRATION_READS; i++) {
    VQ += analogRead(pin);
    delayMicroseconds(sampleInterval);
  }
  VQ /= CALIBRATION_READS;
  return VQ;
}

float determineCQ(int pin, float aqv) {
  float CQ = 0;
  // set reps so the total actual analog reads == CALIBRATION_READS
  int reps = (CALIBRATION_READS / numSamples);
  for (int i = 0; i < reps; i++) {
    CQ += readCurrent(pin, aqv);
  }
  CQ /= reps;
  return CQ;
}


