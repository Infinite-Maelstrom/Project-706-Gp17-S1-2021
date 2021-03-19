// Program: Calibration for Medium Range Infra red (IR) Sensor (2D120X / 2Y0A41SK)
// Author: Ben Wight
// Version: v1 3/3/21

// Notes:
//      Measures 4 to 30 cm
//      2 modes one for checking one for calibrating (toggle measureMode in variables)
// TODO:
//      Connect to sensor
//      Verify Sensor Measurements

// Optional Presteps
// Select analog port (default A0)

// Calibration Steps
//  1. Set measure mode to true
//  1. Measure 4 6 8 10 15 20 25 30 40
//  2. Send to https://mycurvefit.com/
//  3. Reconfigure a, b, c, d

///////////////////////////////////////////////////////////////////////////////////
// Global variables
int analog_port = 15;
int adc = 0;
float voltage = 0;
float distanceCM = 0;

////////////// A B C D
float a = 71.17256;
float b = 1.529853;
float c = 0.3466616;
float d = 1.425554;

// Calibration mode options
int presets[] = {4, 6, 8, 10, 15, 20, 25, 30, 40};
bool measureModeOFF = false;

// Option Variables
bool measureMode = true; // Change modes here

//////////////////////////////////////////////////////////////////////////////////
// Functions

float calculate_MediumRangeIR_DistanceCM(float voltage, float a, float b, float c, float d) {
  // Takes voltage value from medium range sensor
  // Outputs Distance in cm
  // Length = 1.425554 + (71.17256 - 1.425554)/(1 + (voltage/0.3466616)^1.529853)
  return d + (a - d)/(1.0 + pow(voltage/c,b));
}

float getVoltage(int adc) {
  // Takes Analog bit Values (0-1023)
  // Outputs Voltage for 0-5 volts
  return adc / 1024.0 * 5.0; 
}

int getADCvalue(int A) {
  // Takes analog port 0-15
  // Output: maps 0-5v to 0-1023
  return analogRead(A);
}

void printLiveData() {
  // Prints all the data to the serial monitor
  adc = getADCvalue(analog_port); // SET VALUE OF ANALOG PORT
  voltage = getVoltage(adc);
  distanceCM = calculate_MediumRangeIR_DistanceCM(voltage, a, b, c, d);
  Serial.print("ADC: "); Serial.print(adc);
  Serial.print(" --> Voltage: "); Serial.print(voltage);
  Serial.print(" --> Disctance (cm): "); Serial.print(distanceCM); Serial.println();
}

void measure_Mode() {
  if (!measureModeOFF) {
  Serial.println("In this mode the system will record voltage vaules at preset distances");
  Serial.println("There is 5 seconds between each distance");
  Serial.println("The presets are as follows");
  
  int preset_size = sizeof(presets)/2;  //Reason for this explained here https://www.codeproject.com/Questions/5268241/Different-results-with-sizeof
  
  for (int i = 0; i < preset_size; i++) {
     Serial.print(presets[i]); Serial.print("cm, ");
  }
  Serial.println();
  float voltageValues[preset_size];

  delay(10000);
  
  for(int i = 0; i < preset_size;i++) {
    Serial.print("//////////////////////////\nRun for *"); Serial.print(presets[i]); Serial.println("cm*");
    for (int j=5; j> 0;j--) {
      Serial.print(j); Serial.print(" ");
      delay(1000);
    }
   Serial.println();
    voltageValues[i] = getVoltage(getADCvalue(analog_port));
  }
  Serial.println("Values:");
  Serial.println("Cm\tVoltage");
  for (int i = 0; i < preset_size;i++) {
  Serial.print(presets[i]); Serial.print("\t");Serial.println(voltageValues[i]);  
  }
  measureModeOFF = true;
  Serial.println("Measurement Complete!!!");
  }
}
////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Connect to Serial Port
  Serial.begin(115200);
  Serial.println("Connect and ready to go!!!");
  Serial.println("Calibration for Medium Range IR Sensor");
}

void loop() {
  delay(1000);
  if (measureMode) {
    measure_Mode();
  } else {
  printLiveData();
  }
}
