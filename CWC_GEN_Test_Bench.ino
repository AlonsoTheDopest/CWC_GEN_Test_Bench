// Libraries
#include <LiquidCrystal_I2C.h>
#include "HX711.h"
#include <SD.h>


// Constants

  // Timing
  #define ONE_SECOND 1000 // ms
  #define DELAY_TIME 150 // ms
  #define MEASUREMENT_REFRESH_TIME 150 // ms
  #define DEBOUNCE_DURATION 20 // ms 

  // Other
  #define DECIMAL_PLACES 2// MAY CHANGE //
  #define RESOLUTION 1023
  #define NUM_OF_SENSORS 4
  #define MAX_STRING_LENGTH 128
  #define STANDARD_STRING_LENGTH 64
  #define BAUD_RATE 9600 // For serial monitor output // MAY CHANGE //

  // Arduino Specs
  #define REFERENCE_VOLTAGE 4.60 // MAY CHANGE //
  const float VOLTAGE_PER_COUNT = REFERENCE_VOLTAGE / RESOLUTION;
  const float MIDPOINT_VOLTAGE = REFERENCE_VOLTAGE / 2;

  // Characters/Units
  #define SPACE ' '
  #define TAB '\t'
  #define TIME_UNIT "s"
  #define VOLTAGE_UNIT "V"
  #define CURRENT_UNIT "A"
  #define ROTATIONAL_SPEED_UNIT "RPM"
  #define TORQUE_UNIT "N-mm"

  // Voltage Divider
  #define VOLTAGE_DIVIDER_ANALOG_PIN A0
  #define VOLTAGE_DIVIDER_HIGH_RESISTOR_KOHM 9.92
  #define VOLTAGE_DIVIDER_LOW_RESISTOR_KOHM 0.995
  #define VOLTAGE_DIVIDER_SAMPLE_AMT 10
  #define LOW_MEASURED_VOLTAGE 0.16
  #define HIGH_MEASURED_VOLTAGE 26.27
  #define LOW_ACTUAL_VOLTAGE 0
  #define HIGH_ACTUAL_VOLTAGE 26.2

  // Current Sensor
  #define CURRENT_SENSOR_ANALOG_PIN A1
  #define CURRENT_SENSOR_SENSITIVITY 0.066
  #define CURRENT_SENSOR_SAMPLE_AMT 100
  #define LOW_MEASURED_CURRENT -0.89
  #define HIGH_MEASURED_CURRENT 3.18
  #define LOW_ACTUAL_CURRENT 0
  #define HIGH_ACTUAL_CURRENT 5.06

  // Infrared Sensor
  #define INFRARED_SENSOR_OUT_PIN 2
  #define INFRARED_SENSOR_SAMPLE_AMT 10

  // Load Cell Amp
  #define LOAD_CELL_AMP_DAT_PIN 6
  #define LOAD_CELL_AMP_CLK_PIN 7
  #define LOAD_CELL_AMP_SAMPLE_AMT 10

  // SD Card Reader
  #define SD_CARD_READER_CS_PIN 10

  // Load Cell Amp
  #define CALIBRATION_FACTOR 372.5 // MAY CHANGE //

  // LCD Screen
  #define I2C_ADDRESS 0x27
  #define COLUMNS 20
  #define ROWS 4


float getVoltageInput( const int pinNumber );

class Sensor {

  protected:
    float measurement;
    String unit;
    unsigned int sampleAmt;

  public:
    virtual void initialize() = 0;
    virtual void measure() = 0;
    virtual String getUnit() = 0;
    virtual float getMeasurement() = 0;
};

class LCD_Screen {
  public:
    LCD_Screen( const int i2cAddress, const int numOfColumns, const int numOfRows ) : screen( i2cAddress, numOfColumns, numOfRows ) {
      cols = numOfColumns;
      rows = numOfRows;
    }

    void initialize() {
      screen.init();
      screen.backlight();
      screen.clear();
    }

    void print( const char *string, const int line ) {
      screen.setCursor( 0, line );
      screen.print( string );
    }

    void printUnit( const Sensor *sensor, const int line ) {
      String unit = sensor->getUnit();
      int unitStringLength = unit.length();
      int unitCursorPosition = cols - unitStringLength;
      screen.setCursor( unitCursorPosition, line );
      screen.print( unit );
    }

    void printValue( const float value, const int line ) {
      screen.setCursor( 0, line );
      screen.print( value, DECIMAL_PLACES );
    }

    void clearChars( const int line, const int endingColumn = COLUMNS ) {  
      screen.setCursor( 0, line );
      for( int currCol = 0; currCol < endingColumn; currCol++ ) {
        clearChar();
      }
    }

    void clearValue( const int line, const int unitLength ) {
      int endingColumn = cols - unitLength;
      clearChars( line, endingColumn );
    }

    void clearChar(){
      screen.print( SPACE );
    }

    void clearScreen() {
      for ( int line = 0; line < rows; line++ ) {
        clearChars( line );
      }
    }

    void printMeasurement( Sensor *sensor, const int line ) {
      sensor->measure();
      float sensorMeasurement = sensor->getMeasurement();
      String unit = sensor->getUnit();
      int unitLength = unit.length();
      clearValue( line, unitLength );
      printValue( sensorMeasurement, line );
    }

  private:
    LiquidCrystal_I2C screen;
    int cols;
    int rows;
};

  // Voltage Divider
  class VoltageDividerSensor : public Sensor {
    public:
      VoltageDividerSensor( const int analogPin, const float resistance1, const float resistance2, const String voltageUnit, const unsigned int num_of_samples = 1 ) {
        aPin = analogPin;
        multiplier = resistance1 + resistance2;
        unit = voltageUnit;
        measurement = 0;
        
        sampleAmt = num_of_samples;
      }

      void initialize() {
        pinMode( aPin, INPUT );
      }

      void measure() {

        float voltageInputSum = 0;
        float uncorrected_measurement;
        for ( int i = 0; i < sampleAmt; i++ ) {
          voltageInputSum += getVoltageInput( aPin );
        }
        uncorrected_measurement = ( voltageInputSum * multiplier ) / sampleAmt;
        Serial.println( uncorrected_measurement );
        measurement = ( uncorrected_measurement - LOW_MEASURED_VOLTAGE ) / ( ( HIGH_MEASURED_VOLTAGE - LOW_MEASURED_VOLTAGE ) / ( HIGH_ACTUAL_VOLTAGE - LOW_ACTUAL_VOLTAGE ) );
      }

      String getUnit() {
        return unit;
      }

      float getMeasurement() {
        return measurement;
      }

    private:
      float multiplier;
      int aPin;
  };

  // Current Sensor
  class CurrentSensor : public Sensor {

    public:
      CurrentSensor( const int analogPin, const float sensitivity, const String currentUnit, const unsigned int num_of_samples = 1 ) {
        aPin = analogPin;
        s = sensitivity;
        unit = currentUnit;
        sampleAmt = num_of_samples;
        measurement = 0;
      }

      void initialize() {
        pinMode( aPin, INPUT );
      }

      void measure() {
        float voltageIn;
        float ampVoltage;
        float sum = 0;
        float uncorrected_measurement;
        for ( int i = 0; i < sampleAmt; i++ ) {
          voltageIn = getVoltageInput( aPin ); 
          ampVoltage = voltageIn - MIDPOINT_VOLTAGE;
          // measurement = 0; // Just in case ampVoltage is negative
          // if ( ampVoltage > 0 ) {
            sum += ampVoltage / s;
          // }
        }
        uncorrected_measurement = sum / sampleAmt;
        Serial.println( uncorrected_measurement );
        measurement = ( uncorrected_measurement - LOW_MEASURED_CURRENT ) / ( ( HIGH_MEASURED_CURRENT - LOW_MEASURED_CURRENT ) / ( HIGH_ACTUAL_CURRENT - LOW_ACTUAL_CURRENT ) );
      }

      String getUnit() {
        return unit;
      }

      float getMeasurement() {
        return measurement;
      }

    private:
      int aPin;
      float s;
  };

  


  // IR Sensor
  volatile long counter = 0;
  volatile unsigned long lastInterruptTime = 0;

  void InfraredInterrupt() {
    if ( millis() - lastInterruptTime > DEBOUNCE_DURATION ) {
      
      // if ( digitalRead( INFRARED_SENSOR_OUT_PIN ) == LOW ) {
        counter++;
        // Serial.println( counter );
        lastInterruptTime = millis();
      // }
    }
  }

  class InfraredSensor : public Sensor {
    public:
      InfraredSensor( const int digitalPin, const String rpmUnit, const unsigned int num_of_samples = 1, const unsigned int pulsePerRotation = 1 ) {
        dPin = digitalPin;
        unit = rpmUnit;
        sampleAmt = num_of_samples;
        ppr = pulsePerRotation;
        measurement = 0;
        previousTime = millis();
      }

      void initialize() {
        pinMode( dPin, INPUT_PULLUP );
        attachInterrupt( digitalPinToInterrupt( dPin ), InfraredInterrupt, FALLING );
      }

      void measure() {
        unsigned long currentTime = millis() - previousTime;
        long rotations;
        double minutesPassed;
        if ( currentTime >= DELAY_TIME ) {
          noInterrupts();
          rotations = counter;
          minutesPassed = currentTime / 60000.0; // Converts ms to minutes
          measurement = ( rotations / minutesPassed ) / ppr;  // Calculate RPM
          counter = 0;
          interrupts();
          previousTime = millis();
        }
      }

      String getUnit() {
        return unit;
      }

      float getMeasurement() {
        return measurement;
      }

    private:
      unsigned long previousTime;
      int dPin;
      unsigned int ppr;
  };



  // Load Cell Amp
  class LoadCellAmpSensor : public Sensor {
    public:
      LoadCellAmpSensor( const int dataPin, const int clockPin, const float calibrationFactor, const String torqueUnit, const unsigned int num_of_samples = 1 ) {
        dPin = dataPin;
        clkPin = clockPin;
        measurement = 0;
        calFac = calibrationFactor;
        unit = torqueUnit;
        sampleAmt = num_of_samples;
      }

      void initialize() {
        scale.begin( dPin, clkPin );
        if ( scale.is_ready() ) {
          scale.set_scale();
          scale.tare();
        }
      }

      void measure() {
        if ( scale.is_ready() ) { 
          if ( Serial.available() > 0 ) {
            String input = Serial.readString();
            input.trim();
            if ( input == "+" ) {
              calFac += 10;
            }
            else if ( input == "-" ) {
              calFac -= 10;
            }
          }
          scale.set_scale( calFac );
          measurement = scale.get_units( sampleAmt );
        }
      }

      String getUnit() {
        return unit;
      }

      float getMeasurement() {
        return measurement;
      }

    private:
      HX711 scale;
      int dPin;
      int clkPin;
      float torque;
      float calFac;
  };


// SD Card Reader
class Micro_SD_Card_Reader {
  public:
    Micro_SD_Card_Reader( const int chipSelectPin ) {
      csPin = chipSelectPin;
      cardIsAvailable = false;
    }

    bool initialize() {
      if ( SD.begin( csPin ) ) {
        setCardIsAvailable( true );
        createFileName();
        writeHeader();
      }
      else {
        setCardIsAvailable( false );
      }
      return cardIsAvailable;
    }

    void createFileName() {
      int counter = 0;
      do {
        sprintf( fileName, "DATA%d.TXT", counter );
        counter++;
      } while ( SD.exists( fileName ) );
    }

    void writeHeader() {
      File dataFile = getFile();
      String dataHeader;
      if ( cardIsAvailable && dataFile ) {
        dataHeader = "Time(" + String( TIME_UNIT ) +
                        ") Voltage(" + String( VOLTAGE_UNIT ) +
                        ") Current(" + String( CURRENT_UNIT ) +
                        ") RotationalSpeed(" + String( ROTATIONAL_SPEED_UNIT ) +
                        ") Torque(" + String( TORQUE_UNIT ) + ")";
        dataFile.println( dataHeader );
        dataFile.close();
      }
      else {
        setCardIsAvailable( false );
      }
    }

    void writeTime() {
      float seconds = getSeconds();
      String dataToWrite;
      File dataFile = getFile();
      if ( cardIsAvailable && dataFile ) {
        dataToWrite = floatToString( seconds ) + String( SPACE ); 
        dataFile.print( dataToWrite );
        dataFile.close();
      }
      else {
        setCardIsAvailable( false );
      }
    }

    void writeData( const Sensor *sensor ) {
      float measurement;
      String dataToWrite;
      File dataFile = getFile();
      if ( cardIsAvailable && dataFile ) {
        measurement = sensor->getMeasurement();
        dataToWrite = floatToString( measurement ) + String( SPACE );
        dataFile.print( dataToWrite );
        dataFile.close();
      }
      else {
        setCardIsAvailable( false );
      }
    }

    void writeNewLine() {
      File dataFile = getFile();
      if ( cardIsAvailable && dataFile ) {
        dataFile.println();
        dataFile.close();
      }
      else {
        setCardIsAvailable( false );
      }
    }

    File getFile() {
      return SD.open( fileName, FILE_WRITE );
    }

    String getFileName() {
      return String( fileName );
    }

    void setCardIsAvailable( const bool available ) {
      cardIsAvailable = available;
    }

  private:
    int csPin;
    char fileName[ STANDARD_STRING_LENGTH ];
    bool cardIsAvailable;
};

LCD_Screen screen( I2C_ADDRESS, COLUMNS, ROWS );
VoltageDividerSensor voltageDividerSensor( VOLTAGE_DIVIDER_ANALOG_PIN, VOLTAGE_DIVIDER_HIGH_RESISTOR_KOHM, VOLTAGE_DIVIDER_LOW_RESISTOR_KOHM, VOLTAGE_UNIT, VOLTAGE_DIVIDER_SAMPLE_AMT );
CurrentSensor currentSensor( CURRENT_SENSOR_ANALOG_PIN, CURRENT_SENSOR_SENSITIVITY, CURRENT_UNIT, CURRENT_SENSOR_SAMPLE_AMT );
InfraredSensor infraredSensor( INFRARED_SENSOR_OUT_PIN, ROTATIONAL_SPEED_UNIT, INFRARED_SENSOR_SAMPLE_AMT ); 
LoadCellAmpSensor loadCellAmpSensor( LOAD_CELL_AMP_DAT_PIN, LOAD_CELL_AMP_CLK_PIN, CALIBRATION_FACTOR, TORQUE_UNIT, LOAD_CELL_AMP_SAMPLE_AMT );
Micro_SD_Card_Reader cardReader( SD_CARD_READER_CS_PIN );
Sensor *sensors[ NUM_OF_SENSORS ] = { &voltageDividerSensor, &currentSensor, &infraredSensor, &loadCellAmpSensor };

float getVoltageInput( const int pinNumber ) {
  int count = analogRead( pinNumber );
  return digitalToAnalogConverter( count );
}

float digitalToAnalogConverter( const int count ) {
  return count * VOLTAGE_PER_COUNT;
}

char *floatToString( const float value ) {
  static char valueString[ COLUMNS ];
  dtostrf( value, 1, DECIMAL_PLACES, valueString );
  return valueString;
}

void printUnits() {
  for ( unsigned int sensorIndex = 0; sensorIndex < NUM_OF_SENSORS; sensorIndex++ ) {
    screen.printUnit( sensors[ sensorIndex ], sensorIndex );
  }
}

void printMeasurements() {
  for ( unsigned int sensorIndex = 0; sensorIndex < NUM_OF_SENSORS; sensorIndex++ ) {
    screen.printMeasurement( sensors[ sensorIndex ], sensorIndex );
  }
}

void printCardDetectedScreen() {
  String infoToPrint = "File: " + cardReader.getFileName();
  screen.print( "Card Detected", 0 );
  screen.print( infoToPrint.c_str(), 1 );
}

void printCardNotDetectedScreen() {
  screen.print( "Card Not Detected", 0 );
}

void initializeCardReader() {
  bool cardReaderIsInitialized = cardReader.initialize();
  screen.clearScreen();
  if ( cardReaderIsInitialized  ) {
    printCardDetectedScreen();
  }
  else {
    printCardNotDetectedScreen();
  }
  delay( ONE_SECOND );
  screen.clearScreen();
}

void writeSensorData() {
  for ( unsigned int sensorIndex = 0; sensorIndex < NUM_OF_SENSORS; sensorIndex++ ) {
    cardReader.writeData( sensors[ sensorIndex ] );
  }
}

void writeMeasurements() {
  cardReader.writeTime();
  writeSensorData();
  cardReader.writeNewLine();
}


void initializeSensors() {
  for ( unsigned int sensorIndex = 0; sensorIndex < NUM_OF_SENSORS; sensorIndex++ ) {
    sensors[ sensorIndex ]->initialize();
  }
}
 
// void printTimer() {
//   for ( int i = 5; i >= 0; i-- ) {
//     String secondsStr = String( i ) + " s";
//     screen.print( secondsStr.c_str(), 3 );
//     delay( ONE_SECOND );
//   }
// }

// void printIntroScreen() {
//   screen.print( "CWC GEN Fall \'25", 0 );
//   screen.print( "Insert Mirco SD Card", 1 );
//   screen.print( "Time Left:", 2 );
//   printTimer();
// }

void printInitializingScreen() {
  screen.print( "CWC GEN Fall \'25", 0 );
  screen.print( "Initializing card", 2 );
  screen.print( "reader and sensors", 3 );
}

void initializeScreen() {
  screen.initialize();
  // printIntroScreen();
  // screen.clearScreen();
  printInitializingScreen();
}

void initializeDataLogging() {
  initializeScreen();
  initializeCardReader();
  printUnits();
}

float getSeconds() {
  return millis() / 1000;
}

void setup() {
  Serial.begin( BAUD_RATE );
  initializeDataLogging();
  initializeSensors();
}

void loop() {
  printMeasurements();
  writeMeasurements();
  // delay( MEASUREMENT_REFRESH_TIME );
}
