// Libraries
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include "HX711.h"
#include <SPI.h>
#include <SD.h>


// Constants

  // Timing
  #define DELAY_TIME 500 // ms
  #define DEBOUNCE_DURATION 20 // ms

  // Other
  #define DECIMAL_PLACES  1
  #define RESOLUTION 1023
  #define NUM_OF_SENSORS 4
  #define MAX_STRING_LENGTH 128
  #define STANDARD_STRING_LENGTH 64
  #define BAUD_RATE 9600

  // Arduino Specs
  #define REFERENCE_VOLTAGE 4.54
  const float VOLTAGE_PER_COUNT = REFERENCE_VOLTAGE / RESOLUTION;
  const float MIDPOINT_VOLTAGE = REFERENCE_VOLTAGE / 2;

  // Characters/Units
  #define SPACE ' '
  #define TIME_UNIT "s"
  #define VOLTAGE_UNIT "V"
  #define CURRENT_UNIT "A"
  #define ROTATIONAL_SPEED_UNIT "RPM"
  #define TORQUE_UNIT "N-mm"

  // Voltage Divider
  #define VOLTAGE_DIVIDER_ANALOG_PIN A0

  // Current Sensor
  #define CURRENT_SENSOR_ANALOG_PIN A1

  // Infrared Sensor
  #define INFRARED_SENSOR_OUT_PIN 2

  // Load Cell Amp
  #define LOAD_CELL_AMP_DAT_PIN 6
  #define LOAD_CELL_AMP_CLK_PIN 7

  // SD Card Reader
  #define SD_CARD_READER_CS_PIN 10

  // Load Cell Amp
  #define CALIBRATION_FACTOR 372.5

  // LCD Screen
  #define I2C_ADDRESS 0x27
  #define COLUMNS 20
  #define ROWS 4


float getVoltageInput( const int pinNumber );

class LED {

  public:
    LED( int digitalPin ) {
      dPin = digitalPin;
    }

    void initialize() {
      pinMode( dPin, OUTPUT );
    }

    void turnOn() {
      digitalWrite( dPin, HIGH );
    }

    void turnOff() {
      digitalWrite( dPin, LOW );
    }

    void blink() {
      turnOn();
      delay( DELAY_TIME );
      turnOff();
      delay( DELAY_TIME );
    }

  private:
    int dPin;
};

class LCD_Screen {
  public:
    LCD_Screen( int i2cAddress, int numOfColumns, int numOfRows ) : screen( i2cAddress, numOfColumns, numOfRows ) {
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

    template < typename Sensor >
    void printUnit( Sensor *sensor, const int line ) {
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

    template < typename Sensor >
    void printMeasurement( Sensor *sensor, const int line ) {
      float sensorMeasurement = sensor->measure();
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

  class Sensor {
    public:
      virtual void initialize() {}
      virtual float measure() {}
      virtual String getUnit() {}
      virtual float getMeasurement() {}
  };

  // Voltage Divider
  class VoltageDivider : public Sensor {
    public:
      VoltageDivider( int analogPin, float resistance1, float resistance2 ) {
        multiplier = resistance1 + resistance2;
        aPin = analogPin;
        voltage = 0;
      }

      void initialize() {
        pinMode( aPin, INPUT );
      }

      float measure() {
        float voltageIn = getVoltageInput( aPin );
        voltage = voltageIn * multiplier;
        return voltage;
      }

      String getUnit() {
        return VOLTAGE_UNIT;
      }

      float getMeasurement() {
        return voltage;
      }

    private:
      float multiplier;
      float voltage;
      int aPin;
  };

  // Current Sensor
  class CurrentSensor : public Sensor {

    public:
      CurrentSensor( int analogPin, float sensitivity ) {
        aPin = analogPin;
        s = sensitivity;
        current = 0;
      }

      void initialize() {
        pinMode( aPin, INPUT );
      }

      float measure() {
        float voltageIn = getVoltageInput( aPin ); 
        float ampVoltage = voltageIn - MIDPOINT_VOLTAGE;
        current = 0; // Just in case ampVoltage is negative
        if ( ampVoltage > 0 ) {
          current = ampVoltage / s;
        }
        return current;
      }

      String getUnit() {
        return CURRENT_UNIT;
      }

      float getMeasurement() {
        return current;
      }

    private:
      int aPin;
      float s;
      float current;
  };

  


  // IR Sensor
  volatile long counter = 0;
  volatile unsigned long lastInterruptTime = 0;

  void InfraredInterrupt() {
    if ( millis() - lastInterruptTime > DEBOUNCE_DURATION ) {
      
      // if ( digitalRead( INFRARED_SENSOR_OUT_PIN ) == LOW ) {
        counter++;
        Serial.println( counter );
        lastInterruptTime = millis();
      // }
    }
  }

  class InfraredSensor : public Sensor {
    public:
      InfraredSensor( int digitalPin, int pulsePerRotation ) {
        dPin = digitalPin;
        ppr = pulsePerRotation;
        RPM = 0.0;
        previousTime = millis();
      }

      void initialize() {
        pinMode( dPin, INPUT_PULLUP );
        attachInterrupt( digitalPinToInterrupt( dPin ), InfraredInterrupt, FALLING );
      }

      float measure() {
        unsigned long currentTime = millis() - previousTime;
        long rotations;
        double minutesPassed;
        if ( currentTime >= DELAY_TIME ) {
          // detachInterrupt( digitalPinToInterrupt( dPin ) ); // Stop tracking for pulses
          noInterrupts();
          rotations = counter / ( long ) ppr;
          minutesPassed = currentTime / 60000.0; // Converts ms to s, then to minutes
          RPM = rotations / minutesPassed;  // Calculate RPM
          Serial.println( "Reset" );
          counter = 0;
          // attachInterrupt( digitalPinToInterrupt( dPin ), InfraredInterrupt, FALLING ); // Start tracking for pulses
          interrupts();
          previousTime = millis();
        }
        return RPM;
      }

      String getUnit() {
        return ROTATIONAL_SPEED_UNIT;
      }

      float getMeasurement() {
        return RPM;
      }

    private:
      float RPM;
      int ppr;
      unsigned long previousTime;
      int dPin;
  };



  // Load Cell Amp
  class LoadCellAmp : public Sensor {
    public:
      LoadCellAmp( int dataPin, int clockPin, float calibrationFactor ) {
        dPin = dataPin;
        clkPin = clockPin;
        torque = 0.0;
        calFac = calibrationFactor;
      }

      void initialize() {
        scale.begin( dPin, clkPin );
        scale.set_scale();
        scale.tare();
      }

      float measure() {
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
        return scale.get_units(3);
      }

      String getUnit() {
        return TORQUE_UNIT;
      }

      float getMeasurement() {
        return torque;
      }

    private:
      HX711 scale;
      int dPin;
      int clkPin;
      float torque;
      float calFac;
  };
  
  // Button
  class Button {
    public:
      Button( int digitalPin ) : previousState( 0 ), previousPressTime( millis() ), dPin( digitalPin ) {
        pinMode( dPin, INPUT );
      }

      bool wasPressed() {
        int currentState;
        if ( millis() - previousPressTime > DEBOUNCE_DURATION ) {
          currentState = digitalRead( dPin );
          if ( currentState != previousState ) {
            previousPressTime = millis();
            previousState = currentState;
            return currentState == HIGH;
          }
        }
        return false;
      }

  private:
    int previousState;
    long previousPressTime;
    int dPin;
};


// SD Card Reader
class Micro_SD_Card_Reader {
  public:
    Micro_SD_Card_Reader( int chipSelectPin ) {
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

    template < typename Sensor >
    void writeData( Sensor *sensor ) {
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

    void setCardIsAvailable( bool available ) {
      cardIsAvailable = available;
    }

  private:
    int csPin;
    char fileName[ STANDARD_STRING_LENGTH ];
    bool cardIsAvailable;
};

// LED greenLed( 9 );
// LED redLed( 8 );
LCD_Screen screen( I2C_ADDRESS, COLUMNS, ROWS );
VoltageDivider *voltageDivider = new VoltageDivider( VOLTAGE_DIVIDER_ANALOG_PIN, 9.92, 0.995 );
CurrentSensor *currentSensor = new CurrentSensor( CURRENT_SENSOR_ANALOG_PIN, 0.066 );
InfraredSensor *infraredSensor = new InfraredSensor( INFRARED_SENSOR_OUT_PIN, 1 ); 
LoadCellAmp *loadCellAmp = new LoadCellAmp( LOAD_CELL_AMP_DAT_PIN, LOAD_CELL_AMP_CLK_PIN, CALIBRATION_FACTOR );
Micro_SD_Card_Reader cardReader( SD_CARD_READER_CS_PIN );
Sensor *sensors[ NUM_OF_SENSORS ] = { voltageDivider, currentSensor, infraredSensor, loadCellAmp };

float getVoltageInput( const int pinNumber ) {
  int count = analogRead( pinNumber );
  return digitalToAnalogConverter( count );
}

float digitalToAnalogConverter( const int count ) {
  return count * VOLTAGE_PER_COUNT;
}

char *floatToString( const float value ) {
  static char valueString[ 20 ];
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
  if ( cardReader.initialize() ) {
    printCardDetectedScreen();
  }
  else {
    printCardNotDetectedScreen();
  }
  delay( 5000 );
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
 
void printTimer() {
  for ( int i = 5; i >= 0; i-- ) {
    String secondsStr = String( i ) + " s";
    screen.print( secondsStr.c_str(), 3 );
    delay( 1000 );
  }
}

void printIntroScreen() {
  screen.print( "CWC GEN FA '25", 0 );
  screen.print( "Insert Mirco SD Card", 1 );
  screen.print( "Time Left:", 2 );
  printTimer();
}

void printInitializingScreen() {
  screen.print( "Initializing...", 0 );
}

void initializeScreen() {
  screen.initialize();
  printIntroScreen();
  screen.clearScreen();
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
  delay( DELAY_TIME );
}
