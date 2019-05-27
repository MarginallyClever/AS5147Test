// AS5147 test
// dan@marginallyclever.com
// 2019-05-16
// @see https://ams.com/documents/20143/36005/AS5147_DS000307_2-00.pdf


//#define VERBOSE
#define GRAPHABLE

// sensor bits, flags, and masks
#define BOTTOM_14_MASK       (0x3FFF)
#define SENSOR_TOTAL_BITS    (16)
#define SENSOR_DATA_BITS     (15)
#define SENSOR_ANGLE_BITS    (14)
#define SENSOR_ANGLE_PER_BIT (360.0/(float)((long)1<<SENSOR_ANGLE_BITS))  // 0.00549316406



#define PIN_SENSOR_CSEL_0   11
#define  PIN_SENSOR_CLK_0   10
#define PIN_SENSOR_MOSI_0   9
#define PIN_SENSOR_MISO_0   8

#define PIN_SENSOR_CSEL_1   5
#define  PIN_SENSOR_CLK_1   4
#define PIN_SENSOR_MOSI_1   3
#define PIN_SENSOR_MISO_1   2

#define PIN_SENSOR_CSEL_2   14
#define  PIN_SENSOR_CLK_2   15
#define PIN_SENSOR_MOSI_2   16
#define PIN_SENSOR_MISO_2   17

#define PIN_SENSOR_CSEL_3   18
#define  PIN_SENSOR_CLK_3   19
#define PIN_SENSOR_MOSI_3   20
#define PIN_SENSOR_MISO_3   21

#define PIN_SENSOR_CSEL_4   45
#define  PIN_SENSOR_CLK_4   43
#define PIN_SENSOR_MOSI_4   41
#define PIN_SENSOR_MISO_4   39

#define PIN_SENSOR_CSEL_5   29
#define  PIN_SENSOR_CLK_5   27
#define PIN_SENSOR_MOSI_5   25
#define PIN_SENSOR_MISO_5   23


char sensorPins[4*6];
float sensorAngles[6];


void setup() {
  Serial.begin(57600);
  
  int i=0;

  sensorPins[i++]=PIN_SENSOR_CSEL_0;
  sensorPins[i++]=PIN_SENSOR_CLK_0;
  sensorPins[i++]=PIN_SENSOR_MISO_0;
  sensorPins[i++]=PIN_SENSOR_MOSI_0;
  
  sensorPins[i++]=PIN_SENSOR_CSEL_1;
  sensorPins[i++]=PIN_SENSOR_CLK_1;
  sensorPins[i++]=PIN_SENSOR_MISO_1;
  sensorPins[i++]=PIN_SENSOR_MOSI_1;
  
  sensorPins[i++]=PIN_SENSOR_CSEL_2;
  sensorPins[i++]=PIN_SENSOR_CLK_2;
  sensorPins[i++]=PIN_SENSOR_MISO_2;
  sensorPins[i++]=PIN_SENSOR_MOSI_2;
  
  sensorPins[i++]=PIN_SENSOR_CSEL_3;
  sensorPins[i++]=PIN_SENSOR_CLK_3;
  sensorPins[i++]=PIN_SENSOR_MISO_3;
  sensorPins[i++]=PIN_SENSOR_MOSI_3;
  
  sensorPins[i++]=PIN_SENSOR_CSEL_4;
  sensorPins[i++]=PIN_SENSOR_CLK_4;
  sensorPins[i++]=PIN_SENSOR_MISO_4;
  sensorPins[i++]=PIN_SENSOR_MOSI_4;
  
  sensorPins[i++]=PIN_SENSOR_CSEL_5;
  sensorPins[i++]=PIN_SENSOR_CLK_5;
  sensorPins[i++]=PIN_SENSOR_MISO_5;
  sensorPins[i++]=PIN_SENSOR_MOSI_5;

  for(i=0;i<6;++i) {
    pinMode(sensorPins[(i*4)+0],OUTPUT);  // csel
    pinMode(sensorPins[(i*4)+1],OUTPUT);  // clk
    pinMode(sensorPins[(i*4)+2],INPUT);  // miso
    pinMode(sensorPins[(i*4)+3],OUTPUT);  // mosi

    digitalWrite(sensorPins[(i*4)+0],HIGH);  // csel
    digitalWrite(sensorPins[(i*4)+1],LOW);  // clk
    digitalWrite(sensorPins[(i*4)+3],HIGH);  // mosi
  }
}


/**
 * @param index the sensor to read
 * @param result where to store the returned value.  may be changed even if method fails.
 * @return 0 on fail, 1 on success.
// @see https://ams.com/documents/20143/36005/AS5147_DS000307_2-00.pdf
 */
boolean getSensorRawValue(int index, uint16_t &result) {
  result=0;
  uint8_t input,parity=0;

  index*=4;
  
  // Send the request for the angle value (command 0xFFFF)
  // at the same time as receiving an angle.

  // Collect the 16 bits of data from the sensor
  digitalWrite(sensorPins[index+0],LOW);  // csel
  
  for(int i=0;i<SENSOR_TOTAL_BITS;++i) {
    digitalWrite(sensorPins[index+1],HIGH);  // clk
    // this is here to give a little more time to the clock going high.
    // only needed if the arduino is *very* fast.  I'm feeling generous.
    result <<= 1;
    digitalWrite(sensorPins[index+1],LOW);  // clk
    
    input = digitalRead(sensorPins[index+2]);  // miso
#ifdef VERBOSE
    Serial.print(input,DEC);
#endif
    result |= input;
    parity ^= (i>0) & input;
  }
#ifdef VERBOSE
  Serial.print('\t');
#endif

  digitalWrite(sensorPins[index+0],HIGH);  // csel
  
  // check the parity bit
  return ( parity != (result>>SENSOR_DATA_BITS) );
}


/**
 * @param rawValue 16 bit value from as4157 sensor, including parity and EF bit
 * @return degrees calculated from bottom 14 bits.
 */
float extractAngleFromRawValue(uint16_t rawValue) {
  return (float)(rawValue & BOTTOM_14_MASK) * 360.0 / (float)(1<<SENSOR_ANGLE_BITS);
}


void loop() {
#ifdef VERBOSE
  Serial.print(millis());
  Serial.print('\t');
#endif

  uint16_t rawValue;
  for(int i=0;i<6;++i) {
    if(!getSensorRawValue(i,rawValue)) {
      sensorAngles[i] = extractAngleFromRawValue(rawValue);
#ifdef GRAPHABLE
      Serial.print(sensorAngles[i],3);
      Serial.print('\t');
#endif
    }
  }
  Serial.println();
}
