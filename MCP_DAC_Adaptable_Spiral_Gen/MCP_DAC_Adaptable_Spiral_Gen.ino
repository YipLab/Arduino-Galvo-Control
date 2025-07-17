/*____________SineGenNoPWM - Memory Optimized DAC Version__________________________________
 * Optimized for memory efficiency with:
 * - Base waveforms stored in PROGMEM (flash memory)
 * - On-the-fly computation using fixed-point arithmetic
 * - 12-bit native values with 310-point arrays
 * - Precise trigger timing for synchronization
 */

#include "MCP_DAC.h"  // Include DAC library
#include <avr/pgmspace.h>  // For PROGMEM

MCP4921 dac1;         // DAC for Y-axis
MCP4921 dac2;         // DAC for X-axis
#define TRIGGER_PIN 8  // Digital trigger output pin
#define FRACTIONAL_BITS 10  // Fixed-point fractional resolution (10 bits = 1024)
#define FIXED_SCALE (1 << FRACTIONAL_BITS)  // Fixed-point scaling factor

const uint16_t maxSamplesNum = 310;  // Number of points per cycle

// Chip select pins
const byte dac1_CS = 10;  // DAC1 (Y-axis)
const byte dac2_CS = 9;   // DAC2 (X-axis)

// Base waveforms stored in PROGMEM (flash memory)
const uint16_t baseX[maxSamplesNum] PROGMEM = {
  4096, 4090, 4070, 4038, 3994, 3937, 3868, 3788, 3697, 3596, 3485,
  3365, 3236, 3100, 2958, 2809, 2656, 2500, 2340, 2178, 2015, 1853,
  1692, 1533, 1378, 1227, 1081,  941,  808,  682,  566,  458,  361,
    274,  199,  135,   83,   43,   16,    2,    1,   13,   37,   74,
    123,  185,  258,  343,  438,  543,  658,  782,  913, 1052, 1197,
  1347, 1502, 1660, 1821, 1983, 2146, 2308, 2468, 2625, 2779, 2929,
  3072, 3210, 3340, 3461, 3574, 3678, 3771, 3853, 3924, 3983, 4030,
  4065, 4087, 4096, 4092, 4075, 4046, 4004, 3949, 3883, 3805, 3716,
  3617, 3508, 3389, 3262, 3128, 2987, 2840, 2687, 2531, 2372, 2211,
  2049, 2156, 1725, 1310,  929,  601,  340,  158,   63,   58,  143,
    312,  555,  861, 1214, 1596, 1989, 2375, 2734, 3051, 3314, 3510,
  3634, 3682, 3654, 3556, 3394, 3180, 2925, 2645, 2352, 2063, 1789,
  1544, 1338, 1179, 1070, 1013, 1008, 1051, 1136, 1256, 1401, 1564,
  1732, 1899, 2056, 2194, 2310, 2399, 2461, 2494, 2500, 2483, 2447,
  2395, 2334, 2269, 2203, 2142, 2087, 2041, 2007, 1984, 1972, 1968,
  1972, 1981, 1993, 2007, 2019, 2030, 2038, 2044, 2046, 2047, 2049,
  2050, 2052, 2058, 2066, 2077, 2089, 2103, 2115, 2124, 2128, 2124,
  2112, 2089, 2055, 2009, 1954, 1893, 1827, 1762, 1701, 1649, 1613,
  1596, 1602, 1635, 1697, 1786, 1902, 2040, 2197, 2364, 2532, 2695,
  2840, 2960, 3045, 3088, 3083, 3026, 2917, 2758, 2552, 2307, 2033,
  1744, 1451, 1171,  916,  702,  540,  442,  414,  462,  586,  782,
  1045, 1362, 1721, 2107, 2500, 2882, 3235, 3541, 3784, 3953, 4038,
  4033, 3938, 3756, 3495, 3167, 2786, 2371, 1940, 2047, 1885, 1724,
  1565, 1409, 1256, 1109,  968,  834,  707,  588,  479,  380,  291,
    213,  147,   92,   50,   21,    4,    0,    9,   31,   66,  113,
    172,  243,  325,  418,  522,  635,  756,  886, 1024, 1167, 1317,
  1471, 1628, 1788, 1950, 2113, 2275, 2436, 2594, 2749, 2899, 3044,
  3183, 3314, 3438, 3553, 3658, 3753, 3838, 3911, 3973, 4022, 4059,
  4083, 4095
};

const uint16_t baseY[maxSamplesNum] PROGMEM = {
  2049, 2211, 2372, 2531, 2687, 2840, 2987, 3128, 3262, 3389, 3508,
  3617, 3716, 3805, 3883, 3949, 4004, 4046, 4075, 4092, 4096, 4087,
  4065, 4030, 3983, 3924, 3853, 3771, 3678, 3574, 3461, 3340, 3210,
  3072, 2929, 2779, 2625, 2468, 2308, 2146, 1983, 1821, 1660, 1502,
  1347, 1197, 1052,  913,  782,  658,  543,  438,  343,  258,  185,
    123,   74,   37,   13,    1,    2,   16,   43,   83,  135,  199,
    274,  361,  458,  566,  682,  808,  941, 1081, 1227, 1378, 1533,
  1692, 1853, 2015, 2178, 2340, 2500, 2656, 2809, 2958, 3100, 3236,
  3365, 3485, 3596, 3697, 3788, 3868, 3937, 3994, 4038, 4070, 4090,
  4096, 4093, 4070, 3956, 3756, 3480, 3141, 2756, 2344, 1921, 1512,
  1133,  801,  532,  337,  222,  193,  247,  380,  584,  847, 1155,
  1491, 1840, 2184, 2507, 2794, 3035, 3219, 3341, 3398, 3391, 3323,
  3203, 3037, 2839, 2618, 2387, 2158, 1942, 1749, 1587, 1460, 1372,
  1325, 1316, 1342, 1399, 1480, 1579, 1688, 1801, 1910, 2011, 2099,
  2170, 2223, 2257, 2274, 2275, 2263, 2240, 2211, 2178, 2145, 2115,
  2088, 2067, 2052, 2041, 2037, 2036, 2039, 2042, 2045, 2047, 2049,
  2051, 2054, 2057, 2060, 2059, 2055, 2044, 2029, 2008, 1981, 1951,
  1918, 1885, 1856, 1833, 1821, 1822, 1839, 1873, 1926, 1997, 2085,
  2186, 2295, 2408, 2517, 2616, 2697, 2754, 2780, 2771, 2724, 2636,
  2509, 2347, 2154, 1938, 1709, 1478, 1257, 1059,  893,  773,  705,
    698,  755,  877, 1061, 1302, 1589, 1912, 2256, 2605, 2941, 3249,
  3512, 3716, 3849, 3903, 3874, 3759, 3564, 3295, 2963, 2584, 2175,
  1752, 1340,  955,  616,  340,  140,   26,    3,    0,    6,   26,
    58,  102,  159,  228,  308,  399,  500,  611,  731,  860,  996,
  1138, 1287, 1440, 1596, 1756, 1918, 2081, 2243, 2404, 2563, 2718,
  2869, 3015, 3155, 3288, 3414, 3530, 3638, 3735, 3822, 3897, 3961,
  4013, 4053, 4080, 4094, 4095, 4083, 4059, 4022, 3973, 3911, 3838,
  3753, 3658, 3553, 3438, 3314, 3183, 3044, 2899, 2749, 2594, 2436,
  2275, 2113
};

// Control parameters
int32_t scaleX_fixed = FIXED_SCALE;  // Fixed-point X scale (1.0 = 1024)
int32_t scaleY_fixed = FIXED_SCALE;  // Fixed-point Y scale (1.0 = 1024)
int centreX = 2048;  // 12-bit center value
int centreY = 2048;  // 12-bit center value
uint16_t msDelay = 5000;  // Microsecond delay between points

void setup() {
  SPI.begin();
  // Initialize DACs
  dac1.begin(dac1_CS);
  dac2.begin(dac2_CS);
  
  // Initialize trigger pin
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  
  Serial.begin(9600);
  setCenterPosition();
}

void loop() {
  for(uint16_t i = 0; i < maxSamplesNum; i++) {
    // Read base values from PROGMEM
    uint16_t baseX_val = pgm_read_word_near(&baseX[i]);
    uint16_t baseY_val = pgm_read_word_near(&baseY[i]);
    
    // Compute scaled values using fixed-point arithmetic
    uint16_t dacX = computeDACValue(baseX_val, scaleX_fixed, centreX);
    uint16_t dacY = computeDACValue(baseY_val, scaleY_fixed, centreY);
    
    // Update DACs
    dac1.write(dacY);
    dac2.write(dacX);
    
    // Generate precise trigger pulse
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(12);  // Maintain pulse width
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(6);   // Inter-pulse delay
    
    // Apply user-defined delay
    if(msDelay) delayMicroseconds(msDelay);
  }

  // Handle serial commands at end of cycle
  if(Serial.available()) {
    processSerialCommands();
  }
}

// Compute DAC value using fixed-point arithmetic
uint16_t computeDACValue(uint16_t base, int32_t scale_fixed, int centre) {
  // Convert to signed 32-bit for calculations
  int32_t val = (int32_t)base - 2048;  // Center at 0
  
  // Apply scaling (fixed-point multiplication)
  val = (val * scale_fixed) >> FRACTIONAL_BITS;
  
  // Apply center offset and constrain
  val += centre;
  if(val < 0) return 0;
  if(val > 4095) return 4095;
  return (uint16_t)val;
}

void processSerialCommands() {
  char command = Serial.read();
  String input;
  
  switch(command) {
    case 'c':  // Center position
      setCenterPosition();
      break;
      
    case 'p':  // Normal TIRF position
      setTIRFPosition();
      break;
      
    case 'x':  // X scaling
      scaleX_fixed = readFloatFromSerial() * FIXED_SCALE;
      Serial.print("X scale set to: ");
      Serial.println((float)scaleX_fixed / FIXED_SCALE);
      break;
      
    case 'y':  // Y scaling
      scaleY_fixed = readFloatFromSerial() * FIXED_SCALE;
      Serial.print("Y scale set to: ");
      Serial.println((float)scaleY_fixed / FIXED_SCALE);
      break;
      
    case 'd':  // Delay setting
      msDelay = readIntFromSerial();
      Serial.print("Delay set to: ");
      Serial.println(msDelay);
      break;
      
    case '^':  // Y shift
      centreY = constrain(centreY + readIntFromSerial(), 0, 4095);
      Serial.print("Y centre: ");
      Serial.println(centreY);
      break;
      
    case '>':  // X shift
      centreX = constrain(centreX + readIntFromSerial(), 0, 4095);
      Serial.print("X centre: ");
      Serial.println(centreX);
      break;
      
    default:
      Serial.print("Unknown command: ");
      Serial.println(command);
  }
}

void setCenterPosition() {
  digitalWrite(TRIGGER_PIN, HIGH);
  dac1.write(centreY);
  dac2.write(centreX);
  digitalWrite(TRIGGER_PIN, LOW);
  Serial.println("Laser at Center, press 'q' to exit");
  waitForQuit();
}

void setTIRFPosition() {
  // Calculate TIRF position (max X, center Y)
  uint16_t tirfX = computeDACValue(4095, scaleX_fixed, centreX);
  
  digitalWrite(TRIGGER_PIN, HIGH);
  dac1.write(centreY);
  dac2.write(tirfX);
  digitalWrite(TRIGGER_PIN, LOW);
  Serial.println("Laser at TIRF position, press 'q' to exit");
  waitForQuit();
}

void waitForQuit() {
  while(Serial.read() != 'q');  // Wait for 'q'
  Serial.println("Exiting fixed position");
}

float readFloatFromSerial() {
  String input;
  while(!Serial.available());  // Wait for data
  delay(10);  // Allow buffer to fill
  
  while(Serial.available()) {
    char c = Serial.read();
    if(c == '\n' || c == '\r') break;
    if(isDigit(c) || c == '.' || c == '-') input += c;
  }
  return input.toFloat();
}

int readIntFromSerial() {
  String input;
  while(!Serial.available());  // Wait for data
  delay(10);  // Allow buffer to fill
  
  while(Serial.available()) {
    char c = Serial.read();
    if(c == '\n' || c == '\r') break;
    if(isDigit(c) || c == '-') input += c;
  }
  return input.toInt();
}

/* Memory Optimization Notes:
 * - Base waveforms stored in PROGMEM (flash) instead of RAM
 * - Fixed-point arithmetic replaces floating-point calculations
 * - On-the-fly computation eliminates need for output buffers
 * - 310 samples * 2 bytes * 2 buffers = 1240 bytes saved
 * - Total RAM reduction: ~1240 bytes (61% of 2048)
 */