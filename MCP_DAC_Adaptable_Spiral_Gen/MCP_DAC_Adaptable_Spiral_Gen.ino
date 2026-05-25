/*____________SineGen - Memory Optimized & Non-Blocking DAC Version__________
 * Optimized for memory efficiency with:
 * - Base waveforms stored in PROGMEM (flash memory)
 * - On-the-fly computation using fixed-point arithmetic
 * - 12-bit native values with 310-point arrays
 * - Precise trigger timing for synchronization
 * - Non-Blocking mode execution using millis()
 * - Support for Single MCP4922 or Dual MCP4921s via compiler flag
 */

#include <SPI.h>
#include <avr/pgmspace.h>

// Uncomment the following line to use a single MCP4922 instead of dual MCP4921s
// #define USE_MCP4922

#ifndef USE_MCP4922
  #include "MCP_DAC.h"
  MCP4921 dac1;         // DAC for Y-axis
  MCP4921 dac2;         // DAC for X-axis
  const byte dac1_CS = 10;  // DAC1 (Y-axis)
  const byte dac2_CS = 9;   // DAC2 (X-axis)
#else
  // Faster standalone MCP4922 code
  #define MCP4922_CS_PIN 10
  void mcp4922(uint16_t value, uint8_t channel)  //  channel = 0, 1
  {
    uint16_t data = 0x3000 | value;
    if (channel == 1) data |= 0x8000;
    digitalWrite(MCP4922_CS_PIN, LOW);
    SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
    SPI.transfer((uint8_t)(data >> 8));
    SPI.transfer((uint8_t)(data & 0xFF));
    SPI.endTransaction();
    digitalWrite(MCP4922_CS_PIN, HIGH);
  }
#endif

#define TRIGGER_PIN 8  // Digital trigger output pin
#define FRACTIONAL_BITS 10  // Fixed-point fractional resolution (10 bits = 1024)
#define FIXED_SCALE (1 << (FRACTIONAL_BITS))  // Fixed-point scaling factor for without op amp

const uint16_t maxSamplesNum = 310;  // Number of points per cycle
const uint16_t maxSineNum = 79;

// Control parameters
int32_t scaleX_fixed = FIXED_SCALE;  // Fixed-point X scale (1.0 = 1024)
int32_t scaleY_fixed = FIXED_SCALE;  // Fixed-point Y scale (1.0 = 1024)
int centreX = 2048;  // 12-bit center value
int centreY = 2048;  // 12-bit center value
uint16_t msDelay = 10;  // milisecond delay between points
uint16_t usDelay = 0;  // Additional microsecond delay between points

uint16_t wavefromTruncationIdx = maxSamplesNum;
bool isFullWaveform = true;

// State Variables for non-blocking delay
enum ScanMode { MODE_SCAN, MODE_FIXED };
ScanMode currentMode = MODE_FIXED;

unsigned long lastUpdateMillis = 0;
long singleRunRemaining = 0; // >0: steps remaining, -1: infinite
int scan_base_pointer = 0;

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

void setup() {
  SPI.begin();
#ifndef USE_MCP4922
  // Initialize DACs
  dac1.begin(dac1_CS);
  dac2.begin(dac2_CS);
#else
  pinMode(MCP4922_CS_PIN, OUTPUT);
  digitalWrite(MCP4922_CS_PIN, HIGH);
#endif
  
  // Initialize trigger pin
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  
  Serial.begin(9600);
  Serial.println("MCP DAC Ready.");
  printHelp();
  
  enterFixedMode();
}

void loop() {
  if(Serial.available()) {
    processSerialCommands();
  }
  runDACs();
}

void writeDACs(uint16_t dacX, uint16_t dacY) {
#ifndef USE_MCP4922
  dac1.write(dacY);
  dac2.write(dacX);
#else
  mcp4922(dacY, 0); // Channel 0 is Y-axis
  mcp4922(dacX, 1); // Channel 1 is X-axis
#endif
}

void triggerDevice() {
  delayMicroseconds(100); // Settling delay
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(12);
  digitalWrite(TRIGGER_PIN, LOW);
  if (usDelay > 0) delayMicroseconds(usDelay);
}

void runDACs() {
  if (currentMode == MODE_FIXED) {
    delay(10);
    // Fixed mode maintains the position, but we don't continuously spam the trigger
    return;
  }

  if (millis() - lastUpdateMillis >= msDelay) {
    lastUpdateMillis = millis();

    if (singleRunRemaining > 0 || singleRunRemaining == -1) {
      stepDAC();
    } else if (singleRunRemaining == -2) {
       // Last step finished its delay duration, now enter fixed mode
       enterFixedMode();
       singleRunRemaining = 0;
    }
  }
}

void stepDAC() {
  wavefromTruncationIdx = isFullWaveform ? maxSamplesNum : maxSineNum;

  uint16_t baseX_val = pgm_read_word_near(&baseX[scan_base_pointer]);
  uint16_t baseY_val = pgm_read_word_near(&baseY[scan_base_pointer]);

  uint16_t dacX = computeDACValue(baseX_val, scaleX_fixed, centreX);
  uint16_t dacY = computeDACValue(baseY_val, scaleY_fixed, centreY);

  writeDACs(dacX, dacY);
  triggerDevice();

  scan_base_pointer++;
  if (scan_base_pointer >= wavefromTruncationIdx) {
    scan_base_pointer = 0;
  }

  if (singleRunRemaining > 0) {
    singleRunRemaining--;
    if (singleRunRemaining <= 0) {
      singleRunRemaining = -2; // Wait one cycle before entering fixed mode
    }
  }
}

void enterFixedMode() {
  currentMode = MODE_FIXED;
  singleRunRemaining = 0;
  writeDACs(centreX, centreY);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(12);
  digitalWrite(TRIGGER_PIN, LOW);
  Serial.println("Laser at Center (Fixed mode)");
}

void setTIRFPosition() {
  currentMode = MODE_FIXED;
  singleRunRemaining = 0;
  uint16_t tirfY = computeDACValue(4095, scaleY_fixed, centreY);
  writeDACs(centreX, tirfY);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(12);
  digitalWrite(TRIGGER_PIN, LOW);
  Serial.println("Laser at TIRF position (Fixed mode)");
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
  String data;
  while(Serial.available()) {
    data += (char)Serial.read();
    delay(2);
  }
  if (data.length() == 0) return;

  int idx = 0;
  int len = data.length();
  
  // Quick parse char by char for commands
  while(idx < len) {
    char cmd = data[idx];
    if (isWhitespace(cmd)) { idx++; continue; }

    if (cmd == 'c') { enterFixedMode(); idx++; }
    else if (cmd == 'q') { enterFixedMode(); idx++; }
    else if (cmd == 'p') {
      idx++;
      String floatStr = "";
      while(idx < len && (isDigit(data[idx]) || data[idx]=='.' || data[idx]=='-')) {
        floatStr += data[idx];
        idx++;
      }
      if (floatStr.length() > 0) {
        scaleY_fixed = floatStr.toFloat() * FIXED_SCALE;
        Serial.print("TIRF Y scale set to: "); Serial.println((float)scaleY_fixed / FIXED_SCALE);
      }
      setTIRFPosition();
    }
    else if (cmd == 'o') { singleRunRemaining = -1; currentMode = MODE_SCAN; Serial.println("Infinite Run (q to quit)."); idx++; }
    else if (cmd == 's') { 
      // check if it has a value attached
      idx++;
      long steps = 0;
      bool hasVal = false;
      while(idx < len && isDigit(data[idx])) {
        hasVal = true;
        steps = steps * 10 + (data[idx] - '0');
        idx++;
      }
      if (!hasVal) steps = isFullWaveform ? maxSamplesNum : maxSineNum; // Full cycle
      
      scan_base_pointer = 0; 
      singleRunRemaining = steps;
      currentMode = MODE_SCAN;
      Serial.print("Running "); Serial.print(steps); Serial.println(" steps.");
    }
    else if (cmd == 'm') {
      idx++;
      long val = 0;
      while(idx < len && isDigit(data[idx])) {
        val = val * 10 + (data[idx] - '0');
        idx++;
      }
      msDelay = val;
      Serial.print("ms Delay set to: "); Serial.println(msDelay);
    }
    else if (cmd == 'u') {
      idx++;
      long val = 0;
      while(idx < len && isDigit(data[idx])) {
        val = val * 10 + (data[idx] - '0');
        idx++;
      }
      usDelay = val;
      Serial.print("us Delay set to: "); Serial.println(usDelay);
    }
    else if (cmd == 'x') {
      idx++;
      String floatStr = "";
      while(idx < len && (isDigit(data[idx]) || data[idx]=='.' || data[idx]=='-')) {
        floatStr += data[idx];
        idx++;
      }
      if (floatStr.length() > 0) {
        scaleX_fixed = floatStr.toFloat() * FIXED_SCALE;
        Serial.print("X scale set to: "); Serial.println((float)scaleX_fixed / FIXED_SCALE);
      }
    }
    else if (cmd == 'y') {
      idx++;
      String floatStr = "";
      while(idx < len && (isDigit(data[idx]) || data[idx]=='.' || data[idx]=='-')) {
        floatStr += data[idx];
        idx++;
      }
      if (floatStr.length() > 0) {
        scaleY_fixed = floatStr.toFloat() * FIXED_SCALE;
        Serial.print("Y scale set to: "); Serial.println((float)scaleY_fixed / FIXED_SCALE);
      }
    }
    else if (cmd == '^') {
      idx++;
      long val = 0; bool neg=false;
      if (idx < len && data[idx]=='-') { neg=true; idx++; }
      while(idx < len && isDigit(data[idx])) {
        val = val * 10 + (data[idx] - '0');
        idx++;
      }
      if (neg) val = -val;
      centreY = constrain(centreY + val, 0, 4095);
      Serial.print("Y centre: "); Serial.println(centreY);
      if (currentMode == MODE_FIXED) {
        writeDACs(centreX, centreY);
        digitalWrite(TRIGGER_PIN, HIGH);
        delayMicroseconds(12);
        digitalWrite(TRIGGER_PIN, LOW);
      }
    }
    else if (cmd == '>') {
      idx++;
      long val = 0; bool neg=false;
      if (idx < len && data[idx]=='-') { neg=true; idx++; }
      while(idx < len && isDigit(data[idx])) {
        val = val * 10 + (data[idx] - '0');
        idx++;
      }
      if (neg) val = -val;
      centreX = constrain(centreX + val, 0, 4095);
      Serial.print("X centre: "); Serial.println(centreX);
      if (currentMode == MODE_FIXED) {
        writeDACs(centreX, centreY);
        digitalWrite(TRIGGER_PIN, HIGH);
        delayMicroseconds(12);
        digitalWrite(TRIGGER_PIN, LOW);
      }
    }
    else if (cmd == 'f') { Serial.println("Full waveform/spiral"); isFullWaveform = true; idx++; }
    else if (cmd == 't') { Serial.println("Trunc waveform/sine"); isFullWaveform = false; idx++; }
    else if (cmd == 'h') { printHelp(); idx++; }
    else {
      idx++; // skip unknown
    }
  }
}

void printHelp() {
  Serial.println("--- DAC Firmware ---");
  Serial.println("s/s<val>   - Run single cycle or cycles up to <val> steps");
  Serial.println("o          - Run infinite scanning");
  Serial.println("c/q        - Stop scanning and return to center");
  Serial.println("p/p<val>   - Normal TIRF position (Fixed), <val> sets Y scale");
  Serial.println("f          - Full waveform/spiral");
  Serial.println("t          - Truncated waveform/sine");
  Serial.println("x<val>     - X scaling factor (e.g. x1.0)");
  Serial.println("y<val>     - Y scaling factor (e.g. y1.0)");
  Serial.println("><val>     - Shift X center (e.g. >10)");
  Serial.println("^<val>     - Shift Y center (e.g. ^-10)");
  Serial.println("m<val>     - Delay ms (e.g. m10)");
  Serial.println("u<val>     - Delay us (e.g. u100)");
  Serial.println("h          - Print this help message");
  Serial.println("--------------------");
}
