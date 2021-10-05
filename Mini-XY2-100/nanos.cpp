#include <Arduino.h>

uint64_t nanos()  // Code by luni @ https://forum.pjrc.com/threads/60493-A-Higher-Resolution-Micros
{
    static uint32_t oldCycles = ARM_DWT_CYCCNT;
    static uint64_t highValue = 0;

    uint32_t newCycles = ARM_DWT_CYCCNT;
    if (newCycles < oldCycles)
    {
        highValue += 0x0000'0001'0000'0000;
    }
    oldCycles = newCycles;
    return (highValue | newCycles) * (1E9/F_CPU);
}

void nanosleep(uint64_t ns) {
  uint64_t _st = nanos();
  while(nanos() - _st < ns);
  // Serial.printf("%" PRIu64 " ns\t", nanos()-_stamp);
  // while(1); takes about 75ns
}

// EXAMPLE 
//
//void setup()
//{
//    // call nanos every 5s in the background. Only needed if your sketch doesn't call nanos at least once per 7s
//    (new IntervalTimer())->begin([] { nanos(); }, 5'000'000);  
//}
//
//void loop()
//{
//    Serial.printf("%" PRIu64 " ns\n", nanos());
//    delay(500);
//}
