#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdio.h>

void initMiniTinyI2C(const uint16_t baud);
uint8_t readMiniTinyI2C(bool stop);
bool writeMiniTinyI2C(uint8_t data);
bool startMiniTinyI2C(uint8_t address, bool read);
void stopMiniTinyI2C();

#ifdef __cplusplus
}
#endif
