#if defined(TTIMU_I2C_EXAMPLE)
#include "../MPU6050_I2C/MPU6050_I2C.ino"
#elif defined(ARDUINO_ARCH_STM32)
#include "../STM32_SPI/STM32_SPI.ino"
#else
#include "../SerialPlotter/SerialPlotter.ino"
#endif
