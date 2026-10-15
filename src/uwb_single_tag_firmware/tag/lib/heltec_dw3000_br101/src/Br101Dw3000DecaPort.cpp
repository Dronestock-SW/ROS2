#include "Br101Dw3000Port.h"

extern "C" {

static const struct dwt_spi_s dw3000_spi_fct = {
  .readfromspi = dw3000_spi_read,
  .writetospi = dw3000_spi_write,
  .writetospiwithcrc = dw3000_spi_write_crc,
  .setslowrate = dw3000_spi_speed_slow,
  .setfastrate = dw3000_spi_speed_fast,
};

#if CONFIG_DW3000_CHIP_DW3000
extern const struct dwt_driver_s dw3000_driver;
static const struct dwt_driver_s* tmp_ptr[] = {
  &dw3000_driver,
};
#elif CONFIG_DW3000_CHIP_DW3720
extern const struct dwt_driver_s dw3720_driver;
static const struct dwt_driver_s* tmp_ptr[] = {
  &dw3720_driver,
};
#else
#error "No DW3000 chip variant configured"
#endif

const struct dwt_probe_s dw3000_probe_interf = {
  .dw = NULL,
  .spi = (void*)&dw3000_spi_fct,
  .wakeup_device_with_io = dw3000_hw_wakeup,
  .driver_list = (struct dwt_driver_s**)tmp_ptr,
  .dw_driver_num = 1,
};

void wakeup_device_with_io(void) {
  dw3000_hw_wakeup();
}

decaIrqStatus_t decamutexon(void) {
  return Br101Dw3000Port::instance().mutexOn();
}

void decamutexoff(decaIrqStatus_t s) {
  Br101Dw3000Port::instance().mutexOff(s);
}

void deca_sleep(unsigned int time_ms) {
  delay(time_ms);
}

void deca_usleep(unsigned long time_us) {
  delayMicroseconds(time_us);
}

int readfromspi(uint16_t headerLength, uint8_t* headerBuffer,
                uint16_t readLength, uint8_t* readBuffer) {
  return dw3000_spi_read(headerLength, headerBuffer, readLength, readBuffer);
}

int writetospi(uint16_t headerLength, const uint8_t* headerBuffer,
               uint16_t bodyLength, const uint8_t* bodyBuffer) {
  return dw3000_spi_write(headerLength, headerBuffer, bodyLength, bodyBuffer);
}

int writetospiwithcrc(uint16_t headerLength, const uint8_t* headerBuffer,
                      uint16_t bodyLength, const uint8_t* bodyBuffer,
                      uint8_t crc8) {
  return dw3000_spi_write_crc(headerLength, headerBuffer, bodyLength, bodyBuffer, crc8);
}

}

