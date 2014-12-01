#ifndef _DHT22_H_
#define _DHT22_H_

#include <inttypes.h>

#define DHT22_ERROR_VALUE -995

typedef enum
{
    DHT_ERROR_NONE = 0,
    DHT_BUS_HUNG,
    DHT_ERROR_NOT_PRESENT,
    DHT_ERROR_ACK_TOO_LONG,
    DHT_ERROR_SYNC_TIMEOUT,
    DHT_ERROR_DATA_TIMEOUT,
    DHT_ERROR_CHECKSUM,
    DHT_ERROR_TOOQUICK
} DHT22_ERROR_t;

class DHT22
{
    private:
        uint8_t           _bitmask;
        volatile uint8_t *_baseReg;
        int16_t           _lastHumidity;
        int16_t           _lastTemperature;
    public:
        DHT22(uint8_t const pin);
        DHT22_ERROR_t readData();
        inline float getTemperatureF() { return float(_lastTemperature)/10 * 9 / 5 + 32; }
        inline float getTemperatureC() { return float(_lastTemperature)/10; }
        inline float getHumidity() { return float(_lastHumidity)/10; }
};



#endif /*_DHT22_H_*/
