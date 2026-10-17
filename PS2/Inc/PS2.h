#ifndef C8T6_PS2_H
#define C8T6_PS2_H

#include "main.h"
#include "spi.h"
#include <stdint.h>

typedef struct
{
    uint8_t mode;

    uint8_t buttonLow;
    uint8_t buttonHigh;

    uint8_t rightX;
    uint8_t rightY;

    uint8_t leftX;
    uint8_t leftY;

} PS2RawData;

#ifdef __cplusplus

class PS2
{
public:
    PS2(SPI_HandleTypeDef* hspi,
        GPIO_TypeDef* csPort,
        uint16_t csPin);

    void init();

    bool read(PS2RawData& data);

    bool isOnline() const
    {
        return online_;
    }

private:
    uint8_t transfer(uint8_t tx);

    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* csPort_;
    uint16_t csPin_;

    bool online_ = false;
};

#endif


#ifdef __cplusplus
extern "C" {
#endif

    void PS2_Init(void);

    uint8_t PS2_ReadRaw(PS2RawData* data);

#ifdef __cplusplus
}
#endif


#endif // C8T6_PS2_H