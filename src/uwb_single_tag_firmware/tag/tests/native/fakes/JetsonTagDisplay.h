#pragma once
#include <stdint.h>
namespace JetsonTagDisplay {
void begin();
void render(bool ready, uint8_t mask, bool synchronized);
}
