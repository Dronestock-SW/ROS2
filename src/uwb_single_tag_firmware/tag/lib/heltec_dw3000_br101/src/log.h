#pragma once
#include <Arduino.h>

#ifndef LOG_INF
#define LOG_INF(fmt, ...) do { Serial.printf("[DW3000][I] " fmt "\n", ##__VA_ARGS__); } while (0)
#endif
#ifndef LOG_WARN
#define LOG_WARN(fmt, ...) do { Serial.printf("[DW3000][W] " fmt "\n", ##__VA_ARGS__); } while (0)
#endif
#ifndef LOG_ERR
#define LOG_ERR(fmt, ...) do { Serial.printf("[DW3000][E] " fmt "\n", ##__VA_ARGS__); } while (0)
#endif

