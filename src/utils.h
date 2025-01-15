#pragma once
#include "Arduino.h"

enum class RGBLedColor
{
    Off,
    Red,
    Blue,
    Green,
    White,
};

void set_led_color(RGBLedColor color)
{
#ifdef NANO_33_BLE
    switch (color)
    {
    case RGBLedColor::Off:
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_BLUE, HIGH);
        break;

    case RGBLedColor::Red:
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_BLUE, HIGH);
        break;

    case RGBLedColor::Blue:
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_BLUE, HIGH);
        break;

    case RGBLedColor::Green:
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_BLUE, LOW);
        break;

    case RGBLedColor::White:
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_BLUE, LOW);
        break;
    }
#endif
}

enum class LedState
{
    NotConnected,
    Scanning,
    Connected,
};

void set_led_state(LedState state)
{
#ifdef NANO_33_BLE
    switch (state)
    {
    case LedState::NotConnected:
        set_led_color(RGBLedColor::Blue);
        break;
    case LedState::Scanning:
        set_led_color(RGBLedColor::Blue);
        break;
    case LedState::Connected:
        set_led_color(RGBLedColor::Green);
        break;
    }
#endif
#ifdef NANO_33_IOT
    switch (state)
    {
    case LedState::NotConnected:
        digitalWrite(LED_BUILTIN, LOW);
        break;
    case LedState::Scanning:
        digitalWrite(LED_BUILTIN, LOW);
        break;
    case LedState::Connected:
        digitalWrite(LED_BUILTIN, HIGH);
        break;
    }
#endif
}