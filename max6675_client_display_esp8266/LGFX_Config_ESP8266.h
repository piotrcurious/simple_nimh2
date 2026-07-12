#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;

public:
  LGFX(void)
  {
    auto cfg = _bus_instance.config();

    // ESP8266 Standard Hardware SPI (SPI0 / SPI1)
    cfg.spi_host = 1; // SPI-1 / HSPI
    cfg.spi_mode = 0;
    cfg.freq_write = 27000000; // Keep SPI speed stable and conservative for ESP8266
    cfg.freq_read  = 16000000;
    cfg.pin_sclk = 14; // HSPI SCK  (D5 on NodeMCU)
    cfg.pin_mosi = 13; // HSPI MOSI (D7 on NodeMCU)
    cfg.pin_miso = 12; // HSPI MISO (D6 on NodeMCU)
    cfg.pin_dc   = 0;  // D3 on NodeMCU

    _bus_instance.config(cfg);
    _panel_instance.setBus(&_bus_instance);

    auto panel_cfg = _panel_instance.config();

    panel_cfg.pin_cs           = 15; // HSPI CS   (D8 on NodeMCU)
    panel_cfg.pin_rst          = 2;  // D4 on NodeMCU
    panel_cfg.pin_busy         = -1;
    panel_cfg.panel_width      = 240;
    panel_cfg.panel_height     = 320;
    panel_cfg.offset_x         = 0;
    panel_cfg.offset_y         = 0;
    panel_cfg.offset_rotation  = 1; // Rotation 1 for landscape 320x240
    panel_cfg.dummy_read_pixel = 8;
    panel_cfg.dummy_read_bits  = 1;
    panel_cfg.readable         = true;
    panel_cfg.invert           = true;
    panel_cfg.rgb_order        = false;
    panel_cfg.dlen_16bit       = false;
    panel_cfg.bus_shared       = true;

    _panel_instance.config(panel_cfg);

    setPanel(&_panel_instance);
  }
};

extern LGFX tft;
