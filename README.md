# CTN (Cloud Type Nuvoton) Rev02
- Cloud amazon alexa and Embedded hi-amo.
<br><br>
## Overview
- Espressif's amazon alexa app project based on [esp-va-sdk](https://github.com/espressif/esp-va-sdk) for AVS with Nuvoton I94124's Embedded VR based on Nuvoton I94100 Series BSP_CMSIS_V3.05.003 SDK.
<br><br>
## ASK (Alexa Skills Kit) & Blynk (for IoT service)
<a href="https://drive.google.com/file/d/1EBz9SfTMlUxrCrMfmlVNJ3-JVWvinNNR/view?usp=sharing">![ASK (Alexa Skills Kit) & Blynk (for IoT service)](https://github.com/luvinland/ctn-rev02-esp-va-sdk/assets/26864945/ad5a1ce4-bcd2-4805-8d54-c015cf52caaa)</a>
- <a href="https://developer.amazon.com/en-US/alexa/alexa-skills-kit">ASK (Alexa Skills Kit)</a>
- <a href="https://blynk.io/">Blynk (for IoT service)</a>
<br><br>
## Block diagram
![Block Diagram](https://user-images.githubusercontent.com/26864945/69035559-4bcdc400-0a27-11ea-9897-cf64581c2f14.png)
<br><br>
## Hardware
* WiFi & BT SoC : [Espressif ESP32](https://www.espressif.com/en/products/hardware/esp32/overview)
* Preprocessing MCU with DSP SoC : [Nuvoton I94124](http://www.nuvoton.com/hq/products/application-specific-socs/arm-based-audio/?__locale=en)
<br><br>
## Revision
* **[Rev02: 2020-03-13](https://github.com/luvinland/ctn-rev02-esp-va-sdk)**
  * Apply esp-va-sdk [v1.2](https://github.com/espressif/esp-va-sdk/releases/tag/1.2).
  * Apply BSP_CMSIS_V3.05.003 SDK.
  * Run BLYNK Application.
  * Add Factory reset function.
