# **SensiBLE-2.0**

## 1 Build
For build [STM32CubeIDE Version: 1.14.0(https://www.st.com/en/development-tools/stm32cubeide.html)] is used.

## 2 Bootloader
SensiBLE-2.0 uses bootloader [Bootloader.bin](binary/Bootloader.bin)

## 3 Firmware
- [SensiBLE-2.0-V2.3.1-OTA-.bin](binary/SensiBLE-2.0-V2.3.1-OTA.bin) - firmware for downloading over BLE with "[ST BLE Sensor](https://www.st.com/en/embedded-software/stblesensor.html)".
- [SensiBLE-2.0-V2.3.1-OTA-BT.bin](binary/SensiBLE-2.0-V2.3.1-OTA-BT.bin) - firmware for downloading with "[BlueNRG-1 ST-Link Utility](https://www.st.com/en/embedded-software/stsw-bnrg1stlink.html)"

## 4 Firmware Version 2.3.1
- Add Color Ambient Light BLE characteristic
- Add calculating UV index, Color Ambient Light
- Update Acc EVENT logic
- Corrected the formula for calculating LUX

## 5 License

**COPYRIGHT(c) 2024 SensiEDGE **

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the name of SensiEDGE LTD nor the names of its contributors may
     be used to endorse or promote products derived from this software
     without specific prior written permission.
     
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.