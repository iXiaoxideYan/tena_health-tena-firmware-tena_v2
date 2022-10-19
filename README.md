# Load firmware for the device

#### Document history

| Date       | Version  | Author    | Note            |
| ---------- | :------: | --------- | --------------- |
| 2022-10-05 | 1.0.0    |  Y Pham   | Initial version |
| 2022-10-19 | 2.1.0    |  Y Pham   |                 |
#### The steps perform program loading 
1. Connect the device to the PC
2. Open flash_download_tool_3.9.3.exe tool
Download link: https://www.espressif.com/en/support/download/other-tools
3. Select ESP32 DownloadTool
</p>
   <p align="center">
 
  <img src="/doc/Document/tool.png" width=50% alt="" />
</p>
4. Configure for application

 SPI speed: 20MHz
 SPI mode: DIO
 Flash size: 32Mbit
 COM: COM the PC connect the device
 Baud rate: 1152000
</p>
   <p align="center">
 
  <img src="/doc/Document/Config.png" width=50% alt="" />
</p>

**Note:** Select COM device connection by opening Device Manager

</p>
   <p align="center">
 
  <img src="/doc/Document/port.png" width=50% alt="" />
</p>

5. Add file bootloader.bin, partition-table.bin and tena.bin with corresponding address 0x100, 0x800, 0x10000.
</p>
   <p align="center">
 
  <img src="/doc/Document/addfile.png" width=50% alt="" />
</p>
6. Select Start
</p>
   <p align="center">
 
  <img src="/doc/Document/start.png" width=50% alt="" />
</p>

The interface displayed when run is successful
</p>
   <p align="center">
 
  <img src="/doc/Document/run.png" width=50% alt="" />
</p>
#### Connect bluetooth
1. Download Bluetooth Classic app
</p>
   <p align="center">
 
  <img src="/doc/Document/bluetooth.png" width=50% alt="" />
</p>
2. Pair with device "Bluetooth_IMU_D"
</p>
   <p align="center">
 
  <img src="/doc/Document/connect.png" width=50% alt="" />
</p>
3. Connect device
</p>
   <p align="center">
 
  <img src="/doc/Document/device.png" width=50% alt="" />
</p>
4. See data of 6-axis

   </p>
   <p align="center">
 
  <img src="/doc/Document/result.png" width=50% alt="" />
</p># tena_health-tena-firmware-tena_v2
