# Energy2Shelly_ESP

### Getting started
This is a Shelly Pro 3EM Emulator running on ESP8266 or ESP32 using various input sources for power data.<br>
This can be used for zero feed-in with Hoymiles MS-A2 and Marstek Venus (testers needed!).

Kudos to @sdeigms excellent work at https://github.com/sdeigm/uni-meter which made this port easily possible.<br>
SMA Multicast code is based on https://www.mikrocontroller.net/topic/559607


# Installation
## Option 1: Compile yourself
1) compile and flash for your microcontroller using [PlatformIO](https://platformio.org/)
## Option 2: Flash pre-compiled binary via browser 
1) connect your ESP to your PC using USB and follow the instructions on the [webflasher](https://therealmoeder.github.io/Energy2Shelly_ESP/)

## Configuration
1) Power device and wait for a hotspot named "Energy2Shelly"
2) Connect to that hotspot
3) Enter wifi and configuration data using the captive portal or by opening http://192.168.4.1/

  ### On the captive portal you can currently enter:
  - <code>MQTT</code>
    - Server IP, port and topic; power values on the MQTT topic are expected in JSON format. The are multiple fields to define the available values using a JSON Path-style syntax.
      You can also select between monophase and triphase power data.<br>
      
      examples (monophase profile):
        - Total power JSON path -> <code>ENERGY.Power</code> for <code>{"ENERGY":{"Power":9.99}}</code><br>
        - Phase 1 power JSON path -> "no definition" <br>
        - Phase 2 power JSON path -> "no definition" <br>
        - Phase 3 power JSON path -> "no definition" <br>
        - Energy from grid JSON path -> <code>ENERGY.Consumption</code> for <code>{"ENERGY":{"Consumption"77}}</code><br>
        - Energy to grid JSON path ->  <code>ENERGY.Production</code> for  <code>{"ENERGY":{"Production"33}}</code><br>
              -> Energy2Shelly_ESP responds to <br><code>{"ENERGY":{"Power": 9.99,"Consumption":77,"Production":33}}</code><br>

      examples (triphase profile):
        - Total power JSON path -> <code>ENERGY.Power</code> for <code>{"ENERGY":{"Power":7.3}}</code><br>
        - Phase 1 power JSON path -> <code>ENERGY.Pow1</code> for <code>{"ENERGY":{"Pow1":98}}</code><br>
        - Phase 2 power JSON path -> <code>ENERGY.Pow2</code> for <code>{"ENERGY":{"Pow2":196}}</code><br>
        - Phase 3 power JSON path -> <code>ENERGY.Pow3</code> for <code>{"ENERGY":{"Pow3":294}}</code><br>
        - Energy from grid JSON path -> <code>ENERGY.Consumption</code> for <code>{"ENERGY":{"Consumption"98}}</code>
        - Energy to grid JSON path ->  <code>ENERGY.Production</code> for  <code>{"ENERGY":{"Production"131}}</code><br>
              -> Energy2Shelly_ESP responds to <br><code>{"ENERGY":{"Power":7.3,"Pow1":98,"Pow2":196,"Pow3":294,"Consumption":98,"Production":131}}</code><br>
        
  - <code>SMA</code>
    - SMA Energy Meter or Home Manager UDP multicast data; if you have multiple SMA energy meters you can optionally provide the serial number of the source you want to use in the configuration options
  - <code>SHRDZM</code>
      - SHRDZM smart meter interface (common in Austria) with UDP unicast data; please enable UDP broadcasts to the IP of the ESP and port 9522 within SHRDZM
  - <code>HTTP</code>
      - a generic HTTP input; enter a query URL in the second parameter field which delivers JSON data and define at least the JSON Path for total power. For full details on JSONPath configuration, check the section on MQTT above.<br>
  - <code>SUNSPEC</code>
      - generic SUNSPEC register data polling via Modbus TCP; use server for address of Modbus device (e.g. Kostal Smart energy meter), port for Modbus TCP port (usually 502) and Modbus device ID for the unit ID (71 for KSEM)
  - <code>TIBBERPULSE</code>
      - Tibber-Pulse Adapter (support for Tibber-Pluse for reading from optical SML-Interface from german Power-Meter, Parameter:TCP-Port, user:admin, password:xxx-xxx
      
      - HOWTO Hack Tibber-Pulse adapter: https://github.com/SonnenladenGmbH/tibber_local_lib?tab=readme-ov-file (works in local network! You do **not need** a Tibber-account, Tibber-contract, or connect to the external Tibber-API)

  
  

  ### Here are some sample generic HTTP query paths for common devices:
  - Fronius: <code>http://IP-address/solar_api/v1/GetMeterRealtimeData.cgi?Scope=System</code>
  - Tasmota devices: <code>http://IP-address/cm?cmnd=status%2010</code>
  - ioBroker datapoints: <code>http://IP-address:8082/getBulk/smartmeter.0.1-0:1_8_0__255.value,smartmeter.0.1-0:2_8_0__255.value,smartmeter.0.1-0:16_7_0__255.value/?json</code>
  
  The Shelly ID defaults to the ESP's MAC address, you may change this if you want to substitute an existing uni-meter configuration without reconnecting the battery to a new shelly device.
  
4) Check if your device is visible in the WLAN. <code>http://IP-address</code><br>
5) Check the current power data at <code>http://IP-address/status</code><br>
- [ ] \(Optional) If you want to reset you Wifi-Configuration and/or reconfigure other settings go to <code>http://IP-address/reset</code> and reconnect to the Energy2Shelly hotspot.

# Tested microcontrollers
* ESP32 (ESP32-WROOM-32)
* ESP8266

# You found a bug
First, sorry. This software is not perfect.
1. Open a issue
-With helpful title - use descriptive keywords in the title and body so others can find your bug (avoiding duplicates).
- Which branch, what microcontroller, what setup
- Steps to reproduce the problem, with actual vs. expected results
- If you find a bug in our code, post the files and the lines.
<br>
<br>

# some screenshots from project
  ![wifi](./screenshots/screenshot_01.png)

  ### Settings
  ![settings](./screenshots/screenshot_02.png)
  ![settings](./screenshots/screenshot_03.png)

  ### main page <code>http://IP-address</code>
  ![main](./screenshots/screenshot_04.png)

  ### status page <code>http://IP-address/status</code>
  ![status](./screenshots/screenshot_05.png)

  > [!NOTE]
> Images may vary depending on the version. We always try to be up to date.
