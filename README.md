# Pico W Bluetooth Mouse Jiggler

## Arduino IDE

### Settings

* File > Preferences > Additional Boards Manager URLs: https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
* Tools > Board: "Raspberry Pi Pico W"
* Tools > WiFi Region: "Japan"
* Tools > USB Stack: "Adafruit TinyUSB"
* Tools > IP/Bluetooth Stack: "IPv4 + Bluetooth"

### Compile and Upload

* Sketch > Upload

## arduino-cli

### Settings

* arduino-cli config init
* arduino-cli config add board_manager.additional_urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
* arduino-cli core update-index
* arduino-cli core install rp2040:rp2040
* arduino-cli lib update-index
* arduino-cli lib install "Adafruit TinyUSB Library"

### Compile

* (option) arduino-cli board details -b rp2040:rp2040:rpipicow --board-options=usbstack=wificountry=japan,tinyusb,ipbtstack=ipv4btcble
* arduino-cli compile -b rp2040:rp2040:rpipicow --board-options=wificountry=japan,usbstack=tinyusb,ipbtstack=ipv4btcble BLEMouseJiggler

### Upload

* arduino-cli upload -p /dev/ttyACM0 -b rp2040:rp2040:rpipicow BLEMouseJiggler
