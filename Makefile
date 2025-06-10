BOARD = esp32:esp32:adafruit_feather_esp32s3
PORT := $(wildcard /dev/serial/by-id/usb-Silicon_Labs_CP21* /dev/serial/by-id/usb-1a86_USB_Single_Serial_* /dev/cu.usbserial-* /dev/serial/by-id/usb-Adafruit_Feather_ESP32-S3* /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_34:85:18:82:24:24-if00)
PORT := $(strip $(PORT))
# SDL_GAMECONTROLLERCONFIG="030068cd09120000544f000011010000,OpenTX FrSky Taranis Joystick,a:b0,b:b1,x:b3,y:b4,back:b10,guide:b12,start:b11,leftstick:b13,rightstick:b14,leftshoulder:b6,rightshoulder:b7,leftx:a0,lefty:a1,rightx:a2,righty:a3,lefttrigger:a4,righttrigger:a5,crc:cd68,platform:Linux" ./QGroundControl.AppImage
build: .dependencies
	arduino-cli compile --fqbn $(BOARD) flix

upload: build
	arduino-cli upload --fqbn $(BOARD) -p "$(PORT)" flix

monitor:
	arduino-cli monitor -p "$(PORT)" -c baudrate=115200

dependencies .dependencies:
	arduino-cli core update-index --config-file arduino-cli.yaml
	arduino-cli core install esp32:esp32@3.1.0 --config-file arduino-cli.yaml
	arduino-cli lib update-index
	arduino-cli lib install "FlixPeriph"
	arduino-cli lib install "MAVLink"@2.0.12
	touch .dependencies

gazebo/build cmake: gazebo/CMakeLists.txt
	mkdir -p gazebo/build
	cd gazebo/build && cmake ..

build_simulator: .dependencies gazebo/build
	make -C gazebo/build

simulator: build_simulator
	GAZEBO_MODEL_PATH=$$GAZEBO_MODEL_PATH:${CURDIR}/gazebo/models \
	GAZEBO_PLUGIN_PATH=$$GAZEBO_PLUGIN_PATH:${CURDIR}/gazebo/build \
	gazebo --verbose ${CURDIR}/gazebo/flix.world

log:
	PORT=$(PORT) tools/grab_log.py

plot:
	plotjuggler -d $(shell ls -t tools/log/*.csv | head -n1)

clean:
	rm -rf gazebo/build flix/build flix/cache .dependencies

.PHONY: build upload monitor dependencies cmake build_simulator simulator log clean
