# Simple Makefile for PlatformIO + ESP32 auto-monitor
# Run transmitter first and then receiver

TRANSMITTER_PORT := /dev/cu.usbserial-54F70184691 #LilyGO
RECEIVER_PORT:= /dev/cu.usbserial-57450005861 #TTGO
BAUD := 115200

Transmitter_Node: t
Receiver_Node: r

# Default target
t:
	@echo "🚀 Uploading sketch to ESP32..."
	# pio run -t clean
	pio run -e Transmitter_Node -t upload
	@echo "🔍 Opening serial monitor..."
	sleep 1
	pio device monitor -b $(BAUD) -p $(TRANSMITTER_PORT)

r:
	@echo "🚀 Uploading sketch to ESP32..."
	# pio run -t clean
	pio run -e Receiver_Node -t upload
	@echo "🔍 Opening serial monitor..."
	sleep 1
	pio device monitor -b $(BAUD) -p $(RECEIVER_PORT)