# Simple Makefile for PlatformIO + ESP32 auto-monitor

# PORT := /dev/cu.usbserial-57450005861 #TTGO
PORT := /dev/cu.usbserial-54F70184691 #LilyGO
BAUD := 115200

# Default target
Access_Control_System:
	@echo "🚀 Uploading sketch to ESP32..."
	pio run -t clean
	pio run -e Access_Control_System -t upload
	@echo "🔍 Opening serial monitor..."
	sleep 1
	pio device monitor -b $(BAUD) -p $(PORT)
