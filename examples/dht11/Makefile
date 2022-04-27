#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

HOMEKIT_PATH ?= $(abspath $(shell pwd)/../..)
COMMON_COMPONENT_PATH ?= $(abspath $(shell pwd)/../common)

PROJECT_NAME := data_tlv8
EXTRA_COMPONENT_DIRS += $(HOMEKIT_PATH)/components/
EXTRA_COMPONENT_DIRS += $(HOMEKIT_PATH)/components/homekit
EXTRA_COMPONENT_DIRS += $(COMMON_COMPONENT_PATH)
EXTRA_COMPONENT_DIRS += $(HOME)/esp8266/esp-idf-lib/components

EXCLUDE_COMPONENTS := max7219 mcp23x17 led_strip

include $(IDF_PATH)/make/project.mk

