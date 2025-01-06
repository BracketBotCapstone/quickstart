# To install the dependencies and build the config and messages, use:
# CONFIG=bot_quickstart CONFIG_MSGS=bot_quickstart_msgs make

# To only build the config and messages, use:
# CONFIG=bot_quickstart CONFIG_MSGS=bot_quickstart_msgs make build

.PHONY: all debug setup clean build

CONFIG ?= bot_quickstart
CONFIG_MSGS ?= bot_quickstart_msgs
CFG_PATH = $(shell pwd)/config/$(CONFIG).toml

all: setup

build: 
	python3 config/parse_toml_config.py config/$(CONFIG).toml
	python3 config/generate_mqtt_messages.py config/$(CONFIG_MSGS).toml

debug: setup

clean:
	# nothing to clean

setup:
	pip3 install -e .
	python3 config/parse_toml_config.py config/$(CONFIG).toml
	python3 config/generate_mqtt_messages.py config/$(CONFIG_MSGS).toml

install:
	pip3 install -e .
