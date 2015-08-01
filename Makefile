## variables expected to be set on the command line and related
# arduino board architecture
BOARD_TYPE=arduino:avr:
# arduino board
BOARD=uno
# arduino upload port
PORT='/dev/ttyACM0'
# arduino settings header to use
SETTINGS=servo_rhand
# arduino action
ARDUINO_DO=--upload
# arduino flags
ARDUINO_FLAGS=$(ARDUINO_DO) --port $(PORT) --board $(BOARD_TYPE)$(BOARD)

## relevant directories
SKETCH_DIR=arduino/sketch
ARDUINO_LIB_DIR=arduino/lib
BUILD_DIR=build

## command wrappers
ARDUINO=arduino $(ARDUINO_FLAGS)
DOXY=doxygen $(DOC_OPTIONS)

## files left by the build system
ARDUINO_BUILD_FILES=$(BUILD_DIR)
DOC_BUILD_FILES=doc/html doc/latex
BUILD_FILES=$(ARDUINO_BUILD_FILES) $(DOC_BUILD_FILES)

## function macros
BUILD_BASE=$(BUILD_DIR)/$(basename $@)

## pattern targets
# shared arduino files -- updates on any changes to the arduino shared folder
$(BUILD_DIR)/%.ino: $(ARDUINO_LIB_DIR)
	mkdir -p $(dir $@)
	cp $(ARDUINO_LIB_DIR)/$(notdir $@) $@
# make arduino files -- assumes you mean a sketch
%.ino: $(SKETCH_DIR)/*/%.ino
	mkdir -p $(BUILD_BASE)
	cp $< $(BUILD_BASE)/$@
	cp $(dir $<)settings/$(SETTINGS).h $(BUILD_BASE)/settings.h
	$(ARDUINO) $(BUILD_BASE)/$@

## implicit targets
documentation:
	$(DOXY) doc/Doxyfile
	cd doc/latex && make pdf

## implicit arduino targets -- put dependencies here
servo.ino: $(BUILD_DIR)/servo/serial.ino
flex.ino: $(BUILD_DIR)/flex/serial.ino

## cleanup
clean:
	rm -rf $(BUILD_FILES)
