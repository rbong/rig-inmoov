## variables expected to be set on the command line and related
# arduino board architecture
BOARD_TYPE=arduino:avr:
# arduino board
BOARD=uno
# arduino upload port
PORT='/dev/ttyACM0'
# arduino settings header to use
SETTINGS=rhand
# arduino action
ARDUINO_DO=--upload
# arduino flags
ARDUINO_FLAGS=$(ARDUINO_DO) --port $(PORT) --board $(BOARD_TYPE)$(BOARD)

## relevant directories
SKETCH_DIR=arduino/sketch
ARDUINO_LIB_DIR=arduino/lib
BUILD_DIR=build

## command wrappers
CP=cp $(CP_FLAGS)
MKDIR=mkdir $(MKDIR_FLAGS)
ARDUINO=arduino $(ARDUINO_FLAGS)

## function macros
BUILD_BASE=$(BUILD_DIR)/$(basename $@)

## pattern targets
# shared arduino files -- updates on any changes to the arduino shared folder
$(BUILD_DIR)/%.ino: $(ARDUINO_LIB_DIR)
	$(MKDIR) -p $(dir $@)
	$(CP) $(ARDUINO_LIB_DIR)/$(notdir $@) $@
# make arduino files -- assumes you mean a sketch
%.ino: $(SKETCH_DIR)/*/%.ino
	$(MKDIR) -p $(BUILD_BASE)
	$(CP) $< $(BUILD_BASE)/$@
	$(CP) $(dir $<)settings/$(SETTINGS).h $(BUILD_BASE)/settings.h
	$(ARDUINO) $(BUILD_BASE)/$@

## implicit targets
documentation:
	doxygen doc/Doxyfile
	cd doc/latex && make pdf
	mv doc/latex/refman.pdf readme.pdf
	ln -s doc/html/index.html readme.html

## implicit arduino targets -- put dependencies here
servo.ino: $(BUILD_DIR)/servo/serial.ino
flex.ino: $(BUILD_DIR)/flex/serial.ino

## cleanup
clean:
	rm -rf doc $(BUILD_DIR)
