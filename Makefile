BOARD=arduino:avr:uno
PORT='/dev/ttyACM0'

documentation: docclean
	doxygen doc/Doxyfile
	cd doc/latex && make pdf
	mv doc/latex/refman.pdf readme.pdf
	ln -s doc/html/index.html readme.html

rhand_servo:
	cp arduino/servo/settings/rhand.h arduino/servo/settings.h
	arduino --upload --port $(PORT) --board $(BOARD) arduino/servo/servo.ino

rhand_servo_calibrate:
	cp arduino/servo/settings/rhand_calibrate.h arduino/servo/settings.h
	arduino --upload --port $(PORT) --board $(BOARD) arduino/servo/servo.ino

rhand_servo_verify:
	cp arduino/servo/settings/rhand.h arduino/servo/settings.h
	arduino --verify arduino/servo/servo.ino

rhand_flex:
	cp arduino/flex/settings/rhand.h arduino/flex/flex/settings.h
	arduino --upload --port $(PORT) --board $(BOARD) arduino/flex/flex/flex.ino

rhand_flex_verify:
	cp arduino/flex/settings/rhand.h arduino/flex/settings.h
	arduino --verify arduino/flex/flex.ino

clean: docclean arduinoclean

arduinoclean:
	rm -f arduino/*/settings.h &> /dev/null

docclean:
	rm -rf readme.pdf readme.html doc/html doc/latex &> /dev/null
