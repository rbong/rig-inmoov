BOARD=arduino:avr:diecimila
PORT='/dev/ttyUSB0'

documentation: docclean
	doxygen doc/Doxyfile
	cd doc/latex && make pdf
	mv doc/latex/refman.pdf readme.pdf
	ln -s doc/html/index.html readme.html

rhand:
	cp arduino/settings/rhand.h arduino/src/servo/settings.h
	arduino --upload --port $(PORT) --board $(BOARD) arduino/src/servo/servo.ino

rhand_verify:
	cp arduino/settings/rhand.h arduino/src/servo/settings.h
	arduino --verify arduino/src/servo/servo.ino

clean: docclean arduinoclean

arduinoclean:
	rm -f arduino/src/servo/settings.h &> /dev/null

docclean:
	rm -rf readme.pdf readme.html doc/html doc/latex &> /dev/null
