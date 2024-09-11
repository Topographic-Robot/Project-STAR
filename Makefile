# Must run/export idf.py from ESP's installer
# I made an alias saying `alias get_idf=". /Users/bsikar/Documents/git/esp-idf/export.sh"`
# This alias is in my ~/.zshrc
#
# Before running `make build` I have to run (only the first time):
# get_idf
#
# After I then have idf.py and these commands work

build: 
	idf.py build 

flash:
	idf.py flash

clean:
	idf.py clean
	rmdir build 2>/dev/null || true

fullclean:
	idf.py fullclean
	rmdir build 2>/dev/null || true

.PHONY: build
