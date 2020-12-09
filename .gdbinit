#
#
########################################
# This connects to a GDB Server listening
# for commands on localhost at tcp port 3333
set remotetimeout 5
target remote localhost:3333
########################################


########################################

########################################





########################################
# Reset the chip to get to a known state.
monitor reset halt



########################################


# Load the program executable called "image.elf"

file ./_build/app/app
# file ./_build/unit-tests/test-suites/test-example/test-example
# file ./_build/unit-tests/test-suites/test-ledctrl/test-ledctrl


set breakpoint pending on

load
break main
break  main.c:49
