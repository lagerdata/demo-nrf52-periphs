# Lager Demo Project for nRF52840 Development Kits
## Project Description
This is a demo project for the nRF52840 MCU that helps users to learn the many different features of the Lager CLI tool.  
With this demo, users can:  
1. Learn how to build a project inside a docker container
2. Remotely flash and run an application on a DUT (in this case the nRF52840-DK board)
3. See how unit testing can be incorporated into their work flow via the Unity Testing Framework
4. Set up a continuous integration pipeline

## Pre-requisite
Before experimenting with this project there are a couple of pre-requisites  
1. Install docker client for your operating system (https://www.docker.com/products/docker-desktop)
2. Install Lager's CLI tool:  
`pip3 install lager-cli`
  

## Build Instructions
This repository has one submodule (Unity unit testing framework) so when cloning make sure to include the `--recurse-submodules` flag.  
If the repo has already been cloned and the `--recurse-submodules` flag was not included you can run `git submodule update --init --recursive`  
A Lager environment with build instructions is included in this repository.  
To build the project simply run: `lager exec build`  
This will build all the targets in this project.  
To see the different build options run:  
`lager devenv commands`  
If you would like to create a development environment from scratch do the following:  
`lager devenv delete` This deletes the current development environment  
`lager devenv create` This creates a new development environment.  
This will instruct users to choose a development environment image (e.g. cortex-m, stm32, ti, etc), where to mount their project in the docker container, and what shell type to use. For this project the default settings are OK.  
*Note: Search "lagerdata" on hub.docker.com to view other development environments supported by Lager*  
  
To create a new custom command run:  
`lager exec --command "user defined command" --save-as user-defined-shortcut `  

For example, to build this project using CMake + Ninja a user can define the following:  
`lager exec --command "mkdir -p _build;cd _build; cmake .. -G Ninja;cmake --build ." --save-as build`  
Moving forward a user could then run `lager exec build`  
Similarly to create a "clean" command a user could do:  
`lager exec --command "cd _build;ninja -t clean" --save-as clean`  
and run it as `lager exec clean`  
Or, if your project has a Makefile you could do:  
`lager exec --command "make" --save-as make`  
and  
`lager exec --command "make clean" --save-as make-clean`  

## Flashing The Board
#### Connect To board
To flash a board first connect the Lager Gateway to the nrf52840 development board.  
There are two options to do this:  
1. USB2.0 - This will allow users to use the J-Link debug probe built into the nRF52 development board
2. Cortex-Debug 20 Pin header on Gateway - This allows users to use the built-in FTDI debug probe on the Gateway (Note: Board will still need to be powered)  
  
Then run:  
`lager connect --device nrf52 --interface jlink --transport swd --speed 4000`  
or if using built-in debug probe  
`lager connect --device nrf52 --interfact ftdi --transport swd --speed 4000`  
  
#### Flash Image
To flash the board with the project application run the following:  
`lager flash --hexfile _build/app/app.hex`  

## Unit Tests
To run an example unit-test for this project run the following:  
`lager testrun --serial-device /dev/ttyACM0 --hexfile _build/unit-tests/test-suites/test-example/test-example.hex  --test-runner unity`  
The results of the individual tests will be streamed back to the terminal.  
*Note: To view available serial devices that can be used run `lager gateway serial-ports`  
/dev/ttyS0 is the built-in gateway serial port that is accessible via a 20 pin header on the Gateway.  
Other potential usable serial ports is determined on the types of USB devices plugged into the Gateway.*  
  
## CI Setup
For more information about setting up a CI pipeline with lager checkout: https://docs.lagerdata.com/ci/index.html

