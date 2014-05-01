################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../RTC/DS1307RTC.cpp \
../RTC/DateStrings.cpp \
../RTC/Time.cpp \
../RTC/Wire.cpp 

C_SRCS += \
../RTC/twi.c 

OBJS += \
./RTC/DS1307RTC.o \
./RTC/DateStrings.o \
./RTC/Time.o \
./RTC/Wire.o \
./RTC/twi.o 

C_DEPS += \
./RTC/twi.d 

CPP_DEPS += \
./RTC/DS1307RTC.d \
./RTC/DateStrings.d \
./RTC/Time.d \
./RTC/Wire.d 


# Each subdirectory must supply rules for building sources it contributes
RTC/%.o: ../RTC/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"C:\Users\thiago\Dropbox\Circuits\sourceCodes\arduino_core\src" -Wall -g2 -gstabs -Os -ffunction-sections -fdata-sections -ffunction-sections -fdata-sections -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

RTC/%.o: ../RTC/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"C:\Users\thiago\Dropbox\Circuits\sourceCodes\arduino_core\src" -Wall -g2 -gstabs -Os -ffunction-sections -fdata-sections -ffunction-sections -fdata-sections -std=gnu99 -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


