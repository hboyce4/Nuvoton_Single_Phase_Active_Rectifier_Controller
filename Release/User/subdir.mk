################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/PLL.c \
../User/UART_over_DMA.c \
../User/UI.c \
../User/analog.c \
../User/init.c \
../User/interrupt.c \
../User/inverter_control.c \
../User/main.c 

OBJS += \
./User/PLL.o \
./User/UART_over_DMA.o \
./User/UI.o \
./User/analog.o \
./User/init.o \
./User/interrupt.o \
./User/inverter_control.o \
./User/main.o 

C_DEPS += \
./User/PLL.d \
./User/UART_over_DMA.d \
./User/UI.d \
./User/analog.d \
./User/init.d \
./User/interrupt.d \
./User/inverter_control.d \
./User/main.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -I"../Library/CMSIS/Include" -I"../Library/Device/Nuvoton/M480/Include" -I"../Library/StdDriver/inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


