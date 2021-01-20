################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include Library/StdDriver/src/subdir.mk
-include CMSIS/GCC/subdir.mk
-include CMSIS/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
Nuvoton_Single_Phase_Active_Rectifier_Controller.hex \

SECONDARY_SIZE += \
Nuvoton_Single_Phase_Active_Rectifier_Controller.siz \


# All Target
all: Nuvoton_Single_Phase_Active_Rectifier_Controller.elf secondary-outputs

# Tool invocations
Nuvoton_Single_Phase_Active_Rectifier_Controller.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM GNU C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -T gcc_arm.ld -Xlinker --gc-sections -L"../CMSIS/GCC" -Wl,-Map,"Nuvoton_Single_Phase_Active_Rectifier_Controller.map" --specs=nano.specs -o "Nuvoton_Single_Phase_Active_Rectifier_Controller.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Nuvoton_Single_Phase_Active_Rectifier_Controller.hex: Nuvoton_Single_Phase_Active_Rectifier_Controller.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "Nuvoton_Single_Phase_Active_Rectifier_Controller.elf"  "Nuvoton_Single_Phase_Active_Rectifier_Controller.hex"
	@echo 'Finished building: $@'
	@echo ' '

Nuvoton_Single_Phase_Active_Rectifier_Controller.siz: Nuvoton_Single_Phase_Active_Rectifier_Controller.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "Nuvoton_Single_Phase_Active_Rectifier_Controller.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) Nuvoton_Single_Phase_Active_Rectifier_Controller.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
