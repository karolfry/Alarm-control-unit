################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
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
LSS += \
Pilot_at2313.lss \

FLASH_IMAGE += \
Pilot_at2313.hex \

EEPROM_IMAGE += \
Pilot_at2313.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: Pilot_at2313.elf secondary-outputs

# Tool invocations
Pilot_at2313.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,Pilot_at2313.map -mmcu=attiny2313 -o "Pilot_at2313.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Pilot_at2313.lss: Pilot_at2313.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S Pilot_at2313.elf  >"Pilot_at2313.lss"
	@echo 'Finished building: $@'
	@echo ' '

Pilot_at2313.hex: Pilot_at2313.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex Pilot_at2313.elf  "Pilot_at2313.hex"
	@echo 'Finished building: $@'
	@echo ' '

Pilot_at2313.eep: Pilot_at2313.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex Pilot_at2313.elf  "Pilot_at2313.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Pilot_at2313.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=attiny2313 Pilot_at2313.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) Pilot_at2313.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents

-include ../makefile.targets
