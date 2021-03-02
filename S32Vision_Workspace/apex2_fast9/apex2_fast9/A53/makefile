################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include A53_src/subdir.mk
-include A53_gen/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_SIZE += \
apex2_fast9.siz \


# All Target
all:
	$(MAKE) --no-print-directory pre-build
	$(MAKE) --no-print-directory main-build

# Main-build Target
main-build: apex2_fast9.elf secondary-outputs

dependents:
	-cd F:/Spring_2021/Embedded_Autonomous_TA/Current/S32Vision_Workspace/apex2_fast9/apex2_fast9/APU && $(MAKE) all

# Tool invocations
apex2_fast9.elf: $(OBJS) $(USER_OBJS) F:/Spring_2021/Embedded_Autonomous_TA/Current/S32Vision_Workspace/apex2_fast9/apex2_fast9/APU/$(TARGETS)
	@echo 'Building target: $@'
	@echo 'Invoking: Standard S32DS C++ Linker'
	aarch64-linux-gnu-g++ -o "apex2_fast9.elf" "@apex2_fast9.args"  $(USER_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

apex2_fast9.siz: apex2_fast9.elf
	@echo 'Invoking: Standard S32DS Print Size'
	aarch64-linux-gnu-size --format=berkeley apex2_fast9.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES) ./*/*/*.o  ./*/*.o $(SECONDARY_SIZE) ./*/*/*.d  ./*/*.d  apex2_fast9.elf
	-@echo ' '

pre-build:
	-$(MAKE) check_loadh
	-@echo ' '

secondary-outputs: $(SECONDARY_SIZE)

.PHONY: all clean dependents main-build pre-build
F:/Spring_2021/Embedded_Autonomous_TA/Current/S32Vision_Workspace/apex2_fast9/apex2_fast9/APU/$(TARGETS):

-include ../makefile.targets
