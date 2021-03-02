################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../A53_gen/src/acf_host.cpp \
../A53_gen/src/apu_fast9color_process_controller.cpp 

OBJS += \
./A53_gen/src/acf_host.o \
./A53_gen/src/apu_fast9color_process_controller.o 

CPP_DEPS += \
./A53_gen/src/acf_host.d \
./A53_gen/src/apu_fast9color_process_controller.d 


# Each subdirectory must supply rules for building sources it contributes
A53_gen/src/%.o: ../A53_gen/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C++ Compiler'
	aarch64-linux-gnu-g++ "@A53_gen/src/acf_host.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


