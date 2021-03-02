################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Graphs sources folder name variable
GRAPHS_SRC_DIR := APU_gen/src

# Read elf script variables
READ_ELF := tclsh  ../Project_Settings/Scripts/read_elf.tcl
READ_ELF_OPTS := -pPMh=32 -mDMb=8 -mVMb=1024
READ_ELF_OUTPUT_DIR := ../A53_gen/inc
TCL_SCRIPT := ../Project_Settings/Scripts/gen_apu_load.tcl

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include APU_gen/src/subdir.mk
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

# Rules for graphs

GRAPHS := $(patsubst %_nativekernel.hpp,%, $(notdir $(wildcard ../$(GRAPHS_SRC_DIR)/*_nativekernel.hpp)))
GRAPHS_OBJS := $(foreach graph,$(GRAPHS), $(addprefix ./$(GRAPHS_SRC_DIR)/,$(graph).o))
GRAPHS_OBJS += $(foreach graph,$(GRAPHS), $(addprefix ./$(GRAPHS_SRC_DIR)/,$(graph)__MKDBstub.o))
NON_GRAPHS_OBJS := $(filter-out $(GRAPHS_OBJS),$(OBJS))
TARGETS := $(foreach graph,$(GRAPHS),$(graph).elf)
LOADHS := $(foreach graph,$(GRAPHS),$(READ_ELF_OUTPUT_DIR)/$(graph)_APU_LOAD.h)
TARGETS += $(LOADHS)

create-gen-dir:
	@mkdir -p $(READ_ELF_OUTPUT_DIR)

define apu_rules
$(1).elf: ./$(GRAPHS_SRC_DIR)/$(1).o ./$(GRAPHS_SRC_DIR)/$(1)__MKDBstub.o $(NON_GRAPHS_OBJS) F:/Spring_2021/Embedded_Autonomous_TA/Current/S32Vision_Workspace/apex2_fast9/apex2_fast9/Project_Settings/Linker_Files/S32V_APU.lcf | create-gen-dir
	@echo 'Building target: $(1).elf'
	@echo 'Invoking: APU C++ Linker'
	ld-new -L"F:\Spring_2021\Embedded_Autonomous_TA\Installations\S32DS_Vision_v2018.R1\eclipse\../S32DS/APUC/lib-apex" -L"F:\Spring_2021\Embedded_Autonomous_TA\Installations\S32DS_Vision_v2018.R1\eclipse\../S32DS/s32v234_sdk/libs/apex/acf_target/build-apu-nxp-sa-d" -L"F:\Spring_2021\Embedded_Autonomous_TA\Installations\S32DS_Vision_v2018.R1\eclipse\../S32DS/s32v234_sdk/libs/apex/common/build-apu-nxp-sa-d" -L"F:\Spring_2021\Embedded_Autonomous_TA\Installations\S32DS_Vision_v2018.R1\eclipse\../S32DS/s32v234_sdk/kernels/apu/sample_filtering_kernels/build-apu-nxp-sa-d" -L"F:\Spring_2021\Embedded_Autonomous_TA\Installations\S32DS_Vision_v2018.R1\eclipse\../S32DS/s32v234_sdk/kernels/apu/sample_feature_detection_kernels/build-apu-nxp-sa-d" -L"F:\Spring_2021\Embedded_Autonomous_TA\Installations\S32DS_Vision_v2018.R1\eclipse\../S32DS/s32v234_sdk/kernels/apu/sample_display_kernels/build-apu-nxp-sa-d" -L"F:\Spring_2021\Embedded_Autonomous_TA\Installations\S32DS_Vision_v2018.R1\eclipse\../S32DS/s32v234_sdk/kernels/apu/sample_conversion_kernels/build-apu-nxp-sa-d" -L"F:\Spring_2021\Embedded_Autonomous_TA\Installations\S32DS_Vision_v2018.R1\eclipse\../S32DS/s32v234_sdk/kernels/apu/sample_morphology_kernels/build-apu-nxp-sa-d" -T"F:/Spring_2021/Embedded_Autonomous_TA/Current/S32Vision_Workspace/apex2_fast9/apex2_fast9/Project_Settings/Linker_Files/S32V_APU.lcf" --gc-sections -o $(1).elf ./$(GRAPHS_SRC_DIR)/$(1).o ./$(GRAPHS_SRC_DIR)/$(1)__MKDBstub.o $(NON_GRAPHS_OBJS) $(LIBS) -Map=$(1).map
	@echo 'Finished building target: $(1).elf'

$(READ_ELF_OUTPUT_DIR)/$(graph)_APU_LOAD.h: $(1).elf
	@echo 'Generating: $(1)_APU_LOAD.h'
	@echo ' '
	$(READ_ELF) $(READ_ELF_OPTS) $(1).elf -eGb -t $(TCL_SCRIPT) -Tload_name=$(1) > $(READ_ELF_OUTPUT_DIR)/$(1)_APU_LOAD.h 
	@echo 'Finished generating $(1)_APU_LOAD.h'
	@echo ' '
endef

$(foreach graph,$(GRAPHS),$(eval $(call apu_rules,$(graph))))

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(TARGETS)

# Tool invocations
# Other Targets
clean:
	-$(RM) $(EXECUTABLES) ./*/*/*.o  ./*/*/*.d  $(TARGETS)
	-@echo ' '

.PHONY: all clean dependents

-include ../makefile.targets
