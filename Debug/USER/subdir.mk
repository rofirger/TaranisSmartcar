################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USER/Balance.c \
../USER/Cpu0_Main.c \
../USER/Cpu1_Main.c \
../USER/Host_Computer.c \
../USER/TFT_GUI.c \
../USER/control.c \
../USER/debug.c \
../USER/fuzzy_pid.c \
../USER/img_process.c \
../USER/isr.c 

OBJS += \
./USER/Balance.o \
./USER/Cpu0_Main.o \
./USER/Cpu1_Main.o \
./USER/Host_Computer.o \
./USER/TFT_GUI.o \
./USER/control.o \
./USER/debug.o \
./USER/fuzzy_pid.o \
./USER/img_process.o \
./USER/isr.o 

COMPILED_SRCS += \
./USER/Balance.src \
./USER/Cpu0_Main.src \
./USER/Cpu1_Main.src \
./USER/Host_Computer.src \
./USER/TFT_GUI.src \
./USER/control.src \
./USER/debug.src \
./USER/fuzzy_pid.src \
./USER/img_process.src \
./USER/isr.src 

C_DEPS += \
./USER/Balance.d \
./USER/Cpu0_Main.d \
./USER/Cpu1_Main.d \
./USER/Host_Computer.d \
./USER/TFT_GUI.d \
./USER/control.d \
./USER/debug.d \
./USER/fuzzy_pid.d \
./USER/img_process.d \
./USER/isr.d 


# Each subdirectory must supply rules for building sources it contributes
USER/%.src: ../USER/%.c USER/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fD:/projects/TC264/taranis/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

USER/%.o: ./USER/%.src USER/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


