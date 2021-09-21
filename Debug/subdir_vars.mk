################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../28335_RAM_lnk.cmd \
C:/ti/controlSUITE/device_support/f2833x/v140/DSP2833x_headers/cmd/DSP2833x_Headers_nonBIOS.cmd 

ASM_SRCS += \
../DSP2833x_ADC_cal.asm \
C:/ti/controlSUITE/device_support/f2833x/v140/DSP2833x_common/source/DSP2833x_CodeStartBranch.asm \
C:/ti/controlSUITE/device_support/f2833x/v140/DSP2833x_common/source/DSP2833x_usDelay.asm \
../FIR_f32.asm 

C_SRCS += \
C:/ti/controlSUITE/device_support/f2833x/v140/DSP2833x_common/source/DSP2833x_DefaultIsr.c \
C:/ti/controlSUITE/device_support/f2833x/v140/DSP2833x_headers/source/DSP2833x_GlobalVariableDefs.c \
C:/ti/controlSUITE/device_support/f2833x/v140/DSP2833x_common/source/DSP2833x_PieCtrl.c \
../HVACI_Sensorless-DevInit_F2833x.c \
../Mediciones.c \
../TripZoneEnable.c \
../main_control_corriente.c 

C_DEPS += \
./DSP2833x_DefaultIsr.d \
./DSP2833x_GlobalVariableDefs.d \
./DSP2833x_PieCtrl.d \
./HVACI_Sensorless-DevInit_F2833x.d \
./Mediciones.d \
./TripZoneEnable.d \
./main_control_corriente.d 

OBJS += \
./DSP2833x_ADC_cal.obj \
./DSP2833x_CodeStartBranch.obj \
./DSP2833x_DefaultIsr.obj \
./DSP2833x_GlobalVariableDefs.obj \
./DSP2833x_PieCtrl.obj \
./DSP2833x_usDelay.obj \
./FIR_f32.obj \
./HVACI_Sensorless-DevInit_F2833x.obj \
./Mediciones.obj \
./TripZoneEnable.obj \
./main_control_corriente.obj 

ASM_DEPS += \
./DSP2833x_ADC_cal.d \
./DSP2833x_CodeStartBranch.d \
./DSP2833x_usDelay.d \
./FIR_f32.d 

OBJS__QUOTED += \
"DSP2833x_ADC_cal.obj" \
"DSP2833x_CodeStartBranch.obj" \
"DSP2833x_DefaultIsr.obj" \
"DSP2833x_GlobalVariableDefs.obj" \
"DSP2833x_PieCtrl.obj" \
"DSP2833x_usDelay.obj" \
"FIR_f32.obj" \
"HVACI_Sensorless-DevInit_F2833x.obj" \
"Mediciones.obj" \
"TripZoneEnable.obj" \
"main_control_corriente.obj" 

C_DEPS__QUOTED += \
"DSP2833x_DefaultIsr.d" \
"DSP2833x_GlobalVariableDefs.d" \
"DSP2833x_PieCtrl.d" \
"HVACI_Sensorless-DevInit_F2833x.d" \
"Mediciones.d" \
"TripZoneEnable.d" \
"main_control_corriente.d" 

ASM_DEPS__QUOTED += \
"DSP2833x_ADC_cal.d" \
"DSP2833x_CodeStartBranch.d" \
"DSP2833x_usDelay.d" \
"FIR_f32.d" 

ASM_SRCS__QUOTED += \
"../DSP2833x_ADC_cal.asm" \
"C:/ti/controlSUITE/device_support/f2833x/v140/DSP2833x_common/source/DSP2833x_CodeStartBranch.asm" \
"C:/ti/controlSUITE/device_support/f2833x/v140/DSP2833x_common/source/DSP2833x_usDelay.asm" \
"../FIR_f32.asm" 

C_SRCS__QUOTED += \
"C:/ti/controlSUITE/device_support/f2833x/v140/DSP2833x_common/source/DSP2833x_DefaultIsr.c" \
"C:/ti/controlSUITE/device_support/f2833x/v140/DSP2833x_headers/source/DSP2833x_GlobalVariableDefs.c" \
"C:/ti/controlSUITE/device_support/f2833x/v140/DSP2833x_common/source/DSP2833x_PieCtrl.c" \
"../HVACI_Sensorless-DevInit_F2833x.c" \
"../Mediciones.c" \
"../TripZoneEnable.c" \
"../main_control_corriente.c" 


