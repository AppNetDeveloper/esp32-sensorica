#
# Generated Makefile - do not edit!
#
#
# This file contains information about the location of compilers and other tools.
# If you commmit this file into your revision control server, you will be able to 
# to checkout the project and build it from the command line with make. However,
# if more than one person works on the same project, then this file might show
# conflicts since different users are bound to have compilers in different places.
# In that case you might choose to not commit this file and let MPLAB X recreate this file
# for each user. The disadvantage of not commiting this file is that you must run MPLAB X at
# least once so the file gets created and the project can be built. Finally, you can also
# avoid using this file at all if you are only building from the command line with make.
# You can invoke make with the values of the macros:
# $ makeMP_CC="/opt/microchip/mplabc30/v3.30c/bin/pic30-gcc" ...  
#
SHELL=cmd.exe
PATH_TO_IDE_BIN=E:/Microchip/MPLABX/v6.00/mplab_platform/platform/../mplab_ide/modules/../../bin/
# Adding MPLAB X bin directory to path.
PATH:=E:/Microchip/MPLABX/v6.00/mplab_platform/platform/../mplab_ide/modules/../../bin/:$(PATH)
# Path to java used to run MPLAB X when this makefile was created
MP_JAVA_PATH="E:\Microchip\MPLABX\v6.00\sys\java\zulu8.54.0.21-ca-fx-jre8.0.292-win_x64/bin/"
OS_CURRENT="$(shell uname -s)"
MP_CC="E:\Program Files\Microchip\xc32\v4.10\bin\xc32-gcc.exe"
MP_CPPC="E:\Program Files\Microchip\xc32\v4.10\bin\xc32-g++.exe"
# MP_BC is not defined
MP_AS="E:\Program Files\Microchip\xc32\v4.10\bin\xc32-as.exe"
MP_LD="E:\Program Files\Microchip\xc32\v4.10\bin\xc32-ld.exe"
MP_AR="E:\Program Files\Microchip\xc32\v4.10\bin\xc32-ar.exe"
DEP_GEN=${MP_JAVA_PATH}java -jar "E:/Microchip/MPLABX/v6.00/mplab_platform/platform/../mplab_ide/modules/../../bin/extractobjectdependencies.jar"
MP_CC_DIR="E:\Program Files\Microchip\xc32\v4.10\bin"
MP_CPPC_DIR="E:\Program Files\Microchip\xc32\v4.10\bin"
# MP_BC_DIR is not defined
MP_AS_DIR="E:\Program Files\Microchip\xc32\v4.10\bin"
MP_LD_DIR="E:\Program Files\Microchip\xc32\v4.10\bin"
MP_AR_DIR="E:\Program Files\Microchip\xc32\v4.10\bin"
# MP_BC_DIR is not defined
CMSIS_DIR=C:/Users/arducam/.mchp_packs/ARM/CMSIS/5.8.0
DFP_DIR=C:/Users/arducam/.mchp_packs/Microchip/SAME54_DFP/3.7.122