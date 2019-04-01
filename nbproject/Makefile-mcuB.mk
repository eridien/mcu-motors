#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-mcuB.mk)" "nbproject/Makefile-local-mcuB.mk"
include nbproject/Makefile-local-mcuB.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=mcuB
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/mcu-motors.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/mcu-motors.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=clock.c home.c i2c.c main.c motor.c move.c state.c stop.c dist-table.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/clock.o ${OBJECTDIR}/home.o ${OBJECTDIR}/i2c.o ${OBJECTDIR}/main.o ${OBJECTDIR}/motor.o ${OBJECTDIR}/move.o ${OBJECTDIR}/state.o ${OBJECTDIR}/stop.o ${OBJECTDIR}/dist-table.o
POSSIBLE_DEPFILES=${OBJECTDIR}/clock.o.d ${OBJECTDIR}/home.o.d ${OBJECTDIR}/i2c.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/motor.o.d ${OBJECTDIR}/move.o.d ${OBJECTDIR}/state.o.d ${OBJECTDIR}/stop.o.d ${OBJECTDIR}/dist-table.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/clock.o ${OBJECTDIR}/home.o ${OBJECTDIR}/i2c.o ${OBJECTDIR}/main.o ${OBJECTDIR}/motor.o ${OBJECTDIR}/move.o ${OBJECTDIR}/state.o ${OBJECTDIR}/stop.o ${OBJECTDIR}/dist-table.o

# Source Files
SOURCEFILES=clock.c home.c i2c.c main.c motor.c move.c state.c stop.c dist-table.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-mcuB.mk dist/${CND_CONF}/${IMAGE_TYPE}/mcu-motors.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24F16KM202
MP_LINKER_FILE_OPTION=,--script=p24F16KM202.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/clock.o: clock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/clock.o.d 
	@${RM} ${OBJECTDIR}/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  clock.c  -o ${OBJECTDIR}/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/clock.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/clock.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/home.o: home.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/home.o.d 
	@${RM} ${OBJECTDIR}/home.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  home.c  -o ${OBJECTDIR}/home.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/home.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/home.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/i2c.o: i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/i2c.o.d 
	@${RM} ${OBJECTDIR}/i2c.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  i2c.c  -o ${OBJECTDIR}/i2c.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/i2c.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/i2c.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/motor.o: motor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/motor.o.d 
	@${RM} ${OBJECTDIR}/motor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor.c  -o ${OBJECTDIR}/motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/motor.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/motor.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/move.o: move.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/move.o.d 
	@${RM} ${OBJECTDIR}/move.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  move.c  -o ${OBJECTDIR}/move.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/move.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/move.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/state.o: state.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/state.o.d 
	@${RM} ${OBJECTDIR}/state.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  state.c  -o ${OBJECTDIR}/state.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/state.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/state.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/stop.o: stop.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/stop.o.d 
	@${RM} ${OBJECTDIR}/stop.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  stop.c  -o ${OBJECTDIR}/stop.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/stop.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/stop.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/dist-table.o: dist-table.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/dist-table.o.d 
	@${RM} ${OBJECTDIR}/dist-table.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  dist-table.c  -o ${OBJECTDIR}/dist-table.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/dist-table.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/dist-table.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/clock.o: clock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/clock.o.d 
	@${RM} ${OBJECTDIR}/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  clock.c  -o ${OBJECTDIR}/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/clock.o.d"        -g -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/clock.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/home.o: home.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/home.o.d 
	@${RM} ${OBJECTDIR}/home.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  home.c  -o ${OBJECTDIR}/home.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/home.o.d"        -g -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/home.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/i2c.o: i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/i2c.o.d 
	@${RM} ${OBJECTDIR}/i2c.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  i2c.c  -o ${OBJECTDIR}/i2c.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/i2c.o.d"        -g -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/i2c.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"        -g -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/motor.o: motor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/motor.o.d 
	@${RM} ${OBJECTDIR}/motor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor.c  -o ${OBJECTDIR}/motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/motor.o.d"        -g -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/motor.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/move.o: move.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/move.o.d 
	@${RM} ${OBJECTDIR}/move.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  move.c  -o ${OBJECTDIR}/move.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/move.o.d"        -g -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/move.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/state.o: state.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/state.o.d 
	@${RM} ${OBJECTDIR}/state.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  state.c  -o ${OBJECTDIR}/state.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/state.o.d"        -g -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/state.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/stop.o: stop.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/stop.o.d 
	@${RM} ${OBJECTDIR}/stop.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  stop.c  -o ${OBJECTDIR}/stop.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/stop.o.d"        -g -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/stop.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/dist-table.o: dist-table.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/dist-table.o.d 
	@${RM} ${OBJECTDIR}/dist-table.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  dist-table.c  -o ${OBJECTDIR}/dist-table.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/dist-table.o.d"        -g -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -DFORCE_ID_1 -DREV4 -DDEBUG -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/dist-table.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/mcu-motors.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/mcu-motors.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG   -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x800:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x825 -mreserve=data@0x826:0x84F   -Wl,,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/mcu-motors.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/mcu-motors.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_mcuB=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wl,,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/mcu-motors.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/mcuB
	${RM} -r dist/mcuB

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
