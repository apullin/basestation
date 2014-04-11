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
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/basestation.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/basestation.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../imageproc-lib/ipspi1.c ../imageproc-lib/init_default.c ../imageproc-lib/payload.c ../imageproc-lib/radio.c ../imageproc-lib/queue.c ../imageproc-lib/delay.s ../imageproc-lib/at86rf231_driver.c ../imageproc-lib/ppool.c init.c main.c traps.c xbee_handler.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/921515994/ipspi1.o ${OBJECTDIR}/_ext/921515994/init_default.o ${OBJECTDIR}/_ext/921515994/payload.o ${OBJECTDIR}/_ext/921515994/radio.o ${OBJECTDIR}/_ext/921515994/queue.o ${OBJECTDIR}/_ext/921515994/delay.o ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o ${OBJECTDIR}/_ext/921515994/ppool.o ${OBJECTDIR}/init.o ${OBJECTDIR}/main.o ${OBJECTDIR}/traps.o ${OBJECTDIR}/xbee_handler.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/921515994/ipspi1.o.d ${OBJECTDIR}/_ext/921515994/init_default.o.d ${OBJECTDIR}/_ext/921515994/payload.o.d ${OBJECTDIR}/_ext/921515994/radio.o.d ${OBJECTDIR}/_ext/921515994/queue.o.d ${OBJECTDIR}/_ext/921515994/delay.o.d ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d ${OBJECTDIR}/_ext/921515994/ppool.o.d ${OBJECTDIR}/init.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/traps.o.d ${OBJECTDIR}/xbee_handler.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/921515994/ipspi1.o ${OBJECTDIR}/_ext/921515994/init_default.o ${OBJECTDIR}/_ext/921515994/payload.o ${OBJECTDIR}/_ext/921515994/radio.o ${OBJECTDIR}/_ext/921515994/queue.o ${OBJECTDIR}/_ext/921515994/delay.o ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o ${OBJECTDIR}/_ext/921515994/ppool.o ${OBJECTDIR}/init.o ${OBJECTDIR}/main.o ${OBJECTDIR}/traps.o ${OBJECTDIR}/xbee_handler.o

# Source Files
SOURCEFILES=../imageproc-lib/ipspi1.c ../imageproc-lib/init_default.c ../imageproc-lib/payload.c ../imageproc-lib/radio.c ../imageproc-lib/queue.c ../imageproc-lib/delay.s ../imageproc-lib/at86rf231_driver.c ../imageproc-lib/ppool.c init.c main.c traps.c xbee_handler.c


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
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/basestation.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33FJ128MC706A
MP_LINKER_FILE_OPTION=,--script=p33FJ128MC706A.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/921515994/ipspi1.o: ../imageproc-lib/ipspi1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/ipspi1.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/ipspi1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/ipspi1.c  -o ${OBJECTDIR}/_ext/921515994/ipspi1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/ipspi1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1    -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/ipspi1.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/921515994/init_default.o: ../imageproc-lib/init_default.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/init_default.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/init_default.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/init_default.c  -o ${OBJECTDIR}/_ext/921515994/init_default.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/init_default.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1    -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/init_default.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/921515994/payload.o: ../imageproc-lib/payload.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/payload.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/payload.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/payload.c  -o ${OBJECTDIR}/_ext/921515994/payload.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/payload.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1    -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/payload.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/921515994/radio.o: ../imageproc-lib/radio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/radio.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/radio.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/radio.c  -o ${OBJECTDIR}/_ext/921515994/radio.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/radio.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1    -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/radio.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/921515994/queue.o: ../imageproc-lib/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/queue.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/queue.c  -o ${OBJECTDIR}/_ext/921515994/queue.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/queue.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1    -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/queue.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/921515994/at86rf231_driver.o: ../imageproc-lib/at86rf231_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/at86rf231_driver.c  -o ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1    -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/921515994/ppool.o: ../imageproc-lib/ppool.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/ppool.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/ppool.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/ppool.c  -o ${OBJECTDIR}/_ext/921515994/ppool.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/ppool.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1    -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/ppool.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/init.o: init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/init.o.d 
	@${RM} ${OBJECTDIR}/init.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  init.c  -o ${OBJECTDIR}/init.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/init.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1    -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/init.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1    -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/traps.o: traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/traps.o.d 
	@${RM} ${OBJECTDIR}/traps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  traps.c  -o ${OBJECTDIR}/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/traps.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1    -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/xbee_handler.o: xbee_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/xbee_handler.o.d 
	@${RM} ${OBJECTDIR}/xbee_handler.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  xbee_handler.c  -o ${OBJECTDIR}/xbee_handler.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/xbee_handler.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1    -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/xbee_handler.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/921515994/ipspi1.o: ../imageproc-lib/ipspi1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/ipspi1.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/ipspi1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/ipspi1.c  -o ${OBJECTDIR}/_ext/921515994/ipspi1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/ipspi1.o.d"        -g -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/ipspi1.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/921515994/init_default.o: ../imageproc-lib/init_default.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/init_default.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/init_default.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/init_default.c  -o ${OBJECTDIR}/_ext/921515994/init_default.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/init_default.o.d"        -g -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/init_default.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/921515994/payload.o: ../imageproc-lib/payload.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/payload.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/payload.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/payload.c  -o ${OBJECTDIR}/_ext/921515994/payload.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/payload.o.d"        -g -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/payload.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/921515994/radio.o: ../imageproc-lib/radio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/radio.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/radio.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/radio.c  -o ${OBJECTDIR}/_ext/921515994/radio.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/radio.o.d"        -g -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/radio.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/921515994/queue.o: ../imageproc-lib/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/queue.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/queue.c  -o ${OBJECTDIR}/_ext/921515994/queue.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/queue.o.d"        -g -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/queue.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/921515994/at86rf231_driver.o: ../imageproc-lib/at86rf231_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/at86rf231_driver.c  -o ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d"        -g -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/921515994/ppool.o: ../imageproc-lib/ppool.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/ppool.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/ppool.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/ppool.c  -o ${OBJECTDIR}/_ext/921515994/ppool.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/ppool.o.d"        -g -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/ppool.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/init.o: init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/init.o.d 
	@${RM} ${OBJECTDIR}/init.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  init.c  -o ${OBJECTDIR}/init.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/init.o.d"        -g -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/init.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"        -g -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/traps.o: traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/traps.o.d 
	@${RM} ${OBJECTDIR}/traps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  traps.c  -o ${OBJECTDIR}/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/traps.o.d"        -g -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/xbee_handler.o: xbee_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/xbee_handler.o.d 
	@${RM} ${OBJECTDIR}/xbee_handler.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  xbee_handler.c  -o ${OBJECTDIR}/xbee_handler.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/xbee_handler.o.d"        -g -omf=elf -fast-math -fno-short-double -O0 -I"./" -I"../imageproc-lib" -D__BASESTATION -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/xbee_handler.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/921515994/delay.o: ../imageproc-lib/delay.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/delay.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/delay.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../imageproc-lib/delay.s  -o ${OBJECTDIR}/_ext/921515994/delay.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -fast-math -I".." -Wa,-MD,"${OBJECTDIR}/_ext/921515994/delay.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/delay.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/_ext/921515994/delay.o: ../imageproc-lib/delay.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/delay.o.d 
	@${RM} ${OBJECTDIR}/_ext/921515994/delay.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../imageproc-lib/delay.s  -o ${OBJECTDIR}/_ext/921515994/delay.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -fast-math -I".." -Wa,-MD,"${OBJECTDIR}/_ext/921515994/delay.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/delay.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/basestation.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/basestation.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -fast-math  -mreserve=data@0x800:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x825 -mreserve=data@0x826:0x84F   -Wl,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,$(MP_LINKER_FILE_OPTION),--heap=2048,--no-check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="$(BINDIR_)$(TARGETBASE).map",--report-mem$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/basestation.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/basestation.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -fast-math -Wl,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--heap=2048,--no-check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="$(BINDIR_)$(TARGETBASE).map",--report-mem$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/basestation.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
