# *********************************************************************
#
#  $Id: httpsupport.h 2196 2011-07-28 15:58:38Z seb $
#
#  GNU Makefile for MqttBridge

#  - - - - - - - - - License information: - - - - - - - - -
#
#  Copyright (C) 2011 and beyond by Yoctopuce Sarl, Switzerland.
#
#  1) If you have obtained this file from www.yoctopuce.com using
#     a valid customer account established in your proper name,
#     Yoctopuce Sarl (hereafter Licensor) licenses to you (hereafter
#     Licensee) the right to use, modify, copy, and integrate this
#     source file into your own solution for the sole purpose of
#     interfacing a Yoctopuce product integrated into Licensee's
#     solution.
#
#     You should refer to the license agreement accompanying this
#     Software for additional information regarding your rights
#     and obligations.
#
#     THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
#     WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING
#     WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS
#     FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO
#     EVENT SHALL LICENSOR BE LIABLE FOR ANY INCIDENTAL, SPECIAL,
#     INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
#     COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR
#     SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT
#     LIMITED TO ANY DEFENSE THEREOF), ANY CLAIMS FOR INDEMNITY OR
#     CONTRIBUTION, OR OTHER SIMILAR COSTS, WHETHER ASSERTED ON THE
#     BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE), BREACH OF
#     WARRANTY, OR OTHERWISE.
#
#  2) If you have obtained this file from any other source, you
#     are not entitled to use it, read it or create any derived
#     material. You should delete this file immediately.
#
# ********************************************************************

ifeq ($(OS),Windows_NT)
UNAME := Windows
ARCH  := "windows"
RM = $(XDCTOOLS)rm
MKDIR = $(XDCTOOLS)mkdir
_OBJ = obj
_LIB = lib
_INC_FLAG = /I
else
UNAME := $(shell uname)
ARCH  := $(shell uname -m| sed -e s/i.86/i386/ -e s/arm.*/arm/)
RM = rm
MKDIR = mkdir
_OBJ = o
_LIB = a
_INC_FLAG = -I
endif

MIPS_TARGET_ROOT = $(TARGET_MIPS)
MIPSEL_TARGET_ROOT = $(TARGET_MIPSEL)

.SECONDARY:

#ALL directory definitions
DIR_WIN32   = windows/
DIR_WIN64   = windows/amd64/
DIR_I386 	= linux/32bits/
DIR_X64 	= linux/64bits/
DIR_ARMEL 	= linux/armel/
DIR_ARMHF 	= linux/armhf/
DIR_AARCH64	= linux/aarch64/
DIR_MIPS 	= linux/mips/
DIR_MIPSEL 	= linux/mipsel/
DIR_OSX 	= osx/

OBJDIR_WIN32 	= $(DIR_WIN32)obj/
OBJDIR_WIN64    = $(DIR_WIN64)obj/
OBJDIR_I386 	= $(DIR_I386)obj/
OBJDIR_X64 		= $(DIR_X64)obj/
OBJDIR_ARMEL 	= $(DIR_ARMEL)obj/
OBJDIR_ARMHF 	= $(DIR_ARMHF)obj/
OBJDIR_AARCH64	= $(DIR_AARCH64)obj/
OBJDIR_MIPS 	= $(DIR_MIPS)obj/
OBJDIR_MIPSEL 	= $(DIR_MIPSEL)obj/
OBJDIR_OSX 		= $(DIR_OSX)obj/

#CC compiler for each arch
CC_WIN32 = cl
CC_WIN64 = cl
CC_I386 = gcc -std=gnu99
CC_X64 = gcc -std=gnu99
ifdef TOOLCHAIN_QARM
CC_ARMEL = $(TOOLCHAIN_QARM)/bin/arm-none-linux-gnueabi-gcc
else
CC_ARMEL = gcc
endif
ifdef TOOLCHAIN_ARMHF
CC_ARMHF = $(TOOLCHAIN_ARMHF)/bin/arm-linux-gnueabihf-gcc -std=gnu99
else
CC_ARMHF = gcc -std=gnu99
endif
ifdef TOOLCHAIN_AARCH64
CC_AARCH64 = $(TOOLCHAIN_AARCH64)/bin/arm-linux-gnueabihf-gcc -std=gnu99
else
CC_AARCH64 = gcc -std=gnu99
endif
CC_MIPS = mips-openwrt-linux-gcc -std=gnu99
CC_MIPSEL = mipsel-openwrt-linux-gcc -std=gnu99
CC_OSX = gcc

#strip for each arch
STRIP_WIN32 = strip
STRIP_WIN64 = strip
STRIP_I386 = strip
STRIP_X64 = strip
ifdef TOOLCHAIN_QARM
STRIP_ARMEL = $(TOOLCHAIN_QARM)/bin/arm-none-linux-gnueabi-strip
else
STRIP_ARMEL = strip
endif
ifdef TOOLCHAIN_ARMHF
STRIP_ARMHF = $(TOOLCHAIN_ARMHF)/bin/arm-linux-gnueabihf-strip
else
STRIP_ARMHF = strip
endif
ifdef TOOLCHAIN_AARCH64
STRIP_AARCH64 = $(TOOLCHAIN_AARCH64)/bin/arm-linux-gnueabihf-strip
else
STRIP_AARCH64 = strip
endif
STRIP_MIPS = mips-openwrt-linux-strip
STRIP_MIPSEL = mipsel-openwrt-linux-strip
STRIP_OSX = strip


GLOBAL_OPT = -Wall $(DEF_SSL_GCC) -DMQTT_BRIDGE
GLOBAL_OPT_WIN = /nologo $(DEF_SSL_WIN) /W3 /WX- /wd4710 /wd4711 /O2 /Oi /Oy- /GL $(DEF_FLAGS_WIN) /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /D "_USING_V110_SDK71_" /Gm- /EHsc /MT /GS /Gy /Zc:inline /fp:precise /Zc:wchar_t /Zc:forScope /analyze- /Gd

OPTS_WIN32 = $(DEF_FLAGS_WIN)
OPTS_WIN64 = $(DEF_FLAGS_WIN)
OPTS_I386 = -O2 -m32 -march=i586 -D_GNU_SOURCE -Wno-stringop-overflow
OPTS_X64 = -O2 -m64 -D_GNU_SOURCE -Wno-stringop-overflow
OPTS_ARMEL = -I$(STAGING_QARM)/usr/include/ -O2 -mfloat-abi=soft -march=armv5 -marm -DBUILD_ARMEL -D_XOPEN_SOURCE=500 -D_BSD_SOURCE
OPTS_ARMHF = -I$(STAGING_ARMHF)/usr/include/ -O2 -mfloat-abi=hard -march=armv6 -marm -DBUILD_ARMHF
ifdef TOOLCHAIN_AARCH64
OPTS_AARCH64 = -I$(STAGING_AARCH64)/usr/include/ $(OPTS_DBG) -march=native -DBUILD_AARCH64
else
OPTS_AARCH64 = -march=native -DBUILD_AARCH64
endif
OPTS_MIPS = -DCPU_BIG_ENDIAN -O2 -I$(MIPS_TARGET_ROOT)/usr/include/
OPTS_MIPSEL = -O2 -I$(MIPSEL_TARGET_ROOT)/usr/include/
OPTS_OSX = -O1 -m64

RC = "rc.exe"
LINKER_WIN32 = link
LINKER_WIN64 = link

#link optionsfor each arch
LINK_WIN32 = "Advapi32.lib"
LINK_WIN64 = "Advapi32.lib"
LINK_I386 = -lm -lpthread -lusb-1.0
LINK_X64 = -lm -lpthread -lusb-1.0
LINK_ARMEL = -L$(STAGING_QARM)/usr/lib/ -lm -lpthread -lusb-1.0
LINK_ARMHF = -L$(STAGING_ARMHF)/usr/lib/ -lm -lpthread -lusb-1.0
ifdef TOOLCHAIN_AARCH64
LINK_AARCH64 = -L$(STAGING_AARCH64)/usr/lib/ -lm -lpthread -lusb-1.0
else
LINK_AARCH64 = -lm -lpthread -lusb-1.0
endif
LINK_MIPS = -L$(MIPS_TARGET_ROOT)/usr/lib/ -lm -lpthread -lusb-1.0
LINK_MIPSEL = -L$(MIPSEL_TARGET_ROOT)/usr/lib/ -lm -lpthread -lusb-1.0
LINK_OSX = -lstdc++ -framework IOKit -framework CoreFoundation

MQTB_SRC_DIR = ../Sources/
MQTB_OBJS = MqttBridge.o mqtt.o request.o ostools.o

YAPI_SRC_DIR = ../Sources/yapi/
YAPI_OBJS = yapi.o ystream.o yprog.o ypkt_lin.o ypkt_win.o ypkt_osx.o yfifo.o \
		 	ykey.o yhash.o yjson.o ytcp.o ymemory.o ythread.o yssl.o

include mbedtls.inc

MBEDTLS_DIR         = $(YAPI_SRC_DIR)mbedtls/
MBEDTLS_INCLUDE_DIR = $(MBEDTLS_DIR)include/
MBEDTLS_SOURCES_DIR = $(MBEDTLS_DIR)library/

VPATH 		= $(YAPI_SRC_DIR) $(MQTB_SRC_DIR) $(MBEDTLS_SOURCES_DIR)
INCLUDES    = $(YAPI_SRC_DIR) $(MQTB_SRC_DIR) $(MBEDTLS_INCLUDE_DIR)

INC_PATH    = $(patsubst %,-I%,$(subst :, ,$(INCLUDES)))
INC_PATH_WIN = $(patsubst %,/I %,$(subst :, ,$(INCLUDES)))

ALL_H		= $(wildcard $(patsubst %,%*.h,$(subst :, ,$(YAPI_SRC_DIR) $(MBEDTLS_INCLUDE_DIR))))

COMMON_OBJS	= $(YAPI_OBJS) $(OBJS_CRYPTO) $(OBJS_X509) $(OBJS_TLS)

ALL_OBJS    = $(MQTB_OBJS) $(COMMON_OBJS) 

MQTB_WIN32 = $(DIR_WIN32)MqttBridge.exe
MQTB_WIN64 = $(DIR_WIN64)MqttBridge.exe
MQTB_I386 = $(DIR_I386)MqttBridge
MQTB_X64 = $(DIR_X64)MqttBridge
MQTB_ARMEL = $(DIR_ARMEL)MqttBridge
MQTB_ARMHF = $(DIR_ARMHF)MqttBridge
MQTB_MIPS = $(DIR_MIPS)MqttBridge
MQTB_MIPSEL = $(DIR_MIPSEL)MqttBridge
MQTB_AARCH64 = $(DIR_AARCH64)MqttBridge
MQTB_OSX = $(DIR_OSX)MqttBridge

ALL_OBJS_WIN32   = $(patsubst %.o,$(OBJDIR_WIN32)%.$(_OBJ), $(ALL_OBJS) winStartup.o)
ALL_OBJS_WIN64   = $(patsubst %.o,$(OBJDIR_WIN64)%.$(_OBJ), $(ALL_OBJS) winStartup.o)
ALL_OBJS_I386    = $(patsubst %.o,$(OBJDIR_I386)%.$(_OBJ), $(ALL_OBJS) linStartup.o)
ALL_OBJS_X64     = $(patsubst %.o,$(OBJDIR_X64)%.$(_OBJ), $(ALL_OBJS) linStartup.o)
ALL_OBJS_ARMEL   = $(patsubst %.o,$(OBJDIR_ARMEL)%.$(_OBJ), $(ALL_OBJS) linStartup.o)
ALL_OBJS_ARMHF   = $(patsubst %.o,$(OBJDIR_ARMHF)%.$(_OBJ), $(ALL_OBJS) linStartup.o)
ALL_OBJS_MIPS    = $(patsubst %.o,$(OBJDIR_MIPS)%.$(_OBJ), $(ALL_OBJS) linStartup.o)
ALL_OBJS_MIPSEL  = $(patsubst %.o,$(OBJDIR_MIPSEL)%.$(_OBJ), $(ALL_OBJS) linStartup.o)
ALL_OBJS_AARCH64 = $(patsubst %.o,$(OBJDIR_AARCH64)%.$(_OBJ), $(ALL_OBJS) linStartup.o)
ALL_OBJS_OSX     = $(patsubst %.o,$(OBJDIR_OSX)%.$(_OBJ), $(ALL_OBJS) osxStartup.o)

fwd: default

$(OBJDIR_WIN32)%.$(_OBJ): %.c $(ALL_H) | $(OBJDIR_WIN32)
	@echo compiling $@
	@$(CC_WIN32) $(OPTS_WIN32) $(GLOBAL_OPT) $(GLOBAL_OPT_WIN) $(INC_PATH_WIN) /Fo$@ /c $<

$(OBJDIR_WIN64)%.$(_OBJ) : %.c $(ALL_H) | $(OBJDIR_WIN64)
	@echo compiling 64 bits $@
	@$(CC_WIN64) $(OPTS_WIN64) $(GLOBAL_OPT) $(GLOBAL_OPT_WIN) $(INC_PATH_WIN) /Fo$@ /c $<

$(OBJDIR_X64)%.$(_OBJ) : %.c $(ALL_H) | $(OBJDIR_X64)
	@echo compiling 64 bits $@
	@$(CC_X64) $(OPTS_X64) $(GLOBAL_OPT) $(INC_PATH) -c $< -o $@

$(OBJDIR_I386)%.$(_OBJ) : %.c $(ALL_H) | $(OBJDIR_I386)
	@echo compiling $@
	@$(CC_I386) $(OPTS_I386) $(GLOBAL_OPT) $(INC_PATH) -c $< -o $@

$(OBJDIR_ARMEL)%.$(_OBJ) : %.c $(ALL_H) | $(OBJDIR_ARMEL)
	@echo compiling ARM soft-float $@
	@$(CC_ARMEL) $(OPTS_ARMEL) $(GLOBAL_OPT) $(INC_PATH) -c $< -o $@

$(OBJDIR_ARMHF)%.$(_OBJ) : %.c $(ALL_H) | $(OBJDIR_ARMHF)
	@echo compiling ARM hard-floats$@
	@$(CC_ARMHF) $(OPTS_ARMHF) $(GLOBAL_OPT) $(INC_PATH) -c $< -o $@

$(OBJDIR_MIPS)%.$(_OBJ) : %.c $(ALL_H) | $(OBJDIR_MIPS)
	@echo compiling mips $@
	@$(CC_MIPS) $(OPTS_MIPS) $(GLOBAL_OPT) $(INC_PATH) -c $< -o $@

$(OBJDIR_MIPSEL)%.$(_OBJ) : %.c $(ALL_H) | $(OBJDIR_MIPSEL)
	@echo compiling mipsel $@
	@$(CC_MIPSEL) $(OPTS_MIPSEL) $(GLOBAL_OPT) $(INC_PATH) -c $< -o $@

$(OBJDIR_AARCH64)%.$(_OBJ) : %.c $(ALL_H) | $(OBJDIR_AARCH64)
	@echo compiling aarch64 $@
	@$(CC_AARCH64) $(OPTS_AARCH64) $(GLOBAL_OPT) $(INC_PATH) -c $< -o $@

$(OBJDIR_OSX)%.$(_OBJ) : %.c $(ALL_H) | $(OBJDIR_OSX)
	@echo compiling osx $@
	@$(CC_OSX) $(OPTS_OSX) $(GLOBAL_OPT) $(INC_PATH) -c $< -o $@

ALL_DIRS = $(DIR_WIN32) $(DIR_WIN64) $(DIR_I386) $(DIR_X64) $(DIR_ARMEL) \
	$(DIR_ARMHF) $(DIR_MIPS) $(DIR_MIPSEL) $(DIR_OSX) $(DIR_AARCH64)  \
	$(OBJDIR_WIN32) $(OBJDIR_WIN64) $(OBJDIR_I386) $(OBJDIR_X64) $(OBJDIR_ARMEL) \
	$(OBJDIR_ARMHF) $(OBJDIR_MIPS) $(OBJDIR_MIPSEL) $(OBJDIR_OSX) $(OBJDIR_AARCH64)

ifeq ($(UNAME), Windows)
default: win32

release: release_win32

else
ifeq ($(UNAME), Linux)
default: x64

release: release_x64

else
default: build_osx

release: release_osx

endif
endif

win32: $(MQTB_WIN32)

release_win32: $(MQTB_WIN32) clean_obj_win32

win64: $(MQTB_WIN64)

release_win64: $(MQTB_WIN64) clean_obj_win64

i386: $(MQTB_I386)

release_i386: $(MQTB_I386) clean_obj_i386

x64: $(MQTB_X64)

release_x64: $(MQTB_X64) clean_obj_x64

armel: $(MQTB_ARMEL)

release_armel: $(MQTB_ARMEL) clean_obj_armel

armhf: $(MQTB_ARMHF)

release_armhf: $(MQTB_ARMHF) clean_obj_armhf

mips : export STAGING_DIR = $(STAGING_MIPS)
mips: $(MQTB_MIPS)

release_mips : export STAGING_DIR = $(STAGING_MIPS)
release_mips: $(MQTB_MIPS) clean_obj_mips

mipsel : export STAGING_DIR = $(STAGING_MIPSEL)
mipsel: $(MQTB_MIPSEL)

release_mipsel : export STAGING_DIR = $(STAGING_MIPSEL)
release_mipsel: $(MQTB_MIPSEL) clean_obj_mipsel

aarch64: $(MQTB_AARCH64)

release_aarch64: $(MQTB_AARCH64) clean_obj_aarch64

build_osx: $(MQTB_OSX)

release_osx: $(MQTB_OSX) clean_obj_osx

$(MQTB_WIN32): $(DIR_WIN32) $(OBJDIR_WIN32) $(ALL_OBJS_WIN32)
	@echo linking $@
	@$(LINKER_WIN32) $(LINK_WIN32) /OUT:"$@" $(ALL_OBJS_WIN32)

$(MQTB_WIN64): $(DIR_WIN64) $(OBJDIR_WIN64) $(ALL_OBJS_WIN64)
	@echo linking $@
	@$(LINKER_WIN64) $(LINK_WIN64) /OUT:"$@" $(ALL_OBJS_WIN64)

$(MQTB_I386): $(DIR_I386) $(OBJDIR_I386) $(ALL_OBJS_I386)
	@echo linking $@
	@$(CC_I386) $(OPTS_I386) $(OPTS_PROD) -o $@ $(ALL_OBJS_I386) $(LINK_I386)

$(MQTB_X64): $(DIR_X64) $(OBJDIR_X64) $(ALL_OBJS_X64)
	@echo linking $@
	@$(CC_X64) $(OPTS_X64) $(OPTS_PROD) -o $@ $(ALL_OBJS_X64) $(LINK_X64)

$(MQTB_ARMEL): $(DIR_ARMEL) $(OBJDIR_ARMEL) $(ALL_OBJS_ARMEL)
	@echo linking $@
	@$(CC_ARMEL) $(OPTS_ARMEL) $(OPTS_PROD) -o $@ $(ALL_OBJS_ARMEL) $(LINK_ARMEL)

$(MQTB_ARMHF): $(DIR_ARMHF) $(OBJDIR_ARMHF) $(ALL_OBJS_ARMHF)
	@echo linking $@
	@$(CC_ARMHF) $(OPTS_ARMHF) $(OPTS_PROD) -o $@ $(ALL_OBJS_ARMHF) $(LINK_ARMHF)

$(MQTB_MIPS): $(DIR_MIPS) $(OBJDIR_MIPS) $(ALL_OBJS_MIPS)
	@echo linking $@
	@$(CC_MIPS) $(OPTS_MIPS) $(OPTS_PROD) -o $@ $(ALL_OBJS_MIPS) $(LINK_MIPS)

$(MQTB_MIPSEL): $(DIR_MIPSEL) $(OBJDIR_MIPSEL) $(ALL_OBJS_MIPSEL)
	@echo linking $@
	@$(CC_MIPSEL) $(OPTS_MIPSEL) $(OPTS_PROD) -o $@ $(ALL_OBJS_MIPSEL) $(LINK_MIPSEL)

$(MQTB_AARCH64): $(DIR_AARCH64) $(OBJDIR_AARCH64) $(ALL_OBJS_AARCH64)
	@echo linking $@
	@$(CC_AARCH64) $(OPTS_AARCH64) $(OPTS_PROD) -o $@ $(ALL_OBJS_AARCH64) $(LINK_AARCH64)

$(MQTB_OSX): $(DIR_OSX) $(OBJDIR_OSX) $(ALL_OBJS_OSX)
	@echo linking $@
	@$(CC_OSX) $(OPTS_OSX) $(OPTS_PROD) -o $@ $(ALL_OBJS_OSX) $(LINK_OSX)

$(ALL_DIRS):
	@$(MKDIR) -p $@

clean_obj_win32:
	@$(RM) -rf $(OBJDIR_WIN32)

clean_obj_win64:
	@$(RM) -rf $(OBJDIR_WIN64)

clean_obj_i386:
	@$(RM) -rf $(OBJDIR_I386)

clean_obj_x64:
	@$(RM) -rf $(OBJDIR_X64)

clean_obj_armel:
	@$(RM) -rf $(OBJDIR_ARMEL)

clean_obj_armhf:
	@$(RM) -rf $(OBJDIR_ARMHF)

clean_obj_mips:
	@$(RM) -rf $(OBJDIR_MIPS)

clean_obj_mipsel:
	@$(RM) -rf $(OBJDIR_MIPSEL)

clean_obj_aarch64:
	@$(RM) -rf $(OBJDIR_AARCH64)

clean_obj_osx:
	@$(RM) -rf $(OBJDIR_OSX)

clean_win32: clean_obj_win32
	@$(RM) -rf $(MQTB_WIN32)

clean_win64: clean_obj_win64
	@$(RM) -rf $(MQTB_WIN64)

clean_i386: clean_obj_i386
	@$(RM) -rf $(MQTB_I386)

clean_x64: clean_obj_x64
	@$(RM) -rf $(MQTB_X64)

clean_armel: clean_obj_armel
	@$(RM) -rf $(MQTB_ARMEL)

clean_armhf: clean_obj_armhf
	@$(RM) -rf $(MQTB_ARMHF)

clean_mips: clean_obj_mips
	@$(RM) -rf $(MQTB_MIPS)

clean_mipsel: clean_obj_mipsel
	@$(RM) -rf $(MQTB_MIPSEL)

clean_aarch64: clean_obj_aarch64
	@$(RM) -rf $(MQTB_AARCH64)

clean_osx: clean_obj_osx
	@$(RM) -rf $(MQTB_OSX)

clean: clean_win32 clean_win64 clean_i386 clean_x64 clean_armel clean_armhf clean_mips clean_mipsel clean_aarch64 clean_osx


