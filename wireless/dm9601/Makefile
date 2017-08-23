##================================================================
##     Davicom Semiconductor Inc.  	For DM9601 V0.00
##   --------------------------------------------------------
## Description:
##              Compile driver dm9601.c to dm9601.o
##
## Modification List:
## 09/05/2000	Fixed SMPFALGS wrong on smp & smp_mod
## 08/02/2000	Changed some description string & include file path
## 07/25/2000	Append smp_mod and changed some descriptions
## 01/25/2000	by Sten Wang
## 03/24/2009   Modifiy for Linux kernel 2.6.28
##================================================================
# Comment/uncomment the following line to disable/enable debugging
# DEBUG = y

# Add your debugging flag (or not) to CFLAGS
#ifeq ($(DEBUG),y)
#  DEBFLAGS = -O -g # "-O" is needed to expand inlines
#else
#  DEBFLAGS = -O2
#endif

#CFLAGS += $(DEBFLAGS) -I$(LDDINCDIR)

obj-m	:= dm9601.o
