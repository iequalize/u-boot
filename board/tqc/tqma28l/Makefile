#
# (C) Copyright 2000-2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# SPDX-License-Identifier:	GPL-2.0+
#

ifndef	CONFIG_SPL_BUILD
obj-y	:= tqma28l.o
else
obj-y	:= iomux.o
endif

obj-$(CONFIG_TQMA28L_MBA28L)	+= tqma28l_mba28l.o
obj-$(CONFIG_TQMA28L_NONE)	+= tqma28l_none.o
