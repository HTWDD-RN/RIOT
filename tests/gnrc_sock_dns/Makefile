APPLICATION = gnrc_sock_dns
include ../Makefile.tests_common

RIOTBASE ?= $(CURDIR)/../..

BOARD_INSUFFICIENT_MEMORY := chronos telosb nucleo32-f042 nucleo32-f031 nucleo-f030 nucleo-l053 nucleo32-l031 stm32f0discovery

USEMODULE += sock_dns
USEMODULE += gnrc_sock_udp
USEMODULE += gnrc_ipv6_default
USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif

USEMODULE += shell_commands

USEMODULE += posix

CFLAGS += -DDEVELHELP

LOW_MEMORY_BOARDS := nucleo-f334 msb-430 msb-430h

ifeq ($(BOARD),$(filter $(BOARD),$(LOW_MEMORY_BOARDS)))
  CFLAGS += -DGNRC_PKTBUF_SIZE=512 -DGNRC_IPV6_NETIF_ADDR_NUMOF=4 -DGNRC_IPV6_NC_SIZE=1
endif

include $(RIOTBASE)/Makefile.include
