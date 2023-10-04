SRCS += $(PROJECT)/main.c
INCS += $(PROJECT)/parameters.h

INCS += $(INCLUDE)/no_os_delay.h     \
		$(INCLUDE)/no_os_error.h     \
		$(INCLUDE)/no_os_gpio.h      \
		$(INCLUDE)/no_os_print_log.h \
		$(INCLUDE)/no_os_spi.h       \
		$(INCLUDE)/no_os_alloc.h       \
		$(INCLUDE)/no_os_irq.h      \
		$(INCLUDE)/no_os_list.h      \
		$(INCLUDE)/no_os_uart.h      \
		$(INCLUDE)/no_os_lf256fifo.h \
		$(INCLUDE)/no_os_util.h 	\
		$(INCLUDE)/no_os_units.h	\
                $(INCLUDE)/no_os_mutex.h	\
		$(PLATFORM_DRIVERS)/maxim_gpio.h \
		$(PLATFORM_DRIVERS)/maxim_spi.h \
		$(PLATFORM_DRIVERS)/maxim_uart.h \
		$(PLATFORM_DRIVERS)/maxim_uart_stdio.h \
		$(PLATFORM_DRIVERS)/maxim_irq.h 	\
		$(PLATFORM_DRIVERS)/maxim_gpio_irq.h

SRCS += $(DRIVERS)/api/no_os_gpio.c \
		$(NO-OS)/util/no_os_lf256fifo.c \
		$(DRIVERS)/api/no_os_irq.c  \
		$(DRIVERS)/api/no_os_spi.c  \
		$(DRIVERS)/api/no_os_uart.c \
		$(NO-OS)/util/no_os_list.c \
		$(NO-OS)/util/no_os_util.c \
		$(NO-OS)/util/no_os_alloc.c \
                $(NO-OS)/util/no_os_mutex.c	\
		$(PLATFORM_DRIVERS)/maxim_gpio.c \
		$(PLATFORM_DRIVERS)/maxim_spi.c \
		$(PLATFORM_DRIVERS)/maxim_uart.c \
		$(PLATFORM_DRIVERS)/maxim_uart_stdio.c \
		$(PLATFORM_DRIVERS)/maxim_irq.c \
		$(PLATFORM_DRIVERS)/maxim_delay.c 	\
		$(PLATFORM_DRIVERS)/maxim_gpio_irq.c

INCS += $(DRIVERS)/digital-io/max149x6/max14906.h
SRCS += $(DRIVERS)/digital-io/max149x6/max14906.c

INCS += $(DRIVERS)/digital-io/max149x6/max149x6-base.h
SRCS += $(DRIVERS)/digital-io/max149x6/max149x6-base.c

ifeq (y, $(strip $(TINYIIOD)))
LIBRARIES += iio
SRCS += $(NO-OS)/iio/iio_app/iio_app.c	\
	$(DRIVERS)/digital-io/max149x6/iio_max14906.c	\
	$(NO-OS)/iio/iio.c	\
	$(NO-OS)/iio/iiod.c	\
	$(NO-OS)/util/no_os_fifo.c

INCS += $(NO-OS)/iio/iio_app/iio_app.h	\
	$(DRIVERS)/digital-io/max149x6/iio_max14906.h	\
	$(NO-OS)/iio/iio.h	\
	$(NO-OS)/iio/iiod.h	\
	$(NO-OS)/iio/iio_types.h	\
	$(NO-OS)/include/no_os_fifo.h
endif
