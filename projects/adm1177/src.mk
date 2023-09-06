
INCS += $(PROJECT)/src/parameters.h
SRCS += $(PROJECT)/src/main.c

INCS += $(INCLUDE)/no_os_alloc.h		\
	$(INCLUDE)/no_os_error.h		\
	$(INCLUDE)/no_os_delay.h		\
	$(INCLUDE)/no_os_mutex.h		\
	$(INCLUDE)/no_os_util.h			\
	$(INCLUDE)/no_os_lf256fifo.h		\
	$(INCLUDE)/no_os_list.h			\
	$(INCLUDE)/no_os_irq.h			\
	$(INCLUDE)/no_os_uart.h			\
	$(INCLUDE)/no_os_pwm.h			\
	$(INCLUDE)/no_os_i2c.h 			\
	$(INCLUDE)/no_os_units.h		\
	$(PLATFORM_DRIVERS)/maxim_irq.h		\
	$(PLATFORM_DRIVERS)/maxim_uart.h	\
	$(PLATFORM_DRIVERS)/maxim_uart_stdio.h	\
	$(PLATFORM_DRIVERS)/maxim_i2c.h	\

SRCS +=	$(NO-OS)/util/no_os_alloc.c		\
	$(NO-OS)/util/no_os_mutex.c		\
	$(NO-OS)/util/no_os_lf256fifo.c		\
	$(NO-OS)/util/no_os_list.c		\
	$(NO-OS)/util/no_os_util.c		\
	$(DRIVERS)/api/no_os_irq.c		\
	$(DRIVERS)/api/no_os_uart.c		\
	$(DRIVERS)/api/no_os_pwm.c		\
	$(DRIVERS)/api/no_os_i2c.c		\
	$(PLATFORM_DRIVERS)/maxim_irq.c		\
	$(PLATFORM_DRIVERS)/maxim_uart_stdio.c	\
	$(PLATFORM_DRIVERS)/maxim_uart.c	\
	$(PLATFORM_DRIVERS)/maxim_i2c.c	\
	$(PLATFORM_DRIVERS)/maxim_delay.c	\

INCS += $(DRIVERS)/adc/adm1177/adm1177.h	
	
SRCS += $(DRIVERS)/adc/adm1177/adm1177.c	
		

ifeq (y, $(strip $(TINYIIOD)))
LIBRARIES += iio
SRCS += $(NO-OS)/iio/iio_app/iio_app.c	\
	$(DRIVERS)/adc/adm1177/iio_adm1177.c\
	$(NO-OS)/iio/iio.c	\
	$(NO-OS)/iio/iiod.c	\
	$(NO-OS)/util/no_os_fifo.c

INCS += $(NO-OS)/iio/iio_app/iio_app.h	\
	$(DRIVERS)/adc/adm1177/iio_adm1177.h	\
	$(NO-OS)/iio/iio.h	\
	$(NO-OS)/iio/iiod.h	\
	$(NO-OS)/iio/iio_types.h	\
	$(INCLUDE)/no_os_fifo.h

endif
