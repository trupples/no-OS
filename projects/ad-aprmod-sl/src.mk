LIB_FLAGS += -lm
SRC_DIRS += $(PROJECT)/src
SRC_DIRS += $(NO-OS)/iio/iio_app
SRC_DIRS += $(NO-OS)/drivers/photo-electronic/adpd188

SRC_DIRS += $(PLATFORM_DRIVERS)
SRC_DIRS += $(NO-OS)/util
SRC_DIRS += $(INCLUDE) \
		$(DRIVERS)/api \
		$(PLATFORM_DRIVERS)

TINYIIOD=y

