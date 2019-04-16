srcs-$(CFG_BCM_GPIO) += pta_bcm_gpio.c
ifeq ($(CFG_PL022), y)
srcs-$(CFG_SPI_FLASH) += pta_bcm_spi_flash.c
endif
ifeq ($(CFG_BCM_I2C), y)
srcs-$(CFG_I2C_PCA95XX) += pta_bcm_i2c.c
endif
ifeq ($(CFG_BCM_SOTP), y)
srcs-$(CFG_BCM_SOTP) += pta_bcm_sotp.c
endif
srcs-$(CFG_BCM_HWRNG) += pta_bcm_hw_rng.c
