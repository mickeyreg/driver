ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif

RT28xx_MODE                       = STA
CHIPSET                           = 7601U
OSABL                             = NO

# HAS_ATE                           = y
# HAS_QA_SUPPORT                    = y
# HAS_LED_CONTROL_SUPPORT           = y
# HAS_KTHREAD_SUPPORT               = y

HAS_WPA_SUPPLICANT                = y
HAS_NATIVE_WPA_SUPPLICANT_SUPPORT = y
HAS_DOT11N_DRAFT3_SUPPORT         = y
HAS_DOT11_N_SUPPORT               = y
HAS_STATS_COUNT                   = y
HAS_RX_CSO_SUPPORT                = y

EXTRA_CFLAGS += -I$(src)/include
EXTRA_CFLAGS += -Wall -O2 -Wundef -Wstrict-prototypes
EXTRA_CFLAGS += -Wno-trigraphs -Wdeclaration-after-statement 
EXTRA_CFLAGS += -Wno-pointer-sign -fno-strict-aliasing -fno-common 
EXTRA_CFLAGS += -fomit-frame-pointer -ffreestanding -m4-nofpu

WFLAGS := -DAGGREGATION_SUPPORT -DPIGGYBACK_SUPPORT -DWMM_SUPPORT  -DLINUX -Wall -Wstrict-prototypes -Wno-trigraphs 
WFLAGS += -DSYSTEM_LOG_SUPPORT  -DRT28xx_MODE=$(RT28xx_MODE) -DCHIPSET=$(CHIPSET) -DRESOURCE_PRE_ALLOC -DST -D__KERNEL__
# WFLAGS += -DSYSTEM_LOG_SUPPORT  -DRT28xx_MODE=$(RT28xx_MODE) -DCHIPSET=$(CHIPSET) -DRESOURCE_PRE_ALLOC
# WFLAGS += -DSYSTEM_LOG_SUPPORT  -DRT28xx_MODE=$(RT28xx_MODE) -DCHIPSET=$(CHIPSET) -DRESOURCE_PRE_ALLOC -DST

ifeq ($(HAS_KTHREAD_SUPPORT),y)
WFLAGS += -DKTHREAD_SUPPORT
endif

#################################################

# config for STA mode

ifeq ($(RT28xx_MODE),STA)
WFLAGS += -DCONFIG_STA_SUPPORT -DSCAN_SUPPORT -DDBG

ifeq ($(HAS_WPA_SUPPLICANT),y)
WFLAGS += -DWPA_SUPPLICANT_SUPPORT
ifeq ($(HAS_NATIVE_WPA_SUPPLICANT_SUPPORT),y)
WFLAGS += -DNATIVE_WPA_SUPPLICANT_SUPPORT
endif
endif

ifeq ($(HAS_ATE),y)
WFLAGS += -DRALINK_ATE
WFLAGS += -DCONFIG_RT2880_ATE_CMD_NEW
WFLAGS += -I$(RT28xx_DIR)/ate/include
ifeq ($(HAS_QA_SUPPORT),y)
WFLAGS += -DRALINK_QA
endif
endif

ifeq ($(HAS_DOT11_N_SUPPORT),y)
WFLAGS += -DDOT11_N_SUPPORT

ifeq ($(HAS_DOT11N_DRAFT3_SUPPORT),y)
WFLAGS += -DDOT11N_DRAFT3
endif

endif

ifeq ($(HAS_STATS_COUNT),y)
WFLAGS += -DSTATS_COUNT_SUPPORT
endif

endif
# endif of ifeq ($(RT28xx_MODE),STA)

#################################################

ifeq ($(HAS_LED_CONTROL_SUPPORT),y)
WFLAGS += -DLED_CONTROL_SUPPORT
endif

#################################################
# ChipSet specific definitions.
#

ifneq ($(findstring 7601U,$(CHIPSET)),)
WFLAGS += -DMT7601U -DMT7601 -DRLT_MAC -DRLT_RF -DRTMP_MAC_USB -DRTMP_USB_SUPPORT -DRTMP_TIMER_TASK_SUPPORT -DRX_DMA_SCATTER -DVCORECAL_SUPPORT -DRTMP_EFUSE_SUPPORT -DNEW_MBSSID_MODE -DRTMP_INTERNAL_TX_ALC -DCONFIG_ANDES_SUPPORT -DDPD_CALIBRATION_SUPPORT
ifneq ($(findstring $(RT28xx_MODE),AP),)
#WFLAGS += -DSPECIFIC_BCN_BUF_SUPPORT
endif

ifeq ($(HAS_RX_CSO_SUPPORT), y)
WFLAGS += -DCONFIG_RX_CSO_SUPPORT
endif

ifneq ($(findstring $(RT28xx_MODE),AP),)
else
WFLAGS += -DRTMP_FREQ_CALIBRATION_SUPPORT
endif

CHIPSET_DAT = 2870
endif

#################################################

EXTRA_CFLAGS += -o $(WFLAGS) 

MOD_NAME = mt$(CHIPSET)sta

mt$(CHIPSET)sta-objs := \
	common/crypt_md5.o\
        common/crypt_sha2.o\
        common/crypt_hmac.o\
        common/crypt_aes.o\
        common/crypt_arc4.o\
        common/mlme.o\
        common/cmm_wep.o\
        common/action.o\
        common/cmm_data.o\
	common/rtmp_init.o\
        common/rtmp_init_inf.o\
        common/cmm_tkip.o\
        common/cmm_aes.o\
        common/cmm_sync.o\
        common/eeprom.o\
        common/cmm_sanity.o\
        common/cmm_info.o\
        common/cmm_cfg.o\
        common/cmm_wpa.o\
        common/cmm_radar.o\
	common/spectrum.o\
        common/rtmp_timer.o\
        common/rt_channel.o\
        common/cmm_profile.o\
        common/cmm_asic.o\
        common/scan.o\
        common/cmm_cmd.o\
        common/uapsd.o\
	common/ps.o\
	rate_ctrl/ra_ctrl.o\
        rate_ctrl/alg_legacy.o\
        rate_ctrl/alg_ags.o\
	chips/rtmp_chip.o\
		common/txpower.o\
	mac/rtmp_mac.o\
	mgmt/mgmt_hw.o\
	mgmt/mgmt_entrytb.o\
        phy/rtmp_phy.o\
        phy/rlt_phy.o\
	phy/rlt_rf.o

#ifdef DOT11_N_SUPPORT
ifeq ($(HAS_DOT11_N_SUPPORT),y)
mt$(CHIPSET)sta-objs += \
        common/ba_action.o\
        mgmt/mgmt_ht.o

endif
#endif // DOT11_N_SUPPORT //


#ifdef CONFIG_STA_SUPPORT
ifeq ($(RT28xx_MODE), STA)
mt$(CHIPSET)sta-objs += \
	os/linux/rt_profile.o\
	sta/assoc.o\
	sta/auth.o\
	sta/auth_rsp.o\
	sta/sync.o\
	sta/sanity.o\
	sta/rtmp_data.o\
	sta/connect.o\
	sta/wpa.o\
	sta/sta_cfg.o


ifeq ($(OSABL),NO)
mt$(CHIPSET)sta-objs += \
	common/rt_os_util.o\
	os/linux/sta_ioctl.o\
	os/linux/rt_linux.o\
	os/linux/rt_main_dev.o
else
mt$(CHIPSET)sta-objs += \
	os/linux/rt_symb.o
endif

#ifdef LED_CONTROL_SUPPORT
ifeq ($(HAS_LED_CONTROL_SUPPORT),y)
mt$(CHIPSET)sta-objs += \
	common/rt_led.o
endif
#endif // LED_CONTROL_SUPPORT //


ifeq ($(HAS_ATE),y)
mt$(CHIPSET)sta-objs += ate/common/rt_ate.o
endif

ifeq ($(HAS_QA_SUPPORT),y)
mt$(CHIPSET)sta-objs += ate/common/rt_qa.o
endif

endif
#endif // CONFIG_STA_SUPPORT //

#ifdef MT7601U
ifneq ($(findstring 7601U,$(CHIPSET)),)
mt$(CHIPSET)sta-objs += \
                common/cmm_mac_usb.o\
                common/cmm_data_usb.o\
		common/rtusb_io.o\
		common/rtusb_data.o\
		common/rtusb_bulk.o\
                os/linux/rt_usb.o\
                common/ee_prom.o\
                common/ee_efuse.o\
		mcu/rtmp_and.o\
		mcu/rtmp_mcu.o\
		mcu/rtmp_M51.o\
                common/rt_rf.o\
		chips/mt7601.o\
                mac/ral_omac.o

ifeq ($(HAS_RTMP_FLASH_SUPPORT),y)
mt$(CHIPSET)sta-objs += \
        common/ee_flash.o
endif

ifeq ($(HAS_ATE),y)
mt$(CHIPSET)sta-objs += \
	ate/chips/mt7601_ate.o\
	ate/common/ate_usb.o
endif

ifeq ($(HAS_TSO_SUPPORT),y)
mt$(CHIPSET)sta-objs += \
	naf/net_acc.o\
	naf/tso.o\
	naf/cso.o
endif

ifeq ($(OSABL),NO)
mt$(CHIPSET)sta-objs += \
                os/linux/rt_usb.o\
                os/linux/rt_usb_util.o\
                os/linux/usb_main_dev.o\
		common/rtusb_dev_id.o
endif

ifneq ($(findstring $(RT28xx_MODE),STA APSTA),)
mt$(CHIPSET)sta-objs += \
        common/frq_cal.o
endif

endif
#endif // MT7601U //

obj-m := $(MOD_NAME).o
