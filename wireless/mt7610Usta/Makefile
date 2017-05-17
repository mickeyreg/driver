ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif

RT28xx_MODE                       = STA
CHIPSET                           = 7610U
OSABL                             = NO

HAS_ATE                           = y
HAS_QA_SUPPORT                    = y
HAS_LED_CONTROL_SUPPORT           = y
HAS_KTHREAD_SUPPORT               = y

HAS_WPA_SUPPLICANT                = y
HAS_NATIVE_WPA_SUPPLICANT_SUPPORT = y
HAS_DOT11N_DRAFT3_SUPPORT         = y
HAS_DOT11_N_SUPPORT               = y
HAS_DOT11_VHT_SUPPORT             = y
HAS_STATS_COUNT                   = y
HAS_NEW_RATE_ADAPT_SUPPORT        = y

ifeq ($(OSABL),YES)
HAS_OSABL_FUNC_SUPPORT=y
HAS_OSABL_OS_PCI_SUPPORT=y
HAS_OSABL_OS_USB_SUPPORT=y
HAS_OSABL_OS_RBUS_SUPPORT=n
HAS_OSABL_OS_AP_SUPPORT=y
HAS_OSABL_OS_STA_SUPPORT=y
endif

HAS_TEMPERATURE_TX_ALC           = y

EXTRA_CFLAGS += -I$(src)/include
EXTRA_CFLAGS += -Wall -O2 -Wundef -Wstrict-prototypes
EXTRA_CFLAGS += -Wno-trigraphs -Wdeclaration-after-statement 
EXTRA_CFLAGS += -Wno-pointer-sign -fno-strict-aliasing -fno-common 
EXTRA_CFLAGS += -fomit-frame-pointer -ffreestanding -m4-nofpu

WFLAGS := -DAGGREGATION_SUPPORT -DPIGGYBACK_SUPPORT -DWMM_SUPPORT  -DLINUX -Wall -Wstrict-prototypes -Wno-trigraphs 
WFLAGS += -DSYSTEM_LOG_SUPPORT  -DRT28xx_MODE=$(RT28xx_MODE) -DCHIPSET=$(CHIPSET) -DRESOURCE_PRE_ALLOC -DENHANCED_STAT_DISPLAY -DST -D__KERNEL__
# WFLAGS := -g -DAGGREGATION_SUPPORT -DPIGGYBACK_SUPPORT -DWMM_SUPPORT  -DLINUX -Wall -Wstrict-prototypes -Wno-trigraphs
# WFLAGS += -DSYSTEM_LOG_SUPPORT -DRT28xx_MODE=$(RT28xx_MODE) -DCHIPSET=$(MODULE) -DRESOURCE_PRE_ALLOC -DENHANCED_STAT_DISPLAY

ifeq ($(HAS_WIFI_P2P_CONCURRENT_FAST_SCAN),y)
WFLAGS += -DWIFI_P2P_CONCURRENT_FAST_SCAN
endif

ifeq ($(HAS_HE_SUPPORT),y)
WFLAGS += -DHE_SUPPORT
WFLAGS += -DIP_ASSEMBLY
endif

ifeq ($(HAS_HE_BD_SUPPORT),y)
WFLAGS += -DHE_BD_SUPPORT
endif

ifeq ($(HAS_WEPAUTO_OPEN_FIRST_SUPPORT),y)
WFLAGS += -DWEPAUTO_OPEN_FIRST
endif

ifeq ($(HAS_HE_TV_SUPPORT),y)
WFLAGS += -DHE_TV_SUPPORT
endif

ifeq ($(HAS_KTHREAD_SUPPORT),y)
WFLAGS += -DKTHREAD_SUPPORT
endif

ifeq ($(HAS_RTMP_FLASH_SUPPORT),y)
WFLAGS += -DRTMP_FLASH_SUPPORT
endif

ifeq ($(HAS_STREAM_MODE_SUPPORT),y)
WFLAGS += -DSTREAM_MODE_SUPPORT
endif

ifeq ($(HAS_SINGLE_SKU_SUPPORT),y)
WFLAGS += -DSINGLE_SKU_V2
endif

ifeq ($(HAS_HDR_TRANS_SUPPORT),y)
WFLAGS += -DHDR_TRANS_SUPPORT
endif

ifeq ($(HAS_DOT11_VHT_SUPPORT),y)
WFLAGS += -DDOT11_VHT_AC
endif


ifeq ($(HAS_MAC_REPEATER_SUPPORT),y)
WFLAGS += -DMAC_REPEATER_SUPPORT
endif


#################################################

# config for STA mode

ifeq ($(RT28xx_MODE),STA)
WFLAGS += -DCONFIG_STA_SUPPORT -DSCAN_SUPPORT -DDBG

ifeq ($(HAS_HDR_TRANS_SUPPORT),y)
WFLAGS += -DHDR_TRANS_TX_SUPPORT
endif

ifeq ($(HAS_XLINK),y)
WFLAGS += -DXLINK_SUPPORT
endif


ifeq ($(HAS_WPA_SUPPLICANT),y)
WFLAGS += -DWPA_SUPPLICANT_SUPPORT
ifeq ($(HAS_NATIVE_WPA_SUPPLICANT_SUPPORT),y)
WFLAGS += -DNATIVE_WPA_SUPPLICANT_SUPPORT
endif
endif




ifeq ($(HAS_ATE),y)
WFLAGS += -DRALINK_ATE
WFLAGS += -DCONFIG_RT2880_ATE_CMD_NEW
WFLAGS += -I$(src)/ate/include
ifeq ($(HAS_QA_SUPPORT),y)
WFLAGS += -DRALINK_QA
endif
endif


ifeq ($(HAS_SNMP_SUPPORT),y)
WFLAGS += -DSNMP_SUPPORT
endif

ifeq ($(HAS_QOS_DLS_SUPPORT),y)
WFLAGS += -DQOS_DLS_SUPPORT
endif

ifeq ($(HAS_DOT11_N_SUPPORT),y)
WFLAGS += -DDOT11_N_SUPPORT

ifeq ($(HAS_DOT11N_DRAFT3_SUPPORT),y)
WFLAGS += -DDOT11N_DRAFT3
endif

ifeq ($(HAS_TXBF_SUPPORT),y)
WFLAGS += -DTXBF_SUPPORT
endif

ifeq ($(HAS_NEW_RATE_ADAPT_SUPPORT),y)
WFLAGS += -DNEW_RATE_ADAPT_SUPPORT
endif

endif


ifeq ($(HAS_WFD_SUPPORT),y)
HAS_P2P_SUPPORT=y
HAS_P2P_SPECIFIC_WIRELESS_EVENT=y
WFLAGS += -DWFD_SUPPORT
endif


ifeq ($(HAS_CS_SUPPORT),y)
WFLAGS += -DCARRIER_DETECTION_SUPPORT
endif

ifeq ($(HAS_STATS_COUNT),y)
WFLAGS += -DSTATS_COUNT_SUPPORT
endif

ifeq ($(HAS_TSSI_ANTENNA_VARIATION),y)
WFLAGS += -DTSSI_ANTENNA_VARIATION
endif

ifeq ($(HAS_ANDROID_SUPPORT),y)
WFLAGS += -DANDROID_SUPPORT
endif


ifeq ($(HAS_IFUP_IN_PROBE_SUPPORT),y)
WFLAGS += -DIFUP_IN_PROBE
endif

ifeq ($(HAS_USB_SUPPORT_SELECTIVE_SUSPEND),y)
WFLAGS += -DUSB_SUPPORT_SELECTIVE_SUSPEND
endif

ifeq ($(HAS_USB_FIRMWARE_MULTIBYTE_WRITE),y)
WFLAGS += -DUSB_FIRMWARE_MULTIBYTE_WRITE -DMULTIWRITE_BYTES=4
endif

ifeq ($(HAS_CFG80211_SUPPORT),y)
WFLAGS += -DRT_CFG80211_SUPPORT -DEXT_BUILD_CHANNEL_LIST
ifeq ($(HAS_RFKILL_HW_SUPPORT),y)
WFLAGS += -DRFKILL_HW_SUPPORT
endif
endif

ifeq ($(OSABL),YES)
WFLAGS += -DOS_ABL_SUPPORT
ifeq ($(HAS_OSABL_FUNC_SUPPORT),y)
WFLAGS += -DOS_ABL_FUNC_SUPPORT
endif
ifeq ($(HAS_OSABL_OS_PCI_SUPPORT),y)
WFLAGS += -DOS_ABL_OS_PCI_SUPPORT
endif
ifeq ($(HAS_OSABL_OS_USB_SUPPORT),y)
WFLAGS += -DOS_ABL_OS_USB_SUPPORT
endif
ifeq ($(HAS_OSABL_OS_RBUS_SUPPORT),y)
WFLAGS += -DOS_ABL_OS_RBUS_SUPPORT
endif
ifeq ($(HAS_OSABL_OS_AP_SUPPORT),y)
WFLAGS += -DOS_ABL_OS_AP_SUPPORT
endif
ifeq ($(HAS_OSABL_OS_STA_SUPPORT),y)
WFLAGS += -DOS_ABL_OS_STA_SUPPORT
endif
endif


ifeq ($(HAS_TXRX_SW_ANTDIV_SUPPORT),y)
WFLAGS += -DTXRX_SW_ANTDIV_SUPPORT
endif


ifeq ($(HAS_WIDI_SUPPORT),y)
WFLAGS += -DWIDI_SUPPORT

ifeq ($(HAS_P2P_SUPPORT),y)
ifeq ($(HAS_INTEL_WFD_SUPPORT),y)
WFLAGS += -DINTEL_WFD_SUPPORT
endif

ifeq ($(HAS_WFA_WFD_SUPPORT),y)
WFLAGS += -DWFA_WFD_SUPPORT
endif
endif

endif

ifeq ($(HAS_WOW_SUPPORT),y)
WFLAGS += -DWOW_SUPPORT
endif

ifeq ($(HAS_WOW_IFDOWN_SUPPORT),y)
WFLAGS += -DWOW_IFDOWN_SUPPORT
endif

endif
# endif of ifeq ($(RT28xx_MODE),STA)

#################################################

#################################################

#
# Common compiler flag
#






ifeq ($(HAS_EXT_BUILD_CHANNEL_LIST),y)
WFLAGS += -DEXT_BUILD_CHANNEL_LIST
endif

ifeq ($(HAS_IDS_SUPPORT),y)
WFLAGS += -DIDS_SUPPORT
endif


ifeq ($(OSABL),YES)
WFLAGS += -DEXPORT_SYMTAB
endif

ifeq ($(HAS_CLIENT_WDS_SUPPORT),y)
WFLAGS += -DCLIENT_WDS
endif

ifeq ($(HAS_BGFP_SUPPORT),y)
WFLAGS += -DBG_FT_SUPPORT
endif

ifeq ($(HAS_BGFP_OPEN_SUPPORT),y)
WFLAGS += -DBG_FT_OPEN_SUPPORT
endif


ifeq ($(HAS_LED_CONTROL_SUPPORT),y)
WFLAGS += -DLED_CONTROL_SUPPORT
endif

ifeq ($(HAS_FPGA_MODE),y)
WFLAGS += -DCONFIG_FPGA_MODE
endif

ifeq ($(HAS_WIFI_TEST),y)
WFLAGS += -DCONFIG_WIFI_TEST
endif


ifeq ($(HAS_TEMPERATURE_TX_ALC),y)
WFLAGS += -DRTMP_TEMPERATURE_TX_ALC
endif

#################################################
# ChipSet specific definitions.
#


ifneq ($(findstring 7610U,$(CHIPSET)),)
WFLAGS += -DMT76x0 -DRT65xx -DRLT_MAC -DRLT_RF -DRTMP_MAC_USB -DRTMP_USB_SUPPORT -DRTMP_TIMER_TASK_SUPPORT -DA_BAND_SUPPORT -DRTMP_EFUSE_SUPPORT -DNEW_MBSSID_MODE -DCONFIG_ANDES_SUPPORT
#-DRTMP_FREQ_CALIBRATION_SUPPORT
#-DRX_DMA_SCATTER
WFLAGS += -DMT7610

ifneq ($(findstring $(RT28xx_MODE),AP),)
#WFLAGS += -DSPECIFIC_BCN_BUF_SUPPORT
endif

ifeq ($(HAS_CSO_SUPPORT), y)
WFLAGS += -DCONFIG_CSO_SUPPORT -DCONFIG_TSO_SUPPORT
endif

CHIPSET_DAT = 2860
# CHIPSET_DAT = 2870
endif


#################################################

EXTRA_CFLAGS += -o $(WFLAGS) 

# MOD_NAME = mt$(CHIPSET)sta

obj_wsc :=
obj_vht :=
obj_cmm := \
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

ifeq ($(HAS_BLOCK_NET_IF),y)
obj_cmm += common/netif_block.o
endif

ifeq ($(HAS_NEW_RATE_ADAPT_SUPPORT),y)
obj_cmm += rate_ctrl/alg_grp.o
endif

ifeq ($(HAS_RATE_ADAPT_AGS_SUPPORT),y)
obj_cmm += rate_ctrl/alg_ags.o
endif

ifeq ($(HAS_DFS_SUPPORT),y)
obj_cmm += common/cmm_dfs.o
endif

ifeq ($(HAS_CS_SUPPORT),y)
obj_cmm += common/cmm_cs.o
endif

#ifdef DOT11_N_SUPPORT
ifeq ($(HAS_DOT11_N_SUPPORT),y)
obj_cmm += \
        common/ba_action.o\
        mgmt/mgmt_ht.o

#ifdef TXBF_SUPPORT
ifeq ($(HAS_TXBF_SUPPORT),y)
obj_cmm += \
        common/cmm_txbf.o\
        common/cmm_txbf_cal.o
endif
#endif // TXBF_SUPPORT //
endif
#endif // DOT11_N_SUPPORT //

#ifdef DOT11_VHT_SUPPORT
ifeq ($(HAS_DOT11_VHT_SUPPORT),y)
obj_vht += mgmt/mgmt_vht.o\
	common/vht.o
endif
#endif // DOT11_VHT_SUPPORT //


#ifdef CONFIG_STA_SUPPORT
ifeq ($(RT28xx_MODE), STA)
MOD_NAME = mt$(CHIPSET)sta
# MOD_NAME = $(MODULE)_sta
ifeq ($(PLATFORM),MT85XX)
MOD_NAME = rtsta
endif
ifeq ($(PLATFORM),MT53XX)
MOD_NAME = rtsta
endif
DAT_PATH = /etc/Wireless/RT$(CHIPSET_DAT)STA
DAT_FILE_NAME = conf/RT$(CHIPSET_DAT)STA.dat
endif
#endif // CONFIG_STA_SUPPORT //

obj-m := $(MOD_NAME).o


#ifdef CONFIG_STA_SUPPORT
ifeq ($(RT28xx_MODE), STA)
$(MOD_NAME)-objs := \
	os/linux/rt_profile.o\
	sta/assoc.o\
	sta/auth.o\
	sta/auth_rsp.o\
	sta/sync.o\
	sta/sanity.o\
	sta/rtmp_data.o\
	sta/connect.o\
	sta/wpa.o\
	sta/sta_cfg.o\
	$(obj_vht)\
	$(obj_cmm)\
	$(obj_wsc)


ifeq ($(OSABL),NO)
$(MOD_NAME)-objs += \
	common/rt_os_util.o\
	os/linux/sta_ioctl.o\
	os/linux/rt_linux.o\
	os/linux/rt_main_dev.o
else
$(MOD_NAME)-objs += \
	os/linux/rt_symb.o
endif

#ifdef ETH_CONVERT
ifeq ($(HAS_ETH_CONVERT_SUPPORT), y)
$(MOD_NAME)-objs += \
	common/cmm_mat.o \
	common/cmm_mat_iparp.o \
	common/cmm_mat_pppoe.o \
	common/cmm_mat_ipv6.o
endif
#endif // ETH_CONVERT //

ifeq ($(HAS_QOS_DLS_SUPPORT),y)
$(MOD_NAME)-objs += sta/dls.o
endif

#ifdef WFD_SUPPORT
ifeq ($(HAS_WFD_SUPPORT),y)
$(MOD_NAME)-objs += \
	common/wfd.o
endif
#endif // WFD_SUPPORT //

#ifdef LED_CONTROL_SUPPORT
ifeq ($(HAS_LED_CONTROL_SUPPORT),y)
$(MOD_NAME)-objs += \
	common/rt_led.o
endif
#endif // LED_CONTROL_SUPPORT //

ifeq ($(HAS_ATE),y)
$(MOD_NAME)-objs += ate/common/rt_ate.o
endif

ifeq ($(HAS_QA_SUPPORT),y)
$(MOD_NAME)-objs += ate/common/rt_qa.o
endif

#ifdef CRDA_SUPPORT
ifeq ($(OSABL),NO)
ifeq ($(HAS_CFG80211_SUPPORT),y)
$(MOD_NAME)-objs += \
	os/linux/cfg80211.o \
	os/linux/cfg80211drv.o
endif
endif

ifeq ($(OSABL),YES)
ifeq ($(HAS_CFG80211_SUPPORT),y)
$(MOD_NAME)-objs += \
	os/linux/cfg80211drv.o
endif
endif
#endif // CRDA_SUPPORT //

endif
#endif // CONFIG_STA_SUPPORT //

#chip releated

#ifdef MT76x0
ifneq ($(findstring 7610U,$(CHIPSET)),)
$(MOD_NAME)-objs += \
                common/cmm_mac_usb.o\
                common/cmm_data_usb.o\
				common/rtusb_io.o\
				common/rtusb_data.o\
				common/rtusb_bulk.o\
                os/linux/rt_usb.o\
                common/ee_prom.o\
				common/ee_efuse.o\
                common/rt_rf.o\
				chips/rt65xx.o\
                chips/mt76x0.o\
                mac/ral_nmac.o\
				mcu/mcu.o\
                mcu/mcu_and.o

ifeq ($(HAS_RTMP_FLASH_SUPPORT),y)
$(MOD_NAME)-objs += \
        		common/ee_flash.o
endif

ifeq ($(HAS_TSO_SUPPORT),y)
$(MOD_NAME)-objs += \
				naf/net_acc.o\
				naf/cso.o
endif

ifeq ($(HAS_CSO_SUPPORT), y)
$(MOD_NAME)-objs += \
				naf/net_acc.o\
				naf/cso.o
endif

ifeq ($(HAS_ATE),y)
$(MOD_NAME)-objs += \
	ate/chips/mt76x0_ate.o\
	ate/common/ate_usb.o
endif

ifeq ($(OSABL),NO)
$(MOD_NAME)-objs += \
                os/linux/rt_usb.o\
                os/linux/rt_usb_util.o\
                os/linux/usb_main_dev.o\
				common/rtusb_dev_id.o
endif

ifneq ($(findstring $(RT28xx_MODE),STA APSTA),)
$(MOD_NAME)-objs += \
        common/frq_cal.o
endif

endif
#endif // MT76x0 //












obj-m := $(MOD_NAME).o
