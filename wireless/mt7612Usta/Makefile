ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif

RT28xx_MODE                       = STA
CHIPSET                           = 7612U
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
HAS_TXBF_SUPPORT                  = y
HAS_VHT_TXBF_SUPPORT              = y
HAS_RX_CSO_SUPPORT                = y
# HAS_CSO_SUPPORT                 = y

ifeq ($(OSABL),YES)
HAS_OSABL_FUNC_SUPPORT=y
HAS_OSABL_OS_PCI_SUPPORT=y
HAS_OSABL_OS_USB_SUPPORT=y
HAS_OSABL_OS_RBUS_SUPPORT=n
HAS_OSABL_OS_AP_SUPPORT=y
HAS_OSABL_OS_STA_SUPPORT=y
endif

HAS_CAL_FREE_IC_SUPPORT           = y

# HAS_NEW_RATE_ADAPT_SUPPORT      = y
# HAS_RLT_BBP                     = y
# HAS_RLT_MAC                     = y

EXTRA_CFLAGS += -I$(src)/include
EXTRA_CFLAGS += -Wall -O2 -Wundef -Wstrict-prototypes
EXTRA_CFLAGS += -Wno-trigraphs -Wdeclaration-after-statement
EXTRA_CFLAGS += -Wno-pointer-sign -fno-strict-aliasing -fno-common
EXTRA_CFLAGS += -fomit-frame-pointer -ffreestanding -m4-nofpu

WFLAGS := -DAGGREGATION_SUPPORT -DPIGGYBACK_SUPPORT -DWMM_SUPPORT  -DLINUX -Wall -Wstrict-prototypes -Wno-trigraphs -DENHANCED_STAT_DISPLAY -DCONFIG_RA_NAT_NONE
WFLAGS += -DSYSTEM_LOG_SUPPORT  -DRT28xx_MODE=$(RT28xx_MODE) -DCHIPSET=$(CHIPSET) -DRESOURCE_PRE_ALLOC -DST -D__KERNEL__ -DDBG_DIAGNOSE -DDBG_RX_MCS -DDBG_TX_MCS

# WFLAGS := -g -DAGGREGATION_SUPPORT -DPIGGYBACK_SUPPORT -DWMM_SUPPORT  -DLINUX -Wall -Wstrict-prototypes -Wno-trigraphs -DENHANCED_STAT_DISPLAY
# WFLAGS += -DSYSTEM_LOG_SUPPORT -DRT28xx_MODE=$(RT28xx_MODE) -DCHIPSET=$(CHIPSET) -DRESOURCE_PRE_ALLOC -DDBG_DIAGNOSE -DDBG_RX_MCS -DDBG_TX_MCS
# WFLAGS += -DCONFIG_RA_NAT_NONE

ifeq ($(HAS_MULTI_INF),y)
WFLAGS += -DMULTI_INF_SUPPORT
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
WFLAGS += -DSINGLE_SKU
endif

ifeq ($(HAS_SINGLE_SKU_V2_SUPPORT),y)
WFLAGS += -DSINGLE_SKU_V2
endif

ifeq ($(HAS_DOT11_VHT_SUPPORT),y)
WFLAGS += -DDOT11_VHT_AC
endif

ifeq ($(HAS_ANDES_FIRMWARE_SUPPORT),y)
WFLAGS += -DANDES_FIRMWARE_SUPPORT
endif

ifeq ($(HAS_HDR_TRANS_SUPPORT),y)
WFLAGS += -DHDR_TRANS_SUPPORT
endif


ifeq ($(HAS_ATE),y)
WFLAGS += -DRALINK_ATE
WFLAGS += -DCONFIG_RT2880_ATE_CMD_NEW
WFLAGS += -I$(src)/ate/include
endif

ifeq ($(HAS_CAL_FREE_IC_SUPPORT),y)
WFLAGS += -DCAL_FREE_IC_SUPPORT
endif

ifeq ($(HAS_TEMPERATURE_TX_ALC),y)
WFLAGS += -DRTMP_TEMPERATURE_TX_ALC
endif

########################################################
#
# config for STA mode
#
########################################################

ifeq ($(RT28xx_MODE),STA)
WFLAGS += -DCONFIG_STA_SUPPORT -DSCAN_SUPPORT -DDBG

ifeq ($(HAS_MT76XX_BT_COEXISTENCE_SUPPORT),y)
WFLAGS += -DMT76XX_BTCOEX_SUPPORT
endif
ifeq ($(HAS_WPA_SUPPLICANT),y)
WFLAGS += -DWPA_SUPPLICANT_SUPPORT
ifeq ($(HAS_NATIVE_WPA_SUPPLICANT_SUPPORT),y)
WFLAGS += -DNATIVE_WPA_SUPPLICANT_SUPPORT
endif
endif

ifeq ($(HAS_SNMP_SUPPORT),y)
WFLAGS += -DSNMP_SUPPORT
endif

ifeq ($(HAS_DOT11_N_SUPPORT),y)
WFLAGS += -DDOT11_N_SUPPORT

ifeq ($(HAS_DOT11N_DRAFT3_SUPPORT),y)
WFLAGS += -DDOT11N_DRAFT3
endif

ifeq ($(HAS_TXBF_SUPPORT),y)
WFLAGS += -DTXBF_SUPPORT
endif

ifeq ($(HAS_VHT_TXBF_SUPPORT),y)
WFLAGS += -DVHT_TXBF_SUPPORT
endif

ifeq ($(HAS_NEW_RATE_ADAPT_SUPPORT),y)
WFLAGS += -DNEW_RATE_ADAPT_SUPPORT
endif

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
WFLAGS += -DRT_CFG80211_SUPPORT -DEXT_BUILD_CHANNEL_LIST -DWPA_SUPPLICANT_SUPPORT
ifeq ($(HAS_RFKILL_HW_SUPPORT),y)
WFLAGS += -DRFKILL_HW_SUPPORT
endif
ifeq ($(HAS_CFG80211_SCAN_SIGNAL_AVG_SUPPORT),y)
WFLAGS += -DCFG80211_SCAN_SIGNAL_AVG
endif
ifeq ($(HAS_CFG80211_P2P_SUPPORT),y)
WFLAGS += -DRT_CFG80211_P2P_SUPPORT -DUAPSD_SUPPORT -DUAPSD_SP_ACCURATE -DMBSS_SUPPORT -DAP_SCAN_SUPPORT
WFLAGS += -DCONFIG_AP_SUPPORT -DAPCLI_SUPPORT
ifeq ($(HAS_CFG80211_P2P_SINGLE_DEVICE),y)
WFLAGS += -DRT_CFG80211_P2P_SINGLE_DEVICE
else
ifeq ($(HAS_CFG80211_P2P_STATIC_CONCURRENT_DEVICE),y)
WFLAGS += -DRT_CFG80211_P2P_STATIC_CONCURRENT_DEVICE
else
WFLAGS += -DRT_CFG80211_P2P_CONCURRENT_DEVICE
ifeq ($(HAS_CFG80211_P2P_MULTI_CHAN_SUPPORT),y)
WFLAGS += -DRT_CFG80211_P2P_MULTI_CHAN_SUPPORT
endif #HAS_CFG80211_P2P_MULTI_CHAN_SUPPORT
endif #HAS_CFG80211_P2P_STATIC_CONCURRENT_DEVICE
endif #HAS_CFG80211_P2P_SINGLE_DEVICE
endif #HAS_CFG80211_P2P_SUPPORT
ifeq ($(HAS_CFG80211_ANDROID_PRIV_LIB_SUPPORT),y)
WFLAGS += -DRT_RT_CFG80211_ANDROID_PRIV_LIB_SUPPORT
endif #HAS_CFG80211_ANDROID_PRIV_LIB_SUPPORT
ifeq ($(HAS_CFG80211_TDLS_SUPPORT),y)
WFLAGS += -DCFG_TDLS_SUPPORT -DUAPSD_SUPPORT
endif
endif

ifeq ($(HAS_TXRX_SW_ANTDIV_SUPPORT),y)
WFLAGS += -DTXRX_SW_ANTDIV_SUPPORT
endif

ifeq ($(HAS_WIDI_SUPPORT),y)
WFLAGS += -DWIDI_SUPPORT

ifeq ($(HAS_INTEL_L2SD_TOGGLE_SCAN_SUPPORT),y)
WFLAGS += -DINTEL_L2SD_TOGGLE_SCAN_SUPPORT
endif

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

ifeq ($(HAS_NEW_WOW_SUPPORT),y)
WFLAGS += -DNEW_WOW_SUPPORT
endif

endif
# endif of ifeq ($(RT28xx_MODE),STA)

##########################################################
#
# Common compiler flag
#
##########################################################

ifeq ($(HAS_EXT_BUILD_CHANNEL_LIST),y)
WFLAGS += -DEXT_BUILD_CHANNEL_LIST
endif

ifeq ($(HAS_BGFP_SUPPORT),y)
WFLAGS += -DBG_FT_SUPPORT
endif

ifeq ($(HAS_BGFP_OPEN_SUPPORT),y)
WFLAGS += -DBG_FT_OPEN_SUPPORT
endif

ifeq ($(HAS_DOT11W_PMF_SUPPORT),y)
WFLAGS += -DDOT11W_PMF_SUPPORT -DSOFT_ENCRYPT
endif

ifeq ($(HAS_MICROWAVE_OVEN_SUPPORT),y)
WFLAGS += -DMICROWAVE_OVEN_SUPPORT
endif

#################################################
#
# ChipSet specific definitions.
#
#################################################

# MT7662U
ifneq ($(findstring 7612U,$(CHIPSET)),)
WFLAGS += -DMT76x2 -DRT65xx -DRLT_MAC -DRLT_BBP -DMT_RF -DRTMP_MAC_USB -DRTMP_USB_SUPPORT -DRTMP_TIMER_TASK_SUPPORT -DA_BAND_SUPPORT -DRTMP_EFUSE_SUPPORT -DNEW_MBSSID_MODE -DCONFIG_ANDES_SUPPORT -DRTMP_RF_RW_SUPPORT -DDYNAMIC_VGA_SUPPORT
HAS_NEW_RATE_ADAPT_SUPPORT=y
ifeq ($(HAS_NEW_RATE_ADAPT_SUPPORT),y)
WFLAGS += -DNEW_RATE_ADAPT_SUPPORT
endif
WFLAGS += -DFIFO_EXT_SUPPORT
HAS_RLT_BBP=y
HAS_RLT_MAC=y

ifneq ($(findstring 7612U,$(CHIPSET)),)
WFLAGS += -DMT7612
endif

ifeq ($(HAS_CSO_SUPPORT), y)
WFLAGS += -DCONFIG_CSO_SUPPORT -DCONFIG_TSO_SUPPORT
endif

ifneq ($(findstring $(RT28xx_MODE),STA APSTA),)
WFLAGS += -DRTMP_FREQ_CALIBRATION_SUPPORT
endif

CHIPSET_DAT = 2870
endif
## End of 7662U ##

#################################################
# Platform Related definitions
#
#################################################

ifeq ($(HAS_BLOCK_NET_IF),y)
WFLAGS += -DBLOCK_NET_IF
endif

ifeq ($(HAS_DFS_SUPPORT),y)
WFLAGS += -DDFS_SUPPORT
endif

ifeq ($(HAS_MC_SUPPORT),y)
WFLAGS += -DMULTIPLE_CARD_SUPPORT
endif

ifeq ($(HAS_LLTD),y)
WFLAGS += -DLLTD_SUPPORT
endif

EXTRA_CFLAGS += $(WFLAGS)

#OS ABL - YES or NO
OSABL = NO

#RT28xx_DIR = home directory of RT28xx source code
RT28xx_DIR = $(shell pwd)

PLATFORM = PC

#APSOC

#RELEASE Package
RELEASE = DPOA

obj_ap :=
obj_sta :=
obj_p2p :=
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
	common/sys_log.o\
	common/txpower.o\
	rate_ctrl/ra_ctrl.o\
	rate_ctrl/alg_legacy.o\
	rate_ctrl/alg_ags.o\
	chips/rtmp_chip.o\
	mgmt/mgmt_hw.o\
	mgmt/mgmt_entrytb.o\
	tx_rx/wdev_tx.o \
	os/linux/rt_profile.o


obj_phy := phy/phy.o	\
	   phy/rf.o

obj_mac := mac/rtmp_mac.o

ifeq ($(HAS_RTMP_BBP),y)
obj_phy += phy/rtmp_phy.o
endif

ifeq ($(HAS_RLT_BBP),y)
obj_phy += phy/rlt_phy.o
endif

ifeq ($(HAS_RLT_RF),y)
obj_phy += phy/rlt_rf.o
endif

ifeq ($(HAS_MT76XX_BT_COEXISTENCE_SUPPORT),y)
obj_cmm += mcu/bt_coex.o
endif

ifeq ($(HAS_RLT_MAC),y)
obj_mac += mac/ral_nmac.o
endif

ifeq ($(HAS_RTMP_MAC),y)
obj_mac += mac/ral_omac.o
endif

obj_cmm += $(obj_phy) $(obj_mac)

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

#ifdef ANDES_FIRMWARE_SUPPORT
ifeq ($(HAS_ANDES_FIRMWARE_SUPPORT),y)
obj_cmm += mcu/rtmp_and.o
endif
#endif /* ANDES_FIRMWARE_SUPPORT */

#ifdef DOT11W_PMF_SUPPORT
ifeq ($(HAS_DOT11W_PMF_SUPPORT),y)
obj_cmm += common/pmf.o
endif
#endif // DOT11W_PMF_SUPPORT //


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

###############################################################################
#
# config for STA mode
#
###############################################################################

obj_sta += \
	sta/assoc.o\
	sta/auth.o\
	sta/auth_rsp.o\
	sta/sync.o\
	sta/sanity.o\
	sta/rtmp_data.o\
	sta/connect.o\
	sta/wpa.o\
	sta/sta_cfg.o\
	sta/sta.o


obj_sta += os/linux/sta_ioctl.o

MOD_NAME = mt$(CHIPSET)sta

###############################################################################
#
# Module Base
#
###############################################################################

obj-m := $(MOD_NAME).o

#ifdef CONFIG_STA_SUPPORT
ifeq ($(RT28xx_MODE), STA)

$(MOD_NAME)-objs := \
	$(obj_sta)\
	$(obj_p2p)\
	$(obj_vht)\
	$(obj_cmm)\
	$(obj_wsc)\
	$(obj_phy)

$(MOD_NAME)-objs += \
	common/rt_os_util.o\
	os/linux/sta_ioctl.o\
	os/linux/rt_linux.o\
	os/linux/rt_main_dev.o

ifeq ($(HAS_SNIFFER_SUPPORT),y)
$(MOD_NAME)-objs += \
	sniffer/sniffer_prism.o\
	sniffer/sniffer_radiotap.o
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

ifeq ($(HAS_ATE),y)
$(MOD_NAME)-objs += ate/common/rt_ate.o
endif

ifeq ($(HAS_QA_SUPPORT),y)
$(MOD_NAME)-objs += ate/common/rt_qa.o
endif

endif
#endif // CONFIG_STA_SUPPORT //


#ifdef CRDA_SUPPORT
ifeq ($(HAS_CFG80211_SUPPORT),y)
$(MOD_NAME)-objs += \
	os/linux/cfg80211/cfg80211.o\
	os/linux/cfg80211/cfg80211_util.o\
	os/linux/cfg80211/cfg80211_scan.o\
	os/linux/cfg80211/cfg80211_rx.o\
	os/linux/cfg80211/cfg80211_tx.o\
	os/linux/cfg80211/cfg80211_inf.o\
	os/linux/cfg80211/cfg80211_p2p.o\
	os/linux/cfg80211/cfg80211_ap.o\
	os/linux/cfg80211/cfg80211drv.o
endif

#chip releated

#ifdef MT76x2
ifneq ($(or $(findstring mt7662e,$(CHIPSET)),$(findstring mt7612e,$(CHIPSET))),)
$(MOD_NAME)-objs += \
                common/cmm_mac_pci.o\
                common/cmm_data_pci.o\
                os/linux/rt_rbus_pci_drv.o\
				common/ee_efuse.o\
                common/ee_prom.o\
				chips/rt65xx.o\
                chips/mt76x2.o\
				mac/ral_nmac.o\
				mcu/mcu.o\
				mcu/mcu_and.o\
				phy/rt_rf.o\
				phy/mt_rf.o

ifeq ($(HAS_RTMP_FLASH_SUPPORT),y)
$(MOD_NAME)-objs += \
        common/ee_flash.o
endif

ifeq ($(HAS_ATE),y)
$(MOD_NAME)-objs += \
        ate/chips/mt76x2_ate.o\
        ate/common/ate_pci.o
endif

$(MOD_NAME)-objs += \
                os/linux/rt_pci_rbus.o\
                os/linux/rt_rbus_pci_util.o\
                os/linux/pci_main_dev.o

ifneq ($(findstring $(RT28xx_MODE),STA APSTA),)
$(MOD_NAME)-objs += \
        common/frq_cal.o
endif

endif

ifneq ($(findstring 7612U,$(CHIPSET)),)
$(MOD_NAME)-objs += \
	common/cmm_mac_usb.o\
	common/cmm_data_usb.o\
	common/rtusb_io.o\
	common/rtusb_data.o\
	common/rtusb_bulk.o\
	os/linux/rt_usb.o\
	common/ee_prom.o\
	common/ee_efuse.o\
	chips/rt65xx.o\
	chips/mt76x2.o\
	mac/ral_nmac.o\
	mcu/mcu.o\
	mcu/mcu_and.o\
	phy/rt_rf.o\
	phy/mt_rf.o

ifeq ($(HAS_RTMP_FLASH_SUPPORT),y)
$(MOD_NAME)-objs += \
	common/ee_flash.o
endif

ifeq ($(HAS_ATE),y)
$(MOD_NAME)-objs += \
        ate/chips/mt76x2_ate.o\
        ate/common/ate_usb.o
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

$(MOD_NAME)-objs += \
	os/linux/rt_usb.o\
	os/linux/rt_usb_util.o\
	os/linux/usb_main_dev.o\
	common/rtusb_dev_id.o

ifneq ($(findstring $(RT28xx_MODE),STA APSTA),)
$(MOD_NAME)-objs += \
        common/frq_cal.o
endif

endif
#endif // MT76x2 //

