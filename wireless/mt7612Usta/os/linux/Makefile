obj_ap :=
obj_sta :=
obj_p2p :=
obj_wsc :=
obj_vht :=
obj_cmm := \
        ../../common/crypt_md5.o\
        ../../common/crypt_sha2.o\
        ../../common/crypt_hmac.o\
        ../../common/crypt_aes.o\
        ../../common/crypt_arc4.o\
        ../../common/mlme.o\
        ../../common/cmm_wep.o\
        ../../common/action.o\
        ../../common/cmm_data.o\
        ../../common/rtmp_init.o\
        ../../common/rtmp_init_inf.o\
        ../../common/cmm_tkip.o\
        ../../common/cmm_aes.o\
        ../../common/cmm_sync.o\
        ../../common/eeprom.o\
        ../../common/cmm_sanity.o\
        ../../common/cmm_info.o\
        ../../common/cmm_cfg.o\
        ../../common/cmm_wpa.o\
        ../../common/cmm_radar.o\
        ../../common/spectrum.o\
        ../../common/rtmp_timer.o\
        ../../common/rt_channel.o\
        ../../common/cmm_profile.o\
        ../../common/cmm_asic.o\
        ../../common/scan.o\
        ../../common/cmm_cmd.o\
	    ../../common/uapsd.o\
	    ../../common/ps.o\
	    ../../common/sys_log.o\
	    ../../common/txpower.o\
        ../../rate_ctrl/ra_ctrl.o\
        ../../rate_ctrl/alg_legacy.o\
        ../../rate_ctrl/alg_ags.o\
        ../../chips/rtmp_chip.o\
        ../../mgmt/mgmt_hw.o\
        ../../mgmt/mgmt_entrytb.o\
	    ../../tx_rx/wdev_tx.o \
	    ../../os/linux/rt_profile.o


obj_phy := ../../phy/phy.o	\
		   ../../phy/rf.o

obj_mac := ../../mac/rtmp_mac.o

ifeq ($(HAS_RTMP_BBP),y)
obj_phy += ../../phy/rtmp_phy.o
endif

ifeq ($(HAS_RLT_BBP),y)
obj_phy += ../../phy/rlt_phy.o
endif

ifeq ($(HAS_RLT_RF),y)
obj_phy += ../../phy/rlt_rf.o
endif

ifeq ($(HAS_MT76XX_BT_COEXISTENCE_SUPPORT),y)
obj_cmm += ../../mcu/bt_coex.o
endif

ifeq ($(HAS_RLT_MAC),y)
obj_mac += ../../mac/ral_nmac.o
endif

ifeq ($(HAS_RTMP_MAC),y)
obj_mac += ../../mac/ral_omac.o
endif

obj_cmm += $(obj_phy) $(obj_mac)

ifeq ($(HAS_BLOCK_NET_IF),y)
obj_cmm += ../../common/netif_block.o
endif

ifeq ($(HAS_NEW_RATE_ADAPT_SUPPORT),y)
obj_cmm += ../../rate_ctrl/alg_grp.o
endif

ifeq ($(HAS_RATE_ADAPT_AGS_SUPPORT),y)
obj_cmm += ../../rate_ctrl/alg_ags.o
endif

ifeq ($(HAS_DFS_SUPPORT),y)
obj_cmm += ../../common/cmm_dfs.o
endif

ifeq ($(HAS_CS_SUPPORT),y)
obj_cmm += ../../common/cmm_cs.o
endif


#ifdef ANDES_FIRMWARE_SUPPORT
ifeq ($(HAS_ANDES_FIRMWARE_SUPPORT),y)
obj_cmm += ../../mcu/rtmp_and.o
endif
#endif /* ANDES_FIRMWARE_SUPPORT */




#ifdef DOT11W_PMF_SUPPORT
ifeq ($(HAS_DOT11W_PMF_SUPPORT),y)
obj_cmm += ../../common/pmf.o
endif
#endif // DOT11W_PMF_SUPPORT //


#ifdef DOT11_N_SUPPORT
ifeq ($(HAS_DOT11_N_SUPPORT),y)
obj_cmm += \
        ../../common/ba_action.o\
        ../../mgmt/mgmt_ht.o

#ifdef TXBF_SUPPORT
ifeq ($(HAS_TXBF_SUPPORT),y)
obj_cmm += \
        ../../common/cmm_txbf.o\
        ../../common/cmm_txbf_cal.o
endif
#endif // TXBF_SUPPORT //
endif
#endif // DOT11_N_SUPPORT //

#ifdef DOT11_VHT_SUPPORT
ifeq ($(HAS_DOT11_VHT_SUPPORT),y)
obj_vht += ../../mgmt/mgmt_vht.o\
	../../common/vht.o
endif
#endif // DOT11_VHT_SUPPORT //








###############################################################################
#
# config for AP mode
#
###############################################################################

obj_ap += \
	../../ap/ap_mbss.o\
	../../ap/ap.o\
	../../ap/ap_assoc.o\
	../../ap/ap_auth.o\
	../../ap/ap_connect.o\
	../../ap/ap_mlme.o\
	../../ap/ap_sanity.o\
	../../ap/ap_sync.o\
	../../ap/ap_wpa.o\
	../../ap/ap_data.o\
	../../ap/ap_autoChSel.o\
	../../ap/ap_qload.o\
	../../ap/ap_cfg.o

ifeq ($(HAS_QOS_DLS_SUPPORT),y)
obj_ap += ../../ap/ap_dls.o
endif

ifeq ($(HAS_IDS_SUPPORT),y)
obj_ap += ../../ap/ap_ids.o
endif

ifeq ($(HAS_WSC),y)
obj_ap += ../../common/wsc_ufd.o
endif

obj_ap += \
    ../../ap/ap_mbss_inf.o\
	../../os/linux/ap_ioctl.o

ifeq ($(HAS_IGMP_SNOOP_SUPPORT),y)
obj_ap += ../../common/igmp_snoop.o
endif

MOD_NAME = $(MODULE)_ap
DAT_PATH = /etc/Wireless/RT$(CHIPSET_DAT)AP
DAT_FILE_NAME = conf/RT$(CHIPSET_DAT)AP.dat


###############################################################################
#
# config for STA mode
#
###############################################################################

obj_sta += \
	../../sta/assoc.o\
	../../sta/auth.o\
	../../sta/auth_rsp.o\
	../../sta/sync.o\
	../../sta/sanity.o\
	../../sta/rtmp_data.o\
	../../sta/connect.o\
	../../sta/wpa.o\
	../../sta/sta_cfg.o\
	../../sta/sta.o


obj_sta += ../../os/linux/sta_ioctl.o

MOD_NAME = $(MODULE)_sta
DAT_PATH = /etc/Wireless/RT$(CHIPSET_DAT)STA
DAT_FILE_NAME = conf/RT$(CHIPSET_DAT)STA.dat


###############################################################################
#
# config for AP/STA mixed mode
#
###############################################################################

#ifdef CONFIG_APSTA_SUPPORT
ifeq ($(RT28xx_MODE), APSTA)
MOD_NAME = $(MODULE)_apsta
endif
#endif // CONFIG_APSTA_SUPPORT //


###############################################################################
#
# Module Base
#
###############################################################################

obj-m := $(MOD_NAME).o

#ifdef CONFIG_AP_SUPPORT
ifeq ($(RT28xx_MODE),AP)

$(MOD_NAME)-objs := \
	$(obj_ap)\
	$(obj_vht)\
	$(obj_cmm)\
	$(obj_wsc)\
	$(obj_phy)

ifeq ($(HAS_ATE),y)
$(MOD_NAME)-objs += ../../ate/common/rt_ate.o
endif

ifeq ($(HAS_QA_SUPPORT),y)
$(MOD_NAME)-objs += ../../ate/common/rt_qa.o
endif

#ifdef MULTI_INF_SUPPORT
ifeq ($(HAS_MULTI_INF),y)
$(MOD_NAME)-objs += ../../os/linux/multi_main_dev.o
endif
#endif // MULTI_INF_SUPPORT //

#ifdef BG_FT_SUPPORT
ifeq ($(HAS_BGFP_SUPPORT),y)
$(MOD_NAME)-objs += \
	../../os/linux/br_ftph.o
endif
#endif // BG_FT_SUPPORT //

$(MOD_NAME)-objs += \
	../../common/rt_os_util.o\
	../../os/linux/rt_linux.o\
	../../os/linux/rt_main_dev.o

ifeq ($(HAS_SNIFFER_SUPPORT),y)
$(MOD_NAME)-objs += \
	../../sniffer/sniffer_prism.o	\
	../../sniffer/sniffer_radiotap.o
endif

#endif // CRDA_SUPPORT //

ifeq ($(HAS_RT2880_RT2860_COEXIST),y)
RT28XX_AP_OBJ += \
	../../os/linux/rt_pci_rbus.o\
	../../os/linux/rt_rbus_pci_util.o\
	../../os/linux/pci_main_dev.o\
	../../common/dfs.o
endif

endif
#endif // CONFIG_AP_SUPPORT //

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
	../../common/rt_os_util.o\
	../../os/linux/sta_ioctl.o\
	../../os/linux/rt_linux.o\
	../../os/linux/rt_main_dev.o

ifeq ($(HAS_SNIFFER_SUPPORT),y)
$(MOD_NAME)-objs += \
	../../sniffer/sniffer_prism.o\
	../../sniffer/sniffer_radiotap.o
endif

#ifdef ETH_CONVERT
ifeq ($(HAS_ETH_CONVERT_SUPPORT), y)
$(MOD_NAME)-objs += \
	../../common/cmm_mat.o \
	../../common/cmm_mat_iparp.o \
	../../common/cmm_mat_pppoe.o \
	../../common/cmm_mat_ipv6.o
endif
#endif // ETH_CONVERT //




ifeq ($(HAS_QOS_DLS_SUPPORT),y)
$(MOD_NAME)-objs += ../../sta/dls.o
endif



ifeq ($(HAS_ATE),y)
$(MOD_NAME)-objs += ../../ate/common/rt_ate.o
endif

ifeq ($(HAS_QA_SUPPORT),y)
$(MOD_NAME)-objs += ../../ate/common/rt_qa.o
endif

endif
#endif // CONFIG_STA_SUPPORT //


#ifdef CRDA_SUPPORT
ifeq ($(HAS_CFG80211_SUPPORT),y)
$(MOD_NAME)-objs += \
	../../os/linux/cfg80211/cfg80211.o\
	../../os/linux/cfg80211/cfg80211_util.o\
	../../os/linux/cfg80211/cfg80211_scan.o\
	../../os/linux/cfg80211/cfg80211_rx.o\
	../../os/linux/cfg80211/cfg80211_tx.o\
	../../os/linux/cfg80211/cfg80211_inf.o\
	../../os/linux/cfg80211/cfg80211_p2p.o\
	../../os/linux/cfg80211/cfg80211_ap.o\
	../../os/linux/cfg80211/cfg80211drv.o
endif

#ifdef CONFIG_APSTA_SUPPORT
ifeq ($(RT28xx_MODE), APSTA)
$(MOD_NAME)-objs := \
	$(obj_ap)\
	$(obj_sta)\
	$(obj_cmm)\
	$(obj_wsc)

endif
#endif // CONFIG_APSTA_SUPPORT //


#chip releated


#ifdef MT76x2
ifneq ($(or $(findstring mt7662e,$(CHIPSET)),$(findstring mt7612e,$(CHIPSET))),)
$(MOD_NAME)-objs += \
                ../../common/cmm_mac_pci.o\
                ../../common/cmm_data_pci.o\
                ../../os/linux/rt_rbus_pci_drv.o\
				../../common/ee_efuse.o\
                ../../common/ee_prom.o\
				../../chips/rt65xx.o\
                ../../chips/mt76x2.o\
				../../mac/ral_nmac.o\
				../../mcu/mcu.o\
				../../mcu/mcu_and.o\
				../../phy/rt_rf.o\
				../../phy/mt_rf.o

ifeq ($(HAS_RTMP_FLASH_SUPPORT),y)
$(MOD_NAME)-objs += \
        ../../common/ee_flash.o
endif

ifeq ($(HAS_ATE),y)
$(MOD_NAME)-objs += \
        ../../ate/chips/mt76x2_ate.o\
        ../../ate/common/ate_pci.o
endif

$(MOD_NAME)-objs += \
                ../../os/linux/rt_pci_rbus.o\
                ../../os/linux/rt_rbus_pci_util.o\
                ../../os/linux/pci_main_dev.o

ifneq ($(findstring $(RT28xx_MODE),STA APSTA),)
$(MOD_NAME)-objs += \
        ../../common/frq_cal.o
endif

endif

ifneq ($(or $(findstring mt7662u,$(CHIPSET)),$(findstring mt7612u,$(CHIPSET))),)
$(MOD_NAME)-objs += \
                ../../common/cmm_mac_usb.o\
                ../../common/cmm_data_usb.o\
				../../common/rtusb_io.o\
				../../common/rtusb_data.o\
				../../common/rtusb_bulk.o\
                ../../os/linux/rt_usb.o\
                ../../common/ee_prom.o\
				../../common/ee_efuse.o\
				../../chips/rt65xx.o\
                ../../chips/mt76x2.o\
                ../../mac/ral_nmac.o\
				../../mcu/mcu.o\
                ../../mcu/mcu_and.o\
				../../phy/rt_rf.o\
				../../phy/mt_rf.o

ifeq ($(HAS_RTMP_FLASH_SUPPORT),y)
$(MOD_NAME)-objs += \
        		../../common/ee_flash.o
endif

ifeq ($(HAS_ATE),y)
$(MOD_NAME)-objs += \
        ../../ate/chips/mt76x2_ate.o\
        ../../ate/common/ate_usb.o
endif

ifeq ($(HAS_TSO_SUPPORT),y)
$(MOD_NAME)-objs += \
				../../naf/net_acc.o\
				../../naf/cso.o
endif

ifeq ($(HAS_CSO_SUPPORT), y)
$(MOD_NAME)-objs += \
				../../naf/net_acc.o\
				../../naf/cso.o
endif

$(MOD_NAME)-objs += \
                ../../os/linux/rt_usb.o\
                ../../os/linux/rt_usb_util.o\
                ../../os/linux/usb_main_dev.o\
				../../common/rtusb_dev_id.o

ifneq ($(findstring $(RT28xx_MODE),STA APSTA),)
$(MOD_NAME)-objs += \
        ../../common/frq_cal.o
endif

endif
#endif // MT76x2 //

# Declare the contents of the .PHONY variable as phony.  We keep that
# information in a variable so we can use it in if_changed and friends.
.PHONY: $(PHONY)
