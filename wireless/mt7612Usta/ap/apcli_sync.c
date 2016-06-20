/*
 ***************************************************************************
 * Ralink Tech Inc.
 * 4F, No. 2 Technology 5th Rd.
 * Science-based Industrial Park
 * Hsin-chu, Taiwan, R.O.C.
 *
 * (c) Copyright 2002-2006, Ralink Technology, Inc.
 *
 * All rights reserved.	Ralink's source	code is	an unpublished work	and	the
 * use of a	copyright notice does not imply	otherwise. This	source code
 * contains	confidential trade secret material of Ralink Tech. Any attemp
 * or participation	in deciphering,	decoding, reverse engineering or in	any
 * way altering	the	source code	is stricitly prohibited, unless	the	prior
 * written consent of Ralink Technology, Inc. is obtained.
 ***************************************************************************

	Module Name:
	sta_sync.c

	Abstract:

	Revision History:
	Who			When			What
	--------	----------		----------------------------------------------
	Fonchi		2006-06-23      modified for rt61-APClinent
*/

#ifdef APCLI_SUPPORT

#include "rt_config.h"

static VOID ApCliProbeTimeout(
	IN PVOID SystemSpecific1, 
	IN PVOID FunctionContext, 
	IN PVOID SystemSpecific2, 
	IN PVOID SystemSpecific3);

static VOID ApCliMlmeProbeReqAction(
	IN PRTMP_ADAPTER pAd,
	IN MLME_QUEUE_ELEM *Elem);

static VOID ApCliPeerProbeRspAtJoinAction(
	IN PRTMP_ADAPTER pAd, 
	IN MLME_QUEUE_ELEM *Elem);

static VOID ApCliProbeTimeoutAtJoinAction(
	IN PRTMP_ADAPTER pAd,
	IN MLME_QUEUE_ELEM *Elem);

static VOID ApCliInvalidStateWhenJoin(
	IN PRTMP_ADAPTER pAd, 
	IN MLME_QUEUE_ELEM *Elem);

static VOID ApCliEnqueueProbeRequest(
	IN PRTMP_ADAPTER pAd,
	IN UCHAR SsidLen,
	OUT PCHAR Ssid,
	IN USHORT ifIndex);

DECLARE_TIMER_FUNCTION(ApCliProbeTimeout);
BUILD_TIMER_FUNCTION(ApCliProbeTimeout);

/*
    ==========================================================================
    Description:
        The sync state machine, 
    Parameters:
        Sm - pointer to the state machine
    Note:
        the state machine looks like the following
    ==========================================================================
 */
VOID ApCliSyncStateMachineInit(
	IN PRTMP_ADAPTER pAd,
	IN STATE_MACHINE *Sm,
	OUT STATE_MACHINE_FUNC Trans[])
{
	UCHAR i;
#ifdef APCLI_CONNECTION_TRIAL		
	PAPCLI_STRUCT	pApCliEntry;
#endif /*APCLI_CONNECTION_TRIAL*/

	StateMachineInit(Sm, (STATE_MACHINE_FUNC*)Trans,
		APCLI_MAX_SYNC_STATE, APCLI_MAX_SYNC_MSG,
		(STATE_MACHINE_FUNC)Drop, APCLI_SYNC_IDLE,
		APCLI_SYNC_MACHINE_BASE);

	/* column 1 */
	StateMachineSetAction(Sm, APCLI_SYNC_IDLE, APCLI_MT2_MLME_PROBE_REQ, (STATE_MACHINE_FUNC)ApCliMlmeProbeReqAction);

	/*column 2 */
	StateMachineSetAction(Sm, APCLI_JOIN_WAIT_PROBE_RSP, APCLI_MT2_MLME_PROBE_REQ, (STATE_MACHINE_FUNC)ApCliInvalidStateWhenJoin);
	StateMachineSetAction(Sm, APCLI_JOIN_WAIT_PROBE_RSP, APCLI_MT2_PEER_PROBE_RSP, (STATE_MACHINE_FUNC)ApCliPeerProbeRspAtJoinAction);
	StateMachineSetAction(Sm, APCLI_JOIN_WAIT_PROBE_RSP, APCLI_MT2_PROBE_TIMEOUT, (STATE_MACHINE_FUNC)ApCliProbeTimeoutAtJoinAction);
	StateMachineSetAction(Sm, APCLI_JOIN_WAIT_PROBE_RSP, APCLI_MT2_PEER_BEACON, (STATE_MACHINE_FUNC)ApCliPeerProbeRspAtJoinAction);


	for (i = 0; i < MAX_APCLI_NUM; i++)
	{
		/* timer init */
#ifdef APCLI_CONNECTION_TRIAL		
		pApCliEntry = &pAd->ApCfg.ApCliTab[i];
#endif /*APCLI_CONNECTION_TRIAL*/

#ifdef APCLI_CONNECTION_TRIAL	
		RTMPInitTimer(pAd, &pAd->ApCfg.ApCliTab[i].MlmeAux.ProbeTimer, GET_TIMER_FUNCTION(ApCliProbeTimeout), (PVOID)pApCliEntry, FALSE);
#else
		RTMPInitTimer(pAd, &pAd->ApCfg.ApCliTab[i].MlmeAux.ProbeTimer, GET_TIMER_FUNCTION(ApCliProbeTimeout), pAd, FALSE);
#endif /* APCLI_CONNECTION_TRIAL */
		pAd->ApCfg.ApCliTab[i].SyncCurrState = APCLI_SYNC_IDLE;
	}

	return;
}

/* 
    ==========================================================================
    Description:
        Becaon timeout handler, executed in timer thread
    ==========================================================================
 */
static VOID ApCliProbeTimeout(
	IN PVOID SystemSpecific1, 
	IN PVOID FunctionContext, 
	IN PVOID SystemSpecific2, 
	IN PVOID SystemSpecific3)
{
#ifdef APCLI_CONNECTION_TRIAL
	PAPCLI_STRUCT pApCliEntry = (APCLI_STRUCT *)FunctionContext;
	RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)pApCliEntry->pAd;
#else
	RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)FunctionContext;
#endif /*APCLI_CONNECTION_TRIAL*/

	DBGPRINT(RT_DEBUG_TRACE, ("ApCli_SYNC - ProbeReqTimeout\n"));
#ifndef APCLI_CONNECTION_TRIAL
	MlmeEnqueue(pAd, APCLI_SYNC_STATE_MACHINE, APCLI_MT2_PROBE_TIMEOUT, 0, NULL, 0);
#else
	MlmeEnqueue(pAd, APCLI_SYNC_STATE_MACHINE, APCLI_MT2_PROBE_TIMEOUT, 0, NULL, pApCliEntry->ifIndex);
#endif /* APCLI_CONNECTION_TRIAL */
	RTMP_MLME_HANDLER(pAd);

	return;
}

/* 
    ==========================================================================
    Description:
        MLME PROBE req state machine procedure
    ==========================================================================
 */
static VOID ApCliMlmeProbeReqAction(
	IN PRTMP_ADAPTER pAd,
	IN MLME_QUEUE_ELEM *Elem)
{
	BOOLEAN Cancelled;
	APCLI_MLME_JOIN_REQ_STRUCT *Info = (APCLI_MLME_JOIN_REQ_STRUCT *)(Elem->Msg);
	USHORT ifIndex = (USHORT)(Elem->Priv);
	PULONG pCurrState = &pAd->ApCfg.ApCliTab[ifIndex].SyncCurrState;
	APCLI_STRUCT *pApCliEntry = NULL;

	DBGPRINT(RT_DEBUG_TRACE, ("ApCli SYNC - ApCliMlmeProbeReqAction(Ssid %s)\n", Info->Ssid));

	if (ifIndex >= MAX_APCLI_NUM)
		return;

	pApCliEntry = &pAd->ApCfg.ApCliTab[ifIndex];

	/* reset all the timers */
	RTMPCancelTimer(&(pApCliEntry->MlmeAux.ProbeTimer), &Cancelled);

	pApCliEntry->MlmeAux.Rssi = -9999;
	ULONG bss_idx = BSS_NOT_FOUND;
	bss_idx = BssSsidTableSearchBySSID(&pAd->ScanTab, (PCHAR)Info->Ssid, Info->SsidLen);
	if (bss_idx == BSS_NOT_FOUND)
	{
#ifdef APCLI_CONNECTION_TRIAL
		if (pApCliEntry->TrialCh ==0)
			pApCliEntry->MlmeAux.Channel = pAd->CommonCfg.Channel;
		else
			pApCliEntry->MlmeAux.Channel = pApCliEntry->TrialCh;
#else
		pApCliEntry->MlmeAux.Channel = pAd->CommonCfg.Channel;
#endif /* APCLI_CONNECTION_TRIAL */
	}
	else
	{
#ifdef APCLI_CONNECTION_TRIAL
		if (pApCliEntry->TrialCh ==0)
			pApCliEntry->MlmeAux.Channel = pAd->CommonCfg.Channel;
		else
			pApCliEntry->MlmeAux.Channel = pApCliEntry->TrialCh;
#else
		DBGPRINT(RT_DEBUG_TRACE, ("%s, Found %s in scanTable , goto channel %d\n", 
				__FUNCTION__, pAd->ScanTab.BssEntry[bss_idx].Ssid,  
				pAd->ScanTab.BssEntry[bss_idx].Channel));

		pApCliEntry->MlmeAux.Channel = pAd->ScanTab.BssEntry[bss_idx].Channel;
#endif /* APCLI_CONNECTION_TRIAL */
	}

#ifdef RT_CFG80211_P2P_CONCURRENT_DEVICE
	pApCliEntry->MlmeAux.SupRateLen = pAd->cfg80211_ctrl.P2pSupRateLen;
	NdisMoveMemory(pApCliEntry->MlmeAux.SupRate, pAd->cfg80211_ctrl.P2pSupRate, pAd->cfg80211_ctrl.P2pSupRateLen);
	
	pApCliEntry->MlmeAux.ExtRateLen = pAd->cfg80211_ctrl.P2pExtRateLen;
	NdisMoveMemory(pApCliEntry->MlmeAux.ExtRate, pAd->cfg80211_ctrl.P2pExtRate, pAd->cfg80211_ctrl.P2pExtRateLen);
#else
	pApCliEntry->MlmeAux.SupRateLen = pAd->CommonCfg.SupRateLen;
	NdisMoveMemory(pApCliEntry->MlmeAux.SupRate, pAd->CommonCfg.SupRate, pAd->CommonCfg.SupRateLen);

	/* Prepare the default value for extended rate */
	pApCliEntry->MlmeAux.ExtRateLen = pAd->CommonCfg.ExtRateLen;
	NdisMoveMemory(pApCliEntry->MlmeAux.ExtRate, pAd->CommonCfg.ExtRate, pAd->CommonCfg.ExtRateLen);
#endif /* RT_CFG80211_P2P_CONCURRENT_DEVICE */

	RTMPSetTimer(&(pApCliEntry->MlmeAux.ProbeTimer), PROBE_TIMEOUT);

#ifdef APCLI_CONNECTION_TRIAL
	NdisZeroMemory(pAd->ApCfg.ApCliTab[ifIndex].MlmeAux.Bssid, MAC_ADDR_LEN);
	NdisZeroMemory(pAd->ApCfg.ApCliTab[ifIndex].MlmeAux.Ssid, MAX_LEN_OF_SSID);
	NdisCopyMemory(pAd->ApCfg.ApCliTab[ifIndex].MlmeAux.Bssid, pAd->ApCfg.ApCliTab[ifIndex].CfgApCliBssid, MAC_ADDR_LEN);
	NdisCopyMemory(pAd->ApCfg.ApCliTab[ifIndex].MlmeAux.Ssid, pAd->ApCfg.ApCliTab[ifIndex].CfgSsid, pAd->ApCfg.ApCliTab[ifIndex].CfgSsidLen);
#endif /* APCLI_CONNECTION_TRIAL */

	ApCliEnqueueProbeRequest(pAd, Info->SsidLen, (PCHAR) Info->Ssid, ifIndex);

	DBGPRINT(RT_DEBUG_TRACE, ("ApCli SYNC - Start Probe the SSID %s on channel =%d\n", pApCliEntry->MlmeAux.Ssid, pApCliEntry->MlmeAux.Channel));

	*pCurrState = APCLI_JOIN_WAIT_PROBE_RSP;

	return;
}


/* 
    ==========================================================================
    Description:
        When waiting joining the (I)BSS, beacon received from external
    ==========================================================================
 */
static VOID ApCliPeerProbeRspAtJoinAction(
	IN PRTMP_ADAPTER pAd, 
	IN MLME_QUEUE_ELEM *Elem) 
{
	USHORT LenVIE;
	UCHAR *VarIE = NULL;
	NDIS_802_11_VARIABLE_IEs *pVIE = NULL;
	APCLI_CTRL_MSG_STRUCT ApCliCtrlMsg;
	PAPCLI_STRUCT pApCliEntry = NULL;
	struct wifi_dev *wdev;
#ifdef DOT11_N_SUPPORT
	UCHAR CentralChannel;
#endif /* DOT11_N_SUPPORT */
	USHORT ifIndex = (USHORT)(Elem->Priv);
	ULONG *pCurrState;
	BCN_IE_LIST *ie_list = NULL;

	if (ifIndex >= MAX_APCLI_NUM)
		return;

	/* Init Variable IE structure */
	os_alloc_mem(NULL, (UCHAR **)&VarIE, MAX_VIE_LEN);
	if (VarIE == NULL)
	{
		DBGPRINT(RT_DEBUG_ERROR, ("%s: Allocate memory fail!!!\n", __FUNCTION__));
		goto LabelErr;
	}
	pVIE = (PNDIS_802_11_VARIABLE_IEs) VarIE;
	pVIE->Length = 0;
	
	os_alloc_mem(NULL, (UCHAR **)&ie_list, sizeof(BCN_IE_LIST));
	if (ie_list == NULL)
	{
		DBGPRINT(RT_DEBUG_ERROR, ("%s: Allocate ie_list fail!!!\n", __FUNCTION__));
		goto LabelErr;
	}
	NdisZeroMemory(ie_list, sizeof(BCN_IE_LIST));

	pCurrState = &pAd->ApCfg.ApCliTab[ifIndex].SyncCurrState;
	if (PeerBeaconAndProbeRspSanity(pAd, 
								Elem->Msg, 
								Elem->MsgLen, 
								Elem->Channel,
								ie_list,
								&LenVIE,
								pVIE))
	{
		/*
			BEACON from desired BSS/IBSS found. We should be able to decide most
			BSS parameters here.
			Q. But what happen if this JOIN doesn't conclude a successful ASSOCIATEION?
				Do we need to receover back all parameters belonging to previous BSS?
			A. Should be not. There's no back-door recover to previous AP. It still need
				a new JOIN-AUTH-ASSOC sequence.
		*/
		INT ssidEqualFlag = FALSE;
		INT ssidEmptyFlag = FALSE;
		INT bssidEqualFlag = FALSE;
		INT bssidEmptyFlag = FALSE;
		INT matchFlag = FALSE;

		ULONG   Bssidx;
		CHAR RealRssi = -127;

		RealRssi = (LONG)(RTMPMaxRssi(pAd, ConvertToRssi(pAd, Elem->Rssi0, RSSI_0), 
						   ConvertToRssi(pAd, Elem->Rssi1, RSSI_1), 
						   ConvertToRssi(pAd, Elem->Rssi2, RSSI_2)));		

		/* Update ScanTab */
		Bssidx = BssTableSearch(&pAd->ScanTab, ie_list->Bssid, ie_list->Channel);
		if (Bssidx == BSS_NOT_FOUND)
		{
			/* discover new AP of this network, create BSS entry */
			Bssidx = BssTableSetEntry(pAd, &pAd->ScanTab, ie_list, -127, LenVIE, pVIE);
			
			if (Bssidx == BSS_NOT_FOUND) /* return if BSS table full */
			{
				DBGPRINT(RT_DEBUG_ERROR, ("ERROR: Driver ScanTable Full In Apcli ProbeRsp Join\n"));
				goto LabelErr;
			}

			NdisMoveMemory(pAd->ScanTab.BssEntry[Bssidx].PTSF, &Elem->Msg[24], 4);
			NdisMoveMemory(&pAd->ScanTab.BssEntry[Bssidx].TTSF[0], &Elem->TimeStamp.u.LowPart, 4);
			NdisMoveMemory(&pAd->ScanTab.BssEntry[Bssidx].TTSF[4], &Elem->TimeStamp.u.LowPart, 4);
			pAd->ScanTab.BssEntry[Bssidx].MinSNR = Elem->Signal % 10;
			if (pAd->ScanTab.BssEntry[Bssidx].MinSNR == 0)
				pAd->ScanTab.BssEntry[Bssidx].MinSNR = -5;
			
			NdisMoveMemory(pAd->ScanTab.BssEntry[Bssidx].MacAddr, ie_list->Addr2, MAC_ADDR_LEN);
		}

#ifdef RT_CFG80211_P2P_CONCURRENT_DEVICE
                DBGPRINT(RT_DEBUG_TRACE, ("Info: Update the SSID %s in Kernel Table\n", ie_list->Ssid));
                RT_CFG80211_SCANNING_INFORM(pAd, Bssidx, ie_list->Channel, (UCHAR *)Elem->Msg, Elem->MsgLen, RealRssi);
#endif /* RT_CFG80211_P2P_CONCURRENT_DEVICE */


		pApCliEntry = &pAd->ApCfg.ApCliTab[ifIndex];
		wdev = &pApCliEntry->wdev;

		/* Check the Probe-Rsp's Bssid. */
		if(!MAC_ADDR_EQUAL(pApCliEntry->CfgApCliBssid, ZERO_MAC_ADDR))
			bssidEqualFlag = MAC_ADDR_EQUAL(pApCliEntry->CfgApCliBssid, ie_list->Bssid);
		else
			bssidEmptyFlag = TRUE;

		/* Check the Probe-Rsp's Ssid. */
		if(pApCliEntry->CfgSsidLen != 0)
			ssidEqualFlag = SSID_EQUAL(pApCliEntry->CfgSsid, pApCliEntry->CfgSsidLen, ie_list->Ssid, ie_list->SsidLen);
		else
			ssidEmptyFlag = TRUE;


		/* bssid and ssid, Both match. */
		if (bssidEqualFlag && ssidEqualFlag)
			matchFlag = TRUE;

		/* ssid match but bssid doesn't be indicate. */
		else if(ssidEqualFlag && bssidEmptyFlag)
			matchFlag = TRUE;

		/* user doesn't indicate any bssid or ssid. AP-Clinet will auto pick a AP to join by most strong siganl strength. */
		else if (bssidEmptyFlag && ssidEmptyFlag)
			matchFlag = TRUE;


		DBGPRINT(RT_DEBUG_TRACE, ("SYNC - bssidEqualFlag=%d, ssidEqualFlag=%d, matchFlag=%d\n",
					bssidEqualFlag, ssidEqualFlag, matchFlag));
		if (matchFlag)
		{
			/* Validate RSN IE if necessary, then copy store this information */
			if ((LenVIE > 0) 
#ifdef RT_CFG80211_P2P_CONCURRENT_DEVICE
				/* When using CFG80211 and trigger WPS, do not check security. */
				&& ! (pApCliEntry->wpa_supplicant_info.WpaSupplicantUP & WPA_SUPPLICANT_ENABLE_WPS)
#endif /* RT_CFG80211_P2P_CONCURRENT_DEVICE */
                	)
			{
				if (ApCliValidateRSNIE(pAd, (PEID_STRUCT)pVIE, LenVIE, ifIndex))
				{
					pApCliEntry->MlmeAux.VarIELen = LenVIE;
					NdisMoveMemory(pApCliEntry->MlmeAux.VarIEs, pVIE, pApCliEntry->MlmeAux.VarIELen);
				}
				else
				{
					/* ignore this response */
					pApCliEntry->MlmeAux.VarIELen = 0;
					DBGPRINT(RT_DEBUG_ERROR, ("ERROR: The RSN IE of this received Probe-resp is dis-match !!!!!!!!!! \n"));
					goto LabelErr;
				}
			}
			else
			{
				if (pApCliEntry->wdev.AuthMode >= Ndis802_11AuthModeWPA
                    )
				{
					/* ignore this response */
					DBGPRINT(RT_DEBUG_ERROR, ("ERROR: The received Probe-resp has empty RSN IE !!!!!!!!!! \n"));
					goto LabelErr;
				}	
				
				pApCliEntry->MlmeAux.VarIELen = 0;
			}

			DBGPRINT(RT_DEBUG_TRACE, ("SYNC - receive desired PROBE_RSP at JoinWaitProbeRsp... Channel = %d\n",
							ie_list->Channel));

			/* if the Bssid doesn't be indicated then you need to decide which AP to connect by most strong Rssi signal strength. */
			if (bssidEqualFlag == FALSE)
			{
				/* caculate real rssi value. */
				CHAR Rssi0 = ConvertToRssi(pAd, Elem->Rssi0, RSSI_0);
				CHAR Rssi1 = ConvertToRssi(pAd, Elem->Rssi1, RSSI_1);
				CHAR Rssi2 = ConvertToRssi(pAd, Elem->Rssi2, RSSI_2);
				LONG RealRssi = (LONG)(RTMPMaxRssi(pAd, Rssi0, Rssi1, Rssi2));

				DBGPRINT(RT_DEBUG_TRACE, ("SYNC - previous Rssi = %ld current Rssi=%ld\n", pApCliEntry->MlmeAux.Rssi, (LONG)RealRssi));
				if (pApCliEntry->MlmeAux.Rssi > (LONG)RealRssi)
					goto LabelErr;
				else
					pApCliEntry->MlmeAux.Rssi = RealRssi;
			}
			else
			{
				BOOLEAN Cancelled;
				RTMPCancelTimer(&pApCliEntry->MlmeAux.ProbeTimer, &Cancelled);
			}

			NdisMoveMemory(pApCliEntry->MlmeAux.Ssid, ie_list->Ssid, ie_list->SsidLen);
			pApCliEntry->MlmeAux.SsidLen = ie_list->SsidLen;

			NdisMoveMemory(pApCliEntry->MlmeAux.Bssid, ie_list->Bssid, MAC_ADDR_LEN);
			pApCliEntry->MlmeAux.CapabilityInfo = ie_list->CapabilityInfo & SUPPORTED_CAPABILITY_INFO;
			pApCliEntry->MlmeAux.BssType = ie_list->BssType;
			pApCliEntry->MlmeAux.BeaconPeriod = ie_list->BeaconPeriod;
			pApCliEntry->MlmeAux.Channel = ie_list->Channel;
			pApCliEntry->MlmeAux.AtimWin = ie_list->AtimWin;
			pApCliEntry->MlmeAux.CfpPeriod = ie_list->CfParm.CfpPeriod;
			pApCliEntry->MlmeAux.CfpMaxDuration = ie_list->CfParm.CfpMaxDuration;
			pApCliEntry->MlmeAux.APRalinkIe = ie_list->RalinkIe;

			/* Copy AP's supported rate to MlmeAux for creating assoication request */
			/* Also filter out not supported rate */
			pApCliEntry->MlmeAux.SupRateLen = ie_list->SupRateLen;
			NdisMoveMemory(pApCliEntry->MlmeAux.SupRate, ie_list->SupRate, ie_list->SupRateLen);
			RTMPCheckRates(pAd, pApCliEntry->MlmeAux.SupRate, &pApCliEntry->MlmeAux.SupRateLen);
			pApCliEntry->MlmeAux.ExtRateLen = ie_list->ExtRateLen;
			NdisMoveMemory(pApCliEntry->MlmeAux.ExtRate, ie_list->ExtRate, ie_list->ExtRateLen);
			RTMPCheckRates(pAd, pApCliEntry->MlmeAux.ExtRate, &pApCliEntry->MlmeAux.ExtRateLen);
#ifdef APCLI_CERT_SUPPORT
			/*  Get the ext capability info element */
			if (pAd->bApCliCertTest == TRUE 
#ifdef DOT11N_DRAFT3				
				&& pAd->CommonCfg.bBssCoexEnable == TRUE
#endif /* DOT11N_DRAFT3 */			
				)
			{
				NdisMoveMemory(&pApCliEntry->MlmeAux.ExtCapInfo, &ie_list->ExtCapInfo,sizeof(ie_list->ExtCapInfo));
#ifdef DOT11_N_SUPPORT
#ifdef DOT11N_DRAFT3
				DBGPRINT(RT_DEBUG_TRACE, ("\x1b[31m ApCliMlmeAux.ExtCapInfo=%d \x1b[m\n", pApCliEntry->MlmeAux.ExtCapInfo.BssCoexistMgmtSupport)); //zero debug 210121122
					pAd->CommonCfg.ExtCapIE.BssCoexistMgmtSupport = 1;
#endif /* DOT11N_DRAFT3 */
#endif /* DOT11_N_SUPPORT */
			}
#endif /* APCLI_CERT_SUPPORT */
#ifdef DOT11_N_SUPPORT
			NdisZeroMemory(pApCliEntry->RxMcsSet,sizeof(pApCliEntry->RxMcsSet));
			/* filter out un-supported ht rates */
			if ((ie_list->HtCapabilityLen > 0) && 
				(pApCliEntry->wdev.DesiredHtPhyInfo.bHtEnable) &&
				WMODE_CAP_N(pAd->CommonCfg.PhyMode) &&
				/* For Dissallow TKIP rule on STA */
				!(pAd->CommonCfg.HT_DisallowTKIP && IS_INVALID_HT_SECURITY(wdev->WepStatus)))
			{
				RTMPZeroMemory(&pApCliEntry->MlmeAux.HtCapability, SIZE_HT_CAP_IE);
				pApCliEntry->MlmeAux.NewExtChannelOffset = ie_list->NewExtChannelOffset;
				pApCliEntry->MlmeAux.HtCapabilityLen = ie_list->HtCapabilityLen;
				ApCliCheckHt(pAd, ifIndex, &ie_list->HtCapability, &ie_list->AddHtInfo);

				if (ie_list->AddHtInfoLen > 0)
				{
					CentralChannel = ie_list->AddHtInfo.ControlChan;
		 			/* Check again the Bandwidth capability of this AP. */
					CentralChannel = get_cent_ch_by_htinfo(pAd, &ie_list->AddHtInfo,
														&ie_list->HtCapability);
		 			DBGPRINT(RT_DEBUG_TRACE, ("PeerBeaconAtJoinAction HT===>CentralCh = %d, ControlCh = %d\n",
									CentralChannel, ie_list->AddHtInfo.ControlChan));
				}
			}
			else
#endif /* DOT11_N_SUPPORT */
			{
				RTMPZeroMemory(&pApCliEntry->MlmeAux.HtCapability, SIZE_HT_CAP_IE);
				RTMPZeroMemory(&pApCliEntry->MlmeAux.AddHtInfo, SIZE_ADD_HT_INFO_IE);
				pApCliEntry->MlmeAux.HtCapabilityLen = 0;
			}
			ApCliUpdateMlmeRate(pAd, ifIndex);

#ifdef DOT11_N_SUPPORT
			/* copy QOS related information */
			if (WMODE_CAP_N(pAd->CommonCfg.PhyMode))
			{
				NdisMoveMemory(&pApCliEntry->MlmeAux.APEdcaParm, &ie_list->EdcaParm, sizeof(EDCA_PARM));
				NdisMoveMemory(&pApCliEntry->MlmeAux.APQbssLoad, &ie_list->QbssLoad, sizeof(QBSS_LOAD_PARM));
				NdisMoveMemory(&pApCliEntry->MlmeAux.APQosCapability, &ie_list->QosCapability, sizeof(QOS_CAPABILITY_PARM));
			}
			else
#endif /* DOT11_N_SUPPORT */
			{
				NdisZeroMemory(&pApCliEntry->MlmeAux.APEdcaParm, sizeof(EDCA_PARM));
				NdisZeroMemory(&pApCliEntry->MlmeAux.APQbssLoad, sizeof(QBSS_LOAD_PARM));
				NdisZeroMemory(&pApCliEntry->MlmeAux.APQosCapability, sizeof(QOS_CAPABILITY_PARM));
			}

			DBGPRINT(RT_DEBUG_TRACE, ("APCLI SYNC - after JOIN, SupRateLen=%d, ExtRateLen=%d\n", 
				pApCliEntry->MlmeAux.SupRateLen, pApCliEntry->MlmeAux.ExtRateLen));

			if (ie_list->AironetCellPowerLimit != 0xFF)
			{
				/* We need to change our TxPower for CCX 2.0 AP Control of Client Transmit Power */
				ChangeToCellPowerLimit(pAd, ie_list->AironetCellPowerLimit);
			}
			else  /* Used the default TX Power Percentage. */
				pAd->CommonCfg.TxPowerPercentage = pAd->CommonCfg.TxPowerDefault;

			if(bssidEqualFlag == TRUE)
			{
				*pCurrState = APCLI_SYNC_IDLE;

				ApCliCtrlMsg.Status = MLME_SUCCESS;

				MlmeEnqueue(pAd, APCLI_CTRL_STATE_MACHINE, APCLI_CTRL_PROBE_RSP,
					sizeof(APCLI_CTRL_MSG_STRUCT), &ApCliCtrlMsg, ifIndex);
			}
		}
	}

LabelErr:
	if (VarIE != NULL)
		os_free_mem(NULL, VarIE);
	if (ie_list != NULL)
		os_free_mem(NULL, ie_list);

	return;
}

static VOID ApCliProbeTimeoutAtJoinAction(
	IN PRTMP_ADAPTER pAd,
	IN MLME_QUEUE_ELEM *Elem) 
{
	APCLI_CTRL_MSG_STRUCT ApCliCtrlMsg;
	USHORT ifIndex = (USHORT)(Elem->Priv);
	PULONG pCurrState = &pAd->ApCfg.ApCliTab[ifIndex].SyncCurrState;
	APCLI_STRUCT *pApCliEntry = NULL;

	DBGPRINT(RT_DEBUG_TRACE, ("APCLI_SYNC - ProbeTimeoutAtJoinAction\n"));

	if (ifIndex >= MAX_APCLI_NUM)
		return;

	pApCliEntry = &pAd->ApCfg.ApCliTab[ifIndex];
	*pCurrState = SYNC_IDLE;

#ifdef APCLI_CONNECTION_TRIAL
	if (ifIndex == 1)
		*pCurrState = APCLI_CTRL_DISCONNECTED;
#endif /* APCLI_CONNECTION_TRIAL */

	DBGPRINT(RT_DEBUG_TRACE, ("APCLI_SYNC - MlmeAux.Bssid=%02x:%02x:%02x:%02x:%02x:%02x\n",
								PRINT_MAC(pApCliEntry->MlmeAux.Bssid)));

	if(!MAC_ADDR_EQUAL(pApCliEntry->MlmeAux.Bssid, ZERO_MAC_ADDR))
	{
		ApCliCtrlMsg.Status = MLME_SUCCESS;

		MlmeEnqueue(pAd, APCLI_CTRL_STATE_MACHINE, APCLI_CTRL_PROBE_RSP,
			sizeof(APCLI_CTRL_MSG_STRUCT), &ApCliCtrlMsg, ifIndex);
	} else
	{
		MlmeEnqueue(pAd, APCLI_CTRL_STATE_MACHINE, APCLI_CTRL_JOIN_REQ_TIMEOUT, 0, NULL, ifIndex);
	}

	return;
}

/* 
    ==========================================================================
    Description:
    ==========================================================================
 */
static VOID ApCliInvalidStateWhenJoin(
	IN PRTMP_ADAPTER pAd, 
	IN MLME_QUEUE_ELEM *Elem) 
{
	APCLI_CTRL_MSG_STRUCT ApCliCtrlMsg;
	USHORT ifIndex = (USHORT)(Elem->Priv);
	PULONG pCurrState = &pAd->ApCfg.ApCliTab[ifIndex].SyncCurrState;

	*pCurrState = APCLI_SYNC_IDLE;
	ApCliCtrlMsg.Status = MLME_STATE_MACHINE_REJECT;

	MlmeEnqueue(pAd, APCLI_CTRL_STATE_MACHINE, APCLI_CTRL_PROBE_RSP,
		sizeof(APCLI_CTRL_MSG_STRUCT), &ApCliCtrlMsg, ifIndex);

	DBGPRINT(RT_DEBUG_TRACE, ("APCLI_AYNC - ApCliInvalidStateWhenJoin(state=%ld). Reset SYNC machine\n", *pCurrState));

	return;
}

/* 
	==========================================================================
	Description:
	==========================================================================
 */
static VOID ApCliEnqueueProbeRequest(
	IN PRTMP_ADAPTER pAd,
	IN UCHAR SsidLen,
	OUT PCHAR Ssid,
	IN USHORT ifIndex)
{
	NDIS_STATUS     NState;
	PUCHAR          pOutBuffer;
	ULONG           FrameLen = 0;
	HEADER_802_11   Hdr80211;
	UCHAR           SsidIe    = IE_SSID;
	UCHAR           SupRateIe = IE_SUPP_RATES;
	UCHAR ssidLen;
	CHAR ssid[MAX_LEN_OF_SSID];
	APCLI_STRUCT *pApCliEntry = NULL;
	BOOLEAN bHasWscIe = FALSE;

	DBGPRINT(RT_DEBUG_TRACE, ("force out a ProbeRequest ...\n"));

	if (ifIndex >= MAX_APCLI_NUM)
		return;

	pApCliEntry = &pAd->ApCfg.ApCliTab[ifIndex];
	
	NState = MlmeAllocateMemory(pAd, &pOutBuffer);  /* Get an unused nonpaged memory */
	if(NState != NDIS_STATUS_SUCCESS)
	{
		DBGPRINT(RT_DEBUG_TRACE, ("EnqueueProbeRequest() allocate memory fail\n"));
		return;
	}
	else
	{
		if(MAC_ADDR_EQUAL(pAd->ApCfg.ApCliTab[ifIndex].CfgApCliBssid, ZERO_MAC_ADDR))
			ApCliMgtMacHeaderInit(pAd, &Hdr80211, SUBTYPE_PROBE_REQ, 0,
				BROADCAST_ADDR, BROADCAST_ADDR, ifIndex);
		else
			ApCliMgtMacHeaderInit(pAd, &Hdr80211, SUBTYPE_PROBE_REQ, 0,
				pAd->ApCfg.ApCliTab[ifIndex].CfgApCliBssid, pAd->ApCfg.ApCliTab[ifIndex].CfgApCliBssid, ifIndex);

		ssidLen = SsidLen;
		NdisZeroMemory(ssid, MAX_LEN_OF_SSID);
		NdisMoveMemory(ssid, Ssid, ssidLen);

		/* this ProbeRequest explicitly specify SSID to reduce unwanted ProbeResponse */
		MakeOutgoingFrame(pOutBuffer,		&FrameLen,
			sizeof(HEADER_802_11),			&Hdr80211,
			1,								&SsidIe,
			1,								&ssidLen,
			ssidLen,						ssid,
			1,								&SupRateIe,
			1,								&pApCliEntry->MlmeAux.SupRateLen,
			pApCliEntry->MlmeAux.SupRateLen,		pApCliEntry->MlmeAux.SupRate,
			END_OF_ARGS);

		/* Add the extended rate IE */
		if (pApCliEntry->MlmeAux.ExtRateLen != 0)
		{
			ULONG            tmp;
		
			MakeOutgoingFrame(pOutBuffer + FrameLen,    &tmp,
				1,                        &ExtRateIe,
				1,                        &pApCliEntry->MlmeAux.ExtRateLen,
				pApCliEntry->MlmeAux.ExtRateLen,  pApCliEntry->MlmeAux.ExtRate,
				END_OF_ARGS);
			FrameLen += tmp;
		}

#ifdef DOT11_VHT_AC
		if (WMODE_CAP_AC(pAd->CommonCfg.PhyMode) &&
			(pAd->CommonCfg.Channel > 14))
		{
			build_vht_cap_ie(pAd, (UCHAR *)&pApCliEntry->MlmeAux.vht_cap);
			pApCliEntry->MlmeAux.vht_cap_len = sizeof(VHT_CAP_IE);
			FrameLen += build_vht_ies(pAd, (UCHAR *)(pOutBuffer + FrameLen), SUBTYPE_PROBE_REQ);
		}
#endif /* DOT11_VHT_AC */
#ifdef RT_CFG80211_P2P_CONCURRENT_DEVICE
		if ((pAd->StaCfg.wpa_supplicant_info.WpaSupplicantUP != WPA_SUPPLICANT_DISABLE) &&
			(pAd->cfg80211_ctrl.ExtraIeLen != 0))
		{
				ULONG ExtraIeTmpLen = 0;
		
				MakeOutgoingFrame(pOutBuffer + FrameLen,			  &ExtraIeTmpLen,
								  pAd->cfg80211_ctrl.ExtraIeLen,   pAd->cfg80211_ctrl.pExtraIe,
								  END_OF_ARGS);
		
				FrameLen += ExtraIeTmpLen;
		}
#endif /* RT_CFG80211_P2P_CONCURRENT_DEVICE*/		

		MiniportMMRequest(pAd, QID_AC_BE, pOutBuffer, FrameLen);
		MlmeFreeMemory(pAd, pOutBuffer);
	}

	return;
}

#endif /* APCLI_SUPPORT */

