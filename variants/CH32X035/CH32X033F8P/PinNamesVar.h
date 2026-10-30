/* Alternate pin name */

/* SYS_WKUP */
#ifdef PWR_WAKEUP_PIN1
  SYS_WKUP1 = PA_0,
#endif
/* USB */
#ifdef USBCON
  USB_DM = PC_16,
  USB_DP = PC_17,
#endif
