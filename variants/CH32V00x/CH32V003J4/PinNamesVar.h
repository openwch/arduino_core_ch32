
// Alternate pin names
#define PC_4_ALT1              (PC_4  | ALT1)
#define PD_1_ALT1              (PD_1  | ALT1)

#define PA_1_ALT2              (PA_1  | ALT2)
#define PA_2_ALT2              (PA_2  | ALT2)
#define PC_1_ALT2              (PC_1  | ALT2)
#define PC_4_ALT2              (PC_4  | ALT2)
#define PD_1_ALT2              (PD_1  | ALT2)

#define PC_4_ALT3              (PC_4  | ALT3)
#define PC_1_ALT3              (PC_1  | ALT3)
#define PD_4_ALT3              (PD_4  | ALT3)


/* SYS_WKUP */
#ifdef PWR_WAKEUP_PIN1
  SYS_WKUP1 = PA_0,
#endif
#ifdef PWR_WAKEUP_PIN2
  SYS_WKUP2 = NC,
#endif
#ifdef PWR_WAKEUP_PIN3
  SYS_WKUP3 = NC,
#endif
#ifdef PWR_WAKEUP_PIN4
  SYS_WKUP4 = NC,
#endif
#ifdef PWR_WAKEUP_PIN5
  SYS_WKUP5 = NC,
#endif
#ifdef PWR_WAKEUP_PIN6
  SYS_WKUP6 = NC,
#endif
#ifdef PWR_WAKEUP_PIN7
  SYS_WKUP7 = NC,
#endif
#ifdef PWR_WAKEUP_PIN8
  SYS_WKUP8 = NC,
#endif


