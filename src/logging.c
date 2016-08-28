#include "logging.h"

#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "ff.h"

static void RemoveHandler(eventid_t id);
static void InsertHandler(eventid_t id);
static void tmr_init(void *p);

/*===========================================================================*/
/* Card insertion monitor.                                                   */
/*===========================================================================*/
#define POLLING_INTERVAL                10
#define POLLING_DELAY                   10

/**
 * MMC driver instance.
 */
MMCDriver MMCD1;

static event_source_t inserted_event, removed_event;

//Maximum speed 18Mhz
static SPIConfig hs_config = {NULL,
    GPIOB, GPIOB_SD_CS,
    0
};

//Slow speed for configuration 281.25kHz
static SPIConfig ls_config = {NULL,
    GPIOB, GPIOB_SD_CS,
    SPI_CR1_BR_2 | SPI_CR1_BR_1
};


static MMCConfig mmc_config = {&SPID1, &ls_config, &hs_config};


static THD_WORKING_AREA(waCardMonitor, 512);
static THD_FUNCTION(CardMonitor, arg) {
  (void)arg;
  chRegSetThreadName("card_monitor");
  mmcObjectInit(&MMCD1);
  mmcStart(&MMCD1, &mmc_config);

  event_listener_t el0, el1;

  static const evhandler_t evhndl[] = {
      InsertHandler,
      RemoveHandler
    };

  /*
   * Activates the card insertion monitor.
   */
  tmr_init(&MMCD1);

  chEvtRegister(&inserted_event, &el0, 0);
  chEvtRegister(&removed_event, &el1, 1);



  while (true) {
      chEvtDispatch(evhndl, chEvtWaitOneTimeout(ALL_EVENTS, MS2ST(500)));
  }
}

/**
 * @brief   Card monitor timer.
 */
static virtual_timer_t tmr;

/**
 * @brief   Debounce counter.
 */
static unsigned cnt;

/**
 * @brief   Insertion monitor timer callback function.
 *
 * @param[in] p         pointer to the @p BaseBlockDevice object
 *
 * @notapi
 */
static void tmrfunc(void *p) {
  BaseBlockDevice *bbdp = p;

  chSysLockFromISR();
  if (cnt > 0) {
    if (blkIsInserted(bbdp)) {
      if (--cnt == 0) {
        chEvtBroadcastI(&inserted_event);
      }
    }
    else
      cnt = POLLING_INTERVAL;
  }
  else {
    if (!blkIsInserted(bbdp)) {
      cnt = POLLING_INTERVAL;
      chEvtBroadcastI(&removed_event);
    }
  }
  chVTSetI(&tmr, MS2ST(POLLING_DELAY), tmrfunc, bbdp);
  chSysUnlockFromISR();
}

/**
 * @brief   Polling monitor start.
 *
 * @param[in] p         pointer to an object implementing @p BaseBlockDevice
 *
 * @notapi
 */
static void tmr_init(void *p) {

  chEvtObjectInit(&inserted_event);
  chEvtObjectInit(&removed_event);
  chSysLock();
  cnt = POLLING_INTERVAL;
  chVTSetI(&tmr, MS2ST(POLLING_DELAY), tmrfunc, p);
  chSysUnlock();
}


/*===========================================================================*/
/* FatFs related.                                                            */
/*===========================================================================*/

/**
 * @brief FS object.
 */
FATFS MMC_FS;
/* FS mounted and ready.*/
static bool fs_ready = FALSE;

/* Generic large buffer */
uint8_t fbuff[1024];

static FRESULT scan_files(BaseSequentialStream *chp, char *path) {
  FRESULT res;
  FILINFO fno;
  DIR dir;
  int i;
  char *fn;

#if _USE_LFN
  fno.lfname = 0;
  fno.lfsize = 0;
#endif
  res = f_opendir(&dir, path);
  if (res == FR_OK) {
    i = strlen(path);
    for (;;) {
      res = f_readdir(&dir, &fno);
      if (res != FR_OK || fno.fname[0] == 0)
        break;
      if (fno.fname[0] == '.')
        continue;
      fn = fno.fname;
      if (fno.fattrib & AM_DIR) {
        path[i++] = '/';
        strcpy(&path[i], fn);
        res = scan_files(chp, path);
        if (res != FR_OK)
          break;
        path[--i] = 0;
      }
      else {
        //chprintf(chp, "%s/%s\r\n", path, fn);
      }
    }
  }
  return res;
}

/*
 * MMC card insertion event.
 */
static void InsertHandler(eventid_t id) {
  FRESULT err;
  (void)id;
  /*
   * On insertion MMC initialization and FS mount.
   */
  if (mmcConnect(&MMCD1)) {
    return;
  }
  err = f_mount(&MMC_FS, "/", 1);
  if (err != FR_OK) {
    mmcDisconnect(&MMCD1);
    return;
  }
  fs_ready = TRUE;
}

/*
 * MMC card removal event.
 */
static void RemoveHandler(eventid_t id) {
  mmcDisconnect(&MMCD1);
  (void)id;
  fs_ready = FALSE;
}


void init_logging(void)
{
  palSetPad(GPIOB, GPIOB_SD_CS);
  palSetPad(GPIOA, GPIOA_CARD_PWR);

  chThdCreateStatic(waCardMonitor, sizeof(waCardMonitor), NORMALPRIO+1, CardMonitor, NULL);
}

void write_to_file(void)
{
  FIL file;       /* File object */
  FRESULT fr;    /* FatFs return code */


  /* Open a text file */
  fr = f_open(&file, "message.txt", FA_WRITE);
  if (fr) return;

  char test[] = "test\n";

  f_write(&file, test,sizeof(test), NULL);

  /* Close the file */
  f_close(&file);
}

