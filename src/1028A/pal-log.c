#include "pros/apix.h"
#include <stdint.h>
#include <stdio.h>

/* Need to define log level for this file lol */
#define LOG_LEVEL_FILE LOG_LEVEL_WARN
#include "1028A/log.h"

/* Variables which are exported */
FILE *fd;
FILE *dd;

/* Variables which are not exported */
static char dname[64];  /* File name of the data file and log file */
static char fname[64];  /* The size of these strings is guaranteed by the naming
                           convention */
static int dheader = 0; /* Indicate if header needs to be printed */
static int fnum = -1;

/* Function to return the file index */
int log_id() { return fnum; }

/* File open/reopen process (called from the task and from initialize */
void log_reopen(int segment) {
  /* Variable indicating the uSD was previously inserted */
  static int uSD_last = 0;

  /* File index used for file naming */
  int idx = 0;

  /* Check if uSD is inserted */
  int uSD_avail = usd_is_installed();

  /* If segment is requested, perform close of existing file and pretend uSD
   * was not inserted last time
   */
  if (segment) {
    /* Close fd and dd if open */
    LOG_ALWAYS("Segment requested, opening with new file name");
    fclose(fd);
    fclose(dd);
    fd = NULL;
    dd = NULL;
    uSD_last = false;
  }

  /* If it is now installed and wasn't before, go through the opening process */
  if (uSD_avail && !uSD_last) {
    LOG_DEBUG("Opening a new log file, SD was inserted");
    /* Attempt to open the index file to determine the next file name */
    FILE *fidx = fopen("/usd/index.txt", "r");

    /* If no error opening index file, read the contents */
    if (fidx) {
      /* Scan an integer from the index file */
      int ret = fscanf(fidx, "%d", &idx);
      if (ret) {
        /* Successfully read 1 parameter, so idx is valid, increment it
         * so we use the next available filename
         */
        idx++;
      }
      /* Close index for now */
      fclose(fidx);
    }
    LOG_INFO("New file index is %d", idx);
    fnum = idx;

    /* In any case, reopen the index file to write the latest file index */
    fidx = fopen("/usd/index.txt", "w");
    if (fidx) {
      /* No error opening file, so write it and close it */
      fprintf(fidx, "%d", idx);
      fclose(fidx);
    }

    /* Determine filenames of the data log and message log */
    sprintf(fname, "/usd/log%05d.txt", idx);
    sprintf(dname, "/usd/dat%05d.csv", idx);

    /* Open the new files */
    fd = fopen(fname, "w");
    dd = fopen(dname, "w");

    /* Check for errors in the process */
    if (fd) {
      LOG_ALWAYS("Log file opened (%s)", fname);
    } else {
      LOG_ERROR("Error opening log file (%s)", fname);
    }
    if (dd) {
      LOG_ALWAYS("Data file opened (%s)", dname);
    } else {
      LOG_ERROR("Error opening data file (%s)", dname);
    }

    /* Since file is open, reset header status to 2, which will decrement to 1
     * at log_step*/
    dheader = 2;

    /* Now that the file is open, we can write the first log entry */
    LOG_INFO("Log Files Opened");
  }
  /* If it was previously installed and isn't any more, close the files and set
     them null */
  else if (!uSD_avail && uSD_last) {
    LOG_ALWAYS("uSD now unavailable");
    fclose(fd);
    fclose(dd);
    fd = NULL;
    dd = NULL;
    fnum = -1;
  }
  /* If the uSD is currently valid and was previously valid, reopen the file
     again */
  else if (uSD_avail && uSD_last) {
    LOG_DEBUG("About to swap file handles");
    // task_set_priority(CURRENT_TASK,TASK_PRIORITY_MAX);
    FILE *fd_temp = fd;
    fd = NULL;
    FILE *dd_temp = dd;
    dd = NULL;
    // task_set_priority(CURRENT_TASK,TASK_PRIORITY_DEFAULT-1);
    /* Close the fd and reopen it */
    fclose(fd_temp);
    fd_temp = fopen(fname, "a");
    fclose(dd_temp);
    dd_temp = fopen(dname, "a");
    // task_set_priority(CURRENT_TASK,TASK_PRIORITY_MAX);
    fd = fd_temp;
    dd = dd_temp;
    // task_set_priority(CURRENT_TASK,TASK_PRIORITY_DEFAULT-1);

    /* Check for errors */
    if (fd) {
      LOG_DEBUG("Log file reopened (%s)", fname);
    } else {
      LOG_ERROR("Error reopening log file (%s)", fname);
    }
    if (dd) {
      LOG_DEBUG("Data file reopened (%s)", dname);
    } else {
      LOG_ERROR("Error reopening data file (%s)", dname);
    }
    LOG_INFO("Log Files Reopened");
  }
  /* Otherwise, uSD is not available and wasn't before */
  else {
    LOG_INFO("uSD not inserted, unable to log");
  }

  /* Update the last state */
  uSD_last = uSD_avail;
}

/* Call to generate a new log segment (new csv, new txt) i.e. when changing
 * modes */
void log_segment() { log_reopen(true); }

/* Log Step checks if it's been more than a second and calls reopen if necessary
 */
void log_step() {
  /* decrement dheader if it's above 0 so we can write the header row */
  if (dheader) {
    dheader--;
  }
  /* Store previous time */
  static double time_last = 0.0;
  /* Get new time */
  double time_now = millis() / 1000.0;

  /* If it's been a second or more, reopen */
  if ((time_now - time_last) > 1.0) {
    log_reopen(false);
    time_last = time_now;
  }

  /* Make sure log file is valid before writing to it */
  if (dd) {
    /* If printing headers, print TIME, else print the timestamp */
    if (dheader) {
      fprintf(dd, "TIME");
    } else {
      fprintf(dd, "\n%08.03f", time_now);
    }
  }
}

/* Initialize the logger */
void log_init() {
  /* Open the logger if the uSD card is inserted */
  log_reopen(false);
}

/* Internal function to check if a log at level should be printed or not, also
 * prints the log header Return values: 2 = log to both fd and printf 1 = log to
 * printf only (fd is invalid) 0 = don't log
 */
int log_check(const char *fname, const int line, log_level_t level,
              log_level_t flevel) {
  /* Log level strings */
  static const char *log_names[] = {"DEBUG", "INFO", "WARN", "ERROR", "ALWAYS"};

  /* Clamp level to valid values */
  level = (level > LOG_LEVEL_ALWAYS) ? LOG_LEVEL_ALWAYS : level;

  /* If we are below the required log level, exit now */
  if (flevel > level)
    return 0;

  /* We should log this, so print the log header and then return true to allow
   * the user message to be printed after
   */

  /* Print the information */
  double time = millis() / 1000.0;
  printf("%08.3f [%s] in %s line %d: ", time, log_names[level], fname, line);

  /* If fd is invalid, end here, otherwise print to the file as well */
  if (!fd)
    return 1;

  /* Print the same information as earlier to the file */
  fprintf(fd, "\n%08.3f [%s] in %s line %d: ", time, log_names[level], fname,
          line);

  /* Tell the macro to print to the file */
  return 2;
}

/* Functions to log data */
void log_data_int(const char *pname, int data) {
  /* If data is safe to access, print to it */
  if (dd) {
    /* If we need to print the header, do that instead of data */
    if (dheader) {
      fprintf(dd, ",%s", pname);
    } else {
      fprintf(dd, ",%d", data);
    }
  }
}
void log_data_dbl(const char *pname, double data) {
  /* If data is safe to access, print to it */
  if (dd) {
    /* If we need to print the header, do that instead of data */
    if (dheader) {
      fprintf(dd, ",%s", pname);
    } else {
      fprintf(dd, ",%f", data);
    }
  }
}