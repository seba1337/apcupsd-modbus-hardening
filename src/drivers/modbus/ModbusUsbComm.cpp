/*
 * ModbusUsbComm.cpp
 *
 * USB communication layer for MODBUS driver
 */

/*
 * Copyright (C) 2014 Adam Kropelin
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General
 * Public License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1335, USA.
 */

/*
 * Thanks go to APC/Schneider for providing the Apcupsd team with early access
 * to MODBUS protocol information to facilitate an Apcupsd driver.
 *
 * APC/Schneider has published the following relevant application notes:
 *
 * AN176: Modbus Implementation in APC Smart-UPS
 *    <http://www.apc.com/whitepaper/?an=176>
 * AN177: Software interface for Switched Outlet and UPS Management in Smart-UPS
 *    <http://www.apc.com/whitepaper/?an=177>
 * AN178: USB HID Implementation in Smart-UPS
 *    <http://www.apc.com/whitepaper/?an=178>
 */

#include <signal.h>
#include <time.h>

#include "apc.h"
#include "ModbusUsbComm.h"

#define ModbusRTURx 0xFF8600FC
#define ModbusRTUTx 0xFF8600FD

#define S_TO_NS(x)  ( (x) * 1000000000ULL )
#define MS_TO_NS(x) ( (x) * 1000000ULL )
#define US_TO_NS(x) ( (x) * 1000ULL )
#define NS_TO_MS(x) ( ((x)+999999) / 1000000ULL )

ModbusUsbComm::ModbusUsbComm(uint8_t slaveaddr) :
   ModbusComm(slaveaddr), _watchdogRunning(false), _watchdogDeadlineNs(0)
{
}

// Real-time signal used by the watchdog thread to forcibly interrupt a
// runaway blocking USB read. Deliberately not SIGALRM, to avoid any
// possible collision with alarm()-based timing elsewhere; deliberately
// not raw SIGRTMIN, to stay clear of the couple of low real-time signals
// glibc/NPTL reserve for its own internal use.
#define MODBUS_WATCHDOG_SIGNAL (SIGRTMIN + 10)
#define MODBUS_WATCHDOG_POLL_MS 100
// How much longer than a call's own requested timeout we allow before
// concluding it's truly stuck and not just about to return on its own.
#define MODBUS_WATCHDOG_GRACE_MS 500

static void ModbusWatchdogSignalHandler(int)
{
   // Intentionally empty: this signal exists only to interrupt a blocking
   // syscall (EINTR). No SA_RESTART is installed, so the interrupted call
   // returns control to our own timeout-checking loops instead of the
   // kernel silently resuming it.
}

static void EnsureWatchdogSignalHandlerInstalled()
{
   static bool installed = false;
   static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;

   pthread_mutex_lock(&mtx);
   if (!installed)
   {
      struct sigaction sa;
      memset(&sa, 0, sizeof(sa));
      sa.sa_handler = ModbusWatchdogSignalHandler;
      sigemptyset(&sa.sa_mask);
      sa.sa_flags = 0;   // no SA_RESTART -- we want EINTR to stick
      sigaction(MODBUS_WATCHDOG_SIGNAL, &sa, NULL);
      installed = true;
   }
   pthread_mutex_unlock(&mtx);
}

void ModbusUsbComm::ArmWatchdogDeadline(unsigned int timeoutMs)
{
   _watchdogDeadlineNs =
      GetTod() + MS_TO_NS(timeoutMs) + MS_TO_NS(MODBUS_WATCHDOG_GRACE_MS);
}

void ModbusUsbComm::DisarmWatchdogDeadline()
{
   _watchdogDeadlineNs = 0;
}

void *ModbusUsbComm::WatchdogThreadMain(void *arg)
{
   ModbusUsbComm *self = (ModbusUsbComm *)arg;

   while (self->_watchdogRunning)
   {
      struct timespec ts = { 0, MODBUS_WATCHDOG_POLL_MS * 1000000L };
      nanosleep(&ts, NULL);

      // Only interrupt the target thread if it is CURRENTLY inside a call
      // that has already blown past its own requested deadline (plus
      // margin). We never fire on a fixed tick regardless of state --
      // signaling a healthy, in-flight USB transfer mid-syscall can
      // desync libusb's own URB submit/reap bookkeeping and would make
      // things worse than the hang we're trying to route around.
      uint64_t deadline = self->_watchdogDeadlineNs;
      if (deadline != 0 && self->GetTod() > deadline)
      {
         Dmsg(0, "%s: USB read exceeded its deadline -- interrupting\n",
            __func__);
         pthread_kill(self->_targetThread, MODBUS_WATCHDOG_SIGNAL);
      }
   }

   return NULL;
}

void ModbusUsbComm::StartWatchdog()
{
   EnsureWatchdogSignalHandlerInstalled();

   _targetThread = pthread_self();
   _watchdogDeadlineNs = 0;
   _watchdogRunning = true;
   if (pthread_create(&_watchdogThread, NULL, WatchdogThreadMain, this) != 0)
   {
      Dmsg(0, "%s: Unable to start USB read watchdog thread\n", __func__);
      _watchdogRunning = false;
   }
}

void ModbusUsbComm::StopWatchdog()
{
   if (_watchdogRunning)
   {
      _watchdogRunning = false;
      pthread_join(_watchdogThread, NULL);
   }
   _watchdogDeadlineNs = 0;
}

bool ModbusUsbComm::Open(const char *path)
{
   // In case we're already open
   Close();

   // Attempt to locate and open the UPS on USB
   if (!_hidups.Open(path))
      return false;

   // Find ModbusRTUTx report
   hid_item_t item;
   if (!_hidups.LocateItem(ModbusRTUTx, -1, -1, -1, HID_KIND_INPUT, &item))
   {
      Dmsg(0, "%s: Unable to find ModbusRTUTx usage\n", __func__);
      goto error;
   }
   _rxrpt = item.report_ID;
   Dmsg(100, "%s: Found ModbusRTUTx in report id %d\n", __func__, _rxrpt);

   // Find ModbusRTURx report
   if (!_hidups.LocateItem(ModbusRTURx, -1, -1, -1, HID_KIND_OUTPUT, &item))
   {
      Dmsg(0, "%s: Unable to find ModbusRTURx usage\n", __func__);
      goto error;
   }
   _txrpt = item.report_ID;
   Dmsg(100, "%s: Found ModbusRTURx in report id %d\n", __func__, _txrpt);

   _open = true;
   StartWatchdog();
   return true;

error:
   _hidups.Close();
   return false;
}

bool ModbusUsbComm::Close()
{
   StopWatchdog();
   _open = false;
   _hidups.Close();
   return true;
}

bool ModbusUsbComm::ModbusTx(const ModbusFrame *frm, unsigned int sz)
{
   // MODBUS/USB is limited to 63 bytes of payload (we don't bother with
   // fragmentation/reassembly). Since we drop the CRC (last 2 bytes) because 
   // MODBUS/USB doesn't use it, frame size is 2 less than what caller says.
   if (sz-2 > MODBUS_USB_REPORT_MAX_FRAME_SIZE)
      return false;

   // Wait for idle 
   if (!WaitIdle())
      return false;

   Dmsg(100, "%s: Sending frame\n", __func__);
   hex_dump(100, frm, sz);

   // We add HID report id as the first byte of the report, then at most 63
   // bytes of payload.
   uint8_t rpt[MODBUS_USB_REPORT_SIZE] = {0};
   rpt[0] = _txrpt;
   memcpy(rpt+1, frm, sz-2);

   ArmWatchdogDeadline(MODBUS_RESPONSE_TIMEOUT_MS);
   int ret = _hidups.InterruptWrite(USB_ENDPOINT_OUT|1, rpt,
      MODBUS_USB_REPORT_SIZE, MODBUS_RESPONSE_TIMEOUT_MS);
   DisarmWatchdogDeadline();

   return ret == (int)MODBUS_USB_REPORT_SIZE;
}

bool ModbusUsbComm::ModbusRx(ModbusFrame *frm, unsigned int *sz)
{
   struct timeval now, exittime;

   // Determine time at which we need to exit
   gettimeofday(&exittime, NULL);
   exittime.tv_sec += MODBUS_RESPONSE_TIMEOUT_MS / 1000;
   exittime.tv_usec += (MODBUS_RESPONSE_TIMEOUT_MS % 1000) * 1000;
   if (exittime.tv_usec >= 1000000)
   {
      ++exittime.tv_sec;
      exittime.tv_usec -= 1000000;
   }

   int ret;
   uint8_t rpt[MODBUS_USB_REPORT_SIZE];
   while (1)
   {
      gettimeofday(&now, NULL);
      int timeout = TV_DIFF_MS(now, exittime);
      if (timeout <= 0)
      {
         Dmsg(0, "%s: TIMEOUT\n", __func__);
         return false;
      }

      ArmWatchdogDeadline(timeout);
      ret = _hidups.InterruptRead(USB_ENDPOINT_IN|1, rpt,
         MODBUS_USB_REPORT_SIZE, timeout);
      DisarmWatchdogDeadline();

      if (ret == -ETIMEDOUT)
      {
         Dmsg(0, "%s: TIMEOUT\n", __func__);
         return false;
      }

      // Temporary failure
      if (ret == -EINTR || ret == -EAGAIN)
         continue;

      // Fatal error
      if (ret <= 0)
      {
         Dmsg(0, "%s: Read error: %d\n", __func__, ret);
         return false;
      }

      // Filter out non-MODBUS reports
      if (rpt[0] != _rxrpt)
      {
         Dmsg(100, "%s: Ignoring report id %u\n", __func__, rpt[0]);
         continue;
      }

      // Bad report size ... fatal
      if (ret != (int)MODBUS_USB_REPORT_SIZE)
      {
         Dmsg(0, "%s: Bad size %d\n", __func__, ret);
         return false;
      }

      // We always get a full report containing MODBUS_USB_REPORT_MAX_FRAME_SIZE
      // bytes of data. Clip to actual size of live data by looking at the MODBUS 
      // PDU header. This is a blatant layering violation, but no way around it 
      // here. Which byte(s) we look at and how we calculate the length depends
      // on the opcode.
      unsigned frmsz;
      if (rpt[2] == MODBUS_FC_READ_HOLDING_REGS)
      {
         // READ_HOLDING_REGS response includes a size byte.
         // Add 3 bytes to PDU size to account for size byte itself 
         // plus frame header (slaveaddr and op code).
         frmsz = rpt[3] + 3;
      }
      else if (rpt[2] == MODBUS_FC_WRITE_MULTIPLE_REGS)
      {
         // WRITE_MULTIPLE_REGS response is always a fixed length
         // 2 byte frame header (slaveaddr and op code)
         // 2 byte register starting address
         // 2 byte register count
         frmsz = 6;
      }
      else
      {
         // Unsupported response message...we can't calculate its length
         Dmsg(0, "%s: Unknown response type %x\n", __func__, rpt[2]);
         hex_dump(0, rpt,  MODBUS_USB_REPORT_SIZE);
         return false;
      }

      if (frmsz > MODBUS_USB_REPORT_MAX_FRAME_SIZE)
      {
         Dmsg(0, "%s: Fragmented PDU received...not supported\n", __func__);
         return false;
      }

      // Copy data to caller's buffer. Live data starts after USB report id byte.
      memcpy(frm, rpt+1, frmsz);

      // MODBUS/USB doesn't provide a CRC. 
      // Fill one in to make upper layer happy.
      uint16_t crc = ModbusCrc(*frm, frmsz);
      (*frm)[frmsz] = crc & 0xff;
      (*frm)[frmsz+1] = crc >> 8;

      *sz = frmsz + 2;
      hex_dump(100, frm, *sz);
      return true;
   }
}

uint64_t ModbusUsbComm::GetTod()
{
   struct timeval now;
   gettimeofday(&now, NULL);
   return S_TO_NS(now.tv_sec) + US_TO_NS(now.tv_usec);
}

bool ModbusUsbComm::WaitIdle()
{
   // Current TOD
   uint64_t now = GetTod();

   // When we will give up by
   uint64_t exittime = now + MS_TO_NS(MODBUS_IDLE_WAIT_TIMEOUT_MS);

   // Initial idle target
   uint64_t target = now + MS_TO_NS(MODBUS_INTERFRAME_TIMEOUT_MS);

   uint8_t rpt[MODBUS_USB_REPORT_SIZE];
   while (target <= exittime)
   {
      // target and now are both uint64_t, so if a non-MODBUS report (below)
      // has let real time run past target without resetting it, target-now
      // would underflow to a huge value here and could turn this read into
      // an effectively unbounded/blocking wait. Clamp to a small positive
      // minimum instead of trusting the raw subtraction.
      uint64_t remainMs = (target > now) ? NS_TO_MS(target-now) : 1;

      ArmWatchdogDeadline((unsigned int)remainMs);
      int rc = _hidups.InterruptRead(USB_ENDPOINT_IN|1, rpt,
         MODBUS_USB_REPORT_SIZE, remainMs);
      DisarmWatchdogDeadline();

      if (rc == -ETIMEDOUT)
      {
         // timeout: line is now idle
         return true;
      }
      else if (rc <= 0 && rc != -EINTR && rc != -EAGAIN)
      {
         // fatal error
         Dmsg(0, "%s: interrupt_read failed: %s\n", __func__, strerror(-rc));
         return false;
      }

      // Refresh TOD
      now = GetTod();

      if (rc > 0)
      {
         if (rpt[0] == _rxrpt)
         {
            Dmsg(0, "%s: Out of sync\n", __func__);
            hex_dump(0, rpt, rc);

            // Reset wait time
            target = now + MS_TO_NS(MODBUS_INTERFRAME_TIMEOUT_MS);
         }
         else
         {
            // Non-MODBUS reports are not an issue, just continue waiting
            Dmsg(100, "%s: Non-MODBUS report id %u\n", __func__, rpt[0]);
         }
      }
   }

   return false;
}
