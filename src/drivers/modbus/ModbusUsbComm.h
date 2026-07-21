/*
 * ModbusUsbComm.h
 *
 * Public header file for the modbus driver.
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

#ifndef _MODBUSUSBCOMM_H
#define _MODBUSUSBCOMM_H

#include <pthread.h>

#include "ModbusComm.h"
#include "HidUps.h"

class ModbusUsbComm: public ModbusComm
{
public:
   ModbusUsbComm(uint8_t slaveaddr = DEFAULT_SLAVE_ADDR);
   virtual ~ModbusUsbComm() {}

   virtual bool Open(const char *dev);
   virtual bool Close();

private:

   virtual bool ModbusTx(const ModbusFrame *frm, unsigned int sz);
   virtual bool ModbusRx(ModbusFrame *frm, unsigned int *sz);
   bool WaitIdle();
   static uint64_t GetTod();

   // On this system, the underlying USB interrupt read (reached through
   // the libusb-0.1 compatibility shim on top of libusb-1.0) has been
   // observed to occasionally ignore its own timeout and block
   // indefinitely. Rather than trying to fix that at the libusb/kernel
   // level, we run a watchdog thread that can send a non-fatal,
   // no-restart signal to the thread doing MODBUS I/O, forcing a runaway
   // blocking read to return EINTR so our own (already-correct)
   // elapsed-time checks in ModbusRx()/WaitIdle() get a chance to declare
   // a clean timeout instead of hanging forever.
   //
   // Crucially, the watchdog only fires once a specific call has
   // actually overrun ITS OWN requested deadline by a safety margin --
   // never blindly on a fixed tick. Interrupting a healthy, in-flight USB
   // transfer mid-syscall can desynchronize libusb's own URB submit/reap
   // bookkeeping and makes things worse, not better.
   static void *WatchdogThreadMain(void *arg);
   void StartWatchdog();
   void StopWatchdog();
   void ArmWatchdogDeadline(unsigned int timeoutMs);
   void DisarmWatchdogDeadline();

   pthread_t _watchdogThread;
   pthread_t _targetThread;
   volatile bool _watchdogRunning;
   volatile uint64_t _watchdogDeadlineNs;   // 0 == disarmed

   static const unsigned MODBUS_USB_REPORT_SIZE = 64;
   static const unsigned MODBUS_USB_REPORT_MAX_FRAME_SIZE =
      MODBUS_USB_REPORT_SIZE - 1;

   HidUps _hidups;
   uint8_t _rxrpt;
   uint8_t _txrpt;
};

#endif   /* _MODBUSUSBCOMM_H */
