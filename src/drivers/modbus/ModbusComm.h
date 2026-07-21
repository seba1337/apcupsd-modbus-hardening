/*
 * ModbusComm.h
 *
 * Public header file for the modbus communications base class.
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

#ifndef _MODBUSCOMM_H
#define _MODBUSCOMM_H

#include <stdint.h>

class ModbusComm
{
public:
   ModbusComm(uint8_t slaveaddr = DEFAULT_SLAVE_ADDR) :
      _slaveaddr(slaveaddr), _open(false), _lastRxNext(0)
   {
      for (unsigned int i = 0; i < LAST_RX_HISTORY; ++i)
         _haveLastRx[i] = false;
   }
   virtual ~ModbusComm() {}

   virtual bool Open(const char *dev) = 0;
   virtual bool Close() = 0;

   virtual uint8_t *ReadRegister(uint16_t addr, unsigned int nregs);
   virtual bool WriteRegister(uint16_t reg, unsigned int nregs, const uint8_t *data);

protected:

   uint16_t ModbusCrc(const uint8_t *data, unsigned int sz);

   // MODBUS constants
   static const uint8_t DEFAULT_SLAVE_ADDR = 1;

   // MODBUS timeouts
   static const unsigned int MODBUS_INTERCHAR_TIMEOUT_MS = 25; // Spec is 15, increase for compatibility with USB serial dongles
   // Spec is 35. On this UPS's USB/HID MODBUS gateway we've observed it
   // asynchronously re-emit a stale copy of a prior response up to ~900ms
   // after that response was already accepted -- unrelated to our own
   // request/response cadence. WaitIdle() (the drain gate before every
   // send) requires this many ms of continuous silence before allowing a
   // send, so this directly multiplies the latency of every single
   // request. A straggling duplicate that slips past this gate is still
   // caught by SendAndWait's content-based dedup ring, so this doesn't
   // need to be long enough to catch everything by itself -- 1000ms made
   // capability probing painfully slow in practice without a measurable
   // correctness gain over this shorter value.
   static const unsigned int MODBUS_INTERFRAME_TIMEOUT_MS = 500;
   static const unsigned int MODBUS_IDLE_WAIT_TIMEOUT_MS = 2000;
   static const unsigned int MODBUS_RESPONSE_TIMEOUT_MS = 1000;

   // Total time budget, per transmission attempt, for draining stale/
   // mismatched responses left over from earlier requests while waiting
   // for the real response to arrive. This must be generous enough to
   // absorb a backlog of delayed responses without triggering a
   // retransmit, since retransmitting while a stale response is still in
   // flight only makes the desync worse. Combined with SendAndWait's own
   // retry count and get_capabilities()'s per-register retry count, this
   // multiplies out fast -- keep it modest.
   static const unsigned int MODBUS_TOTAL_REQUEST_TIMEOUT_MS = 3000;

   // MODBUS function codes
   static const uint8_t MODBUS_FC_ERROR = 0x80;
   static const uint8_t MODBUS_FC_READ_HOLDING_REGS = 0x03;
   static const uint8_t MODBUS_FC_WRITE_REG = 0x06;
   static const uint8_t MODBUS_FC_WRITE_MULTIPLE_REGS = 0x10;

   // MODBUS message sizes
   static const unsigned int MODBUS_MAX_FRAME_SZ = 256;
   static const unsigned int MODBUS_MAX_PDU_SZ = MODBUS_MAX_FRAME_SZ - 4;

   typedef uint8_t ModbusFrame[MODBUS_MAX_FRAME_SZ];
   typedef uint8_t ModbusPdu[MODBUS_MAX_PDU_SZ];

   virtual bool ModbusTx(const ModbusFrame *frm, unsigned int sz) = 0;
   virtual bool ModbusRx(ModbusFrame *frm, unsigned int *sz) = 0;

   uint8_t _slaveaddr;
   bool _open;

private:

   virtual bool SendAndWait(
      uint8_t fc,
      const ModbusPdu *txpdu, unsigned int txsz,
      ModbusPdu *rxpdu, unsigned int rxsz);

   // The APC MODBUS-over-HID gateway occasionally re-emits a response
   // asynchronously, well after the original request/response exchange
   // already completed, with no relation to our current request/response
   // cadence. Because MODBUS read-holding-registers responses don't echo
   // back the register address, a same-sized stale duplicate like this is
   // indistinguishable from a real response by address/size/CRC alone. We
   // remember a short history of responses we've accepted so SendAndWait
   // can recognize and discard an exact repeat of any of them, rather than
   // misattributing it to a different, newly-requested register. A single
   // slot isn't enough -- a straggler can be several requests stale, not
   // just one.
   static const unsigned int LAST_RX_HISTORY = 6;
   bool _haveLastRx[LAST_RX_HISTORY];
   uint8_t _lastRxFc[LAST_RX_HISTORY];
   unsigned int _lastRxSz[LAST_RX_HISTORY];
   ModbusPdu _lastRxPdu[LAST_RX_HISTORY];
   unsigned int _lastRxNext;
};

#endif   /* _MODBUSCOMM_H */
