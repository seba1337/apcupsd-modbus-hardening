/*
 * modbus.h
 *
 * Public header file for the modbus driver.
 */

/*
 * Copyright (C) 2013 Adam Kropelin
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

#ifndef _MODBUS_H
#define _MODBUS_H

#include <stdint.h>
#include "mapping.h"

class astring;
class ModbusComm;

class ModbusUpsDriver: public UpsDriver
{
public:
   ModbusUpsDriver(UPSINFO *ups);
   virtual ~ModbusUpsDriver() {}

   static UpsDriver *Factory(UPSINFO *ups)
      { return new ModbusUpsDriver(ups); }

   virtual bool get_capabilities();
   virtual bool read_volatile_data();
   virtual bool read_static_data();
   virtual bool kill_power();
   virtual bool shutdown();
   virtual bool check_state();
   virtual bool Open();
   virtual bool Close();
   virtual bool entry_point(int command, void *data);

   bool write_string_to_ups(const APCModbusMapping::RegInfo &reg, const char *str);
   bool write_int_to_ups(const APCModbusMapping::RegInfo &reg, uint64_t val);
   bool write_dbl_to_ups(const APCModbusMapping::RegInfo &reg, double val);
   bool read_string_from_ups(const APCModbusMapping::RegInfo &reg, astring *val);
   bool read_int_from_ups(const APCModbusMapping::RegInfo &reg, uint64_t *val);
   bool read_dbl_from_ups(const APCModbusMapping::RegInfo &reg, double *val);

private:

   struct CiInfo
   {
      int ci;
      bool dynamic;
      const APCModbusMapping::RegInfo *reg;
   };

   static const CiInfo CI_TABLE[];
   const CiInfo *GetCiInfo(int ci);
   bool UpdateCis(bool dynamic);
   bool UpdateCi(const CiInfo *info, bool forceLiveRead = false);
   bool UpdateCi(int ci, bool forceLiveRead = false);

   // A cluster of adjacent measurement registers (battery %, battery
   // voltage, temperature, load %, output current/voltage, frequency,
   // input status/voltage) were found to intermittently cross-contaminate
   // each other when polled as ~10 separate back-to-back MODBUS
   // transactions -- almost certainly racing the UPS's own internal
   // measurement refresh cycle. Fetching each cluster in a single
   // transaction instead removes the back-to-back polling pattern that
   // triggers it, and is faster besides. Two blocks, not one: 128-146 is
   // fully contiguous per AN176, whereas 147-149 are undocumented and we
   // don't want to risk an ILLEGAL_DATA_ADDRESS exception by reading
   // through them, so 150-151 is fetched as a second, separate block.
   static const uint16_t MEAS_BLOCK_A_ADDR = 128;
   static const uint16_t MEAS_BLOCK_A_NREGS = 19;   // covers 128..146
   static const uint16_t MEAS_BLOCK_B_ADDR = 150;
   static const uint16_t MEAS_BLOCK_B_NREGS = 2;    // covers 150..151

   // Status/error cluster (UPS status, transfer cause, low-battery,
   // power-system/battery-system errors, test/calibration status). Fully
   // contiguous per AN176 with no undocumented gaps, so one block covers
   // the whole thing safely. This is also where CI_NeedReplacement and
   // CI_BatteryPresent both read the exact same register (22) as two
   // separate transactions today -- batching collapses that redundancy
   // too.
   static const uint16_t MEAS_BLOCK_C_ADDR = 0;
   static const uint16_t MEAS_BLOCK_C_NREGS = 27;   // covers 0..26

   void FetchMeasurementBlocks();
   bool GetRegisterData(const APCModbusMapping::RegInfo *reg, uint8_t *out,
      bool forceLiveRead = false);

   uint8_t _measBlockA[MEAS_BLOCK_A_NREGS * 2];
   bool _measBlockAValid;
   uint8_t _measBlockB[MEAS_BLOCK_B_NREGS * 2];
   bool _measBlockBValid;
   uint8_t _measBlockC[MEAS_BLOCK_C_NREGS * 2];
   bool _measBlockCValid;

   time_t _commlost_time;
   ModbusComm *_comm;

   // Consecutive failed CI_STATUS polls in check_state(). A single missed
   // poll used to declare COMMLOST and close the port immediately; that's
   // exactly the granularity at which this UPS's USB/Modbus channel
   // produces brief, self-recovering noise bursts. Require a few in a row
   // before actually declaring the connection lost.
   int _statusFailCount;

   // Last time all three measurement blocks (see FetchMeasurementBlocks())
   // fetched cleanly. check_state()'s CI_STATUS poll reads a single
   // register live and can keep succeeding -- resetting is_commlost() --
   // even while these much larger batched blocks stay desynced, so that
   // path alone can't be relied on to ever trigger a recovery here. See
   // read_volatile_data().
   time_t _lastVolatileOk;
};

#endif   /* _MODBUS_H */
