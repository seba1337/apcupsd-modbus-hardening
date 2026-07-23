/*
 * modbus.cpp
 *
 * Driver for APC MODBUS protocol
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

#include "apc.h"
#include "modbus.h"
#include "astring.h"
#include "ModbusRs232Comm.h"

#ifdef HAVE_MODBUS_USB_DRIVER
#include "ModbusUsbComm.h"
#endif

using namespace APCModbusMapping;

const ModbusUpsDriver::CiInfo ModbusUpsDriver::CI_TABLE[] =
{
//   ci                  dynamic  addr ln  type
   { CI_UPSMODEL,        false,   &REG_MODEL                        },
   { CI_SERNO,           false,   &REG_SERIAL_NUMBER                },
   { CI_IDEN,            false,   &REG_NAME                         },
   { CI_MANDAT,          false,   &REG_MANUFACTURE_DATE             },
   { CI_BATTDAT,         false,   &REG_BATTERY_DATE_SETTING         },
   { CI_NOMOUTV,         false,   &REG_OUTPUT_VOLTAGE_SETTING       },
   { CI_REVNO,           false,   &REG_FW_VERSION                   },
   { CI_BUPSelfTest,     false,   &REG_MODBUS_MAP_ID                }, // Purposely after CI_REVNO
   { CI_NOMPOWER,        false,   &REG_OUTPUT_REAL_POWER_RATING     },
   { CI_NomApparent,     false,   &REG_OUTPUT_APPARENT_POWER_RATING },
   { CI_DSHUTD,          false,   &REG_MOG_TURN_OFF_COUNT_SETTING   },
   { CI_DWAKE,           false,   &REG_MOG_TURN_ON_COUNT_SETTING    },
// { CI_NOMBATTV,        false,   upsAdvBatteryNominalVoltage,
// { CI_AlarmTimer,      false,   upsAdvConfigAlarmTimer,
// { CI_DALARM,          false,   upsAdvConfigAlarm,
// { CI_DLBATT,          false,   upsAdvConfigLowBatteryRunTime,
// { CI_RETPCT,          false,   upsAdvConfigMinReturnCapacity,
// { CI_SENS,            false,   upsAdvConfigSensitivity,
// { CI_EXTBATTS,        false,   upsAdvBatteryNumOfBattPacks,
// { CI_STESTI,          false,   
// { CI_LTRANS,          false,    
// { CI_HTRANS,          false,    

   { CI_VLINE,           true,    &REG_INPUT_0_VOLTAGE              },
   { CI_VOUT,            true,    &REG_OUTPUT_0_VOLTAGE             },
   { CI_OutputCurrent,   true,    &REG_OUTPUT_0_CURRENT             },
   { CI_VBATT,           true,    &REG_BATTERY_VOLTAGE              },
   { CI_FREQ,            true,    &REG_OUTPUT_FREQUENCY             },
   { CI_ITEMP,           true,    &REG_BATTERY_TEMPERATURE          },
   { CI_BATTLEV,         true,    &REG_STATE_OF_CHARGE_PCT          },
   { CI_RUNTIM,          true,    &REG_RUNTIME_REMAINING            },
   { CI_STATUS,          true,    &REG_UPS_STATUS                   },
   { CI_WHY_BATT,        true,    &REG_UPS_STATUS_CHANGE_CAUSE      },// purposely after CI_STATUS
   { CI_Calibration,     true,    &REG_CALIBRATION_STATUS           },
   { CI_ST_STAT,         true,    &REG_BATTERY_TEST_STATUS          },
   { CI_Overload,        true,    &REG_POWER_SYSTEM_ERROR           },
   { CI_NeedReplacement, true,    &REG_BATTERY_SYSTEM_ERROR         },
   { CI_BatteryPresent,  true,    &REG_BATTERY_SYSTEM_ERROR         },
   { CI_Boost,           true,    &REG_INPUT_STATUS                 },
   { CI_Trim,            true,    &REG_INPUT_STATUS                 },
   { CI_LOAD,            true,    &REG_OUTPUT_0_REAL_POWER_PCT      },
   { CI_LoadApparent,    true,    &REG_OUTPUT_0_APPARENT_POWER_PCT  },
   { CI_LowBattery,      true,    &REG_SIMPLE_SIGNALLING_STATUS     },
// { CI_VMIN,            true,    upsAdvInputMinLineVoltage
// { CI_VMAX,            true,    upsAdvInputMaxLineVoltage
                                  
// { CI_ATEMP,           true,    mUpsEnvironAmbientTemperature
// { CI_HUMID,           true,    mUpsEnvironRelativeHumidity
// { CI_BADBATTS,        true,    upsAdvBatteryNumOfBadBattPacks

   { -1 }   /* END OF TABLE */   
};

ModbusUpsDriver::ModbusUpsDriver(UPSINFO *ups) :
   UpsDriver(ups),
   _measBlockAValid(false),
   _measBlockBValid(false),
   _measBlockCValid(false),
   _commlost_time(0),
   _comm(NULL),
   _statusFailCount(0),
   _lastVolatileOk(time(NULL))
{
}

/*
 * Read UPS events. I.e. state changes.
 */
bool ModbusUpsDriver::check_state()
{
   bool ret, onbatt, online;

   // Determine when we need to exit by
   struct timeval exittime, now;
   gettimeofday(&exittime, NULL);
   exittime.tv_sec += _ups->wait_time;

   while(1)
   {
      // Shutdown in progress (see apcupsd_terminate()/do_device()): return
      // promptly instead of running out the rest of wait_time (up to 60s)
      // before the outer loop in do_device() gets a chance to notice.
      if (shutdown_requested)
         return false;

      // See if it's time to exit
      gettimeofday(&now, NULL);
      if (now.tv_sec > exittime.tv_sec ||
          (now.tv_sec == exittime.tv_sec &&
           now.tv_usec >= exittime.tv_usec))
      {
         return false;
      }

      // Try to recover from commlost by reopening port
      ret = !_ups->is_commlost() || _comm->Open(_ups->device);
      if (ret)
      {
         // Remember current online/onbatt state
         online = _ups->is_online();
         onbatt = _ups->is_onbatt();

         // Poll CI_STATUS. This must always be a live read, never a hit
         // on the cached measurement block -- this loop's whole purpose
         // is to detect an online/on-battery transition within about a
         // second, and serving a stale cached status would silently
         // degrade that to the full read_volatile_data() cycle interval.
         // forceLiveRead=true bypasses the block cache entirely for this
         // one call (see GetRegisterData()) -- note this must NOT be
         // done by invalidating _measBlockCValid instead, since
         // GetRegisterData() has no individual-read fallback for
         // block-covered registers when the block is invalid; that
         // combination made this poll fail unconditionally, every time,
         // which in turn meant COMMLOST could never clear.
         ret = UpdateCi(CI_STATUS, /*forceLiveRead=*/true);
      }

      if (ret)
      {
         _statusFailCount = 0;

         // If we were commlost, we're not any more
         if (_ups->is_commlost())
         {
            _ups->clear_commlost();
            generate_event(_ups, CMDCOMMOK);
         }

         // Exit immediately if on battery state changed
         if ((online ^ _ups->is_online()) ||
             (onbatt ^ _ups->is_onbatt()))
         {
            return true;
         }
      }
      else
      {
         time_t now = time(NULL);
         if (_ups->is_commlost())
         {
            // We already know we're commlost.
            // Log an event every 10 minutes.
            if ((now - _commlost_time) >= 10*60)
            {
               _commlost_time = now;
               log_event(_ups, event_msg[CMDCOMMFAILURE].level,
                  event_msg[CMDCOMMFAILURE].msg);
            }
         }
         else
         {
            // This UPS's USB/Modbus channel produces brief,
            // self-recovering noise bursts at roughly the same
            // granularity as this poll -- declaring COMMLOST (and
            // closing/reopening the port) on the very first missed poll
            // was firing on noise, not real communication loss. Require
            // a few consecutive failures before actually declaring it.
            static const int STATUS_FAIL_THRESHOLD = 3;
            if (++_statusFailCount >= STATUS_FAIL_THRESHOLD)
            {
               _commlost_time = now;
               _ups->set_commlost();
               generate_event(_ups, CMDCOMMFAILURE);
               _comm->Close();
            }
         }
      }

      sleep(1);
   }
}

bool ModbusUpsDriver::Open()
{
   if (!_comm)
   {
#ifdef HAVE_MODBUS_USB_DRIVER
      if (_ups->cable.type == CABLE_SMART)
         _comm = new ModbusRs232Comm();
      else
         _comm = new ModbusUsbComm();
#else
      _comm = new ModbusRs232Comm();
#endif
   }

   _ups->fd = 1;
   return _comm->Open(_ups->device);
}

bool ModbusUpsDriver::Close()
{
   bool ret = true;
   if (_comm)
   {
      ret = _comm->Close();
      delete _comm;
      _comm = NULL;
   }

   _ups->fd = -1;
   return ret;
}

/*
 * Setup capabilities structure for UPS
 */
bool ModbusUpsDriver::get_capabilities()
{
   // A capability probe that fails here permanently disables that CI for
   // the life of the connection -- read_volatile_data()/read_static_data()
   // only ever query CIs with UPS_Cap[] already set. This probe runs right
   // after Open(), the noisiest point in the connection (most likely to
   // still be settling from earlier traffic), so a single failed attempt
   // isn't a reliable signal that the register is actually unsupported.
   // Retry a few times before giving up on a given CI.
   static const int CAPABILITY_PROBE_ATTEMPTS = 3;

   FetchMeasurementBlocks();

   for (const CiInfo *info = CI_TABLE; info->reg; info++)
   {
      for (int attempt = 0; attempt < CAPABILITY_PROBE_ATTEMPTS; ++attempt)
      {
         uint8_t data[64];
         if (GetRegisterData(info->reg, data))
         {
            _ups->UPS_Cap[info->ci] = true;
            break;
         }

         // GetRegisterData() no longer decomposes into an individual read
         // for anything a batched block covers (that's exactly what
         // reintroduces the same-size collision risk we're trying to
         // avoid). So if this failed, re-fetch the blocks before retrying
         // -- otherwise a block that failed on the one-time fetch above
         // would fail this exact same way on every remaining attempt for
         // every CI it covers, permanently disabling all of them instead
         // of just needing one more try.
         FetchMeasurementBlocks();
      }
   }

   return _ups->UPS_Cap[CI_STATUS];
}

void ModbusUpsDriver::FetchMeasurementBlocks()
{
   // A couple of retries here is cheap (it's one transaction either way)
   // and means a single transient failure doesn't knock an entire block
   // out for the cycle -- which matters a lot now that GetRegisterData()
   // no longer falls back to decomposed individual reads for anything a
   // block covers (see below).
   static const int BLOCK_FETCH_ATTEMPTS = 2;
   uint8_t *data;

   _measBlockAValid = false;
   for (int i = 0; !_measBlockAValid && i < BLOCK_FETCH_ATTEMPTS; ++i)
   {
      data = _comm->ReadRegister(MEAS_BLOCK_A_ADDR, MEAS_BLOCK_A_NREGS);
      if (data)
      {
         memcpy(_measBlockA, data, sizeof(_measBlockA));
         delete [] data;
         _measBlockAValid = true;
      }
   }

   _measBlockBValid = false;
   for (int i = 0; !_measBlockBValid && i < BLOCK_FETCH_ATTEMPTS; ++i)
   {
      data = _comm->ReadRegister(MEAS_BLOCK_B_ADDR, MEAS_BLOCK_B_NREGS);
      if (data)
      {
         memcpy(_measBlockB, data, sizeof(_measBlockB));
         delete [] data;
         _measBlockBValid = true;
      }
   }

   _measBlockCValid = false;
   for (int i = 0; !_measBlockCValid && i < BLOCK_FETCH_ATTEMPTS; ++i)
   {
      data = _comm->ReadRegister(MEAS_BLOCK_C_ADDR, MEAS_BLOCK_C_NREGS);
      if (data)
      {
         memcpy(_measBlockC, data, sizeof(_measBlockC));
         delete [] data;
         _measBlockCValid = true;
      }
   }
}

bool ModbusUpsDriver::GetRegisterData(const APCModbusMapping::RegInfo *reg,
   uint8_t *out, bool forceLiveRead)
{
   const unsigned int nbytes = reg->nregs * sizeof(uint16_t);

   if (!forceLiveRead)
   {
      bool inBlockA = reg->addr >= MEAS_BLOCK_A_ADDR &&
         (reg->addr + reg->nregs) <= (MEAS_BLOCK_A_ADDR + MEAS_BLOCK_A_NREGS);
      bool inBlockB = reg->addr >= MEAS_BLOCK_B_ADDR &&
         (reg->addr + reg->nregs) <= (MEAS_BLOCK_B_ADDR + MEAS_BLOCK_B_NREGS);
      bool inBlockC = reg->addr >= MEAS_BLOCK_C_ADDR &&
         (reg->addr + reg->nregs) <= (MEAS_BLOCK_C_ADDR + MEAS_BLOCK_C_NREGS);

      if (inBlockA || inBlockB || inBlockC)
      {
         // Deliberately no fallback to an individual read here, even if
         // the corresponding block fetch failed this cycle. Decomposing
         // back into a one-register-at-a-time read is exactly what
         // reintroduces the same-size stale-response collision risk that
         // batching this cluster was meant to close -- many of these
         // registers are a single word (identical response size), so a
         // stale duplicate from one can slip in and be misattributed to
         // another. Better to skip this field for the cycle (it'll retry
         // next cycle, and FetchMeasurementBlocks() already retries the
         // block fetch itself a couple of times before giving up) than
         // to risk a wrong value.
         //
         // This entire check is bypassed when forceLiveRead is set (see
         // check_state()'s CI_STATUS poll) -- that call's whole point is
         // a guaranteed-fresh read, and there is no reasonable fallback
         // for it to skip a cycle the way this one can.
         if (inBlockA && _measBlockAValid)
         {
            memcpy(out, _measBlockA + (reg->addr - MEAS_BLOCK_A_ADDR) * 2, nbytes);
            return true;
         }
         if (inBlockB && _measBlockBValid)
         {
            memcpy(out, _measBlockB + (reg->addr - MEAS_BLOCK_B_ADDR) * 2, nbytes);
            return true;
         }
         if (inBlockC && _measBlockCValid)
         {
            memcpy(out, _measBlockC + (reg->addr - MEAS_BLOCK_C_ADDR) * 2, nbytes);
            return true;
         }
         return false;
      }
   }

   // Outside all batched blocks (e.g. the static identity/nameplate
   // fields), or the caller explicitly requested a guaranteed-fresh
   // individual read bypassing the cache entirely -- no batch
   // alternative applies here either way, so an individual read is the
   // only option. Outside the blocks, these are read once at startup and
   // rarely revisited, so the same-size collision window is far smaller
   // in practice.
   uint8_t *data = _comm->ReadRegister(reg->addr, reg->nregs);
   if (!data)
      return false;

   memcpy(out, data, nbytes);
   delete [] data;
   return true;
}

const ModbusUpsDriver::CiInfo *ModbusUpsDriver::GetCiInfo(int ci)
{
   const CiInfo *info = CI_TABLE;
   while (info->reg && info->ci != ci)
      info++;
   return info->reg ? info : NULL;
}

bool ModbusUpsDriver::UpdateCi(int ci, bool forceLiveRead)
{
   const CiInfo *info = GetCiInfo(ci);
   return info ? UpdateCi(info, forceLiveRead) : false;
}

bool ModbusUpsDriver::UpdateCi(const CiInfo *info, bool forceLiveRead)
{
   uint8_t data[64];
   if (!GetRegisterData(info->reg, data, forceLiveRead))
   {
      Dmsg(0, "%s: Failed reading %u/%u\n", __func__,
         info->reg->addr, info->reg->nregs);
      return false;
   }

   const unsigned int nbytes = info->reg->nregs * sizeof(uint16_t);

   uint64_t uint = 0;
   int64_t sint = 0;
   double dbl = 0;
   astring str;

   if (info->reg->type == DT_STRING)
   {
      // String type...
      for (unsigned int i = 0; i < nbytes; ++i)
      {
         // Extract from response message
         // Constrain to valid ASCII subset defined by APC
         if (data[i] < 0x20 || data[i] > 0x7E)
            str += ' ';
         else
            str += data[i];
      }

      // Strip leading and trailing whitespace
      str.trim();
   }
   else
   {
      // Integer type...
      for (unsigned int i = 0; i < nbytes; ++i)
      {
         // Extract from response message, MSB first
         uint = (uint << 8) | data[i];
      }

      if (info->reg->type == DT_INT)
      {
         // Sign extend
         sint = uint;
         sint <<= (8 * (sizeof(uint) - nbytes));
         sint >>= (8 * (sizeof(uint) - nbytes));
         // Scale
         if (info->reg->scale)
            dbl = (double)sint / (1ULL << info->reg->scale);
      }
      else if (info->reg->scale)
      {
         // Scale
         dbl =  (double)uint / (1ULL << info->reg->scale);
      }
   }

   astring tmpstr;
   struct tm tmp;
   time_t date;

   // Everything above this point is slow, retry-prone MODBUS I/O and
   // pure local decoding -- none of it touches _ups. Lock only for the
   // brief moment of copying the already-decoded value into the shared
   // struct below, instead of holding the lock for the whole fetch. A
   // single read cycle's I/O can take anywhere from a few seconds to
   // (with retries) tens of seconds on this UPS's USB/HID gateway, and
   // holding _ups's lock for that whole span was starving the NIS
   // server (apcaccess) of the lock for just as long, causing client
   // timeouts even though the daemon itself was healthy and working.
   write_lock(_ups);
   switch (info->ci)
   {
   case CI_UPSMODEL:
      Dmsg(80, "Got CI_UPSMODEL: %s\n", str.str());
      strlcpy(_ups->upsmodel, str, sizeof(_ups->upsmodel));
      break;
   case CI_SERNO:
      Dmsg(80, "Got CI_SERNO: %s\n", str.str());
      strlcpy(_ups->serial, str, sizeof(_ups->serial));
      break;
   case CI_IDEN:
      Dmsg(80, "Got CI_IDEN: %s\n", str.str());
      strlcpy(_ups->upsname, str, sizeof(_ups->upsname));
      break;
   case CI_REVNO:
      Dmsg(80, "Got CI_REVNO: %s\n", str.str());
      strlcpy(_ups->firmrev, str, sizeof(_ups->firmrev));
      break;
   case CI_BUPSelfTest:
      Dmsg(80, "Got MODBUS_MAP_ID: %s\n", str.str());
      tmpstr = _ups->firmrev; // Append to REVNO
      tmpstr += " / " + str;
      strlcpy(_ups->firmrev, tmpstr, sizeof(_ups->firmrev));
      break;
   case CI_MANDAT:
      Dmsg(80, "Got CI_MANDAT: %llu\n", uint);
      // uint is in days since 1/1/2000
      date = ModbusRegTotime_t(uint);
      gmtime_r(&date, &tmp);
      strftime(_ups->birth, sizeof(_ups->birth), "%Y-%m-%d", &tmp);
      break;
   case CI_BATTDAT:
      Dmsg(80, "Got CI_BATTDAT: %llu\n", uint);
      // uint is in days since 1/1/2000
      date = ModbusRegTotime_t(uint);
      gmtime_r(&date, &tmp);
      strftime(_ups->battdat, sizeof(_ups->battdat), "%Y-%m-%d", &tmp);
      break;
   case CI_NOMOUTV:
      Dmsg(80, "Got CI_NOMOUTV: %llx\n", uint);
      if (uint & OVS_100VAC)
         _ups->NomOutputVoltage = 100;
      else if (uint & OVS_120VAC)
         _ups->NomOutputVoltage = 120;
      else if (uint & OVS_200VAC)
         _ups->NomOutputVoltage = 200;
      else if (uint & OVS_208VAC)
         _ups->NomOutputVoltage = 208;
      else if (uint & OVS_220VAC)
         _ups->NomOutputVoltage = 220;
      else if (uint & OVS_230VAC)
         _ups->NomOutputVoltage = 230;
      else if (uint & OVS_240VAC)
         _ups->NomOutputVoltage = 240;
      else
         _ups->NomOutputVoltage = -1;
      break;
   case CI_NOMPOWER:
      Dmsg(80, "Got CI_NOMPOWER: %llu\n", uint);
      _ups->NomPower = uint;
      break;
   case CI_NomApparent:
      Dmsg(80, "Got CI_NomApparent: %llu\n", uint);
      _ups->NomApparentPower = uint;
      break;
   case CI_DSHUTD:
      Dmsg(80, "Got CI_DSHUTD: %lld\n", sint);
      _ups->dshutd = sint;
      break;
   case CI_DWAKE:
      Dmsg(80, "Got CI_DWAKE: %lld\n", sint);
      _ups->dwake = sint;
      break;
   case CI_LowBattery:
      Dmsg(80, "Got CI_LowBattery: %llu\n", uint);
      _ups->set_battlow(uint & SSS_SHUTDOWN_IMMINENT);
      break;
#if 0
   case CI_STESTI:
      Dmsg(80, "Got CI_STESTI: %llx\n", uint);
      if (uint & 0x1)
         strlcpy(_ups->selftest, "OFF", sizeof(_ups->selftest));
      else if (uint & 0x2)
         strlcpy(_ups->selftest, "ON", sizeof(_ups->selftest));
      else if (uint & 0x14)
         strlcpy(_ups->selftest, "168", sizeof(_ups->selftest));
      else if (uint & 0x28)
         strlcpy(_ups->selftest, "336", sizeof(_ups->selftest));
      break;
   case CI_LTRANS:
      Dmsg(80, "Got CI_LTRANS: %llu\n", uint);
      _ups->lotrans = uint;
      break;
   case CI_HTRANS:
      Dmsg(80, "Got CI_HTRANS: %llu\n", uint);
      _ups->hitrans = uint;
      break;
#endif
   case CI_LOAD:
      Dmsg(80, "Got CI_LOAD: %f\n", dbl);
      _ups->UPSLoad = dbl;
      break;
   case CI_LoadApparent:
      Dmsg(80, "Got CI_LoadApparent: %f\n", dbl);
      _ups->LoadApparent = dbl;
      break;
   case CI_VLINE:
      Dmsg(80, "Got CI_VLINE: %f\n", dbl);
      _ups->LineVoltage = dbl;
      break;
   case CI_VOUT:
      Dmsg(80, "Got CI_VOUT: %f\n", dbl);
      _ups->OutputVoltage = dbl;
      break;
   case CI_OutputCurrent:
      Dmsg(80, "Got CI_OutputCurrent: %f\n", dbl);
      _ups->OutputCurrent = dbl;
      break;
   case CI_VBATT:
      Dmsg(80, "Got CI_VBATT: %f\n", dbl);
      _ups->BattVoltage = dbl;
      break;
   case CI_FREQ:
      Dmsg(80, "Got CI_FREQ: %f\n", dbl);
      _ups->LineFreq = dbl;
      break;
   case CI_ITEMP:
      Dmsg(80, "Got CI_ITEMP: %f\n", dbl);
      _ups->UPSTemp = dbl;
      break;
   case CI_BATTLEV:
      Dmsg(80, "Got CI_BATTLEV: %f\n", dbl);
      _ups->BattChg = dbl;
      break;
   case CI_RUNTIM:
      Dmsg(80, "Got CI_RUNTIM: %llu\n", uint);
      _ups->TimeLeft = uint / 60; // secs to mins
      break;
   case CI_ST_STAT:
      Dmsg(80, "Got CI_ST_STAT: 0x%llx\n", uint);
      if (uint & (BTS_PENDING|BTS_IN_PROGRESS))
         _ups->testresult = TEST_INPROGRESS;
      else if (uint & BTS_PASSED)
         _ups->testresult = TEST_PASSED;
      else if (uint & BTS_FAILED)
         _ups->testresult = TEST_FAILED;
      else if (uint == 0)
         _ups->testresult = TEST_NONE;
      else
         _ups->testresult = TEST_UNKNOWN;
      break;
   case CI_WHY_BATT:
      Dmsg(80, "Got CI_WHY_BATT: %llx\n", uint);
      // Only update if we're on battery now
      if (_ups->is_onbatt())
      {
         switch(uint)
         {
         case USCC_HIGH_INPUT_VOLTAGE:
            _ups->lastxfer = XFER_OVERVOLT;
            break;
         case USCC_LOW_INPUT_VOLTAGE:
            _ups->lastxfer = XFER_UNDERVOLT;
            break;
         case USCC_DISTORTED_INPUT:
            _ups->lastxfer = XFER_NOTCHSPIKE;
            break;
         case USCC_RAPID_CHANGE:
            _ups->lastxfer = XFER_RIPPLE;
            break;
         case USCC_HIGH_INPUT_FREQ:
         case USCC_LOW_INPUT_FREQ:
         case USCC_FREQ_PHASE_DIFF:
            _ups->lastxfer = XFER_FREQ;
            break;
         case USCC_AUTOMATIC_TEST:
            _ups->lastxfer = XFER_SELFTEST;
            break;
         case USCC_LOCAL_UI_CMD:
         case USCC_PROTOCOL_CMD:
            _ups->lastxfer = XFER_FORCED;
            break;
         default:
            _ups->lastxfer = XFER_UNKNOWN;
            break;
         }
      }
      break;
   case CI_STATUS:
      Dmsg(80, "Got CI_STATUS: 0x%llx\n", uint);
      // Clear the following flags: only one status will be TRUE
      _ups->clear_online();
      _ups->clear_onbatt();
      if (uint & US_ONLINE)
         _ups->set_online();
      else if (uint & US_ONBATTERY)
         _ups->set_onbatt();
      break;
   case CI_Calibration:
      Dmsg(80, "Got CI_Calibration: 0x%llx\n", uint);
      _ups->set_calibration(uint & (CS_PENDING|CS_IN_PROGRESS));
      break;
   case CI_Overload:
      Dmsg(80, "Got CI_Overload: 0x%llx\n", uint);
      _ups->set_overload(uint & PSE_OUTPUT_OVERLOAD);
      break;
   case CI_NeedReplacement:
      Dmsg(80, "Got CI_NeedReplacement: 0x%llx\n", uint);
      _ups->set_replacebatt(uint & BSE_NEEDS_REPLACEMENT);
      break;
   case CI_BatteryPresent:
      Dmsg(80, "Got CI_BatteryPresent: 0x%llx\n", uint);
      _ups->set_battpresent(!(uint & BSE_DISCONNECTED));
      break;
   case CI_Boost:
      Dmsg(80, "Got CI_Boost: 0x%llx\n", uint);
      _ups->set_boost(uint & IS_BOOST);
      break;
   case CI_Trim:
      Dmsg(80, "Got CI_Trim: 0x%llx\n", uint);
      _ups->set_trim(uint & IS_TRIM);
      break;
   }
   write_unlock(_ups);

   return true;
}

bool ModbusUpsDriver::UpdateCis(bool dynamic)
{
   // Best-effort: a single register that hits a transient sync hiccup
   // shouldn't block every other register in this pass from updating.
   // Keep going and report success if at least one field made it through
   // this cycle -- the next cycle will retry whatever didn't.
   bool any_ok = false;

   for (unsigned int i = 0; CI_TABLE[i].ci != -1; i++)
   {
      if (_ups->UPS_Cap[CI_TABLE[i].ci] && CI_TABLE[i].dynamic == dynamic)
      {
         if (UpdateCi(CI_TABLE+i))
            any_ok = true;
      }
   }

   return any_ok;
}

/*
 * Read UPS info that remains unchanged -- e.g. transfer
 * voltages, shutdown delay, ...
 *
 * This routine is called once when apcupsd is starting
 */
bool ModbusUpsDriver::read_static_data()
{
   // UpdateCis()/UpdateCi() now take _ups's lock themselves, briefly,
   // only around the in-memory apply step -- not for the whole (slow,
   // retry-prone) MODBUS I/O -- so no lock is taken here.
   //
   // This runs once at startup. Unlike read_volatile_data(), which gets a
   // fresh attempt every poll cycle, a static field that fails here never
   // gets read again for the life of the connection. Retry the whole pass
   // a few times -- UpdateCi() only writes a field into _ups on success,
   // so a failed retry attempt can't clobber an already-good value an
   // earlier attempt already wrote.
   static const int STATIC_DATA_ATTEMPTS = 3;

   bool ret = false;
   for (int attempt = 0; attempt < STATIC_DATA_ATTEMPTS; ++attempt)
      ret = UpdateCis(false) || ret;

   return ret;
}

/*
 * Read UPS info that changes -- e.g. Voltage, temperature, ...
 *
 * This routine is called once every N seconds to get
 * a current idea of what the UPS is doing.
 */
bool ModbusUpsDriver::read_volatile_data()
{
   // Refresh the batched measurement-cluster cache once per cycle, up
   // front, so every CI in UpdateCis() below sees consistent, freshly
   // fetched data instead of each issuing its own back-to-back request.
   // Doesn't touch _ups, so no lock needed here either.
   FetchMeasurementBlocks();

   // If every block came back clean, we're healthy -- reset the clock.
   // Otherwise, once we've gone too long without a single fully-clean
   // cycle, the per-block retries in FetchMeasurementBlocks() aren't
   // recovering on their own: this UPS's USB/MODBUS channel is stuck in a
   // longer-lived desync than the brief, self-recovering noise bursts
   // those retries (and check_state()'s own COMMLOST handling) are tuned
   // for. Force a close/reopen to make the OS and UPS renegotiate the USB
   // HID session. The threshold is set well above the longest
   // self-recovering burst seen historically (well under two minutes) so
   // this doesn't fire on ordinary noise, only on a genuine stuck channel.
   static const time_t VOLATILE_STALE_RECOVERY_SECS = 180;

   if (_measBlockAValid && _measBlockBValid && _measBlockCValid)
   {
      _lastVolatileOk = time(NULL);
   }
   else if (time(NULL) - _lastVolatileOk >= VOLATILE_STALE_RECOVERY_SECS)
   {
      log_event(_ups, LOG_WARNING,
         "MODBUS: measurement data stale for over %d seconds -- "
         "reopening device to force resync",
         (int)VOLATILE_STALE_RECOVERY_SECS);
      _comm->Close();
      _comm->Open(_ups->device);
      _lastVolatileOk = time(NULL);
   }

   bool ret = UpdateCis(true);
   if (ret)
   {
      // Successful query, update timestamp
      write_lock(_ups);
      _ups->poll_time = time(NULL);
      write_unlock(_ups);
   }
   return ret;
}

bool ModbusUpsDriver::kill_power()
{
   return write_int_to_ups(REG_SIMPLE_SIGNALLING_CMD, SSC_REQUEST_SHUTDOWN);
}

bool ModbusUpsDriver::shutdown()
{
   return write_int_to_ups(REG_SIMPLE_SIGNALLING_CMD, SSC_REMOTE_OFF);
}

bool ModbusUpsDriver::entry_point(int command, void *data)
{
   switch (command) {
   case DEVICE_CMD_CHECK_SELFTEST:
      Dmsg(80, "Checking self test.\n");
      /* Reason for last transfer to batteries */
      if (_ups->UPS_Cap[CI_WHY_BATT] && UpdateCis(true))
      {
         Dmsg(80, "Transfer reason: %d\n", _ups->lastxfer);

         /* See if this is a self test rather than power failure */
         if (_ups->lastxfer == XFER_SELFTEST) {
            /*
             * set Self Test start time
             */
            _ups->SelfTest = time(NULL);
            Dmsg(80, "Self Test time: %s", ctime(&_ups->SelfTest));
         }
      }
      break;

   case DEVICE_CMD_GET_SELFTEST_MSG:
   default:
      return false;
   }

   return true;
}

bool ModbusUpsDriver::write_string_to_ups(const APCModbusMapping::RegInfo &reg, const char *str)
{
   if (reg.type != DT_STRING)
      return false;

   int len = reg.nregs * sizeof(uint16_t);
   astring strcpy = str;
   if (strcpy.len() > len)
   {
      // Truncate
      strcpy = strcpy.substr(0, len);
   }
   else
   {
      // Pad
      while (strcpy.len() < len)
         strcpy += ' ';
   }

   return _comm->WriteRegister(reg.addr, reg.nregs, (const uint8_t*)strcpy.str());
}

bool ModbusUpsDriver::write_int_to_ups(const APCModbusMapping::RegInfo &reg, uint64_t val)
{
   if (reg.type != DT_UINT && reg.type != DT_INT)
      return false;

   unsigned int len = reg.nregs * sizeof(uint16_t);
   uint8_t data[len];
   for (unsigned int i = 0; i < len; ++i)
      data[i] = val >> (8*(len-i-1));

   return _comm->WriteRegister(reg.addr, reg.nregs, data);
}

bool ModbusUpsDriver::read_string_from_ups(const APCModbusMapping::RegInfo &reg, astring *val)
{
   if (reg.type != DT_STRING)
      return false;

   unsigned int len = reg.nregs * sizeof(uint16_t);
   uint8_t *data = _comm->ReadRegister(reg.addr, reg.nregs);
   if (!data)
      return false;

   *val = "";
   for (unsigned int i = 0; i < len; ++i)
   {
      // Extract from response message
      // Constrain to valid ASCII subset defined by APC
      if (data[i] < 0x20 || data[i] > 0x7E)
         *val += ' ';
      else
         *val += data[i];
   }
   delete [] data;

   // Strip leading and trailing whitespace
   val->trim();

   return true;
}

bool ModbusUpsDriver::read_int_from_ups(const APCModbusMapping::RegInfo &reg, uint64_t *val)
{
   if (reg.type != DT_UINT && reg.type != DT_INT)
      return false;

   unsigned int len = reg.nregs * sizeof(uint16_t);
   uint8_t *data = _comm->ReadRegister(reg.addr, reg.nregs);
   if (!data)
      return false;

   *val = 0;
   for (unsigned int i = 0; i < len; ++i)
      *val = (*val << 8) | data[i];
   delete [] data;

   return true;
}

bool ModbusUpsDriver::read_dbl_from_ups(const APCModbusMapping::RegInfo &reg, double *val)
{
   if (reg.type != DT_UINT && reg.type != DT_INT)
      return false;

   // Read raw unscaled value from UPS
   uint64_t uint;
   if (!read_int_from_ups(reg, &uint))
      return false;

   // Scale
   if (reg.type == DT_INT)
   {
      // Sign extend
      int64_t sint = uint;
      unsigned int nbytes = reg.nregs * sizeof(uint16_t);
      sint <<= (8 * (sizeof(uint) - nbytes));
      sint >>= (8 * (sizeof(uint) - nbytes));
      *val = (double)sint / (1ULL << reg.scale);
   }
   else
   {
      *val = (double)uint / (1ULL << reg.scale);
   }

   return true;
}
