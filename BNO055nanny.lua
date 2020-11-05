-- jmdasnoy scripsit 2020 copyright MIT license
-- ESP32 nodeMCU Lua 5.1 handler for Bosch Sensortech IMU BNO055
-- according to datasheet: BST-BNO055-DS000-16 | Revision 1.6 | February 2020
--
-- *** TODO FIX ME
-- power modes PWR_MODE register

-- axis remap
-- defaut sensor settings

-- registers in pages ?
-- soft iron calibration and matrix reuse ?
-- heading, tilt and pitch offsets to correct mounting imprecision
-- tilt and pitch could be automatic if sufficient period of no-change ?
--
-- Startup and mode mode changes:
-- startup time (from OFF to config mode) is typically 400 ms
-- power on reset time (from reset to config mode) is typ 600 ms
-- during these periods, the i2c will not respond !
-- time to switch from config to operation is 7 ms
-- time to switch from operation to config is 19 ms
--
-- Default orientation:
-- the dot on the chip is in the North-East corner
-- North is +Y, East is +X, Up is +Z
-- heading is 0 for North, 90 for East ie increasing when CW on Z axis
-- pitch increasing for nose-down (Z moving towards Y) ie CW on Y axis
-- roll increasing for right wing up (X moving towards Z) ie CW on X axis
-- only the direction of increasing pitch can be chosen (Android vs Windows convention)
--
-- accelerometer is factory calibrated
-- gyro calibration needs a few seconds of stationary
-- magneto calibration 3 (&2) are ok, 1 requires figure-8 calibration, 0 means environment changed
-- for COMPASS mode, no gyro calibration !!! CALIB_STAT is Oxc3
-- not using Built In Self Test BIST
-- The CALIB_STAT readout (bits: ssggaamm) is buggy, system and accelerometer bits should be ignored !
--
-- *** FIX ME store and retrieve calibration values for faster/easier startup
--
-- Notes:
--
-- the ESP-32 Huzzah has weakup (10 K) pullups on the i2c bus pins
-- additional pullups SCL 3K3 SDA 2K2 are advised for reliable operation
--
-- The BNO uses clock stretching. With fast i2c clocks, this may trigger the ESP32 timeout on hardware i2c subsystems.
-- A i2c clock speed of 30KHz or slower is advised.
--
-- The CALIB_STAT readout (bits: ssggaamm) is buggy, system and accelerometer bits should be ignored !
--
-- populate table with register addresses, reset/read values, symbolic settings, expected values ie settings
local function populatetable()
  local bno={}
-- i2c particulars and defaults
  bno.i2cinterface =i2c.HW0
  bno.i2caddress = 0x28  -- if pin ADR/COM3 is low (like on Adafruit breakout board), otherwise 0x29
-- 
  bno.register = {} -- register hardware addresses
  bno.value = {} -- last read value from chip
  bno.lookup = {} -- lookup table for symbolic value settings
  bno.setting = {} -- expected/to be value as provided by configure() function
--
  bno.register.CHIP_ID = 0x0
  bno.value.CHIP_ID = 0xa0
--
--  bno.register.ACC_ID = 0x1
  bno.value.ACC_ID = 0xfb
--
--  bno.register.MAG_ID = 0x2
  bno.value.MAG_ID = 0x32
--
--  bno.register.GYR_ID = 0x3
  bno.value.GYR_ID = 0x0f
--
--  bno.register.SW_REV_ID_LSB=0x4
  bno.value.SW_REV_ID_LSB = 0x11 -- higher than 0x08 mentioned in datasheet
--
--  bno.register.SW_REV_ID_MSB = 0x5
  bno.value.SW_REV_ID_MSB = 0x03
--
-- bno.register.BL_REV_ID = 0x06
-- bno.value.BL_REV_ID = 0x15
-- bno.register.PageID = 0x7
  bno.register.ACC_DATA_X_LSB = 0x8
  bno.register.MAG_DATA_X_LSB = 0x0e
  bno.register.GYR_DATA_X_LSB = 0x14
-- note lower case for fusion data registers
  bno.register.EUL_heading_LSB = 0x1a
  bno.register.QUA_data_w_LSB = 0x20
  bno.register.LIA_data_X_LSB = 0x28
  bno.register.GRAV_data_X_LSB = 0x2e
--
  bno.register.TEMP = 0x34
  bno.register.CALIB_STAT = 0x35
  bno.register.ST_RESULT = 0x36
  bno.lookup.ST_RESULT = { OK = 0xf}
  bno.register.SYS_STATUS = 0x39
  bno.lookup.SYS_STATUS = { IDLE = 0, ERROR = 1, FUSIONRUN = 5, RAWRUN = 6 }
-- other SYS_STATUS values  InitializingPeripherals = 2, SystemInitialisation = 3, Executing Selftest = 4
--  bno.register.SYS_ERR = 0x3a  -- contents are defined only if SYS_STATUS is ERROR
  bno.register.UNIT_SEL = 0x3b
  bno.value.UNIT_SEL = 0x80
--
  bno.register.OPR_MODE = 0x3d
  bno.value.OPR_MODE = 0x00  -- According to datasheet should be 0x1C (NDOF) on reset?!? should rather be CONFIG mode after reset ?
  bno.lookup.OPR_MODE = {CONFIGMODE = 0, ACCONLY = 1, MAGONLY = 2, GYROONLY = 3, ACCMAG = 4, ACCGYRO = 5, MAGGYRO = 6, AMG = 7, IMU = 8, COMPASS = 9, M4G = 0xa, NDOF_FMC_OFF = 0xb, NDOF = 0xc}

--
  bno.register.SYS_TRIGGER = 0x3f
  bno.lookup.SYS_TRIGGER = { RST_SYS = 0x20, Self_Test = 0x1}
  bno.register.TEMP_SOURCE = 0x40
  bno.value.TEMP_SOURCE = 0x2
  bno.lookup.TEMP_SOURCE = { ACC = 0x0, GYR = 0x1}
--
  bno.register.ACC_OFFSET_X_LSB  = 0x55
--
-- error counters and limits
  bno.i2c_err_count = 0
  bno.i2c_err_max = 5 -- max number of cumulative i2c errors
  bno.reset_count = 0
  bno.reset_max = 5  -- max number of chip reset attempts
  bno.wait_count = 0
  bno.wait_max = 5  -- max number of tick waits wait setting OPR_MODE to CONFIGMODE or for initialisation to complete
-- default name for bno
  bno.name = "BNO055"
-- health indicator
  bno.health = "STOP" -- other values are INIT , CALIB, RUN or ERROR
-- values derived by handler routines
  bno.HEADING = 0
  bno.ROLL = 0
  bno.PITCH = 0
  bno.W = 0
  bno.X = 0
  bno.Y = 0
  bno.Z = 0
  bno.GRAV_X = 0
  bno.GRAV_Y = 0
  bno.GRAV_Z = 0
  bno.LIA_X = 0
  bno.LIA_Y = 0
  bno.LIA_Z = 0
  bno.GYRO_X = 0
  bno.GYRO_Y = 0
  bno.GYRO_Z = 0
  bno.MAG_X = 0
  bno.MAG_Y = 0
  bno.MAG_Z = 0
  bno.ACC_X = 0
  bno.ACC_Y = 0
  bno.ACC_Z = 0
--
  return bno
end
--
local function tick( self ) -- execute function acccording to FSM state
  self:state()
end
-- functions for FSM states
--
-- forward declarations of state functions and associates
local checkchip
local checkpost
local checkstatus
local configreset
local loadcalib
local checkconfig
local checkcalib
local showcalib
local monitor
-- forward declarations of read functions
local getmagnetic
local getgyro
local getacc
local geteuler
local getquat
local getgravity
local getlia
local getmagnetic
local getgyro
-- forward declarations of functions called from states
local readbytes
local writebytes
local writeandreadbackbytes
local resetchip
local forceconfig
local fsm_disable
local i2c_ok
local i2c_err
local chip_err
local post_err
local status_err
local status_wait
local status_nowait
--
checkchip = function( self ) -- FSM initial state: attempt to get a response from the bus/address and compare with expected signature
---[[
  print( "checking i2c connectivity and chip signature" )
--]]
  readbytes( self.i2cinterface, self.i2caddress, self.register.CHIP_ID, 6,
    function( data, ack )
      if not ack then
        i2c_err( self )
      else
        i2c_ok( self )
        if data == self.signature then
---[[
          print ( "Chip signature ok")
--]]
          self.state = checkpost
          self.health = "INIT"
        else
          chip_err( self, data )
        end
      end
    end
  )
end
--
checkpost = function( self )  -- FSM state: check POST
---[[
  print( "checking BNO055 self test results" )
--]]
  readbytes( self.i2cinterface, self.i2caddress, self.register.ST_RESULT, 1,
    function( data, ack )
      if not ack then
        i2c_err( self )
      else
        i2c_ok( self )
        if data:byte( 1,1 ) == self.lookup.ST_RESULT.OK  then
          self.state = checkstatus
---[[
          print ( "BNO055 POST ok" )
--]]
        else
          post_err( self, data:byte( 1,1 ) )
        end
      end
    end
  )
end
--
checkstatus = function ( self )  -- FSM state: check SYS_STATUS and SYS_ERR
---[[
  print( "checking BNO055 system status and errors" )
--]]
  readbytes( self.i2cinterface, self.i2caddress, self.register.SYS_STATUS, 2,
    function( data, ack )
      if not ack then
        i2c_err( self )
      else
        i2c_ok( self )
        local status = data:byte( 1, 1 )
        local error = data:byte( 2 ,2 )
---[[
        print ( "BNO055 SYS_STATUS SYS_ERR: ",status,error)
--]]
        if status == self.lookup.SYS_STATUS.IDLE then
          status_nowait( self )
          forceconfig( self )
          self.state = loadcalib  -- system idle, ok to configure
        elseif status == self.lookup.SYS_STATUS.ERROR then -- system error
-- *** FIX ME sys_error codes of 4 and above can be caused by usr program, not chip failure
-- *** this status seems to persist, no hint on how to clear this except with a full reset
          status_nowait( self )
          status_err( self, error )
        elseif status == self.lookup.SYS_STATUS.FUSIONRUN or status == self.lookup.SYS_STATUS.RAWRUN then
---[[
          print( "System already running, must go back to config mode" )
--]]
          status_nowait( self )
          self.state = configreset  -- system running, need to set OPR_MODE to CONFIGMODE before configuring
        else
---[[
          print( "BNO055 still busy initialising, please wait...")
--]]
          status_wait( self, status ) -- codes 2,3,4 for peripheral init, system init and selftest, wait for next tick or timeout
        end
      end
    end
  )
end
--
configreset = function( self ) -- FSM state: set OPR_MODE to CONFIGMODE, clear effective read values to reset values
---[[
  print( "Setting OPR_MODE back to CONFIGMODE" )
--]]
  writeandreadbackbytes( self.i2cinterface, self.i2caddress, self.register.OPR_MODE, self.lookup.OPR_MODE.CONFIGMODE,
    function( data, ack )
      if not ack then
        i2c_err( self )
      else
        i2c_ok( self )
        if data:byte( 1, 1) == self.lookup.OPR_MODE.CONFIGMODE then
          status_nowait( self )
          forceconfig( self )
          self.state = loadcalib
        else
          status_wait( self, data ) -- try again until time out
---[[
          print( "Attempt to put chip back into config mode did not succeed, trying again" )
--]]
        end
      end
    end
  )
end
--
loadcalib = function( self )  -- FSMstate: load and apply previously stored calibration values, if any
  local calibfn = self.name.."calibration"
  if file.exists( calibfn ) then
---[[
    print( "found calibration file: " , calibfn )
--]]
    dofile( calibfn )
    if self.calib then
---[[
      print( "calibration recovery succesful, writing data to chip" )
--]]
---
      writebytes( self.i2cinterface, self.i2caddress, self.register.ACC_OFFSET_X_LSB, self.calib ,
        function( data, ack )
          if not ack then
            i2c_err( self )
          else
            i2c_ok( self )
---[[
      print( "sucessfully wrote back calibration data to chip" )
--]]
            self.state = checkconfig
          end
        end
      )
---
    else
---[[
      print( "calibration recovery did not succeed" )
--]]
      self.state = checkconfig
    end
  else
---[[
    print( "calibration file not found: " , calibfn )
--]]
    self.state = checkconfig
  end
end
--
checkconfig = function ( self )  -- FSM state: check configuration requests and apply them
---[[
  print( "checking/setting BNO055 configuration" )
--]]
-- check settings needed,changing OPR_MODE must be done last when all others are ok, so there will be time for OPR_MODE to establish before next tick
  local tryk, tryv
  for k,v in pairs( self.setting ) do
    print( k, v, self.value[ k] )
    if v ~= self.value[ k ] then
      if not (k == "OPR_MODE" and tryk) then tryk, tryv = k, v end -- ignore OPR_MODE request until last
--[[
      print( "setup required for register", k)
--]]
--[[
    else
      print( "setting ok for register: ", k)
--]]
    end
  end
--
  if tryk then
--[[
    print( " attempting configuration of : ", tryk, tryv )
--]]
    writeandreadbackbytes( self.i2cinterface, self.i2caddress, self.register[ tryk ], tryv,
      function( data, ack )
        if not ack then
          i2c_err( self )
        else
          i2c_ok( self )
          local newv = data:byte( 1, 1 )
          self.value[ tryk] = newv
--[[
          print ( "Read back of configured value for register: ", tryk, newv )
--]]
        end
      end
    )
  else
-- [[
   print ( "Configuration complete" )
--]]
   self.state = checkcalib -- move to next FSM state
   self.health = "CALIB"
  end
---[[
  print( "exiting checkconfig BNO055" )
--]]
end
--
checkcalib = function( self )  -- FSM state: check for effective calibration
--[[
  print( "Check for BNO055 calibration" )
--]]
  readbytes( self.i2cinterface, self.i2caddress, self.register.CALIB_STAT, 1,
    function( data, ack )
      if not ack then
        i2c_err( self )
      else
        i2c_ok( self )
        local calibvalue = data:byte( 1 )
        self.value.CALIB_STAT = calibvalue
-- very kludgy, system and accelerometer bits are buggy, compass mode has no gyro but ndof does...
-- read value mask and expected values are different for each operation mode :-
        if ( bit.rshift( bit.band( calibvalue, 0x30 ) , 4) + bit.band( calibvalue, 0x03 )) >= 2 then  -- sum of gyro and magneto calib at least 2
          self.state = showcalib
---[[
          print( ("BNO055 calibration effective: 0x%02x"):format( calibvalue ) )
--]]
        else
---[[
          print( ("Incomplete BNO055 calibration: 0x%02x"):format( calibvalue ) , " ...waiting" )
--]]
--          post_err( self, calibvalue) ) -- *** FIX ME add a max limit ? -- Don't use post_err for this
        end
      end
    end
  )
end
--
showcalib = function( self )  -- FSM state: show and store calibration values
---[[
  print( "Showing calibration values" )
--]]
-- datasheet recommends reading out the calibration values while in config mode, but this seems to work well also in running mode
  readbytes( self.i2cinterface, self.i2caddress, self.register.ACC_OFFSET_X_LSB, 22,
    function( data, ack )
      if not ack then
        i2c_err( self )
      else
        i2c_ok( self )
--[[
        for i=1, 5 , 2 do
          print( ("ACC offset: 0x%x"):format( data:byte( i ) + 256 * data:byte( i+1 ) ) )
        end
        for i=7, 11 , 2 do
          print( ("MAG offset: 0x%x"):format( data:byte( i ) + 256 * data:byte( i+1 ) ) )
        end
        for i=13, 17, 2 do
          print( ("GYR offset: 0x%x"):format( data:byte( i ) + 256 * data:byte( i+1 ) ) )
        end
        print( ("ACC radius: 0x%x"):format( data:byte( 19 ) + 256 * data:byte( 20 ) ) )
        print( ("MAG radius: 0x%x"):format( data:byte( 21 ) + 256 * data:byte( 22 ) ) )
--]]
        local calibv = ("%q"):format( data )
        local calibfn = self.name.."calibration"
        if not file.exists( calibfn ) then
          print( "creating calibration file: " , calibfn , "with value: " , calibv )
          local fh = file.open( calibfn , "w+")
          if fh then
            fh:writeline( self.name..".calib="..calibv )
            fh:flush()
            fh:close()
            fh = nil
          else
-- *** FIX ME improve error reporting ?
            print( "Could not write calibration values to file :", calibfn )
          end
        end
        self.state = monitor 
        self.health = "RUN"
        self.reset_count = 0
        self.wait_count = 0
      end
    end
  )
end
--
monitor = function( self )  -- FSM normal running state: monitor system health
--[[
  print( "Monitoring BNO055 health" )
--]]
  readbytes( self.i2cinterface, self.i2caddress, self.register.TEMP, 10,
    function( data, ack )
      if not ack then
---[[
        print( "monitor didn't get an answer" ) --*** FIX ME remove after debugging
--]]
        i2c_err( self )
      else
        i2c_ok( self )
        self.value.TEMP = data:byte( 1 )
        local calibvalue = data:byte( 2 )
        self.value.CALIB_STAT = calibvalue
        local sysstatus = data:byte( 6 )
        self.value.SYS_STATUS = sysstatus
        local syserror = data:byte( 7 )
        self.value.SYS_ERROR = syserror
        self.value.UNIT_SEL = data:byte( 8 )
        local oprmode = data:byte( 10 )
        self.value.OPR_MODE = oprmode
        if sysstatus < self.lookup.SYS_STATUS.FUSIONRUN then -- if not providing data, move back to state checkstatus
--- *** FIX ME SYS_ERROR of 4 and above can result from programming mistakes, like reading/writing out of range registers not chip failure
--- *** No hint on how to reset these !!!
---[[
          print( "BNO055 SYS_STATUS problem at time:" , time.get() )
          print( ("SYS_STATUS: %d SYS_ERROR: %d CALIB_STAT: 0x%x OPR_MODE: 0x%x"):format( sysstatus, syserror, calibvalue, oprmode ) )
          print( "BNO system status idle, error or setting up, moving back to check status")
--]]  
          self.state = checkstatus
          self.health = "INIT"
        elseif ( bit.rshift( bit.band( calibvalue, 0x30 ) , 4) + bit.band( calibvalue, 0x03 )) < 2  then -- back to calibration state
---[[
          print( "BNO055 calibration problem at time:" , time.get() )
          print( ("SYS_STATUS: %d SYS_ERROR: %d CALIB_STAT: 0x%x OPR_MODE: 0x%x"):format( sysstatus, syserror, calibvalue, oprmode ) )
--]]
          self.state = checkcalib
          self.health = "INIT"
        else
          self.health = "RUN"
        end
      end
    end
  )
end
--
-- read and store measured values
--
geteuler = function( self )
-- return euler angles heading, roll +/-90, pitch +/-180
-- raw heading is 0-5760 ie 360*16
-- pitch and roll are full 16 bit 2's complement, 16 LSB per degree
  if not self.health == "RUN" then return end
  readbytes( self.i2cinterface, self.i2caddress, self.register.EUL_heading_LSB, 6,
    function( data, ack )
      if not ack then
---[[
        print( "geteuler didn't get an answer" ) --*** FIX ME remove after debugging
--]]
        i2c_err( self )
      else
        i2c_ok( self )
        -- decode and store values
        self.HEADING = ( data:byte( 1 ) + bit.lshift( data:byte( 2 ), 8 ) ) / 16
        local val = ( data:byte( 3 ) + bit.lshift( data:byte( 4 ), 8 ) )
        if val>32767 then self.ROLL = (val-65536) / 16 else self.ROLL = val / 16 end
        local val = ( data:byte( 5 ) + bit.lshift( data:byte( 6 ), 8 ) )
        if val>32767 then self.PITCH = (val-65536) / 16 else self.PITCH = val / 16 end
      end
    end
  )
end
--
getquat = function( self )
-- return quaternion
-- values are full 16 bit 2's complement
  if not self.health == "RUN" then return end
  readbytes( self.i2cinterface, self.i2caddress, self.register.QUA_data_w__LSB, 8,
    function( data, ack )
      if not ack then
---[[
        print( "getquat didn't get an answer" ) --*** FIX ME remove after debugging
--]]
        i2c_err( self )
      else
        i2c_ok( self )
        -- decode and store values
        local val = ( data:byte( 1 ) + bit.lshift( data:byte( 2 ), 8 ) )
        if val>32767 then self.W = (val-65536)/ 32767 else self.W = val / 32767 end
        local val = ( data:byte( 3 ) + bit.lshift( data:byte( 4 ), 8 ) )
        if val>32767 then self.X = (val-65536) / 32767 else self.X = val / 32767 end
        val = ( data:byte( 5 ) + bit.lshift( data:byte( 6 ), 8 ) )
        if val>32767 then self.Y = (val-65536) / 32767 else self.Y = val / 32767 end
        val = ( data:byte( 7 ) + bit.lshift( data:byte( 8 ), 8 ) )
        if val>32767 then self.Z = (val-65536) / 32767 else self.Z = val / 32767 end
---[[
        local norm = math.sqrt( self.W * self.W +self.X * self.X +self.Y * self.Y + self.Z * self.Z)
        print( "Quaternion: ",self.W , self.X , self.Y , self.Z , norm)
--]]
      end
    end
  )
end
--
getgravity = function( self )
-- return gravity vector
-- vector components are full 16 bit 2's complement
-- according to settings either 100 LSB for 1m/s2 or 1 LSB for 1 mg
-- *** FIX ME code here assumes m/s2 setting
  if not self.health == "RUN" then return end
  readbytes( self.i2cinterface, self.i2caddress, self.register.GRAV_data_X_LSB, 6,
    function( data, ack )
      if not ack then
---[[
        print( "getgravity didn't get an answer" ) --*** FIX ME remove after debugging
--]]
        i2c_err( self )
      else
        i2c_ok( self )
        -- decode and store values
        local val = ( data:byte( 1 ) + bit.lshift( data:byte( 2 ), 8 ) )
        if val>32767 then self.GRAV_X = (val-65536) / 100 else self.GRAV_X = val / 100 end
        val = ( data:byte( 3 ) + bit.lshift( data:byte( 4 ), 8 ) )
        if val>32767 then self.GRAV_Y = (val-65536) / 100 else self.GRAV_Y = val / 100 end
        val = ( data:byte( 5 ) + bit.lshift( data:byte( 6 ), 8 ) )
        if val>32767 then self.GRAV_Z = (val-65536) / 100 else self.GRAV_Z = val / 100 end
---[[
        local norm = math.sqrt( self.GRAV_X * self.GRAV_X +self.GRAV_Y * self.GRAV_Y + self.GRAV_Z * self.GRAV_Z)
        print( "Gravity: ",self.GRAV_X, self.GRAV_Y, self.GRAV_Z, norm)
--]]
      end
    end
  )
end
--
getlia = function( self )
-- return linear acceleration vector
-- vector components are full 16 bit 2's complement
-- according to settings either 100 LSB for 1m/s2 or 1 LSB for 1 mg
-- *** FIX ME code here assumes m/s2 setting
  if not self.health == "RUN" then return end
  readbytes( self.i2cinterface, self.i2caddress, self.register.LIA_data_X_LSB, 6,
    function( data, ack )
      if not ack then
---[[
        print( "getlia didn't get an answer" ) --*** FIX ME remove after debugging
--]]
        i2c_err( self )
      else
        i2c_ok( self )
        -- decode and store values
        local val = ( data:byte( 1 ) + bit.lshift( data:byte( 2 ), 8 ) )
        if val>32767 then self.LIA_X = (val-65536) / 100 else self.LIA_X = val / 100 end
        val = ( data:byte( 3 ) + bit.lshift( data:byte( 4 ), 8 ) )
        if val>32767 then self.LIA_Y = (val-65536) / 100 else self.LIA_Y = val / 100 end
        val = ( data:byte( 5 ) + bit.lshift( data:byte( 6 ), 8 ) )
        if val>32767 then self.LIA_Z = (val-65536) / 100 else self.LIA_Z = val / 100 end
---[[
        local norm = math.sqrt( self.LIA_X * self.LIA_X +self.LIA_Y * self.LIA_Y + self.LIA_Z * self.LIA_Z)
        print( "Linear acceleration: ",self.LIA_X, self.LIA_Y, self.LIA_Z, norm)
--]]
      end
    end
  )
end
--
getmagnetic = function( self )
-- *** FIX ME results are absurd in fusion modes, not tested in other modes, despite euler angles being ok ??!!
-- return magnetic vector
-- vector components are 13 bits for x and y, 15 bits for z
-- single unit setting 16 LSB for 1 uT (2.5° heading accuracy with 30 uT horizontal component)
  if not self.health == "RUN" then return end
  readbytes( self.i2cinterface, self.i2caddress, self.register.MAG_DATA_X_LSB, 6,
    function( data, ack )
      if not ack then
---[[
        print( "getmagnetic didn't get an answer" ) --*** FIX ME remove after debugging
--]]
        i2c_err( self )
      else
        i2c_ok( self )
        -- decode and store values
        local val = ( data:byte( 1 ) + bit.lshift( data:byte( 2 ), 8 ) )
        print( ("raw magx %d 0x%x"):format( val , val ) )
        if val>32767 then self.MAG_X = (val-65536) / 16 else self.MAG_X = val / 16 end
        val = ( data:byte( 3 ) + bit.lshift( data:byte( 4 ), 8 ) )
        print( ("raw magy %d 0x%x"):format( val , val ) )
        if val>32767 then self.MAG_Y = (val-65536) / 16 else self.MAG_Y = val / 16 end
        val = ( data:byte( 5 ) + bit.lshift( data:byte( 6 ), 8 ) )
        print( ("raw magz %d 0x%x"):format( val , val ) )
        if val>32767 then self.MAG_Z = (val-65536) / 16 else self.MAG_Z = val / 16 end
---[[
        local norm = math.sqrt( self.MAG_X * self.MAG_X +self.MAG_Y * self.MAG_Y + self.MAG_Z * self.MAG_Z)
        print( "Magnetic: ",self.MAG_X, self.MAG_Y, self.MAG_Z, norm)
--]]
      end
    end
  )
end
--
getgyro = function( self )
-- return gyro rates
-- default unit setting 16 LSB for 1 DPS
  if not self.health == "RUN" then return end
  readbytes( self.i2cinterface, self.i2caddress, self.register.GYR_DATA_X_LSB, 6,
    function( data, ack )
      if not ack then
---[[
        print( "getgyro didn't get an answer" ) --*** FIX ME remove after debugging
--]]
        i2c_err( self )
      else
        i2c_ok( self )
        -- decode and store values
        local val = ( data:byte( 1 ) + bit.lshift( data:byte( 2 ), 8 ) )
        if val>32767 then self.GYRO_X = (val-65536) / 16 else self.GYRO_X = val / 16 end
        val = ( data:byte( 3 ) + bit.lshift( data:byte( 4 ), 8 ) )
        if val>32767 then self.GYRO_Y = (val-65536) / 16 else self.GYRO_Y = val / 16 end
        val = ( data:byte( 5 ) + bit.lshift( data:byte( 6 ), 8 ) )
        if val>32767 then self.GYRO_Z = (val-65536) / 16 else self.GYRO_Z = val / 16 end
--[[
        print( "Gyro rates: ",self.GYRO_X, self.GYRO_Y, self.GYRO_Z )
--]]
      end
    end
  )
end
--
getacc = function( self )
-- return acceleromter data rates
-- default unit setting 100 LSB for 1 m/s2
  if not self.health == "RUN" then return end
  readbytes( self.i2cinterface, self.i2caddress, self.register.ACC_DATA_X_LSB, 6,
    function( data, ack )
      if not ack then
---[[
        print( "getgyro didn't get an answer" ) --*** FIX ME remove after debugging
--]]
        i2c_err( self )
      else
        i2c_ok( self )
        -- decode and store values
        local val = ( data:byte( 1 ) + bit.lshift( data:byte( 2 ), 8 ) )
        if val>32767 then self.GYRO_X = (val-65536) / 100 else self.GYRO_X = val / 100 end
        val = ( data:byte( 3 ) + bit.lshift( data:byte( 4 ), 8 ) )
        if val>32767 then self.GYRO_Y = (val-65536) / 100 else self.GYRO_Y = val / 100 end
        val = ( data:byte( 5 ) + bit.lshift( data:byte( 6 ), 8 ) )
        if val>32767 then self.GYRO_Z = (val-65536) / 100 else self.GYRO_Z = val / 100 end
--[[
        print( "Gyro rates: ",self.GYRO_X, self.GYRO_Y, self.GYRO_Z )
--]]
      end
    end
  )
end
--
--
--
--
--
--
readbytes = function( interface, address, register, nbytes, callback) -- block read multiple bytes from BNO registers and execute callback when done
  i2c.start( interface )
  i2c.address( interface, address, i2c.TRANSMITTER, true )
  i2c.write( interface, register, true )
  i2c.start( interface )
  i2c.address( interface, address, i2c.RECEIVER, true)
  i2c.read( interface,nbytes )
  i2c.stop( interface)
  i2c.transfer( interface,callback )
end
--
writebytes = function( interface, address, register, value, callback) -- write value(s) to BNO register and execute callback when done
  i2c.start( interface )
  i2c.address( interface , address, i2c.TRANSMITTER, true)
  i2c.write( interface, register, value, true) -- value may be multiple bytes 
  i2c.stop( interface)
  i2c.transfer( interface,callback )
  end
--
-- 
writeandreadbackbytes = function( interface, address, register, value, callback) -- write value(s) to BNO register, read back and execute callback when done
  i2c.start( interface )
  i2c.address(interface , address, i2c.TRANSMITTER, true )
  local nbytes = i2c.write( interface, register, value, true )  -- return value is number of bytes written including register address
  i2c.start( interface )
  i2c.address( interface, address, i2c.TRANSMITTER, true )
  i2c.write( interface, register, true )
  i2c.start( interface )
  i2c.address( interface, address, i2c.RECEIVER, true )
  i2c.read( interface, nbytes - 1 )  -- read the same number of bytes as written previously
  i2c.stop( interface)
  i2c.transfer( interface, callback )
end
--
resetchip = function( self )
---[[
  print( "Resetting chip" )
--]]
  writebytes( self.i2cinterface, self.i2caddress, self.register.SYS_TRIGGER, self.lookup.SYS_TRIGGER.RST_SYS,
    function( data, ack )
      if not ack then
        i2c_err( self )
      else
        if self.reset_count < self.reset_max then
          self.reset_count = self.reset_count + 1
          self.state = checkchip
        else
          fsm_disable( self )
        end
      end
  end
  )
end
--
forceconfig = function( self ) -- for each setting, set effective value to nil
---[[
  print( "Forcing values for complete configuration" )
--]]
  for k, _ in pairs( self.setting ) do
    self.value[ k ] = nil
  end
end
--
fsm_disable = function( self )
  self.health = "ERROR"
  self.state = function() end
  self:fatal_err_log()
end
--
i2c_ok = function( self )
  if self.i2c_err_count > 0 then
    self.i2c_err_count = self.i2c_err_count - 1
  end
end
--
status_nowait = function( self )
  if self.wait_count > 0 then
    self.wait_count = self.wait_count - 1
  end
end
-- error handlers
--
i2c_err = function( self )
  self:i2c_err_log()
  if self.i2c_err_count < self.i2c_err_max then
    self.i2c_err_count = self.i2c_err_count + 1
  else
    fsm_disable( self )
  end
end
--
chip_err = function( self, data )
  self:chip_err_log( data )
  resetchip( self )
end
--
post_err = function( self, data )
  self:post_err_log( data )
  resetchip( self )
end
--
status_err = function( self, data )
  self:status_err_log( data )
  resetchip( self )
end
--
status_wait = function( self, data )
  self:status_wait_log( data )
  if self.wait_count < self.wait_max then
    self.wait_count = self.wait_count + 1
  else
    fsm_disable( self )
  end
end
--
--
local function configure( self, t ) -- add requested configuration settings
-- keys are register names, lookup value in lookup table if any or use the value as such (8bit int)
---[[
  print( "Entering configuration settings")
--]]
  for k,v in pairs( t ) do
---[[
    print("configure: ",k,v)
--]]
    if self.register[ k ] then  -- valid register name
      if self.lookup[ k ] then  -- register has a lookup table for symbolic values
        if self.lookup[ k ][ v] then -- symbolic value is known
          self.setting[ k ] = ( self.lookup[ k ] )[ v]
---[[
          print("Configure setting register: ",k,  "with symbolic value: ", v )
--]]
        else  -- unknown symbolic value
---[[
          print("Configure ignoring unknown symbolic value: ", v, "for register: ", k)
--]]
        end
      else -- no lookup table, use value as such
        if type(v) == "number" and v >= 0 and v<=255 then -- valid 8 bit literal value
          self.setting[ k ] =  v
---[[
          print("Configure setting register: ", k, "with literal value: ",v )
--]]
        else -- literal value is longer than 8 bits
---[[
          print("Configure ignoring invalid literal value: ",v, "for register: ", k )
--]]
        end
      end
---[[
    else print( "Configure ignoring unknown BNO055 register: ", k )
--]]
    end
  end
---[[
  print( "Leaving configuration settings")
--]]
-- *** FIX ME check FSM state and request adjustment if needed
end
--
local function new( interface, address )
  local bno = populatetable()
  bno.i2cinterface = interface or bno.i2cinterface
  bno.i2caddress = address or bno.i2caddress
  bno.signature = string.char( bno.value.CHIP_ID, bno.value.ACC_ID, bno.value.MAG_ID, bno.value.GYR_ID, bno.value.SW_REV_ID_LSB, bno.value.SW_REV_ID_MSB )
--
  bno.state = checkchip -- initial FSM state
-- default error reporting functions are set to do nothing
  local function donothing() end
  bno.i2c_err_log = donothing
  bno.chip_err_log = donothing
  bno.post_err_log = donothing
  bno.status_err_log = donothing
  bno.status_wait_log = donothing
  bno.fatal_err_log = donothing
-- exported functions
  bno.tick = tick
  bno.configure = configure
  bno.geteuler = geteuler
  bno.getquat = getquat
  bno.getlia = getlia
  bno.getgravity = getgravity
  bno.getmagnetic = getmagnetic
  bno.getgyro = getgyro
  bno.getacc = getacc
  return bno
end
--
return { new = new }
