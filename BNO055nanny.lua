-- handler for Bosch Sensortech IMU BNO055
-- for more details refer to datasheet: BST-BNO055-DS000-14 | Revision 1.4 | June 2016
--
-- the ESP-32 Huzzah has weakup (10 K) pullups on the i2c bus pins
-- additional pullups SCL 3K3 SDA 2K2 are advised for reliable operation
--
-- CALIB_STAT (bits: ssggaamm) is buggy, system and accelerometer bits can be ignored !
-- populate table with register addresses, reset/read values, symbolic settings, expected values ie settings
local function populatetable()
  local bno={}
-- i2c particulars and defaults
  bno.i2cinterface =i2c.HW0
  bno.i2caddress = 0x28  -- if pin COM3 is low like on Adafruit board, otherwise 0x29
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
  bno.register.EULER_HEADING_LSB = 0x1a
  bno.register.TEMP = 0x34
  bno.register.CALIB_STAT = 0x35
  bno.register.ST_RESULT = 0x36
  bno.lookup.ST_RESULT = { OK = 0xf}
  bno.register.SYS_STATUS = 0x39
  bno.lookup.SYS_STATUS = { IDLE = 0, ERROR = 1, FUSIONRUN = 5, RAWRUN = 6 }
--  bno.register.SYS_ERR = 0x3a
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
  bno.i2c_err_max = 5 -- initially 5 attempts for i2c ack detection
  bno.reset_count = 0
  bno.reset_max = 5  -- 5 attempts for chip reset
  bno.wait_count = 0
  bno.wait_count_max = 5  -- wait for OPR_MODE in CONFIGMODE or initialisation to complete
--
-- values derived by handler routines
  bno.HEALTH = false
  bno.HEADING = 0
  bno.ROLL = 0
  bno.PITCH = 0
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
local checkconfig
local checkcalib
local showcalib
local monitor
-- forward declarations of read  functions
local geteuler
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
--
checkchip = function( self ) -- state 1: attempt to get a response from the bus/address and compare with expected signature
  print( "checking i2c connectivity and chip signature" )
  readbytes( self.i2cinterface, self.i2caddress, self.register.CHIP_ID, 6,
    function( data, ack )
      if not ack then
        i2c_err( self )
      else
        i2c_ok( self )
        if data == self.signature then
          print ( "Chip signature ok")
          self.state = checkpost
        else
          chip_err( self, data )
        end
      end
    end
  )
end
--
checkpost = function( self )  -- state 2: check POST
  print( "checking BNO055 self test results" )
  readbytes( self.i2cinterface, self.i2caddress, self.register.ST_RESULT, 1,
    function( data, ack )
      if not ack then
        i2c_err( self )
      else
        i2c_ok( self )
        if data:byte( 1,1 ) == self.lookup.ST_RESULT.OK  then
          self.state = checkstatus
          print ( "BNO055 POST ok" )
        else
          post_err( self, data:byte( 1,1 ) )
        end
      end
    end
  )
end
--
checkstatus = function ( self )  -- state 3: check SYS_STATUS and SYS_ERR
  print( "checking BNO055 system status and errors" )
  readbytes( self.i2cinterface, self.i2caddress, self.register.SYS_STATUS, 2,
    function( data, ack )
      if not ack then
        i2c_err( self )
      else
        i2c_ok( self )
        local status = data:byte( 1, 1 )
        local error = data:byte( 2 ,2 )
        print ( "BNO055 SYS_STATUS SYS_ERR: ",status,error)
        if status == self.lookup.SYS_STATUS.IDLE then
          forceconfig( self )
          self.state = checkconfig  -- system idle, ok to configure
        elseif status == self.lookup.SYS_STATUS.ERROR then -- system error
          status_err( self, error )
        elseif status == self.lookup.SYS_STATUS.FUSIONRUN or status == self.lookup.SYS_STATUS.RAWRUN then
          print( "System running already, must go back to config mode" )
          self.state = configreset  -- system running, need to set OPR_MODE to CONFIGMODE before configuring
        else
          print( "BNO055 still busy initialising, please wait...")  -- codes 2,3,4 for initialising and selftest, wait for next tick
          status_wait( self, status )
        end
      end
    end
  )
end
--
configreset = function( self ) -- state 4 : set OPR_MODE to CONFIGMODE, clear effective read values to reset values
  print( "Setting OPR_MODE back to CONFIGMODE" )
  writeandreadbackbytes( self.i2cinterface, self.i2caddress, self.register.OPR_MODE, self.lookup.OPR_MODE.CONFIGMODE,
    function( data, ack )
      if not ack then
        i2c_err( self )
      else
        i2c_ok( self )
        if data:byte( 1, 1) == self.lookup.OPR_MODE.CONFIGMODE then 
          forceconfig( self )
          self.state = checkconfig
        else
          status_wait( self, data )
          print( "Attempt to put chip back into config mode did not succeed" )
        end
      end
    end
  )
end
--
checkconfig = function ( self )  -- state 5: check configuration requests and apply them
  print( "checking/setting BNO055 configuration" )
-- check settings needed,changing OPR_MODE must be done last when all others are ok, so there will be time for OPR_MODE to establish before next tick
  local tryk, tryv
  for k,v in pairs( self.setting ) do
    print( k, v, self.value[ k] )
    if v ~= self.value[ k ] then
      if not (k == "OPR_MODE" and tryk) then tryk, tryv = k, v end -- ignore OPR_MODE request if there is any other already
      print( "setup required for register", k)
    else
      print( "setting ok for register: ", k)
    end
  end
--
  if tryk then
    print( " attempting configuration of : ", tryk, tryv )
    writeandreadbackbytes( self.i2cinterface, self.i2caddress, self.register[ tryk ], tryv,
      function( data, ack )
        if not ack then
          i2c_err( self )
        else
          i2c_ok( self )
          local newv = data:byte( 1, 1 )
          self.value[ tryk] = newv
          print ( "Read back of configured value for register: ", tryk, newv )
        end
      end
    )
  else
   print ( "Configuration complete" )
   self.state = checkcalib -- move to next FSM state
  end
  print( "exiting checkconfig BNO055" )
end
--
checkcalib = function( self )  -- state 6: check for effective calibration
  print( "Check for BNO055 calibration" )
  readbytes( self.i2cinterface, self.i2caddress, self.register.CALIB_STAT, 1,
    function( data, ack )
      if not ack then
        i2c_err( self )
      else
        i2c_ok( self )
        local calibvalue = data:byte( 1 )
        self.value.CALIB_STAT = calibvalue
        if bit.band( calibvalue, 0x30 ) > 0 and bit.band( calibvalue, 0x03 ) > 0 then  -- gyro and magneto calib at least 1
          self.state = showcalib
          print( ("BNO055 calibration effective: 0x%x"):format( calibvalue ) )
        else
          print( ("Incomplete BNO055 calibration: 0x%x"):format( calibvalue ) )
--          post_err( self, calibvalue) ) -- *** FIX ME add a max limit ?
        end
      end
    end
  )
end
--
showcalib = function( self )  -- state 7: show and store calibration values
  print( "Showing calibration values" )
  readbytes( self.i2cinterface, self.i2caddress, self.register.ACC_OFFSET_X_LSB, 22,
    function( data, ack )
      if not ack then
        i2c_err( self )
      else
        i2c_ok( self )
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
        print( ("Serialized string value:%q"):format( data ) )
        self.state = monitor -- *** FIX ME write out calib values to file -- ** FIX ME update ok indicator
      end
    end
  )
end
--
monitor = function( self )  -- state 8: monitor system health
--  print( "Monitoring BNO055 health" )
  readbytes( self.i2cinterface, self.i2caddress, self.register.TEMP, 10,
    function( data, ack )
      if not ack then
        print( "monitor didn't get an answer" ) --*** FIX ME remove after debugging
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
        if bit.band( calibvalue, 0x33 ) < 0x11 or sysstatus < self.lookup.SYS_STATUS.FUSIONRUN then
          print( "BNO055 health problem at time:" , time.get() )
          print( ("SYS_STATUS: %d SYS_ERROR: %d CALIB_STAT: 0x%x OPR_MODE: 0x%x"):format( sysstatus, syserror, calibvalue, oprmode ) )  
          self.HEALTH = false --*** FIX ME do something about it, try to fix this
          fsm_disable( self )
        else
          self.HEALTH = true
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
  if not self.HEALTH then return end
  readbytes( self.i2cinterface, self.i2caddress, self.register.EULER_HEADING_LSB, 6,
    function( data, ack )
      if not ack then
        print( "geteuler didn't get an answer" ) --*** FIX ME remove after debugging
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
  print( "Resetting chip" )
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
forceconfig = function( self )
-- for each setting, set effective value to nil
  print( "Forcing values for complete configuration" )
  for k, _ in pairs( self.setting ) do
    self.value[ k ] = nil
  end
end
--
fsm_disable = function( self )
  self.state = function() end
  self:fatal_err_log()
end
--
i2c_ok = function( self )
  if self.i2c_err_count > 0 then
    self.i2c_err_count = self.i2c_err_count - 1
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
  if self.wait_count < self.wait_count_max then
    self.wait_count = self.wait_count + 1
  else
    fsm_disable( self )
  end
end
-- default error loggers
local function i2cnoresponse( self )
  print( time.get() , "no response from i2c address : ", self.i2caddress, " bus: " , self.i2cinterface , "check address and connectivity")
end
--
--
local function badchipsignature( self, data )
    print( "i2c device on bus: ", self.i2cinterface, " at address: ", self.i2caddress, "does not have the expected signature for BNO055")
    print( "expected chip signature", self.signature:byte( 1, 6 ) )
    print( "received chip signature", data:byte( 1, 6 ) )
end
--
local function badpost( self, data)
  print( "BNO055 Power On Self Test failed with value: ", data )
end
--
local function badstatus( self, data )
  print( "BNO055 system error: ", data )
end
--
local function waitstatus( self, data )
  print( "BNO055 OPR_MODE is still not CONFIGMODE: ", data )
end
--
local function fatal( self )
  print( "BNO055 handler is now disabled" )
end
--
--
local function configure( self, t ) -- add requested configuration settings
-- keys are register names, lookup value in lookup table if any or use the value as such (8bit int)
  print( "Entering configuration settings")
  for k,v in pairs( t ) do
    print("configure: ",k,v)
    if self.register[ k ] then  -- valid register name
      if self.lookup[ k ] then  -- register has a lookup table for symbolic values
        if self.lookup[ k ][ v] then -- symbolic value is known
          self.setting[ k ] = ( self.lookup[ k ] )[ v]
          print("Configure setting register: ",k,  "with symbolic value: ", v )
        else  -- unknown symbolic value
          print("Configure ignoring unknown symbolic value: ", v, "for register: ", k)
        end
      else -- no lookup table, use value as such
        if type(v) == "number" and v >= 0 and v<=255 then -- valid 8 bit literal value
          self.setting[ k ] =  v 
          print("Configure setting register: ", k, "with literal value: ",v )
        else -- literal value is longer than 8 bits
          print("Configure ignoring invalid literal value: ",v, "for register: ", k )
        end
      end
    else print( "Configure ignoring unknown BNO055 register: ", k )
    end
  end
  print( "Leaving configuration settings")
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
-- default error reporting functions
  bno.i2c_err_log = i2cnoresponse
  bno.chip_err_log = badchipsignature
  bno.post_err_log = badpost
  bno.status_err_log = badstatus
  bno.status_wait_log = waitstatus
  bno.fatal_err_log = fatal
-- exported functions
  bno.tick = tick
  bno.configure = configure
  bno.geteuler = geteuler
  return bno
end
--
return { new = new }
--FileView done.