-- standalone execution of BNO055nanny
-- *** FIX ME adapt to default nanny donothing() error handlers
print( "starting BNO handler")
--
bh = require( "BNO055nanny")
--
led_pin = 13
gpio.config( {gpio=led_pin , dir=gpio.OUT} )
gpio.write( led_pin , 0 )
--
i2cid = i2c.HW1  -- hardware i2c, must use queuing and callbacks
pinSDA = 23 -- Huzzah32 silkscreen
-- pinSDA=21 -- Expressif default
pinSCL = 22 -- according to Huzzah32 silk screen
--i2cspeed = i2c.SLOW  -- BNO supports speeds up to 400k (ie i2c.SLOW 100 KHz or i2c.FAST 400 KHz) but not i2c.FASTPLUS 1MHz
i2cspeed = 30000

i2cstretch = 5 -- allow more than default clock stretching
--
print( "bnorun: i2c setup returned: ", i2c.setup( i2cid , pinSDA , pinSCL, i2cspeed , i2cstretch ))
--
print( "creating IMU1" )
IMU1 = bh.new( i2cid )
--
-- chatty error loggers
local function i2cnoresponse( self )
  print( time.get() , "no response from i2c address : ", self.i2caddress, " bus: " , self.i2cinterface , "check address and connectivity")
end
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
-- tie in error reporting functions
IMU1.i2c_err_log = i2cnoresponse
IMU1.chip_err_log = badchipsignature
IMU1.post_err_log = badpost
IMU1.status_err_log = badstatus
IMU1.status_wait_log = waitstatus
IMU1.fatal_err_log = fatal
--
print( "Configuring IMU1 - real values " )
IMU1:configure( {TEMP_SOURCE="GYR", OPR_MODE="COMPASS"} )
--
cycle = 0
hdg_prev, roll_prev , pitch_prev = IMU1.HEADING, IMU1.ROLL, IMU1.PITCH
--
imucompass = function()
  if IMU1.health == "RUN" then
    gpio.write( led_pin, 1 )
    hdg, roll, pitch = IMU1.HEADING, IMU1.ROLL, IMU1.PITCH
    if hdg ~= hdg_prev or roll ~= roll_prev or pitch ~= pitch_prev then
      print( ("HDG: %.4f ROLL: %.4f PITCH: %.4f"):format( hdg, roll, pitch ) )
      hdg_prev, roll_prev , pitch_prev = hdg, roll, pitch
    end
  else
    gpio.write( led_pin, 0 )
--    print( "Not yet healthy" )
  end
  if cycle < 10 and IMU1.health == "RUN" then
--    print( "getting euler ",cycle, IMU1.HEALTH )
    IMU1:geteuler()
    cycle = cycle + 1
  else
--    print( "ticking", cycle, IMU1.HEALTH )
    IMU1:tick()
    cycle = 0
  end
end
--
bnotimer = tmr.create()
bnotimer:register(1000, tmr.ALARM_AUTO, imucompass )
bnotimer:start()
--
print( "stop the timer tick by bnotimer:stop()" )
--
