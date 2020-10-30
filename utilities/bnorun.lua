-- standalone execution of BNO055nanny
-- *** FIX ME adapt to default nanny donothing() error handlers
print( "starting BNO handler")
--
bh = require( "BNO055nanny")
--
i2cid=i2c.HW1  -- hardware i2c, must use queuing and callbacks
pinSDA=23 -- Huzzah32 silkscreen
-- pinSDA=21 -- Expressif default
pinSCL=22 -- according to Huzzah32 silk screen
--i2cspeed=i2c.SLOW  -- BNO supports speeds up to 400k (ie i2c.SLOW 100 KHz or i2c.FAST 400 KHz) but not i2c.FASTPLUS 1MHz
i2cspeed=30000

i2cstretch=5 -- allow more than default clock stretching
--
print( "bnorun: i2c setup returned: ", i2c.setup( i2cid, pinSDA, pinSCL, i2cspeed, i2cstretch ))
--
print( "creating IMU1" )
IMU1 = bh.new( i2cid )
--
print( "Configuring IMU1 - real values " )
IMU1:configure( {TEMP_SOURCE="GYR", OPR_MODE="NDOF"} )
--
cycle = 0
hdg_prev, roll_prev , pitch_prev = IMU1.HEADING, IMU1.ROLL, IMU1.PITCH
--
imucompass = function()
  if IMU1.HEALTH then
    gpio.write( 13, 1 )
    hdg, roll, pitch = IMU1.HEADING, IMU1.ROLL, IMU1.PITCH
    if hdg ~= hdg_prev or roll ~= roll_prev or pitch ~= pitch_prev then
      print( ("HDG: %.4f ROLL: %.4f PITCH: %.4f"):format( hdg, roll, pitch ) )
      hdg_prev, roll_prev , pitch_prev = hdg, roll, pitch
    end
  else
    gpio.write( 13, 0 )
--    print( "Not yet healthy" )
  end
  if cycle < 10 and IMU1.HEALTH then
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
