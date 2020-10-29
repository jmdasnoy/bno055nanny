--
print( "starting BNO handler")
--
bh = require( "BNO055")
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
--FileView done.
