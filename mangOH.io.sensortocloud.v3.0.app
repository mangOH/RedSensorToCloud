<?xml version="1.0" encoding="ISO-8859-1" ?>
<app:application
    xmlns:app="http://www.sierrawireless.com/airvantage/application/1.0"
    type="mangoh.io.sensortocloud.app"
    name="RedSensorToCloud"
    revision="3.0">
  <application-manager use="LWM2M_SW"/>
  <capabilities>
    <data>
      <encoding type="LWM2M">
        <asset default-label="MangOH Red" id="MangOH">
          <node path="Sensors" default-label="Sensors">
            <node path="Accelerometer" default-label="Accelerometer">
              <node path="Acceleration" default-label="Acceleration">
                <variable default-label="X" path="X" type="double" />
                <variable default-label="Y" path="Y" type="double" />
                <variable default-label="Z" path="Z" type="double" />
              </node>
              <node path="Gyro" default-label="Gyro">
                <variable default-label="X" path="X" type="double" />
                <variable default-label="Y" path="Y" type="double" />
                <variable default-label="Z" path="Z" type="double" />
              </node>
            </node>
            <node path="GPS" default-label="Gps">
              <variable default-label="VerticalAccuracy" path="VerticalAccuracy" type="double" />
            </node>
            <node path="Light" default-label="Light">
              <variable default-label="Level" path="Level" type="int" />
            </node>
            <node path="Pressure" default-label="Pressure">
              <variable default-label="Pressure" path="Pressure" type="double" />
              <variable default-label="Temperature" path="Temperature" type="double" />
            </node>
          </node>
          <node path="Commands" default-label="Commands">
            <command default-label="ActivateLED" id="redSensorToCloud/ActivateLED" />
            <command default-label="DeactivateLED" id="redSensorToCloud/DeactivateLED" />
            <command default-label="Set LED Interval" id="redSensorToCloud/SetLedBlinkInterval">
              <parameter default-label="LedBlinkInterval" id="LedBlinkInterval" type="string" />
            </command>
          </node>
        </asset>
      </encoding>
    </data>
  </capabilities>
</app:application>
