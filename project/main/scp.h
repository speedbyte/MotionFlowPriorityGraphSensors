#ifndef _SCP_H
#define _SCP_H

/*
 * Order
 * Projectname
 * <TaskControl><RDB client="false" enable="true" interface="eth0" portRx="48190" portTx="48190" portType="TCP" /></TaskControl>
 * ScenarioName
 */



/*
 *
------------------------------------------------------------
VT-GUI
<SimCtrl><Stop /></SimCtrl>
------------------------------------------------------------
VT-GUI
<SimCtrl><Stop /></SimCtrl>
------------------------------------------------------------
VT-GUI
<SimCtrl><Apply /></SimCtrl>
------------------------------------------------------------
TaskControl
<TaskControl><RDB client="false" enable="true" interface="eth0" portRx="48190" portTx="48190" portType="TCP" /></TaskControl>
------------------------------------------------------------
TaskControl
<TaskControl><Interface name="eth0" /><ImageGenerator autoConfig="true" autoHeadlight="true" autoRestart="true" ctrlPortConnect="true" dynPlayerConfig="true" idleImgId="20" imgPortConnect="true" portType="TCP" protocol="precision10" showIdleImg="true" /><Traffic controlHeadlights="true" /><Mockup arguments="" force="false" lookupBrakePedal="" lookupThrottlePedal="" player="" type="none" /><Joystick debug="false" type="custom"><Axis /><Axis /><Axis /><Button cmd="" index="0" /><Button cmd="" index="0" /><Button cmd="" index="0" /><Button cmd="" index="0" /><Button cmd="" index="0" /><Button cmd="" index="0" /><Button cmd="" index="0" /><Button cmd="" index="0" /><Button cmd="" index="0" /><Button cmd="" index="0" /></Joystick><Sound enable="false" interface="" multicastAddress="" starterSound="" /><DataVis enable="false" /><ScVis enable="true" portType="loopback" /><Instruments enable="false" /><RDB enable="true" imageTransfer="false" portType="TCP" sendEnvironment="true" sendLightPos="true" sendLightSources="true" sendScoring="true" targetAddress="" /><RDB enable="false" name="" server="" targetAddress="" /><RDB enable="false" name="" server="" targetAddress="" /><RecPlay autoRec="false" dummyPedestrians="auto" playSCP="true" /><Video buffer="color" liveStream="false" motionBlurMs="0" motionBlurRes="0" saveToFile="false" /><Dynamics syncMode="frame" /><Sync frameTimeMs="40" realTime="true" source="extern" /><Config publish="true" /><Debug egoSpeed="true" enable="true" simTime="true" /></TaskControl>
------------------------------------------------------------
IG
<Reply entity="imageGenerator"><VisualDatabase file="Data/Projects/Current/Databases/Town/Ive/townGer.opt.osgb" /></Reply>
------------------------------------------------------------
IG
<Reply entity="imageGenerator"><Window height="600" width="800" x="12" y="90" /></Reply>


 */

/*
"<TaskControl><RDB client="false" enable="true" interface="eth0" portRx="48190" portTx="48190" portType="TCP" /></TaskControl>"

"<Environment><Friction value="1.000000" /><TimeOfDay headlights="false" value="39600" /><Sky cloudState="4/8" visibility="100000.000000" /><Precipitation intensity="0.000000" type="none" /><Road effectScale="0.500000" state="dry" /></Environment>"

"<Camera name="VIEW_CAMERA" showOwner="false"><Frustum far="1500.000000" fovHor="40.000000" fovVert="30.000000" near="1.000000" offsetHor="0.000000" offsetVert="0.000000" /><PosEyepoint /><ViewRelative dh="0.000000" dp="0.000000" dr="0.000000" /><Set /></Camera>"

"<Display><SensorSymbols enable="false" sensor="Sensor_MM" showCone="false" /><Database enable="true" streetLamps="false" /><VistaOverlay enable="false" /></Display>"

"<VIL><Imu dbElevation="true" /></VIL>"

"<VIL><EyepointOffset hDeg="0.000000" pDeg="0.000000" rDeg="0.000000" x="0.000000" y="0.000000" z="0.000000" /></VIL>"

read -p "Darn, that weather!Press enter to let it snow"


./scpGenerator -p $PORT_SCP -i '<Info level="info"> <Message popup="true" text="snow!!!!"/> </Info>'
./scpGenerator -p $PORT_SCP -i '<Symbol name="expl01" > <Text data="Let it snow(Let it snow, let it snow)" colorRGB="0xffffff" size="50.0" /> <PosScreen x="0.01" y="0.05" /></Symbol>'
./scpGenerator -p $PORT_SCP -i '<Environment><Friction value="1.000000" /><TimeOfDay headlights="false" value="39600" /><Sky cloudState="4/8" visibility="100000.000000" /><Precipitation intensity="1.000000" type="snow" /><Road effectScale="0.500000" state="wet" /></Environment>'

read -p "Press enter for summer"

./scpGenerator -p $PORT_SCP -i '<Info level="info"> <Message popup="true" text="Summer!!!!"/> </Info>'
./scpGenerator -p $PORT_SCP -i '<Symbol name="expl01" > <Text data="Time for Summer" colorRGB="0xffff00" size="50.0" /> <PosScreen x="0.01" y="0.05" /></Symbol>'
./scpGenerator -p $PORT_SCP -i '<Environment><Friction value="1.000000" /><TimeOfDay headlights="false" value="39600" /><Sky cloudState="0/8" visibility="100000.000000" /><Precipitation intensity="1.000000" type="none" /><Road effectScale="0.500000" state="dry" /></Environment>'


 DRY

"<Environment><Friction value="1.000000" /><TimeOfDay headlights="false" value="39600" /><Sky cloudState="4/8" visibility="100000.000000" /><Precipitation intensity="0.000000" type="none" /><Road effectScale="0.500000" state="dry" /></Environment>"

WET


"<Environment><Friction value="1.000000" /><TimeOfDay headlights="false" value="39600" /><Sky cloudState="4/8" visibility="100000.000000" /><Precipitation intensity="1.000000" type="rain" /><Road effectScale="0.500000" state="wet" /></Environment>"




 */

#endif _SCP_H