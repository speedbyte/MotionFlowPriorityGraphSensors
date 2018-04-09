#ifndef _SCP_H
#define _SCP_H


/*
 *
 * 
 * 
std::string apply = "<SimCtrl><Apply/></SimCtrl>";

std::string project_name = "<SimCtrl><Project name=\"Movement\"path=\"/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/Current\" /></SimCtrl>";

std::string image_generator = <ImageGenerator> <Window width="800" height="600" x="0" y="0" screen="0" border="true"/></ImageGenerator>

std::string scenario_name = "<SimCtrl><UnloadSensors /><LoadScenario "filename=\"/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2".0/Data/Projects/Current/Scenarios/traffic_demo".xml\" /><Start mode=\"operation\" /></SimCtrl>";

std::string module_manager = "<Sensor name="Sensor_MM" type="video" > <Load lib="libModuleCameraSensor.so" path="/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/../Distros/Distro/Plugins/ModuleManager" /> <Player name="SpyingCar"/> <Frustum near="1.000000" far="40.000000" left="12.500000" right="12.500000" bottom="15.000000" top="15.000000" /> <Position dx="0.000000" dy="0.000000" dz="0.000000" dhDeg="0.000000" dpDeg="0.000000" drDeg="0.000000" /> <Origin type="usk" /> <Cull maxObjects="10" enable="true" /> <Port name="RDBout" number="48185" type="TCP" sendEgo="true" /> <Filter objectType="none" /><Filter objectType="pedestrian" /> <Debug enable="false" detection="false" road="false" position="false" dimensions="false" camera="false" packages="false" culling="false" /> </Sensor>";

// Precipitation intensity needs to be > 0 for snow and rain.
 std::string environment_parameters = "<Environment> <Friction value="1.000000" /> <TimeOfDay value="39600"
 headlights="false" /> <Sky cloudState="4/8" visibility="100000.000000" /><Precipitation type="none" intensity="0.000000" /><Road state="dry" effectScale="0.500000" /></Environment>";
//Rain
 std::string environment_parameters = "<Environment> <Friction value="1.000000" /> <TimeOfDay value="39600"
 headlights="false" /> <Sky cloudState="4/8" visibility="100000.000000" /><Precipitation type="rain" intensity="0.500000" /><Road state="dry" effectScale="0.500000" /></Environment>";

std::string view_parameters = "<Camera name="VIEW_CAMERA" showOwner="true"> <Frustum near="0.100000" far="1501.000000" fovHor="60.000000" fovVert="40.000000" offsetHor="0.000000" offsetVert="0.000000" /> <PosTether player="StaticCar" distance="6.000000" azimuth="0.000000" elevation="0.261799" slew="1" /> <ViewRelative dh="0.000000" dp="0.000000" dr="0.000000" /><Set /> </Camera>";

std::string display_parameters = "<Display>  <SensorSymbols enable="false" sensor="Sensor_MM" showCone="true" /> <SensorSymbols enable="false" sensor="Sensor_MM" showCone="true" /> <Database enable="true" streetLamps="false"/> <VistaOverlay enable="false" /> </Display>";

std::string elevation = "<VIL><Imu dbElevation=\"true\" /></VIL>";

std::string eyepoint = "<VIL><EyepointOffset hDeg=\"0.000000\" pDeg=\"0.000000\" rDeg=\"0.000000\" x=\"0.000000\"y=\"0.000000\" z=\"0.000000\" /></VIL>";

std::string pause = "<SimCtrl> <Pause/> </SimCtrl>"



INTERNAL: GUI: XMIT SCP: <SimCtrl><Apply/></SimCtrl>
INTERNAL: GUI: XMIT SCP: <SimCtrl> <Project name="Movement" path="/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/Current" /> </SimCtrl>
INTERNAL: GUI: XMIT SCP: <ImageGenerator> <Window width="800" height="600" x="0" y="0" screen="0" border="true"/> </ImageGenerator>
INTERNAL: GUI: XMIT SCP: <SimCtrl> <UnloadSensors/> <LoadScenario filename="/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/Current/Scenarios/two.xml" /> <Start mode="operation" /> </SimCtrl>
INTERNAL: GUI: XMIT SCP: <Sensor name="Sensor_MM" type="video" > <Load lib="libModuleCameraSensor.so" path="/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/../Distros/Distro/Plugins/ModuleManager" /> <Player name="SpyingCar"/> <Frustum near="1.000000" far="40.000000" left="12.500000" right="12.500000" bottom="15.000000" top="15.000000" /> <Position dx="0.000000" dy="0.000000" dz="0.000000" dhDeg="0.000000" dpDeg="0.000000" drDeg="0.000000" /> <Origin type="usk" /> <Cull maxObjects="10" enable="true" /> <Port name="RDBout" number="48185" type="TCP" sendEgo="true" /> <Filter objectType="none" /><Filter objectType="pedestrian" /> <Debug enable="false" detection="false" road="false" position="false" dimensions="false" camera="false" packages="false" culling="false" /> </Sensor>
INTERNAL: GUI: XMIT SCP: <Environment> <Friction value="1.000000" /> <TimeOfDay value="39600" headlights="false" /> <Sky cloudState="4/8" visibility="100000.000000" /><Precipitation type="none" intensity="0.000000" /><Road state="dry" effectScale="0.500000" /></Environment>
INTERNAL: GUI: XMIT SCP: <Camera name="VIEW_CAMERA" showOwner="true"> <Frustum near="0.100000" far="1501.000000" fovHor="60.000000" fovVert="40.000000" offsetHor="0.000000" offsetVert="0.000000" /> <PosTether player="StaticCar" distance="6.000000" azimuth="0.000000" elevation="0.261799" slew="1" /> <ViewRelative dh="0.000000" dp="0.000000" dr="0.000000" /><Set /> </Camera>
INTERNAL: GUI: XMIT SCP: <Display>  <SensorSymbols enable="false" sensor="Sensor_MM" showCone="true" /> <SensorSymbols enable="false" sensor="Sensor_MM" showCone="true" /> <Database enable="true" streetLamps="false"/> <VistaOverlay enable="false" /> </Display>
INTERNAL: GUI: XMIT SCP: <VIL> <Imu  dbElevation="true" /> </VIL>
INTERNAL: GUI: XMIT SCP: <VIL> <EyepointOffset x="0.000000" y="0.000000" z="0.000000" hDeg="0.000000" pDeg="0.000000" rDeg="0.000000" /> </VIL>
INTERNAL: GUI: XMIT SCP: <SimCtrl> <Pause/> </SimCtrl> 



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
<TaskControl><Interface name="eth0" /><ImageGenerator autoConfig="true" autoHeadlight="true" autoRestart="true" ctrlPortConnect="true" dynPlayerConfig="true" idleImgId="20" imgPortConnect="true" portType="TCP" protocol="precision10" showIdleImg="true" /><Traffic controlHeadlights="true" /><Mockup arguments=" force="false" lookupBrakePedal=" lookupThrottlePedal=" player=" type="none" /><Joystick debug="false" type="custom"><Axis /><Axis /><Axis /><Button cmd=" index="0" /><Button cmd=" index="0" /><Button cmd=" index="0" /><Button cmd=" index="0" /><Button cmd=" index="0" /><Button cmd=" index="0" /><Button cmd=" index="0" /><Button cmd=" index="0" /><Button cmd=" index="0" /><Button cmd=" index="0" /></Joystick><Sound enable="false" interface=" multicastAddress=" starterSound=" /><DataVis enable="false" /><ScVis enable="true" portType="loopback" /><Instruments enable="false" /><RDB enable="true" imageTransfer="false" portType="TCP" sendEnvironment="true" sendLightPos="true" sendLightSources="true" sendScoring="true" targetAddress=" /><RDB enable="false" name=" server=" targetAddress=" /><RDB enable="false" name=" server=" targetAddress=" /><RecPlay autoRec="false" dummyPedestrians="auto" playSCP="true" /><Video buffer="color" liveStream="false" motionBlurMs="0" motionBlurRes="0" saveToFile="false" /><Dynamics syncMode="frame" /><Sync frameTimeMs="40" realTime="true" source="extern" /><Config publish="true" /><Debug egoSpeed="true" enable="true" simTime="true" /></TaskControl>
------------------------------------------------------------
IG
<Reply entity="imageGenerator"><VisualDatabase file="Data/Projects/Current/Databases/Town/Ive/townGer.opt.osgb" /></Reply>
------------------------------------------------------------
IG
<Reply entity="imageGenerator"><Window height="600" width="800" x="12" y="90" /></Reply>

AutoCgfDatabase.xml

 <IGconfig>
   <Database>
     <Entity attachToScene="1" ID="Terrain" filename="Data/Projects/Current/Databases/Crossing8Course/Crossing8Course.opt.osgb" />
     <Entity attachToScene="0" ID="ShadowTerrain" filename="Data/Projects/Current/Databases/Crossing8Course/Crossing8Course.shadow.osgb" />
    <OcclusionFile file="Data/Projects/Current/Databases/Crossing8Course/Crossing8Course.occl" />
  </Database>
 </IGconfig>


 <IGconfig>
   <Database>
     <Entity attachToScene="1" ID="Terrain" filename="Data/Projects/Current/Databases/Town/Ive/townGer.opt.osgb" />
     <Entity attachToScene="0" ID="ShadowTerrain" filename="Data/Projects/Current/Databases/Town/Ive/townGer.shadow.osgb" />
    <OcclusionFile file="Data/Projects/Current/Databases/Town/Ive/townGer.occl" />
  </Database>
 </IGconfig>

 */


/*
 *
------------------------------------------------------------
VT-GUI
<SimCtrl><Stop /></SimCtrl>
------------------------------------------------------------
ExampleConsole
<SimCtrl><Apply /></SimCtrl>
------------------------------------------------------------
TaskControl
<TaskControl><RDB client="false" enable="true" interface="eth0" portRx="48190" portTx="48190" portType="TCP" /></TaskControl>
------------------------------------------------------------
TaskControl
<TaskControl><Interface name="eth0" /><ImageGenerator autoConfig="true" autoHeadlight="true" autoRestart="true" ctrlPortConnect="true" dynPlayerConfig="true" idleImgId="20" imgPortConnect="true" portType="TCP" protocol="precision10" showIdleImg="true" /><Traffic controlHeadlights="true" /><Mockup force="false" type="none" /><Joystick debug="false" type="custom"><Axis /><Axis /><Axis /><Button /><Button /><Button /><Button /><Button /><Button /><Button /><Button /><Button /><Button /></Joystick><Sound enable="false" /><DataVis enable="false" /><ScVis enable="true" portType="loopback" /><Instruments enable="false" /><RDB enable="true" imageTransfer="false" portType="TCP" sendEnvironment="false" sendLightPos="false" sendLightSources="false" sendScoring="false" /><RDB enable="false" name="rdb2" /><RDB enable="false" name="rdb3" /><RecPlay autoRec="false" dummyPedestrians="auto" playSCP="true" /><Video buffer="color" liveStream="false" motionBlurMs="0" motionBlurRes="0" saveToFile="false" /><Dynamics syncMode="frame" /><Sync frameTimeMs="40" realTime="true" source="RDB" /><Config publish="true" /><Debug egoSpeed="true" enable="false" simTime="true" /></TaskControl>
------------------------------------------------------------
IG
<Reply entity="imageGenerator"><VisualDatabase file="Data/Projects/Current/Databases/Town/Ive/townGer.opt.osgb" /></Reply>
------------------------------------------------------------
IG
<Reply entity="imageGenerator"><Window height="600" width="800" x="12" y="90" /></Reply>
------------------------------------------------------------
ExampleConsole
<SimCtrl><Project name="Movement" path="/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/Current" /></SimCtrl>
------------------------------------------------------------
ExampleConsole
<TaskControl><RDB client="false" enable="true" interface="eth0" portRx="48190" portTx="48190" portType="TCP" /></TaskControl>
------------------------------------------------------------
TaskControl
<TaskControl><RDB client="false" enable="true" interface="eth0" portRx="48190" portTx="48190" portType="TCP" /></TaskControl>
------------------------------------------------------------
ExampleConsole
<SimCtrl><UnloadSensors /><LoadScenario filename="/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/Current/Scenarios/two.xml" /><Start mode="operation" /></SimCtrl>
------------------------------------------------------------
TaskControl
<SimCtrl><LayoutFile filename="/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Distros/Distro/Databases/Town/Odr/townGer.xodr" /></SimCtrl>
------------------------------------------------------------
TaskControl
<Set entity="player" id="1" name="StaticCar"><PosInertial hDeg="126.3381" pDeg="0.0000" rDeg="0.0000" x="-43.234" y="2453.641" z="9.500" /><Path id="-1" s="0.000" /></Set>
------------------------------------------------------------
TaskControl
<Player name="StaticCar"><Config dynamics="default" initialSpeed="1.389" scenarioEntity="1" visualModel="1" /></Player>
------------------------------------------------------------
TaskControl
<Set entity="player" id="2" name="SpyingCar"><PosInertial hDeg="36.3381" pDeg="0.0000" rDeg="0.0000" x="-70.314" y="2460.254" z="9.500" /><Path id="-1" s="0.000" /></Set>
------------------------------------------------------------
TaskControl
<Player name="SpyingCar"><Config dynamics="default" initialSpeed="0.000" scenarioEntity="2" visualModel="71" /></Player>
------------------------------------------------------------
Traffic
<Player name="SpyingCar"><Driver visible="false" /></Player>
------------------------------------------------------------
Traffic
<Player name="StaticCar"><VehicleDef airDragCoefficient="0.26" distCoupling="3.467 0 0.385" distFront="3.467" distHeight="1.423" distHitch="-0.833 0 0.385" distLeft="0.888" distRear="0.833" distRight="0.888" enginePower="103000" frontSurfaceEffective="2" gradientPitchAngle="-1" gradientRollAngle="-1" mass="1560" maxDecel="-9.5" maxSpeed="58.4" maxSteering="0.48" maxTorque="320" overallEfficiency="0.75" rollingResistance="0.013" trackWidth="-1" wheelBase="2.591" wheelDiameter="0.641" wheelDiameterDynamic="0.705" wheelDrive="wheel_drive_front" wheelSkewStiffness="12" /></Player>
------------------------------------------------------------
Traffic
<Player name="SpyingCar"><VehicleDef airDragCoefficient="0.3" distCoupling="2.158 0 0.385" distFront="2.158" distHeight="1.52" distHitch="-0.356 0 0.385" distLeft="0.74" distRear="0.356" distRight="0.74" enginePower="45000" frontSurfaceEffective="2.1" gradientPitchAngle="-1" gradientRollAngle="-1" mass="890" maxDecel="-9.5" maxSpeed="40" maxSteering="0.48" maxTorque="95" overallEfficiency="0.75" rollingResistance="0.013" trackWidth="-1" wheelBase="1.765" wheelDiameter="0.536" wheelDiameterDynamic="0.59" wheelDrive="wheel_drive_rear" wheelSkewStiffness="12" /></Player>
------------------------------------------------------------
Traffic
<Player name="StaticCar"><VehicleDef airDragCoefficient="0.26" distCoupling="3.467 0 0.385" distFront="3.467" distHeight="1.423" distHitch="-0.833 0 0.385" distLeft="0.888" distRear="0.833" distRight="0.888" enginePower="103000" frontSurfaceEffective="2" gradientPitchAngle="-1" gradientRollAngle="-1" mass="1560" maxDecel="-9.5" maxSpeed="58.4" maxSteering="0.48" maxTorque="320" overallEfficiency="0.75" rollingResistance="0.013" trackWidth="-1" wheelBase="2.591" wheelDiameter="0.641" wheelDiameterDynamic="0.705" wheelDrive="wheel_drive_front" wheelSkewStiffness="12" /></Player>
------------------------------------------------------------
Traffic
<Player name="SpyingCar"><VehicleDef airDragCoefficient="0.3" distCoupling="2.158 0 0.385" distFront="2.158" distHeight="1.52" distHitch="-0.356 0 0.385" distLeft="0.74" distRear="0.356" distRight="0.74" enginePower="45000" frontSurfaceEffective="2.1" gradientPitchAngle="-1" gradientRollAngle="-1" mass="890" maxDecel="-9.5" maxSpeed="40" maxSteering="0.48" maxTorque="95" overallEfficiency="0.75" rollingResistance="0.013" trackWidth="-1" wheelBase="1.765" wheelDiameter="0.536" wheelDiameterDynamic="0.59" wheelDrive="wheel_drive_rear" wheelSkewStiffness="12" /></Player>
------------------------------------------------------------
Traffic
<Player name="StaticCar"><ModelDescription name="Car_Audi_A3"><Mirrors><Mirror name="MirrorCenter" x="1.671" y="0" z="1.257" /><Mirror name="MirrorLeft" x="1.799" y="0.898" z="1.027" /><Mirror name="MirrorRight" x="1.799" y="-0.898" z="1.027" /></Mirrors><EyePoints><EyePoint name="Driver" x="1.152" y="0.355" z="1.161" /></EyePoints><LightSources><LightSource ID="DefaultHeadlight" name="HeadLightLeft" type="Headlight" x="3.15" y="0.654" z="0.678" /><LightSource ID="DefaultHeadlight" name="HeadLightRight" type="Headlight" x="3.15" y="-0.654" z="0.678" /><LightSource name="TaillightLeft" type="Taillight" x="-0.572" y="0.668" z="0.903" /><LightSource name="TaillightRight" type="Taillight" x="-0.572" y="-0.668" z="0.903" /></LightSources><Switches brakelights="true" daytimeRunningLight="true" driver="true" headlights="true" indicatorLeft="true" indicatorRight="true" litNumberPlate="true" reverseDrivinglight="true" /></ModelDescription></Player>
------------------------------------------------------------
Traffic
<Set entity="player" name="StaticCar"><PosInertial hDeg="126.338" jumpToPos="true" x="-43.234" y="2453.64" z="9.5" /></Set>
------------------------------------------------------------
Traffic
<Player name="SpyingCar"><ModelDescription name="Car_Smart"><Mirrors><Mirror name="MirrorCenter" x="1.078" y="0" z="1.332" /><Mirror name="MirrorLeft" x="1.318" y="0.727" z="1.027" /><Mirror name="MirrorRight" x="1.318" y="-0.727" z="1.027" /></Mirrors><EyePoints><EyePoint name="Driver" x="0.541" y="0.268" z="1.34" /></EyePoints><LightSources><LightSource ID="DefaultHeadlight" name="HeadLightLeft" type="Headlight" x="1.797" y="0.513" z="0.799" /><LightSource ID="DefaultHeadlight" name="HeadLightRight" type="Headlight" x="1.797" y="-0.513" z="0.799" /><LightSource name="TaillightLeft" type="Taillight" x="-0.28" y="0.595" z="0.806" /><LightSource name="TaillightRight" type="Taillight" x="-0.28" y="-0.595" z="0.806" /></LightSources><Switches brakelights="true" driver="true" foglightFront="true" foglightRear="true" headlights="true" indicatorLeft="true" indicatorRight="true" litNumberPlate="true" reverseDrivinglight="true" /></ModelDescription></Player>
------------------------------------------------------------
Traffic
<Set entity="player" name="SpyingCar"><PosInertial hDeg="36.3381" jumpToPos="true" x="-70.3135" y="2460.25" z="9.5" /></Set>
------------------------------------------------------------
Traffic
<Player name="StaticCar"><VehicleDef airDragCoefficient="0.26" distCoupling="3.467 0 0.385" distFront="3.467" distHeight="1.423" distHitch="-0.833 0 0.385" distLeft="0.888" distRear="0.833" distRight="0.888" enginePower="103000" frontSurfaceEffective="2" gradientPitchAngle="-1" gradientRollAngle="-1" mass="1560" maxDecel="-9.5" maxSpeed="58.4" maxSteering="0.48" maxTorque="320" overallEfficiency="0.75" rollingResistance="0.013" trackWidth="-1" wheelBase="2.591" wheelDiameter="0.641" wheelDiameterDynamic="0.705" wheelDrive="wheel_drive_front" wheelSkewStiffness="12" /></Player>
------------------------------------------------------------
Traffic
<Player name="SpyingCar"><VehicleDef airDragCoefficient="0.3" distCoupling="2.158 0 0.385" distFront="2.158" distHeight="1.52" distHitch="-0.356 0 0.385" distLeft="0.74" distRear="0.356" distRight="0.74" enginePower="45000" frontSurfaceEffective="2.1" gradientPitchAngle="-1" gradientRollAngle="-1" mass="890" maxDecel="-9.5" maxSpeed="40" maxSteering="0.48" maxTorque="95" overallEfficiency="0.75" rollingResistance="0.013" trackWidth="-1" wheelBase="1.765" wheelDiameter="0.536" wheelDiameterDynamic="0.59" wheelDrive="wheel_drive_rear" wheelSkewStiffness="12" /></Player>
------------------------------------------------------------
TaskControl
<SimCtrl><InitDone place="checkInitConfirmation" /></SimCtrl>
------------------------------------------------------------
TaskControl
<SimCtrl><Run currentState="6" /></SimCtrl>
------------------------------------------------------------
viTrafficDyn
<Player name="StaticCar"><Driver ctrlLatLong="ghostdriver" /></Player>
------------------------------------------------------------
ExampleConsole
<Sensor name="Sensor_MM" type="video"><Load lib="libModuleCameraSensor.so" path="/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/../Distros/Distro/Plugins/ModuleManager" /><Player name="New Player" /><Frustum bottom="15.000000" far="40.000000" left="20.000000" near="1.000000" right="20.000000" top="15.000000" /><Position dhDeg="0.000000" dpDeg="0.000000" drDeg="0.000000" dx="0.000000" dy="0.000000" dz="0.000000" /><Origin type="usk" /><Cull enable="true" maxObjects="10" /><Port name="RDBout" number="48185" sendEgo="true" type="TCP" /><Filter objectType="none" /><Filter objectType="pedestrian" /><Debug camera="false" culling="false" detection="false" dimensions="false" enable="false" packages="false" position="false" road="false" /></Sensor>
------------------------------------------------------------
ExampleConsole
<Camera name="VIEW_CAMERA" showOwner="false"><Frustum far="1500.000000" fovHor="40.000000" fovVert="30.000000" near="1.000000" offsetHor="0.000000" offsetVert="0.000000" /><PosEyepoint /><ViewRelative dh="0.000000" dp="0.000000" dr="0.000000" /><Set /></Camera>
------------------------------------------------------------
ExampleConsole
<Display><SensorSymbols enable="false" sensor="Sensor_MM" showCone="false" /><Database enable="true" streetLamps="false" /><VistaOverlay enable="false" /></Display>
------------------------------------------------------------
ExampleConsole
<Environment><Friction value="1.000000" /><TimeOfDay headlights="false" value="39600" /><Sky cloudState="4/8" visibility="100000.000000" /><Precipitation intensity="4.000000" type="snow" /><Road effectScale="0.500000" state="wet" /></Environment>
------------------------------------------------------------
ExampleConsole
<Symbol name="expl01"><Text colorRGB="0xffff00" data="Time for snow" size="50.0" /><PosScreen x="0.01" y="0.05" /></Symbol>
------------------------------------------------------------
ExampleConsole
<VIL><EyepointOffset hDeg="30.000000" pDeg="0.000000" rDeg="0.000000" x="0.000000" y="0.000000" z="0.000000" /></VIL>
------------------------------------------------------------
ExampleConsole
<VIL><Imu dbElevation="true" /></VIL>
------------------------------------------------------------
ScpGenerator
<VIL><EyepointOffset hDeg="0.000000" pDeg="0.000000" rDeg="0.000000" x="0.000000" y="0.000000" z="0.000000" /></VIL>

Traffic
<Traffic><InfoAction id="1" owner="New Character" status="start" trgName=" trgPosX="-58.1298" trgPosY="2478.43" type="CharacterMotion"><Param name="move" value=" /><Param name="speed" value="1.22222" /><Param name="force" value="false" /></InfoAction></Traffic>
------------------------------------------------------------
Traffic
<Traffic><InfoAction id="1" owner="New Character" status="stop" trgName=" type="CharacterMotion" /></Traffic>
------------------------------------------------------------
Traffic
<Traffic><InfoAction id="2" owner="New Character" status="start" trgName=" trgPosX="-58.1298" trgPosY="2478.43" type="CharacterPath"><Param name="pathShape" value="2" /><Param name="beam" value="true" /><Param name="loop" value="false" /></InfoAction></Traffic>
------------------------------------------------------------
Traffic
<Traffic><InfoAction id="2" owner="New Character" status="stop" trgName=" type="CharacterPath" /></Traffic>
------------------------------------------------------------
Traffic
<Traffic><InfoAction id="1" owner="New Character01" status="start" trgName=" trgPosX="-68.237" trgPosY="2470.43" type="CharacterMotion"><Param name="move" value=" /><Param name="speed" value="1.77778" /><Param name="force" value="false" /></InfoAction></Traffic>
------------------------------------------------------------
Traffic
<Traffic><InfoAction id="1" owner="New Character01" status="stop" trgName=" type="CharacterMotion" /></Traffic>
------------------------------------------------------------
Traffic
<Traffic><InfoAction id="2" owner="New Character01" status="start" trgName=" trgPosX="-68.237" trgPosY="2470.43" type="CharacterPath"><Param name="pathShape" value="1" /><Param name="beam" value="true" /><Param name="loop" value="false" /></InfoAction></Traffic>
------------------------------------------------------------
Traffic
<Traffic><InfoAction id="2" owner="New Character01" status="stop" trgName=" type="CharacterPath" /></Traffic>

 */



#endif _SCP_H