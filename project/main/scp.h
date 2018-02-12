#ifndef _SCP_H
#define _SCP_H


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


#endif _SCP_H