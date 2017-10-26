/********************************************************
VTD Connector
 * Enables communication with simple (custom) messages. *
********************************************************
*
*
* (Proxy-)Message containing:
* ---------------------------
* uint16 - protocol
* uint16 - package/message ID
* uint32 spare[6] - spares
* uint32 - data size (size of message data following this entry) in bytes
*
*
* Message data must contain:
* ---------------------------
* message type
* sender information
* actual data to be processed (traffic signs, road information, other vehicles, etc.)
*
*/

/**

 VIRES Virtual Test Drive is a tool chain and modular framework for the provision of virtual environments in
 engineering simulations for the automotive and railroad industry.
 The dongle can be configured using export VI_LIC_DEVICE="name_of_the_dongle" in vtdStart.sh
 VTD Starts in config mode - REd in Task Control and green is ParameterServer. This is called Config mode.


 If you do not click the Apply buttong, then you can change the paramters using the parameter browser. In case you
 have already clicked the Apply button, then you can go back to the config mode, by pressing the Configure button.
 Ofcourse you have to stop a running simualtion first. Thereafter all processes but Task Control, Parameter Server
 and GUI will be stopped.

 The video is started by clicking on the video record button on the left and is stopped by clicking on the video stop
 buton on the right. The conversion processes are described in Setups/Common/Scripts/vrecConvert.sh.
 Instructions from operating from command line only, AdvancedSetup, autoStart, autoConfig.
 GUI-less and IG-less Operation (auto-started / auto-configured simulation)
 purpose is to run VTD in a headless test automation environment without any rendering (i.e. only RDB data is of
 interest)

 One needs to add the post processing step in the IGBase.xml in order to write the images to the shared memory. The
 IG post processing pipeline allows rendering of a scene with subsequent modification and combination of generated
 images. Sample use cases of the pipeline include corrective image warping for projection on uneven surfaces, tone
 mapping of HDR images and various visual effects including depth of view.
 Post processing is implemented by the IG Component PostProcessing.
  - PostProcessing with a name.
  - PostProcessingPipelineConfigurator with a name.
  - single pipeline child with one or more steps as children.

 Rendering the scene into a texture and then display it.

 IGBase.xml <ShmLayoutImage key="0x0811b" freeFlag="0x00000000" releaseFlag="0x00000002"/>
 ProjectIGBase.xml

 autoStart, autoConfig. autoStart invokes the scpGenerator with the parameter tcpAutoStart.scp

 ------ shared memory identifiers ------
 #define RDB_SHM_BUFFER_FLAG_NONE                    0x00000000      /**< no bits set, buffer may be overwritten
 #define RDB_SHM_BUFFER_FLAG_LOCK                    0x00000001      /**< buffer is locked by producer
 #define RDB_SHM_BUFFER_FLAG_TC                      0x00000002      /**< buffer is to be processed by TC
 #define RDB_SHM_BUFFER_FLAG_IG                      0x00000004      /**< buffer is to be processed by IG

 Setups -> AutoCfgDatabase.xml  -> here is your scene in Projects/Current/Databases/*.ive file
                                -> here is your occlusion in Projects/Current/Databases/*.occl file
                                all of them are soft link to Distros/Databases

 The primary source and receiver of RDB is the Task Control.
 Module Manager sends and receives data to and from RDB. The data is then forwarded to the plugins. The plugins
 refactors/exterds the data with additional information and then sends it again on the RDB. The data can only be sent
 by plugins and not received directly.
 RDB_MSG_HDR_t        headerSize  = sizeof( RDB_MSG_HDR_t )
                      dataSize    = sum of all following "headerSize" and "dataSize" information, i.e.
                                     5 * sizeof( RDB_MSG_ENTRY_HDR_t ) +
                                     3 * sizeof( TypeA ) +
                                     1 * sizeof( TypeB ) +
                                     2 * sizeof( TypeC )
 RDB_MSG_ENTRY_HDR_t  pkgId       = RDB_PKG_ID_START_OF_FRAME
                      headerSize  = sizeof( RDB_MSG_ENTRY_HDR_t )
                      dataSize    = 0
                      elementSize = 0
 RDB_MSG_ENTRY_HDR_t  pkgId       = TypeA
                      headerSize  = sizeof( RDB_MSG_ENTRY_HDR_t )
                      dataSize    = 3 * sizeof( TypeA )
                      elementSize = sizeof( TypeA )
     entryOfTypeA
     entryOfTypeA
     entryOfTypeA
 RDB_MSG_ENTRY_HDR_t  pkgId       = TypeB
                      headerSize  = sizeof( RDB_MSG_ENTRY_HDR_t )
                      dataSize    = 1 * sizeof( TypeB )
                      elementSize = sizeof( TypeB )
     entryOfTypeB
 RDB_MSG_ENTRY_HDR_t  pkgId = TypeC
                      headerSize  = sizeof( RDB_MSG_ENTRY_HDR_t )
                      dataSize    = 2 * sizeof( TypeC )
                      elementSize = sizeof( TypeC )
     entryOfTypeC
     entryOfTypeC
 RDB_MSG_ENTRY_HDR_t  pkgId       = RDB_PKG_ID_END_OF_FRAME
                      headerSize  = sizeof( RDB_MSG_ENTRY_HDR_t )
                      dataSize    = 0
                      elementSize = 0

 execute the script as follows: sudo ./instMultiUser.sh -r VTD.2.0

 Single Ray

 It computes the intersection of this ray and the bounding box of other objects (vehicles); it returns the position
 of these objects in sensor co-ordinates with the origin being at the sensor's mounting point.

 The multi-ray sensor provides a means to interact with the actual geometry of the 3d database from within the
 moduleManager. For this purpose, there is a communication channel between MM and the imageGenerator. This means
 that you may also take continuous features of the environment (e.g. hills, road surface) into account for the sensing.

 Real-Time Ray-Tracing (via Optix)

 The VIG-OptiX SDK is a real-time ray tracing plugin using NVIDIA's OptiX ray tracing engine.

 woody./PathToYourWorkingDir/>cp vtd.x.x.addOns.*optix*.yyyymmdd.tgz .
 woody./PathToYourWorkingDir/>tar -xzvf vtd.x.x.addOns.*optix*.yyyymmdd.tgz .

 switch your setup to OptiX.Stream or OptiX.NonVisualSpectrum

 woody./PathToYourWorkingDir/VTD.X.X>cd Data/Setups
 woody./PathToYourWorkingDir/VTD.X.X/Data/Setups/>./selectSetup

 OptiXPluginExample

 Plugin Example
 see Plugin Example

 IG plugin development
 see VTD IG plugins

 Optix4.1

 Coordinate Systems
 VIRES Installation

 Demo
 VTD.2.0/bin
 VTD.2.0/Data/Distros/Current
 VTD.2.0/Data/Setups/Current
 VTD.2.0/Data/Setups/Standard
 VTD.2.0/Data/Setups/Common
 VTD.2.0/Data/Setups/Stereo
 VTD.2.0/Data/Setups/Common
 VTD.2.0/Data/Setups/Joystick
 VTD.2.0/Data/Setups/DualHost
 VTD.2.0/Data/Setups/Standard.noIG
 VTD.2.0/Data/Setups/OpenCRG
 VTD.2.0/Data/Projects/Default
 VTD.2.0/Data/Projects/Current
 VTD.2.0/Data/Projects/SampleProject
 VTD.2.0/Data/Projects/OpenCRG
 VTD.2.0/doc
 VTD.2.0/Develop/Framework
 VTD.2.0/Develop/Communication
 VTD.2.0/Runtime/Core/IG64
 VTD.2.0/Runtime/Tools/ModelConverter
 VTD.2.0/Runtime/Tools/Bugreport
 VTD.2.0/Runtime/Tools/Installation
 VTD.2.0/Runtime/Tools/VehicleController
 VTD.2.0/Runtime/Tools/Drivers
 VTD.2.0/Runtime/Tools/LicServer
 VTD.2.0/Runtime/Tools/RDBSniffer
 
 
 Plugin
 VTD.2.0/Develop/IG64/Lib
 VTD.2.0/Develop/IG64/doc
 VTD.2.0/Develop/IG64/bin
 VTD.2.0/Develop/IG64/Plugins/CarSim
 VTD.2.0/Develop/IG64/Plugins/FramebufferReader
 VTD.2.0/Develop/IG64/Plugins/OptiXPluginExample
 VTD.2.0/Develop/IG64/Plugins/PedestriansBDI_Base
 VTD.2.0/Develop/IG64/Plugins/OptiXLidar
 VTD.2.0/Develop/IG64/Plugins/EnvironmentManager
 VTD.2.0/Develop/IG64/Plugins/Symbols
 VTD.2.0/Develop/IG64/Plugins/PedestriansBDI_v13
 VTD.2.0/Develop/IG64/Plugins/SpecialEffects
 VTD.2.0/Develop/IG64/Framework
 VTD.2.0/Develop/IG64/ViresLibs/VTDFramework
 VTD.2.0/Develop/IG64/Cmake/
 VTD.2.0/Develop/IG64/3rdParty/Optix
 VTD.2.0/Develop/IG64/3rdParty/CUDA
 VTD.2.0/Develop/IG64/3rdParty/Boost
 VTD.2.0/Develop/IG64/3rdParty/OpenSceneGraph
 VTD.2.0/Develop/IG64/Tools/RDBComTester
 VTD.2.0/Develop/IG64/Tools/BuildTools
 
 VTD.2.0/Data/Projects/Demo.VIGPlugin
 VTD.2.0/Data/Projects/Optix.NonVisualSpectrum
 VTD.2.0/Data/Projects/Optix
 VTD.2.0/Data/Projects/Demo.OptiXPlugin
 VTD.2.0/Data/Projects/OptiXLidar
 VTD.2.0/Data/Distros/Distro/Databases/OptiX/
 VTD.2.0/Data/Setups/Demo.VIGPlugin
 
 ROD Package
 VTD.2.0/Runtime/Tools/RodDistro_2638_Rod64b_4.5.5
 
 instMultiUser.sh : This file simply creates symbolic links. The file is interactive and hence just run the file.
 
 selectStarup.sh : This file simply changes the setup. One can change setup from default to NoIG or something new for
 example.
 
 https://secure.vires.com/demo/vtd/vtd.2.0.3.Demo.Road.20170131.tgz
 https://secure.vires.com/demo/vtd/vtd.2.0.3.addOns.ROD64b.Standard_Dongle_20170209.tgz
 
 login: demo
 OpenSesame: viresDemo
 
 tar -xvzf vtd.2.0.3.Demo.Road.20170131.tgz -C /local/development
 tar -xvzf vtd.2.0.3.addOns.ROD64b.Standard_Dongle_20170209.tgz -C /local/development
 Run instMultiUser.sh from /local/development. It will move the binaries and other generic stuff to /opt and /var and
 make symlinks to /local/development.
 Its better to chown /local/development/bin to root.
 
 \subsection{License}
 
 \textbf{License File and Location}
 The license file needs to be placed under VTD2.0/bin/
 Please make sure the license file obtained has the same MAC Address as the Dongle.
 
 \textbf{License Hardware Configuration}
 Wireless Dongle
 /etc/modules.load/modules.conf - write mt7601u to load this module at run time.
 To check simply modprobe mt7601u
 
 14.04
 The precondition is that the wifi dongle should be loaded in ifconfig.
 sudo apt-add-repository ppa:thopiekar/mt7601
 sudo apt-get update
 sudo apt-get install mt7601-sta-dkms
 
 Sempre Wireless Dongles use MediaTek drivers ( mt7601u ). The following will load the driver mt7601Usta to be found
 under
 /lib/modules/3.13.0-65-generic/updates/dkms/mt7601Usta.ko
 Please check using modinfo mt7601Usta
 
 16.04 - automatic because the drivers is already built in the kernel
 To check please invoke lsmod | grep mt7601u
 
 if the device is not present in ifconfig, then check for ifconfig -a
 Then start the link ip link set dev  up
 
 Open the file Data/Setups/Current/Config/SimServer/simServer.xml
 Add an environment variable: <EnvVar name="VI_LIC_DEVICE" val="ed8sf0" /> (replace ed8sf0 with the name of your device)
 Find the name of the device by using ifconfig.
 
 In order for VTD to work correctly in terms of network communication, please make sure that your system has a valid
 hostname and a valid host address. If you are not connected to any network, please set a static address for your
 system. You may also just edit the file /etc/hosts and add the entry
 127.0.0.2 nameOfYourHost
 
Single Ray


It computes the intersection of this ray and the bounding box of other objects (vehicles); it returns the position of these objects in sensor co-ordinates with the origin being at the sensor's mounting point.


The multi-ray sensor provides a means to interact with the actual geometry of the 3d database from within the moduleManager. For this purpose, there is a communication channel between MM and the imageGenerator. This means that you may also take continuous features of the environment (e.g. hills, road surface) into account for the sensing.


    Real-Time Ray-Tracing (via Optix)

The VIG-OptiX SDK is a real-time ray tracing plugin using NVIDIA's OptiX ray tracing engine.

       woody./PathToYourWorkingDir/>cp vtd.x.x.addOns.*optix*.yyyymmdd.tgz .
   woody./PathToYourWorkingDir/>tar -xzvf vtd.x.x.addOns.*optix*.yyyymmdd.tgz .

    switch your setup to OptiX.Stream or OptiX.NonVisualSpectrum

   woody./PathToYourWorkingDir/VTD.X.X>cd Data/Setups
   woody./PathToYourWorkingDir/VTD.X.X/Data/Setups/>./selectSetup

 StartVIRES

 VTD provides a development environment for the creation of custom moduleManager plug-ins. First, make sure you have
 a license and the libraries for the moduleManager plug-in API. These are located at Develop/Modules


 \textbf{Plugin Development OptiX}
 
 OptiXPluginExample

 Plugin Example
 see Plugin Example
 IG plugin development
 see VTD IG plugins

 Optix4.1

 
 First step: compile the example
 
 The source code of the PerfectSensor is provided as an example under Develop/Modules/PerfectSensor. This example may
 be compiled with the following steps:
 
     Prerequisites:
         g++ 4.2.1
         32bit compilation
     Go to Develop/Modules/PerfectSensor
     Call qmake -spec linux-g++-32
     Call make
     The plugin will be compiled and will be stored at Develop/bin/Plugins
 
 Shared Memory
 If you are rendering large images and transfer them to shared memory, the system's shared memory limit might not be
 sufficient. You may determine the current maximum shared memory size with the command
 # cat /proc/sys/kernel/shmmax
 The setting may be changed temporarily using
 # sysctl -w kernel.shmmax=66123456
 In order to permanently set the limit, add or edit the line in /etc/sysctl.conf
 # echo "kernel.shmmax=66123456 >> /etc/sysctl.conf@
 
 
 If you tend to ignore our advice and use Unity desktop nevertheless ;-), ROD may have issues with the correct
 display of menus etc. In this case, please do the following:
 open the file ~/.config/Trolltech.conf
 add the following entry under the tag [Qt]
 [Qt]
 style=Plastique
 GNOME 3
 GUI elements (VtGui, ROD) will be screwed up. You may fix this by editing
 ~/.config/Trolltech.conf
 and adding the following lines in this file
 [Qt]
 style=Platinum
 
 Unless you have a custom configuration, the image generator will derive the display settings from the file
 Data/Setups/Current/Config/ImageGenerator/AutoCfgDisplay.xml. Here, look for the variables displayNum and screenNum
 and set them appropriately.
 Note: the image generator's config file is created by a script, so if you have auto-configuration of the IG enabled,
 you will need to change the script instead of the config file. Please perform the following steps:
 create a directory Data/Setups/Current/Scripts
 copy the file Data/Setups/Common/Scripts/configureDisplay.sh to the newly created directory
 edit the new copy and set displayNum and screenNum accordingly
 next time you load and INIT the simulation, the new settings will be written to
 Data/Setups/Current/Config/ImageGenerator/AutoCfgDisplay.xml
 displayNum is the environment variable $DISPLAY
 

 
 # 18.11.2016 by M. Dupuis
 # (c) 2016 by VIRES Simulationstechnologie GmbH
 #
 # PURPOSE:
 #   tests automatic start/stop of a simulation
 #
 #
    0 "<SimCtrl> <Stop/> <LoadScenario filename="traffic_demo.xml" /> <Init mode="operation"/> </SimCtrl>"
 #
 wait "<SimCtrl> <InitDone place="checkInitConfirmation"/> </SimCtrl>"
 #
   +1 "<SimCtrl> <Start/> </SimCtrl>"
  +5s "<Symbol name="expl01" > <Text data="End of Test" colorRGB="0xffff00" size="50.0" /> <PosScreen x="0.01" y="0
  .05" /></Symbol>"
  +1s  "<SimCtrl> <Stop/> </SimCtrl>"
 
 tcpdump 'tcp port 48190 and (((ip[2:2] - ((ip[0]&0xf)<<2)) - ((tcp[12]&0xf0)>>2)) != 0)'
 rdb image is on TCP 48190
 scp commands are on UDP 32512
 48179
 
 Real time rendering of high dynamic range images for validation of driver assistance systems.
 
 High dynamic range rendering in computer graphics is a well known method to create synthetic images. Some parts of
 the generated images, which can be measured in luminance are wrong – road too bright and traffic sign too dark.
 
 Digital camera and another luminance meter – luminance to pixel value.
 RGBE Image file format – HDR imaging film format.
 Light mapping through different image operations. Tone mapping resulted in more visible result.
 
 HDR technique : combining differently exposed images to produce a single HDR image.
 
 HDR images are used as input for camera systems, in order to accurately simulate the extreme lighting conditions of
 reality.
 
 Ray tracing radar prototypes have detailed material properties, multiple reflections, absorption, attenuation,
 scattering etc.
 
 VTD Vehicle Library
 All the vehicles.
 
 Scenario Editor Manual
  - Action Trigger. Actions are triggered at trigger points. Triggers can be chosen from Trigger List.
 <Traffic> <Trigger id=”myTrigger” active=”true | false” /> </Traffic>
  - SCP Gui
 How to control players ( cars ) , characters ( pedesterias ) and objects ( vegetation )
 How to control drivers
 
 VTD User Manual
 
 ROD Tutorial
 How to create a layout of roads, cities etc.
 
 VTD 2.0 - sneak preview
 Some improvements in VTD2.0. Powerpoint.
 
 VIRES Driver Characteristics
 Please see Scenario Editor Manual
 
 HDR Validation of v-IG
 The sensor simulator input data contains image, object list or ray tracing information. HDR images are used as input
 for camera systems, in order
 to accurately simulate the extreme lighting conditions of reality.
 
 TestReports
 Vehicle Dynamics - simplified
 VT-MÄK License Management

     ModuleManager
     Vehicle Configuration Files
     Material System
     Working with OpenCRG Data
     Working with Map and Terrain Data
 
 Image Generator (vIG)
 
     Sky Model
     OS Optimization
     vIG plugins
         Plugin Example
     OptiX SDK
         OptiX-related FAQs
     RDBInterface
     Post Processing
 
 0.01s, making the manager run at 100Hz.
 Sensor plug-ins are used for the extraction of data from the virtual world within a given sub-space which is usually
 connected to the own vehicle. The standard sensors work like filter which are adding some information about occlusion etc.
 

 <RDB>
     <Port name="RDBraw" number="48190" type="TCP" />
 </RDB>
 <Debug    enable="true"
           :
           performance="true" />
 <Sensor name="perfectFront" type="video">
     <Load     lib="libModulePerfectSensor.so" path="" persistent="true" />
     <Frustum  near="0.0" far="50.0" left="10.0" right="10.0" bottom="3.0" top="3.0" />
     <Cull     maxObjects="5" enable="true" />
     <Port     name="RDBout" number="48195" type="UDP" sendEgo="true" />
     <Player   default="true" />
     <Position dx="3.5" dy="0.0" dz="0.5" dhDeg="0.0" dpDeg="0.0" drDeg="0.0" />
     <Filter   objectType="pedestrian"/>
     <Filter   objectType="vehicle"/>
     <Filter   objectType="trafficSign"/>
     <Filter   objectType="obstacle"/>
     <Filter   objectType="roadInfo"/>
     <Filter   objectType="laneInfo"/>
     <Filter   objectType="roadMarks" tesselate="true"/>
 
 </Sensor>
     <Filter   objectType="trafficSign" signType="274"/>
     <Filter   objectType="trafficSign" signType="281"/>
 <Debug enable="false" />
 <Cull maxObjects="5" enable="true" maxOcclusion="0.3"/>
 <Sensor name="perfect" type="video">
     <Load     lib="libModulePerfectSensor.so" path="" persistent="true" />
     :
     <Output   sendOcclusionMatrix="true"/>
     :
 </Sensor>
 
 The occlusion matrix will be sent as an RDB package of type RDB_PKG_ID_OCCLUSION_MATRIX.
 After detecting the relevant objects (or calculating the respective information), each sensor will compose an RDB
 output data package containing the relevant information (object lists etc.). The output will be sent via the ports
 defined in the <Port> section of each sensor. Each sensor will open its own output port
 <Origin type="{USK|inertial|sensor|relative|gps|road}" dx=... dy=... dz=... dhDeg=... dpDeg=... drDeg=.../>
 RDB_IMAGE_t On the sensor's RDB output port, there will be - among others - one package of type SENSOR_OBJECT_t and
 one package of type OBJECT_STATE_t for each detected element.
 plugin: libModuleCameraSensor.so.
 
 
 <Sensor name="multiRay" type="video">

     <Load     lib="libModuleMultiRaySensor.so" path="" persistent="true" />
     <Frustum  near="0.0" far="50.0" left="10.0" right="10.0" bottom="3.0" top="3.0" />
     <Config   noRaysHorizontal="3" noRaysVertical="3" verbose="false" />
     <Port     name="RDBout" number="48195" type="TCP" sendEgo="false" />
     <Player   default="true" />
     <Position dx="3.5" dy="0.0" dz="0.5" dhDeg="0.0" dpDeg="0.0" drDeg="0.0" />
     <Debug    enable="false" />
 </Sensor>

 <TaskControl>
   :
   <Debug ...
     rayHits="true"/>
 </TaskControl>
 
 <Sensor>
   :
   <Noise axis="x" amplitude="1.0" frequency="40.0"/>
   <Noise axis="p" amplitude="0.2" frequency="20.0"/>
   :
 </Sensor>
 <Sensor>
   :
   <Noise axis="x" amplitude="1.0" frequency="40.0"/>
   <Noise axis="p" amplitude="0.2" frequency="20.0"/>
   <DataLoss duration="0.2" frequency="0.5"/>
   :
 </Sensor>
 
 
 
 
 
 
 
 
 
 Dynamics Plugins
 For test purposes you may add sinusoidal offsets to the resulting  x/y/z/h/p/r data of the vehicle dynamics before
 it is sent to the taskControl.
 Each Plug-in has one input Iface and one output Iface. The input Iface represents the whole data received by
 the moduleManager's input routines (i.e. RDB) and the output Iface represents all data generated within the plug-in
 (e.g.
 copies of the incoming data that have been found to be relevant within a sensor).
 
 
 
 
 
 As noted above, custom sensors must be derived from the class Module::SensorPlugin. At least the method
 update() must be implemented by the user. This routine is called by the ModuleManager in each
 simulation frame with the current frame number and a pointer to the interface class which contains all
 data that has been received from the TC.
 
 
 Before calling the update()-method, the routines of the base class SensorPlugin will have performed the
 following steps:
 
 - receiving of RDB data
 - object filtering (by type)
 - object detection (by frustum)
 

 
 RDB
     <RDB            name="default"
                     enable="true"
                     portType="SHM"
                     imageTransfer="false"
                     send="true"/>
 
     <RDB            name="shmIn"
                     enable="true"
                     portType="SHM"
                     imageTransfer="false"
                     receive="true"/>
 <RDB>
     <Port name="RDBraw" type="SHM" receive="true" />
     <Port name="RDBraw" type="SHM" send="true" />
 </RDB>
 

     <RDB            name="default"
                     portType="UDP"
                     imageTransfer="false"/>
     <RDB>
         <Port name="RDBraw" number="48190" type="UDP" />
     </RDB>
 
 
 The database contains the graphical elements of the virtual world. It is
 available in binary .ive-format (OpenSceneGraph). The corresponding
 logical database of the road network is stored in the OpenDRIVE format,
 an XML-based description. For the design and modification of both
 database, the editor RoadDesigner (ROD) may be purchased optionally.

  Scenario

 The scenario is stored in an XML format which is proprietary to
 VIRES. It contains references to the visual database and the logical
 database. Actions of all players are also contained in the scenario
 file. The file may be edited using the graphical ScenarioEditor.
 VTD provides an extensive set of binary data for each simulation step. The overall amount of data that is available
 on RDB is listed in
 the respective documentation. Some of the more important / frequent
 contents are given in the following list:
 
 
 Player Data (Vehicles, Pedestrians)
 
 
 position, orientation (heading, pitch, roll) and size (bounding-box)
 category of the player (vehicle, pedestrian)
 Details of Vehicles
 
 size, mass, axle distance
 information about drivetrain (speed, torque, gear...)
 information about driver-vehicle-interface (e.g. pedals, steering wheel)
 detailed information about wheels (steering angle, radius, forces etc.)
 brake pressure, spring compression (per wheel)
 status of vehicle lights
 Details of current road sections
 drive lane information (width, id, ...)
 road marks (type, color, ...)
 traffic signs and traffic lights (including current and future states)
 Data from VTD
 
 
 Images of the Image Generator
 height, width
 image data (visual range, depth information, infrared, ...)
 information about the camera which is used for generating the image (make sure you configure the TaskControl to the
 latest IG protocol version for this feature, i.e. "precision10" or higher)
 
 Sensor Data
 Perfect sensors may be positioned at arbitrary locations of
 vehicles. These may be used to simulate real sensors. Sensor output data
  is provided on dedicated connections of each sensor. The protocol also
 follows the RDB definition.
 position of the sensor
 distance to detected objects (with global object reference)
 
 Common Data
 time-of-day
 sky state
 weather conditions (fog, rain, snow)
 visibility
 road conditions (dry, wet)
 SCP provides the interface to all non-periodic communication within VTD. Among these are:
 
 Common Information About the Simulation
 filename and path of the current scenario and OpenDRIVE file
 simulation state (start, stop, ...)
 Information About Actions Within the Simulation
 execution of triggers from within the scenario
 state changes of traffic lights
 The full extent of the SCP command interface can be retrieved from the documentation in VTD/Doc/SCP_HTML/index.html.
 <Query label="a58s7" entity="player" id="1"/>"
 
 <Reply label="a58s7" entity="player" id="1" name="Ego"/>
 
 
 Database
 
 ALL SENSORS accept Crash Sensors
 
 - all
 - vehicle
 - pedestrian
 - light
 - trafficSign
 - obstacle
 - laneInfo (new in VTD 1.2.2)
 - roadMarks (new in VTD 1.2.2)
 
 
 What is a crash: Crash between bounding box of self and surrounding.
 If a crash is detected, it will issue an SCP message.

 <Filter rdbCategory="RDB_OBJECT_CATEGORY_PLAYER" rdbType="RDB_OBJECT_TYPE_PLAYER_CAR"/>
 <Filter rdbCategory="RDB_OBJECT_CATEGORY_PLAYER"/>

 The JitterSensor may be used for deteriorating the accuracy of information retrieved by the perfect sensor. On each
 component, a sinusoidal noise will be added to the actual signal. The user may specify frequency and amplitude of
 the noise per channel.
 Detection like a perfect sensor - this provides the basic object data of vehicles, pedestrians etc. Additional
 detection AND occlusion calculation for the following types of objects:traffic signs, vehicle lights (headlights,
 rear lights), street lamps, common obstacles (e.g. houses) which may occlude the former objects. The information
 about the sensor itself (contained in a package of type RDB_SENSOR_STATE_t) complements the data stream.
 
 -----

 ATZ . Automobil Technisch Zeitschrift
 VDI - Verein Deutsche Ingineure


 Class 1, Class 2 ( laser pointers ) , Class 3R, Class 3B and Class 4

 The laser power varies from 1 mw ( Laser pointers ) to  100s of watts ( Outdoor shows for example Audience scanning
 ) to MW ( Laser telescopes )


 Faster and still Reliable Sensor Fusion.

 Trying to solve this by making a generic framework and inducing 4 important details in the framework: Confusion,
 Consistency, Compromise and Confidence.

 High Dynamic Range: well known method to create synthetic images. 32 bit
 Low Dynamic Range: 8 bit
 Generated images cn be measured in Luminance, Illuminance

 High Density Rendering

 Difference between ray tracing and beam tracing:
 Rays have no thickness, but beams are shaped as unbounded pyramids with polygonal cross sections. The idea is since
 1984.
 Primarily two things: Ray tracing and scan line tracing.
 Ray tracing is a very complex algorithm, but in short there are

*/