const char *project_name = "<SimCtrl><Project name=\"Movement\" "
        "path=\"/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/Current\" "
        "/></SimCtrl>";


const char *scenario_nam = "<SimCtrl><UnloadSensors /><LoadScenario "
        "filename=\"/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/Current/Scenarios/truck.xml\" /><Start mode=\"operation\" /></SimCtrl>";

std::string scenario_name = "<SimCtrl><UnloadSensors /><LoadScenario "
        "filename=\"/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/Current/Scenarios/truck.xml\" /><Start mode=\"operation\" /></SimCtrl>";


const char *module_manager = "<Sensor name=\"Sensor_MM\" type=\"video\"><Load lib=\"libModuleCameraSensor.so\" "
        "path=\"/local/git/MotionFlowPriorityGraphSensors/VIRES/VTD.2.0/Data/Projects/../Distros/Distro/Plugins/ModuleManager\" /><Player name=\"New Player\" /><Frustum bottom=\"15.000000\" far=\"40.000000\" left=\"20.000000\" near=\"1.000000\" right=\"20.000000\" top=\"15.000000\" /><Position dhDeg=\"0.000000\" dpDeg=\"0.000000\" drDeg=\"0.000000\" dx=\"0.000000\" dy=\"0.000000\" dz=\"0.000000\" /><Origin type=\"usk\" /><Cull enable=\"true\" maxObjects=\"10\" /><Port name=\"RDBout\" number=\"48185\" sendEgo=\"true\" type=\"TCP\" /><Filter objectType=\"none\" /><Filter objectType=\"pedestrian\" /><Debug camera=\"false\" culling=\"false\" detection=\"false\" dimensions=\"false\" enable=\"false\" packages=\"false\" position=\"false\" road=\"false\" /></Sensor>";

/*
"<TaskControl><RDB client="false" enable="true" interface="eth0" portRx="48190" portTx="48190" portType="TCP" /></TaskControl>"

"<Environment><Friction value="1.000000" /><TimeOfDay headlights="false" value="39600" /><Sky cloudState="4/8" visibility="100000.000000" /><Precipitation intensity="0.000000" type="none" /><Road effectScale="0.500000" state="dry" /></Environment>"

"<Camera name="VIEW_CAMERA" showOwner="false"><Frustum far="1500.000000" fovHor="40.000000" fovVert="30.000000" near="1.000000" offsetHor="0.000000" offsetVert="0.000000" /><PosEyepoint /><ViewRelative dh="0.000000" dp="0.000000" dr="0.000000" /><Set /></Camera>"

"<Display><SensorSymbols enable="false" sensor="Sensor_MM" showCone="false" /><Database enable="true" streetLamps="false" /><VistaOverlay enable="false" /></Display>"

"<VIL><Imu dbElevation="true" /></VIL>"

"<VIL><EyepointOffset hDeg="0.000000" pDeg="0.000000" rDeg="0.000000" x="0.000000" y="0.000000" z="0.000000" /></VIL>"

traffic_demo.xml
car.xml
moving_car.xml
moving_car_near.xml
truck.xml
moving.xml
moving_truck.xml
two.xml




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
