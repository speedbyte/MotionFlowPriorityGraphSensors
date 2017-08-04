import sys
from DeviceConfigurationInterface import ConfigurationInterface
import DeviceFrame

# ###########
# Main part
# ###########
if __name__ == "__main__":    
    # place of config file
    ConfigDir= './/config//'
    ConfigFile="DeviceTracer.ini"    
    # Process now data of config file 
    
    
    ConfigData = ConfigurationInterface(ConfigDir + ConfigFile) # create ConfigurationInterface class   
    if (ConfigData.SteeringData == None):
        # problems with config file 
        sys.exit(4)
    # start DeviceFrame
    DeviceFrame.Start(ConfigData) 
    sys.exit(0)