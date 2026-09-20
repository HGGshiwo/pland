cd /tmp
git clone --recurse-submodules --branch Copter-4.5 https://github.com/ArduPilot/ardupilot
cd ardupilot

export DO_AP_STM_ENV=0 
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
# ./waf configure --board sitl
# ./waf copter
