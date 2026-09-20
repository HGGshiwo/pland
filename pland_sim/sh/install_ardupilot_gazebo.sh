cd /tmp

if [ ! -d "ardupilot_gazebo" ]; then
    git clone https://github.com/HGGshiwo/ardupilot_gazebo.git ardupilot_gazebo
fi
cd ardupilot_gazebo
rm -rf build
mkdir build && cd build
cmake ..

make -j$(nproc)

sudo make install