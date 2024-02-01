# Installation from Source Code

First install the dependencies:
- [Install CMake 3.16+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-cmake.md/)
- [Install YCM 0.11+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-ycm.md/)
- [Install YARP 3.8+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-yarp.md/)
- [Install YARP 3.8+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-yarp.md/)

### Components with known additional/specific dependencies
- [../libraries/YarpPlugins/DextraCanControlBoard](../libraries/YarpPlugins/DextraCanControlBoard#requirements)

## Installation (Ubuntu)

Once the required dependencies have been installed, the code has to be compiled and installed. Note that you will be prompted for your password upon using `sudo` a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it doesn't exist; then, enter it
git clone https://github.com/roboticslab-uc3m/dextra-yarp-devices.git  # Download yarp-devices software from the repository
cd dextra-yarp-devices; mkdir build; cd build; cmake ..  # Configure the dextra-yarp-devices software
make -j$(nproc)  # Compile
sudo make install  # Install :-)
sudo ldconfig  # Just in case
```

For additional options use `ccmake` instead of `cmake`.
