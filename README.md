# ubm-rrt

This is an implementation of the *Theta\*-RRT* algorithm for our RoboRacer (formerly F1Tenth) stack. The algorithm has been replicated from [this paper](http://www.spencer.eu/papers/palmieriICRA16.pdf):


> L. Palmieri, S. Koenig and K. O. Arras
> 
> RRT-Based Nonholonomic Motion Planning Using Any-Angle Path Biasing

Since the package is not maintained anymore, documentation will not be updated according to UEP 11.

<img src="rrt/img/screenshot.png" alt="rrt_screenshot" width="1000" height="563">

## Authors
- UniBo Motorsport Driverless team
- [@Scheggetta](https://github.com/Scheggetta) - for the overall package and RRT logic
- [@TorioCrema](https://github.com/TorioCrema) - for the Theta* path generation algorithm and the speed profile generation
- [@Noce99](https://github.com/Noce99) - for the ROI generation from the map
- [@SamueleCrimi](https://github.com/SamueleCrimi) - for the POSQ controller adaptation and implementation
- [@andreaalboni](https://github.com/andreaalboni) - for the POSQ controller adaptation and implementation
- [@ParsaMK](https://github.com/ParsaMK) - for the speed profile generation

## How to make this ROS2 package work
First of all, this package has been tested on ROS 2 Foxy, but it should work without any problems also with the more recent versions like Humble and Jazzy.

> [!NOTE]
> **Only for organization members:**
> These files can be directly copied into `ubm-f1tenth` repo to make the package work as of 12/22/2024. To automatically build it in the docker container, you would have to add the link in the `~/f1tenth_ws/src` folder as usual.

### Missing `f1tenth_msgs` package
The only package that is not available online that you need to build this package is indeed `f1tenth_msgs`. Though, just by understanding the code you can create your own messages' implementations.

### What are `map_filepath` and `map_yamlpath` variables?
They are simply the filepaths of the map occupancy grid in .pgm format and its associated .yaml file, respectively. Those can be generated using the [slam_toolbox repo](https://github.com/SteveMacenski/slam_toolbox).

### What are `ax_max_machines_path` and `ggv_path` variables?
They correspond to the `ax_max_machines.csv` and `ggv.csv` file locations (filepaths), respectively. Those can be obtained and edited from the [TUM racing line generation repo](https://github.com/TUMFTM/global_racetrajectory_optimization).

### Build C++ dependencies

This package has some C++ dependencies that are exclusive to the `rrt` package: GEOS, FLANN, nanoflann and Armadillo. Once you have installed those, you can build the package as any other ROS 2 package using `colcon build`.

#### GEOS 3.12.1

```bash
cd ~ && \
wget https://github.com/libgeos/geos/releases/download/3.12.1/geos-3.12.1.tar.bz2 && \
tar xvfj geos-3.12.1.tar.bz2 && \
cd geos-3.12.1 && \
mkdir _build && cd _build && \
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
make -j12 && ctest && make install
```

#### FLANN 1.9.2

```bash
cd ~ && \
curl -L https://github.com/flann-lib/flann/archive/refs/tags/1.9.2.tar.gz > flann.tar.gz && \
tar xf flann.tar.gz && \
cd flann-1.9.2 && \
mkdir build && cd build && \
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_C_BINDINGS=OFF -DBUILD_PYTHON_BINDINGS=OFF \
      -DBUILD_MATLAB_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_DOC=OFF .. && \
make -j12 && make install
```

#### nanoflann v1.6.2

```bash
cd ~ && \
curl -L https://github.com/jlblancoc/nanoflann/archive/refs/tags/v1.6.2.tar.gz > nanoflann.tar.gz && \
tar xf nanoflann.tar.gz && \
cd nanoflann-1.6.2 && \
mkdir build && cd build && \
cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
make -j12 && make install
```

#### Armadillo 12.8.4

```bash
sudo apt install -y cmake libopenblas-dev liblapack-dev libarpack2-dev libsuperlu-dev && \
curl -L https://sourceforge.net/projects/arma/files/armadillo-12.8.4.tar.xz > ~/armadillo-12.8.4.tar.xz && \
cd ~ && tar -xf ~/armadillo-12.8.4.tar.xz && cd ~/armadillo-12.8.4 && \
cmake . && \
sudo make install && \
cd ~ && rm -rf ~/armadillo-12.8.4.tar.xz && rm -rf ~/armadillo-12.8.4
```
