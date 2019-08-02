# Pioneer 3-AT (All Terrain) mobile base driver in DUNE/IMC framework

Basic driver for MobileRobot's (discontinued) [P3AT mobile base](https://www.generationrobots.com/en/402397-robot-mobile-pioneer-3-at.html) for the [DUNE](https://lsts.fe.up.pt/toolchain/dune) framework, using [IMC](https://lsts.fe.up.pt/toolchain/imc) messages, both part of the [LSTS toolchain](https://lsts.fe.up.pt/toolchain).
The driver consists of a DUNE task which uses a modified version of the [Aria library](https://github.com/rgmaidana/libaria) to communicate to the robot.
The task implements basic movement capabilities (sets motor translation and rotation speeds), as well as dispatching the robot's pose (from odometry).
The task's code was based on Amanda Whitbrook's [*Programming Mobile Robots with Aria and Player*](https://www.amazon.com/Programming-Mobile-Robots-Aria-Player/dp/1848828632) book, as well as the [ROSARIA](https://github.com/amor-ros-pkg/rosaria) ROS package.

This driver, as well as the DUNE framework, was designed to work with Linux/UNIX operating systems. The driver was tested with LSTS' [GLUED](https://lsts.fe.up.pt/toolchain/glued) linux distribution (in a Beaglebone Black embedded computer board), although it should work with most POSIX-compliant operating systems (e.g., Debian, Ubuntu).

# Installation/Build

* Clone this repository (for example, to your system's home folder)

```
git clone https://github.com/rgmaidana/P3AT-dune-driver $HOME/P3AT-dune-driver
```

* Create a directory following DUNE's install instructions structure

```
mkdir -p $HOME/dune/build
```

* Clone the [DUNE framework repository](https://github.com/LSTS/dune), with the "dune" name, into the "dune" directory

```
git clone https://github.com/LSTS/dune $HOME/dune/dune
```

* Copy the P3AT DUNE task (src/P3AT directory), from this repository, to DUNE's "src" directory

```
cp -r $HOME/P3AT-dune-driver/src/P3AT $HOME/dune/dune/src
```

* Copy the P3AT driver configutation file task (etc/P3AT.ini), from this repository, to DUNE's "etc" directory

```
cp -r $HOME/P3AT-dune-driver/etc/P3AT.ini $HOME/dune/dune/etc
```

* Clone the modified Aria library repository to DUNE's "vendor/libraries" directory, with the "aria" name

```
git clone https://github.com/rgmaidana/libaria $HOME/dune/dune/vendor/libraries/aria
```

* Go to DUNE's build directory and configure the project for compilation

```
cd $HOME/dune/build
cmake ../dune
```

* In DUNE's build directory, build the IMC messages and then build the project (don't be alarmed by the amount of warnings when compiling the Aria library)

```
git clone https://github.com/lsts/imc IMC
make imc
make
```

## Cross-compilation for ARM processors

As DUNE is a large project, it is wise to cross-compile the framework to be used in ARM architectures (e.g., beaglebone, raspberry). To perform cross-compilation in DUNE, follow the installation steps above until the second to last step: When running cmake, use the "-DCROSS" argument to indicate the directory with the C and C++ compilers for ARM. For example (using a GLUED-generated toolchain for Beaglebone):

```
cmake -DCROSS=$HOME/beaglebone/glued/lctr-b2xx/toolchain/bin/armv7-lsts-linux-gnueabi- ../dune/
```

If using GLUED, you can build the project as a monolithic package (with static libraries), using the "package" option in make:

```
make package
```

This produces a single "dune" tarball, with the whole framework.


## Dependencies

* [DUNE](https://lsts.fe.up.pt/toolchain/dune)
* [IMC](https://lsts.fe.up.pt/toolchain/imc)
* [ARIA (modified to fit DUNE's compilation scheme)](https://github.com/rgmaidana/libaria)

# Usage

* Connect a computer (or embedded board) to the robot's serial port, for communication
* Build the project in your robot's computer following the build instructions above (or copy the built project if cross-compiling)
* On the robot's computer, go to the dune build directory and run the DUNE application, indicating the P3AT configuration file:

```
cd $HOME/dune/build
./dune -c P3AT
```

* If the project was compiled as a package ("make package"), the dune application will be located in the "bin" folder:

```
./bin/dune -c P3AT
```

The driver expects [DesiredVelocity](https://www.lsts.pt/docs/imc/master/Guidance.html?highlight=desiredvelocity#desired-velocity) messages (for teleop) over UDP, in port 6002, and sends [EstimatedState](https://www.lsts.pt/docs/imc/master/Navigation.html?highlight=estimatedstate#estimated-state) messages (with the robot pose), also in UDP, to a static destination and port 6004.
The [P3AT.ini](https://github.com/rgmaidana/P3AT-dune-driver/blob/master/etc/P3AT.ini) configuration file sets the IP destination address for EstimatedState as static. If you wish to receive the robot pose information somewhere, **you must change the Static Destinations IP** to the address where you wish to send the pose information.

# Acknowledgements

* [Laboratório de Sistemas e Tecnologia Subaquática (LSTS)](https://lsts.fe.up.pt/)
* [Laboratório de Sistemas Autônomos (LSA)](https://lsa-pucrs.github.io/)
* [ARIA Library](https://github.com/amor-ros-pkg/libaria)
* [ROSARIA package](https://github.com/amor-ros-pkg/rosaria)

# Contributors

* [Renan Maidana](https://github.com/rgmaidana)
