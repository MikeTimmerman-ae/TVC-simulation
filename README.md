# TVC Simulation

The simulation tool has the system's equations of motion and the numerical integration written in C++ with an interface to a python UI for easy analysation of the simulation data.

The C++ simulation program is split up in four building blocks with wich different control loops can be build: dynamics, controller, actuator, sensor.
### Dynamics
The dynamics class contains all information about the system's dynamics and state. Using the 'step' class method, a control input is fed into the system and the output response of the system to this input is obtained by integrating the system's equations of motion using the RK45.  The parameters characterizing the system are all contained within the private data members of the class. External forces and moments for a given system can be specificied in the 'calculateForce' and 'calculateMoment' class methods.

### Controller
The controller is an abstract class from which specific controllers are derived, such as a PID controller. The controller class provides the generic interface and reference specification with each derived controller class providing the control logic. 

The derived PID controller class can be configured to have multiple input and outputs as well as multiple inputs and one output. The gains of each input channel can easily be set are reset using the class methods.

### Actuator
The actuator class allows for modelling of the actuator dynamics, namely control input saturation as well as rate saturation. Noise and bias can also be specified on the actuator signal. Multiple channels can be specified for a specific actuator.

### Sensor
The sensor class allows for providing realistic output data of the system. Several derived classes contain a certain type of sensor, such as the IMU sensor class which gives access to gyroscopic and accelerometer data. The main sensor class provides an interface to specify sensor bias and noise for each sensor.

## Structure

The simulator uses Eigen as its linear algebra module. It is structured as containing each class in a separate file with the header files of the class being stored in the include directory and the code in the src directory. The main header file contains all includes to these header files. The project structure is as follows:
- TVC-simulation
    * GUI
      * GUI.py
      * __init\__.py
      * helpers.py
    * include
      * PIDcontroller.h
      * actuator.h
      * controller.h
      * controller.ipp
      * dynamics.h
      * helpers.h 
      * sensor.h
    * libraries
      * eigen (@submodule)
    * src
        * PIDcontroller.cpp
        * actuator.cpp
        * controller.cpp
        * dynamics.cpp
        * helpers.cpp
        * sensor.cpp
    * header.h
    * main.cpp
    * setup.py
    * main.py

## Installation

The project uses cmake to compile and link the project. This means that the user should have cmake installed to run the code as well as a C++ compiler, such as GCC. The user should also have git and python3 installed.

Following steps will enable you to set-up and build the project:

1. Set-up directory and initialize with git:

```console
foo@bar:~$ git init
```

2. Clone this project into the directory:
```console
foo@bar:~$ git clone https://github.com/MikeTimmerman-ae/TVC-simulation.git
```

3. Navigate to the project root and remove the submodules from the libraries directory and from git:
```console
foo@bar:~$ cd TVC-simulation
foo@bar:~$ rm -r libraries/eigen
foo@bar:~$ git rm -r --cached libraries/eigen
```
4. Install Eigen as submodule
```console
foo@bar:~$ git submodule add https://gitlab.com/libeigen/eigen.git libraries/eigen
```
5. Create a build directory and create the cmake files by running cmake from it:
```console
foo@bar:~$ mkdir build
foo@bar:~$ cd build
foo@bar:~$ cmake ..
```
6. Build and link the project
```console
foo@bar:~$ make
```

7. The simulation can now be run using the executable in the build directory
```console
foo@bar:~$ ./Simulator.exe
```
8. Navigate back to the root directory and install the python UI using pip
```console
foo@bar:~$ cd ..
foo@bar:~$ pip install -e .
```
10. The python UI can be launched using following command:
```console
foo@bar:~$ Simulator 
```
