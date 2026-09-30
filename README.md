<p align="center">
    <img
      src="https://raw.githubusercontent.com/pyomeca/biorbd_design/main/logo_png/biorbd_full.png"
      alt="logo"
    />
</p>

# Table of contents
- [Table of contents](#table-of-contents)
- [How to install](#how-to-install)
  - [Anaconda (For Windows, Linux and Mac)](#anaconda-for-windows-linux-and-mac)
  - [PyPI (For Windows, Linux and Mac)](#pypi-for-windows-linux-and-mac)
  - [Download binaries (For MATLAB users on Windows, Linux and Mac)](#download-binaries-for-matlab-users-on-windows-linux-and-mac)
  - [Compiling (For Windows, Linux and Mac)](#compiling-for-windows-linux-and-mac)
    - [Dependencies](#dependencies)
    - [CMake](#cmake)
- [How to use](#how-to-use)
  - [The C++ API](#the-c-api)
    - [Create an empty yet valid model](#create-an-empty-yet-valid-model)
    - [Read and write a bioMod file](#read-and-write-a-biomod-file)
    - [Perform some analyses](#perform-some-analyses)
  - [MATLAB](#matlab)
    - [Perform some analyses](#perform-some-analyses-1)
    - [Help](#help)
  - [Python 3](#python-3)
    - [Perform some analyses](#perform-some-analyses-2)
- [Model files](#model-files)
  - [*bioMod* files](#biomod-files)
    - [Header](#header)
      - [version](#version)
      - [mesh\_folder\_prefix](#mesh_folder_prefix)
      - [gravity](#gravity)
      - [variables / endvariables](#variables--endvariables)
    - [Definition of the model](#definition-of-the-model)
      - [Segment](#segment)
      - [Marker](#marker)
      - [Imu](#imu)
      - [Contact](#contact)
      - [Loopconstraint](#loopconstraint)
      - [Actuators](#actuators)
  - [Convert from OpenSim models](#convert-from-opensim-models)
- [How to contribute](#how-to-contribute)
- [Graphical User Interface (GUI)](#graphical-user-interface-gui)
- [Documentation](#documentation)
- [Troubleshoots](#troubleshoots)
  - [Slow BIORBD](#slow-biorbd)
- [Cite](#cite)





BIORBD is a library to analyze biomechanical data. It provides several useful functions for the direct and inverse flow including rigid body (based on *Featherstone* equations implemented in RBDL) and muscle elements.

Biomechanical data are often analyzed using similar flow, that is inverse or direct. BIORBD implements these common analyses providing high-level and easy to use Python and MATLAB interfaces of an efficient C++ implementation. 

So, without further ado, let's begin our investigation of BIORBD!

You can get the online version of the paper for BIORBD here: [![DOI](https://joss.theoj.org/papers/10.21105/joss.02562/status.svg)](https://doi.org/10.21105/joss.02562)

Furthermore, anyone can play with BIORBD with a working (but slightly limited in terms of graphics) MyBinder by clicking the following badge

[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/pyomeca/biorbd-tutorial/HEAD?urlpath=lab)

# How to install
There are several ways to install BIORBD on your computer: installing the binaries from Anaconda or PyPI (easiest, but limited to C++ and Python3), downloading the MATLAB binaries, or compiling the source code yourself (more versatile and up to date; for C++, Python3 and MATLAB).

## Anaconda (For Windows, Linux and Mac)
The easiest way to install BIORBD is to download the binaries from Anaconda (https://anaconda.org/) repositories (binaries are not available though for MATLAB). The project is hosted on the conda-forge channel (https://anaconda.org/conda-forge/biorbd).

After having installed properly an anaconda client [my suggestion would be Miniconda (https://conda.io/miniconda.html)] and loaded the desired environment to install BIORBD in, just type the following command:
```bash
conda install -c conda-forge biorbd
```
The library and the headers of the core of BIORBD will be installed in the `lib` and `include` folders of the environment respectively. Moreover, the Python3 binder will also be installed in the environment.

Please note that because of the way `Ipopt` is compiled on conda-forge, it was not possible to link it with `biorbd`. Therefore, the `MODULE_STATIC_OPTIM` was set to `OFF` for the conda-forge binaries.

## PyPI (For Windows, Linux and Mac)
The Python3 binder with the `Eigen3` backend is also published on PyPI when a new release is made (see `.github/workflows/publish_eigen_to_pypi.yml`):
```bash
pip install biorbd
```
Python 3.10 or newer is required (`requires-python` in `pyproject.toml`). The high-level Python wrapper (`biorbd.Biorbd`, see [Python 3](#python-3)) is only available with Python 3.12 or newer. Please note that we did not check which platforms and Python versions are covered by the wheels of each release.

The current building status for Anaconda release is as follow.

| License | Name | Downloads | Version | Platforms |
| --- | --- | --- | --- | --- |
|   <a href="https://opensource.org/licenses/MIT"><img src="https://img.shields.io/badge/license-MIT-success" alt="License"/></a> | [![Conda Recipe](https://img.shields.io/badge/recipe-biorbd-green.svg)](https://anaconda.org/conda-forge/biorbd) | [![Conda Downloads](https://img.shields.io/conda/dn/conda-forge/biorbd.svg)](https://anaconda.org/conda-forge/biorbd) | [![Conda Version](https://img.shields.io/conda/vn/conda-forge/biorbd.svg)](https://anaconda.org/conda-forge/biorbd) | [![Conda Platforms](https://img.shields.io/conda/pn/conda-forge/biorbd.svg)](https://anaconda.org/conda-forge/biorbd) |

## Download binaries (For MATLAB users on Windows, Linux and Mac)

The MATLAB users can download the binaries directly from the Release page of `biorbd` at this URL: [https://github.com/pyomeca/biorbd/releases/latest](https://github.com/pyomeca/biorbd/releases/latest). 

Once the folder is download, you simply unzip it, add it to the MATLAB's path and enjoy `biorbd`!

## Compiling (For Windows, Linux and Mac)
The main drawback with downloading the pre-compiled version from Anaconda is that this version may be out-of-date (even if we do our best to keep the release versions up-to-date). Moreover, since it is already compiled, it doesn't allow you to modify BIORBD if you need to. Therefore, a more versatile way to enjoy BIORBD is to compile it by yourself.

The building status for the current BIORBD branches is as follow

| Name | Status |
| --- | --- |
| Eigen3 tests | [![Eigen tests](https://github.com/pyomeca/biorbd/actions/workflows/run_eigen_tests.yml/badge.svg)](https://github.com/pyomeca/biorbd/actions/workflows/run_eigen_tests.yml) |
| CasADi tests | [![CasADi tests](https://github.com/pyomeca/biorbd/actions/workflows/run_casadi_tests.yml/badge.svg)](https://github.com/pyomeca/biorbd/actions/workflows/run_casadi_tests.yml) |
| Code coverage | [![codecov](https://codecov.io/gh/pyomeca/biorbd/branch/master/graph/badge.svg)](https://codecov.io/gh/pyomeca/biorbd) |
| DOI | [![DOI](https://zenodo.org/badge/124423173.svg)](https://zenodo.org/badge/latestdoi/124423173) |

### Dependencies
BIORBD relies on several libraries (namely Eigen (https://eigen.tuxfamily.org) with RBDL (https://github.com/rbdl/rbdl), or CasADi (https://web.casadi.org/) with rbdl-casadi (https://github.com/pyomeca/rbdl-casadi), tinyxml2 (https://github.com/leethomason/tinyxml2) and Ipopt (https://github.com/coin-or/Ipopt)) that one must install prior to compiling. Fortunately, all these dependencies are also hosted on the *conda-forge* channel of Anaconda or will automatically be compiled when building BIORBD. Therefore the following command will install everything you need to compile BIORBD:
```bash
conda install -c conda-forge rbdl tinyxml2 [ipopt] [pkgconfig] [cmake] [scipy]
```
Please note:
- ```tinyxml2``` is required if `MODULE_VTP_FILES_READER` is set to `ON` (the default); it is found on your system or, if not found, downloaded and compiled by CMake;
- the files `environment_eigen.yml` and `environment_casadi.yml` at the root of the repository list a full development environment (Python 3.10 or newer, `rbdl`, `scipy`, `ezc3d`, `ipopt`, ...);
- ```ipopt``` is optional, but is required for the *Static optimization* module;
- ```pkgconfig``` and ```cmake``` are very useful tools that can prevents lot of headaches when compiling; 


Additionally, the Python3 interface requires *numpy* (https://numpy.org/) and *SWIG* (http://www.swig.org/) (SWIG 3 or 4; with the `Casadi` backend, SWIG must be older than 4.4.0). Again, one can easily install these dependencies from Anaconda using the following command:
```bash
conda install -c conda-forge numpy swig
```

Finally, the MATLAB interface (indeed) requires MATLAB to be installed.

If you are interested in developing BIORBD, the ```googletest``` suite is required to test your modifications. It is included as a git submodule (`external/googletest`), so clone with `--recurse-submodules` (or run `git submodule update --init`) before setting `BUILD_TESTS` to `ON`.

### CMake
BIORBD comes with a CMake (https://cmake.org/) project. If you don't know how to use CMake, you will find many examples on Internet. The main variables to set are:

> `CMAKE_INSTALL_PREFIX` Which is the `path/to/install` BIORBD in. If you compile the Python3 binder, a valid installation of Python with Numpy should be installed relatived to this path.
>
> `BUILD_SHARED_LIBS` If you wan to build BIORBD in a shared `TRUE` or static `FALSE` library manner. Default is `TRUE`. Please note that due to the dependencies, on Windows BIORBD must be statically built.
>
> `CMAKE_BUILD_TYPE` Which type of build you want. Options are `Debug`, `RelWithDebInfo`, `MinSizeRel` or `Release`. This is relevant only for the build done using the `make` command. Please note that you will experience a slow BIORBD library if you compile it without any optimization (i.e. `Debug`), especially for all functions that requires linear algebra. 
>
> `MATH_LIBRARY_BACKEND` Choose between the two linear algebra backends, either `Eigen3` or `Casadi`. Default is `Eigen3`.
>
> `BUILD_EXAMPLE` If you want (`TRUE`) or not (`FALSE`) to build the C++ example. Default is `TRUE`.
>
> `BUILD_TESTS` If you want (`ON`) or not (`OFF`) to build the tests of the project. Please note that this requires gtest (https://github.com/google/googletest), which is the `external/googletest` git submodule. Default is `OFF`.
>
> `BUILD_DOC` If you want (`ON`) or not (`OFF`) to build the documentation of the project. Default is `OFF`.
>
> `BINDER_C` If you want (`ON`) or not (`OFF`) to build the low level C binder. Default is `OFF`. Please note that this binder is very light and will not contain most of BIORBD features, and that it can't be set to `ON` alongside the `Casadi` backend.
>
> `BINDER_PYTHON3` If you want (`ON`) or not (`OFF`) to build the Python binder. Default is `OFF`.
>
> `SWIG_EXECUTABLE`  If `BINDER_PYTHON3` is set to `ON` then this variable should point to the SWIG executable. This variable should be found automatically.
>
> `BINDER_MATLAB` If you want (`ON`) or not (`OFF`) to build the MATLAB binder. Default is `OFF`. Please note that `BINDER_MATLAB` can't be set to `ON` alongside the `Casadi` backend.
>
> `Matlab_ROOT_DIR` If `BINDER_MATLAB` is set to `ON` then this variable should point to the root path of MATLAB directory. Please note that the MATLAB binder is based on MATLAB R2018a API and won't compile on earlier versions. This variable should be found automatically, except on Mac where the value should manually be set to the MATLAB in the App folder. 
>
> `Matlab_biorbd_eigen_INSTALL_DIR` (more generally `Matlab_<BIORBD_NAME>_INSTALL_DIR`, where `BIORBD_NAME` is `biorbd_eigen` or `biorbd_casadi`) If `BINDER_MATLAB` is set to `ON` then this variable should point to the path where you want to install BIORBD. Typically, this is `{MY DOCUMENTS}/MATLAB`. The default value is the toolbox folder of MATLAB. Please note that if you leave the default value, you will probably need to grant administrator rights to the installer. In all cases, after the installation, you will have to add the path to the MATLAB search path by typing the following command in the MATLAB's prompt (or to add it to the `startup.m`) `addpath(genpath($Matlab_biorbd_eigen_INSTALL_DIR))`, and replacing `Matlab_biorbd_eigen_INSTALL_DIR` by your own path.
>
> `MODULE_ACTUATORS` If you want (`ON`) or not (`OFF`) to build with the actuators module. Default is `ON`. This allows to use exotic joint torques. 
>
> `MODULE_KALMAN` If you want (`ON`) or not (`OFF`) to build the Kalman filter module. Default is `ON` (forced to `OFF` with the `Casadi` backend). The main reason to skip Kalman is that in `Debug` mode `Eigen3` will perform this very slowly and `CasADi` will always perform this slowly. 
>
> `MODULE_MUSCLES` If you want (`ON`) or not (`OFF`) to build with the muscle module. Default is `ON`. This allows to read and interact with models that include muscles.
>
> `MODULE_PASSIVE_TORQUES` If you want (`ON`) or not (`OFF`) to build with the passive torques module. Default is `ON`.
>
> `MODULE_LIGAMENTS` If you want (`ON`) or not (`OFF`) to build with the ligaments module. Default is `ON`.
>
> `MODULE_STATIC_OPTIM` If you want (`ON`) or not (`OFF`) to build the Static optimization module. Default is `ON`, but it is forced to `OFF` if `ipopt` is not found or with the `Casadi` backend.
>
> `MODULE_VTP_FILES_READER` If you want (`ON`) or not (`OFF`) to build with the vtp files reader module. Default is `ON` (`tinyxml2` is then found or downloaded and compiled). This allows to read mesh files produced by `OpenSim`.
> 
> `SKIP_ASSERT` If you want (`ON`) or not (`OFF`) to skip the asserts in the functions (e.g. checks for sizes). Default is `OFF` (except when `CMAKE_BUILD_TYPE` is `Debug`, where it is `ON`). Putting this to `OFF` reduces the risks of Segmentation Faults, it will however slow down the code when using `Eigen3` backend.
>
> `INSTALL_DEPENDENCIES_PREFIX` The path where the dependencies are installed, if they are not in the default search paths (e.g. the prefix of your conda environment).
>
> `SKIP_LONG_TESTS` If you want (`ON`) or not (`OFF`) to skip the tests that are long to perform. Default is `OFF`. This is useful when debugging. 


# How to use
BIORBD provides as much as possible explicit names for the filter so one can intuitively find what he wants from the library. Still, this is a C++ library and it can be sometimes hard to find what you need. Due to the varity of functions implemented in the library, minimal examples are shown here. One is encourage to have a look at the `examples` and `test` folders to get a better overview of the possibility of the API. For an in-depth detail of the API, the Doxygen documentation (see [Documentation](#documentation)) is the way to go.

## The C++ API
The core code is written in C++, meaning that you can fully use BIORBD from C++.  Moreover, the linear algebra is using the Eigen library which makes it fairly easy to perform further computation and analyses.
The informations that follows is a basic guide that should allow you to perform everything you want to do.

### Create an empty yet valid model
To create a new valid yet empty model, just call the `Model` class without parameter. All BIORBD classes live in the `BIORBD_NAMESPACE` namespace (its name depends on the chosen backend, e.g. `Biorbdeigen3`), which is why the examples use `using namespace BIORBD_NAMESPACE;`.
```C++
#include "biorbd.h"
using namespace BIORBD_NAMESPACE;
int main()
{
    Model myModel;
}
```
This model can thereafter be populated using the *biorbd* add methods. Even if this is not the prefered way of loading a model, one can have a look at the *src/ModelReader.cpp* in order to know what functions that must be called to populate the model manually. 

### Read and write a bioMod file
The prefered method to load a model is to read the in-house *.bioMod* format file.  To do so, one must simply call the `Model` constructor with a valid path to the model. Afterward, one can modify manually the model and write it back to a new file. 
```C++
#include "biorbd.h"
using namespace BIORBD_NAMESPACE;
int main()
{
    Model myModel("path/to/mymodel.bioMod");
    // Do some changes...
    Writer::writeModel(myModel, "path/to/newFile.bioMod");
    return 0;
}
```
Please note that on Windows, the path must be `/` or `\\` separated (and not only`\`), for obvious reasons. 

### Perform some analyses
BIORBD is made to work with the RBDL functions (the code and doc can be found in https://github.com/rbdl/rbdl). Therefore, every functions available in RBDL is also available on BIORBD. Additionnal are of course also made available, for example the whole muscle module. 

The most obvious and probably the most used function is the forward kinematics, where one knows the configuration of the body and is interested in the resulting position of skin markers. The following code performs that task.
```C++
#include "biorbd.h"
using namespace BIORBD_NAMESPACE;
int main()
{
    // Load the model
    Model model("path/to/model.bioMod");

    // Prepare the model
    rigidbody::GeneralizedCoordinates Q(model);
    Q.setOnes();
    Q = Q / 10; // Set the model position

    // Perform forward kinematics
    std::vector<rigidbody::NodeSegment> markers(model.markers(Q));
    
    // Print the results
    for (auto marker : markers)
        std::cout << marker.name() << " is at the coordinates: " << marker.transpose() << std::endl;
    return 0;
}
```

Another common analysis to perform is to compute the effect of the muscles on the acceleration of the model. Assuming that the model that is loaded has muscles, the following code perform this task.
```C++
#include "biorbd.h"
using namespace BIORBD_NAMESPACE;
int main()
{
    // Load the model
    Model model("path/to/model.bioMod");

    // Prepare the model
    rigidbody::GeneralizedCoordinates Q(model); // position
    rigidbody::GeneralizedVelocity Qdot(model); // velocity
    Q.setOnes();
    Q = Q / 10; // Set the model position
    Qdot.setOnes();
    Qdot = Qdot / 10; // Set the model velocity
    // Muscles activations
    std::vector<std::shared_ptr<internal_forces::muscles::State>> states = model.stateSet();
    for (auto& state : states){
        state->setActivation(0.5); // Set the muscle activation
    }

    // Compute the joint torques based on muscle
    rigidbody::GeneralizedTorque muscleTorque(model.muscularJointTorque(states, Q, Qdot));

    // Compute the acceleration of the model due to these torques
    rigidbody::GeneralizedAcceleration Qddot(model.ForwardDynamics(Q, Qdot, muscleTorque));

    // Print the results
    std::cout << " The joints accelerations are: " << Qddot.transpose() << std::endl;
    return 0;
}
```

These snippets follow `examples/forwardKinematicsExample.cpp` and `examples/forwardDynamicsFromMusclesExample.cpp`, which are compiled with the library and are the maintained reference. There are many other analyses and filters that are available. Please refer to the BIORBD and RBDL Docs to see what is available.  

## MATLAB
MATLAB (https://www.mathworks.com/) is a prototyping langage largely used in industry and fairly used by the biomechanical scientific community. Despite the existence of Octave as an open-source and very similar language or the growing popularity of Python as a free and open-source alternative, MATLAB remains an important player as a programming languages. Therefore BIORBD comes with a binder for MATLAB (that can theoretically used with Octave as well with some minor changes to the CMakeLists.txt file).

Most of the functions available in C++ are also available in MATLAB. Still, they were manually binded, therefore it may happen that some important one (for you) are not there. If so, do not hesitate to open an issue on GitHub to required the add of that particular function. The philosophy behind the MATLAB binder is that you open a particular model and a reference to that model is gave back to you. Thereafter, the functions can be called, assuming the pass back that model reference. That implies, however, that ones must himself deallocate the memory of the model when it is no more needed. Failing to do so results in an certain memory leak.

### Perform some analyses
Please find here the same tasks previously described for the C++ interface done in the MATLAB interface. Notice that the MATLAB interface takes advantage of the matrix nature of MATLAB and therefore can usually perform the analyses on multiple frames at once. 

Forward kinematics can be performed as follow
```MATLAB
nFrames = 10; % Set the number of frames to simulate

% Load the model
model = biorbd('new', 'path/to/model.bioMod');

% Prepare the model
Q = ones(biorbd('nQ', model), nFrames)/10; % Set the model position

% Perform the forward kinematics
markers = biorbd('markers', model, Q);

% Print the results
disp(markers);

% Deallocate the model
biorbd('delete', model);
```

The joint accelerations from muscle activations can be performed as follow
```MATLAB
nFrames = 10; % Set the number of frames to simulate

% Load the model
model = biorbd('new', 'path/to/model.bioMod');

% Prepare the model
Q = ones(biorbd('nQ', model), nFrames)/10; % Set the model position
Qdot = ones(biorbd('nQdot', model), nFrames)/10; % Set the model velocity
activations = ones(biorbd('nMuscles', model), nFrames)/2; % Set muscles activations

% Compute the joint torques based on muscle
jointTorque = biorbd('jointTorqueFromActivation', model, activations, Q, Qdot);

% Compute the acceleration of the model due to these torques
Qddot = biorbd('forwardDynamics', model, Q, Qdot, jointTorque);

% Print the results
disp(Qddot);

% Deallocate the model
biorbd('delete', model);
```

### Help
One can print all the available functions by type the `help` command
```MATLAB
biorbd('help')
```
Please note that it was reported that on Windows, the command returns nothing. One must therefore look in the source code (`biorbd/binding/matlab/Matlab_help.h`) what should the command have returned.


## Python 3
Python (https://www.python.org/) is a scripting language that has taken more and more importance over the past years. So much that now it is one of the preferred language of the scientific community. Its simplicity yet its large power to perform a large variety of tasks makes it a certainty that its popularity won't decrease for the next years.

To interface the C++ code with Python, SWIG is a great tool. It creates very rapidly an interface in the target language with minimal code to write. However, the resulting code in the target language can be far from being easy to use. In effect, it gives a mixed-API not far from the original C++ language, which may not comply to best practices of the target language. When this is useful to rapidly create an interface, it sometime lacks of user-friendliness and expose the user to the possibility of the C++ such as segmentation fault (unlike the MATLAB API which won't suffer from this devil problem). 

BIORBD interfaces the C++ code using SWIG. While it has some inherent limit as discussed previously, it has the great advantage of providing almost for free the complete API. Because of that, much more of the C++ API is interfaced in Python than the MATLAB one. Again, if for some reason, part of the code which is not accessible yet is important for you, don't hesitate to open an issue asking for that particular feature!

On top of this low-level SWIG API (`biorbd.Model`, camelCase methods), a high-level and more pythonic wrapper is available for Python 3.12 or newer: the `biorbd.Biorbd` class (`binding/python3/wrapper/`), with snake_case names (`nb_q`, `markers`, `muscles.joint_torque`, `forward_dynamics`, ...). The examples below use this wrapper; the `examples/python3` folder is the maintained reference. When built with the `Casadi` backend, the Python package is called `biorbd_casadi`.

### Perform some analyses
Please find here the same tasks previously described for the C++ interface done in the Python3 interface. Please note that the interface usually takes advantage of the numpy arrays in order to interact with the user while a vector is needed. 

Forward kinematics can be performed as follow
```Python
import numpy as np
import biorbd

# Load the model
model = biorbd.Biorbd('path/to/model.bioMod')

# Prepare the model
q = np.ones(model.nb_q) / 10  # Set the model position

# Perform the forward kinematics
markers = model.markers(q)

# Print the results
for marker in markers:
    print(marker.world)
```

The joint accelerations from muscle activations can be performed as follow
```Python
import numpy as np
import biorbd

# Load the model
model = biorbd.Biorbd('path/to/model.bioMod')

# Prepare the model
q = np.ones(model.nb_q) / 10  # Set the model position
qdot = np.ones(model.nb_qdot) / 10  # Set the model velocity
activations = [0.5] * len(model.muscles)  # Set muscles activations

# Compute the joint torques based on muscle
joint_torque = model.muscles.joint_torque(activations=activations, q=q, qdot=qdot)

# Compute the acceleration of the model due to these torques
qddot = model.forward_dynamics(q, qdot, joint_torque)

# Print the results
print(qddot)
```
# Model files
## *bioMod* files
The preferred method to load a model is by using a *.bioMod* file. This type of file is an in-house language that describes the segments of the model, their interactions and additionnal elements attached to them. The following section describe the structure of the file and all the tags that exists so far. 

Comments can be added to the file in a C-style way, meaning that everything a on line following a `//` will be considered as a comment and everything between `/*` and `*/` will also be ignored. 

Please note that the *bioMod* is not case dependent, so `Version`and `version` are for instance fully equivalent. The *bioMod* reader also ignore the tabulation, which is therefore only aesthetic. 

When a tag waits for multiple values, they must be separate by a space, a tabulation or a return of line. Also, anytime a tag waits for a value, it is possible to use simple equations (assuming no spaces are used) and/or variables. For example, the following snippet is a valid way to set the gravity parameter to $(0, 0, -9.81)$. 
```c
variables
    $my_useless_variable 0
endvariables
gravity 2*(1-1) -2*$my_useless_variable
        -9.81
```

### Header
#### version
The very first tag that **must** appear at the first line in file is the version of the file. The current version of the *.bioMod* files is $4$. Please note that most of the version are backward compatible, unless specified. This tag waits for $1$ value.
```c
version 4
```
From that point, the order of the tags is not important, header can even be at the end of the file. For simplicity though we suggest to put everything related to the header at the top of the file. 

#### mesh_folder_prefix
The `mesh_folder_prefix` is to prefix all the [meshfile](#meshfile-or-ply) path. Warning, this is undefined if the paths are given as absolute paths. 

#### gravity
The `gravity` tag is used to reorient and/or change the magnitude of the gravity. The default value is $(0, 0, -9.81)$. This tag waits for $3$ values.
```c
// Restate the default value
gravity 0 0 -9.81
```

#### variables / endvariables
The `variables / endvariables` tag pair allows to declare variables that can be used within the file. This allows for example to template the *bioMod* file by only changing the values in the variables. Please note that contrary to the rest of the file, the actual variables are case dependent. 

The `\$` sign is mandatory and later in the file, everything with that sign followed by the same name will be converted to the values specified in the tag. 
```c
// Restate the default value
variables
    $my_first_variable_is_an_int 10
    $my_second_variable_is_a_double 10.1
    $myThirdVariableIsCamelCase 1
    $myLastVariableIsPi pi
endvariables
```
As you may have noticed, the constant PI is defined as $3.141592653589793$.

### Definition of the model
A BIORBD model consists of a chain of segment, linked by joints with up to six DoF (3 translations, 3 rotations). It is imperative when attaching something to a segment of the model that particular segment must have been previously defined. For instance, if the `thorax` is attached to the `pelvis`, then the latter must be defined before the former in the file. 

#### Segment
The `segment xxx / endsegment`tag pair is the core of a *bioMod* file. It describes a segment of the model with the name `xxx`, that is most of the time a bone of the skeleton. For internal reasons, the name cannot be `root` (case insensitive), which is reserved for the environment. The `xxx` must be present and consists of $1$ string. The segment is composed of multiple subtags, described here. 

```c
segment default_segment
    parent ROOT
    rtinmatrix 0
    rt 0 0 0 xyz 0 0 0
    translations xyz
    rotations xyz
    mass 1
    com 0 0 0
    inertia 
        1 0 0
        0 1 0
        0 0 1
endsegment

segment second_segment
    parent default_segment
endsegment
```

##### parent <!-- omit from toc -->
The `parent` tag is the name of the segment that particular segment is attached to. If no segment parent is provided, or if it is `ROOT` (case insensitive), it is considered to be attached to the environment. The parent must be defined earlier in the file and is case dependent. This tag is waits for $1$ string.

##### rt <!-- omit from toc -->
The `homogeneous matrix` of transformation (rototranslation) of the current segment relative to its parent. If `rtinmatrix` is defined to `1`, `rt` waits for a 4 by 4 matrix (the 3 x 3 matrix of rotation and the 4th column being the translation and the last row being 0 0 0 1); if it is defined to `0` it waits for the 3 rotations, the rotation sequence and the translation. The default value is the identity matrix.

##### rtinmatrix <!-- omit from toc -->
The tag that defines if the `rt` is in matrix or not. If the `version` of the file is higher or equal than $3$, the default value is false ($0$), otherwise, it true ($1$). 

##### translations <!-- omit from toc -->
The `translations` tag specifies the number of degrees-of-freedom in translation and their order. The possible values are `x`, `y` and/or `z` combined whatever fits the model. Please note that the vector of generalized coordinate will comply the the order wrote in this tag. If no translations are provided, then the segment has no translation relative to its parent. This tag waits for $1$ string.

##### rotations <!-- omit from toc -->
The `rotations` tag specifies the number of degrees-of-freedom in rotation and their order. The possible values are `x`, `y` and/or `z` combined whatever fits the model. Please note that the vector of generalized coordinate will comply the the order wrote in this tag. If no rotations are provided, then the segment has no rotation relative to its parent. This tag waits for $1$ string.

##### mass <!-- omit from toc -->
The `mass` tag specifies the mass of the segment in kilogram. This tag waits for $1$ value. The default value is $0.00000001$. Please note that a pure $0$ can create a singularity. 

##### com <!-- omit from toc -->
The $3$ values position of the `center of mass` relative to the local reference of the segment. The default values are `0 0 0`. 

##### inertia <!-- omit from toc -->
The `inertia` tag allows to specify the matrix of inertia of the segment ( computed with respect to the center of mass of the segment ). It waits for $9$ values. The default values are the `identity matrix`

##### forceplate or externalforceindex <!-- omit from toc -->
This tag is deprecated and should therefore not be used.

##### meshfile or ply <!-- omit from toc -->
The path of the meshing `.bioBone`, `.ply`, `.stl` file respectively. It can be relative to the current running folder or absolute (relative being preferred) and UNIX or Windows formatted (`/` vs `\\`, UNIX being preferred).

If the folder is relative, one can pass the [`mesh_folder_prefix`](#mesh_folder_prefix) in order to prefix all the mesh path. 

##### mesh <!-- omit from toc -->
If the mesh is not written in a file, it can be written directly in the segment. If so, the `mesh` tag stands for the vertex. Therefore, there are as many `mesh` tags as vertex. It waits for $3$ values being the position relative to reference of the segment. 

##### meshcolor <!-- omit from toc -->
The color of the segment mesh given in RGB values `[0, 1]`. Default is `0.89, 0.855, 0.788`, that is bone color-ish.

##### meshscale <!-- omit from toc -->
The scaling to apply to the provided mesh, given in `X Y Z` values. Default is `1 1 1`.

##### meshrt <!-- omit from toc -->
The RT to apply to the provided mesh, given in `RX RY RZ seq TX TY TZ` as for RT. The default value is `0 0 0 xyz 0 0 0`. 


##### patch <!-- omit from toc -->
The patches to define the orientation of the patches of the mesh. It waits for $3$ values being the $0-based$ of the index of the vertex defined by the `mesh`.

#### Marker
The marker with a unique name attached to a body segment. 

```c
marker my_marker
    parent segment_name
    position 0 0 0
    technical 1
    anatomical 0
endmarker
```

##### parent <!-- omit from toc -->
The `parent` tag is the name of the segment that particular segment is attached to. The parent must be defined earlier in the file and is case dependent. This tag is waits for $1$ string.

##### position <!-- omit from toc -->
The `position` of the marker in the local reference frame of the segment. 

##### technical <!-- omit from toc -->
If the marker will be taged as technical (will be returned when asking technical markers). Default value is true ($1$).

##### anatomical <!-- omit from toc -->
If the marker will be taged as anatomical (will be returned when asking anatomical markers). Default value is false ($0$).

##### axestoremove <!-- omit from toc -->
It is possible to project the marker onto some axes, if so, write the name of the axes to project onto here. Waits for the axes in a string.

#### Imu
Same as a marker, but for inertial measurement unit. 
```c
imu my_imu
    parent segment_parent
    rtinmatrix 0
    rt 0 0 0 xyz 0 0 0
    technical 1
    anatomical 0
endimu
```
##### parent <!-- omit from toc -->
The `parent` tag is the name of the segment that particular segment is attached to. The parent must be defined earlier in the file and is case dependent. This tag is waits for $1$ string.

##### rt <!-- omit from toc -->
The `homogeneous matrix` of transformation (rototranslation) of the current segment relative to its parent. If `rtinmatrix` is defined to `1`, `rt` waits for a 4 by 4 matrix (the 3 x 3 matrix of rotation and the 4th column being the translation and the last row being 0 0 0 1); if it is defined to `0` it waits for the 3 rotations, the rotation sequence and the translation. The default value is the identity matrix.

##### rtinmatrix <!-- omit from toc -->
The tag that defines if the `rt` is in matrix or not. If the `version` of the file is higher or equal than $3$, the default value is false ($0$), otherwise, it true ($1$). 

##### technical <!-- omit from toc -->
If the marker will be taged as technical (will be returned when asking technical markers). Default value is true ($1$).

##### anatomical <!-- omit from toc -->
If the marker will be taged as anatomical (will be returned when asking anatomical markers). Default value is false ($0$).

#### Contact <!-- omit from toc -->
The position of a non acceleration point while computing the forward dynamics. 
```c
contact my_contact
    parent parent_segment
    position 0 0 0
    axis xyz
endcontact
```
##### parent <!-- omit from toc -->
The `parent` tag is the name of the segment that particular segment is attached to. The parent must be defined earlier in the file and is case dependent. This tag is waits for $1$ string.

##### position <!-- omit from toc -->
The `position` of the marker in the local reference frame of the segment. 

##### axis <!-- omit from toc -->
The name of the `axis` that the contact acts on. If the version of the file is $1$, this tag has no effect. 

##### normal <!-- omit from toc -->
The `normal` that the contact acts on. This tags waits for $3$ values with a norm $1$. If the version of the file is not $1$, this tag has no effect. To get the `x`, `y` and `z` axes, one must therefore define three separate contacts. 

##### acceleration <!-- omit from toc -->
The constant `acceleration` of the contact point. The default values are `0, 0, 0`. 

#### Loopconstraint
A closed kinematic loop constraint between two segments (the reader calls `AddLoopConstraint`, see `src/ModelReader.cpp`). The name of the constraint is generated as `Loop_<predecessor>_<successor>`.
```c
loopconstraint
    predecessor segment_a
    successor segment_b
    rtpredecessor 0 0 0 xyz 0 0 0
    rtsuccessor 0 0 0 xyz 0 0 0
    axis 0 0 0 1 0 0
    stabilizationparameter 0.1
endloopconstraint
```

##### predecessor and successor <!-- omit from toc -->
The names of the two segments linked by the constraint. Both must have been defined earlier in the file. Each waits for $1$ string.

##### rtpredecessor and rtsuccessor <!-- omit from toc -->
The RT of the constraint frame relative to the predecessor and the successor respectively, given in `RX RY RZ seq TX TY TZ` as for the segment RT. The default value is the identity.

##### axis <!-- omit from toc -->
The $6$ values of the spatial vector that defines the constrained axis.

##### stabilizationparameter <!-- omit from toc -->
The parameter of the constraint stabilization. The stabilization is enabled only if this value is strictly positive; by default, it is disabled.

#### Actuators
The Actuators specifies the different parameters used to express the torque generated by a particular movement at a joint. 
These different parameters were described by M. Jackson (The mechanics of the table contact phase of gymnastics vaulting, 2019).

##### type <!-- omit from toc -->
Different types of actuator can be defined with the `type` tag : `Constant`, `Linear`, `Gauss3p`, `Gauss6p`, `Sigmoidgauss3p`.
In function of the type of actuator the parameters are different.

```c
actuator    default_segment
            type    Gauss3p
            dof     RotX
            direction   positive
            Tmax    220.3831
            T0      157.4165
            wmax    475.0000
            wc      190.0000
            amin    0.9900
            wr      40.0000
            w1     -90.0000
            r       56.4021
            qopt    25.6939
        endactuator

``` 

##### dof (Constant, Linear, Gauss3p, Gauss6p, Sigmoidgauss3p) <!-- omit from toc -->
The `dof` tag defines the degree of fredoom of the segment. It can be a rotation (`Rot`) or a translation (`Trans`) on one 
of the 3 axis (`x`, `y`, `z`). This argument is not required.

##### direction (Constant, Linear, Gauss3p, Gauss6p, Sigmoidgauss3p) <!-- omit from toc -->
The `direction` of the torque can be positive or negative. This argument is not required.

##### Tmax (Constant, Gauss3p, Gauss6p) <!-- omit from toc -->
`Tmax` is the maximum eccentric torque.

##### T0 (Linear, Gauss3p, Gauss6p) <!-- omit from toc -->
The tag `T0` defines the maximum concentric torque.

##### wmax (Gauss3p, Gauss6p) <!-- omit from toc -->
The values of `wmax` is the maximum angular velocity above which torque cannot be produced.

##### wc (Gauss3p, Gauss6p) <!-- omit from toc -->
The `wc` tag specifies the angular velocity of the vertical asymptote of the concentric hyperbola based of the relation between tetanic torque and contractile component angular velocity.

##### amin (Gauss3p, Gauss6p) <!-- omit from toc -->
The `amin` tag allows to specify the plateau low activation level (values between 0.5 and 0.99) based of the differential activation-velocity relationship.

##### wr (Gauss3p, Gauss6p) <!-- omit from toc -->
The tag `wr` is the angular velocity range over which the ramp occurs based of the differential activation-velocity relationship.

##### w1 (Gauss3p, Gauss6p) <!-- omit from toc -->
`w1` represents the angular velocity of the midpoint between the maximum and the low plateau activation level based of the differential activation-velocity relationship.

##### qopt (Gauss3p, Gauss6p, Sigmoidguass3p) <!-- omit from toc -->
The `qopt` tag allows to specify the optimum angle for torque production. This argument is required, the default value is `0`.

##### r (Gauss3p, Gauss6p, Sigmoidguass3p) <!-- omit from toc -->
The tag `r` represented the width of the curve based of the torque-angle relationship. This argument is required, the default value is `0`.

##### facteur (Gauss6p) <!-- omit from toc -->
The weight of the second Gaussian of the torque-angle relationship: `Ta = exp(-(qopt - q)^2 / (2 r^2)) + facteur * exp(-(qopt2 - q)^2 / (2 r2^2))` (see `src/InternalForces/Actuators/ActuatorGauss6p.cpp`).

##### r2 (Gauss6p) <!-- omit from toc -->
The width `r2` of the second Gaussian of the torque-angle relationship (see `facteur`).

##### qopt2 (Gauss6p) <!-- omit from toc -->
The optimum angle `qopt2` of the second Gaussian of the torque-angle relationship (see `facteur`).

##### theta (Sigmoidgauss3p) <!-- omit from toc -->
The amplitude of the sigmoid of the torque-velocity relationship: `Tmax = theta / (1 + exp(lambda * speed)) + offset` (see `src/InternalForces/Actuators/ActuatorSigmoidGauss3p.cpp`).

##### lambda (Sigmoidgauss3p) <!-- omit from toc -->
The slope of the sigmoid of the torque-velocity relationship (see `theta`).

##### offset (Sigmoidgauss3p) <!-- omit from toc -->
The vertical offset of the sigmoid of the torque-velocity relationship (see `theta`).

## Convert from OpenSim models
For users who have well-established models in OpenSim and wish to convert them to a `.bioMod`, there's a handy tool called BioBuddy. 
This package facilitates the conversion process, ensuring that OpenSim models can be seamlessly integrated into our ecosystem.

To get started with the BioBuddy package, please refer to its documentation and source code available at [BioBuddy](https://github.com/pyomeca/biobuddy) GitHub Repository.


# How to contribute

You are very welcome to contribute to the project! There are two main ways to contribute. 

The first way is to actually code new features for BIORBD. The easiest way to do so is to fork the project, make the modifications and then open a pull request to the main project. Please have a look at `doc/contributing.md` and `doc/pull_request_template.md` before opening a pull request.

The second way is to open issues to report bugs or to ask for new features. I am trying to be as reactive as possible, so don't hesitate to do so!

# Graphical User Interface (GUI)
For now, there is no GUI for the C++ interface and the MATLAB one is so poor I decided not to release it. However, there is a Python interface (`bioviz`) that is worth having a look at. Installation procedure and documentation can be found at the GitHub repository (https://github.com/pyomeca/bioviz).

# Documentation
The documentation is automatically generated using Doxygen (http://www.doxygen.org/). You can compile it yourself if you want (by setting `BUILD_DOC` to `ON`). Otherwise, you can access a copy of it that I try to keep up-to-date in the Documentation project of pyomeca (https://pyomeca.github.io/Documentation/) by selecting `biorbd`. 

# Troubleshoots
Despite my efforts to make a bug-free library, BIORBD may fails sometimes. If it does, please refer to the section below to know what to do. I will fill this section with the issue over time.

## Slow BIORBD
If you experience a slow BIORBD, you are probably using a non optimized version, that is compiled with `Debug` level. Please use at least `RelWithDebInfo` level of optimization while compiling BIORBD. 

If you actually are using a released level of optimization, you may actually experiencing a bug. You are therefore welcomed to provide me with a minimal example of your slow code and I'll see how to improve the speed!

# Cite
If you use BIORBD, we would be grateful if you could cite it as follows:

```

@article{michaudBiorbd2021,
  title = {Biorbd: {{A C}}++, {{Python}} and {{MATLAB}} Library to Analyze and Simulate the Human Body Biomechanics},
  shorttitle = {Biorbd},
  author = {Michaud, Benjamin and Begon, Mickaël},
  date = {2021-01-19},
  journaltitle = {Journal of Open Source Software},
  volume = {6},
  pages = {2562},
  issn = {2475-9066},
  doi = {10.21105/joss.02562},
  url = {https://joss.theoj.org/papers/10.21105/joss.02562},
  urldate = {2021-01-19},
  abstract = {Michaud et al., (2021). biorbd: A C++, Python and MATLAB library to analyze and simulate the human body biomechanics. Journal of Open Source Software, 6(57), 2562, https://doi.org/10.21105/joss.02562},
  langid = {english},
  number = {57}
}
```
