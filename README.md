PSO mobility model for ns-3
=========================

## Features

This module comprises the following features:

* Adds `PSOMobilityModel` as an ns-3 application, allowing an integration of multi-agent, spatial convergence in three-dimensions upon a pre-set position
* Compatibility with other ns-3 models, such as the DeviceEnergyModel, provides robust capabilities for dynamic simulations.

Future releases will aim to:
* Add functional trace mobility filing.
* Include inertia weight; instilling a steady decrement from 0.9 to 0.4 over the lifetime of the PSO algorithm.
* Allow for a moving solution, which will be converged upon continually.

## Install

### Prerequisites ###

To run simulations using this module, you will need to install ns-3, and clone
this repository inside the `contrib` directory. 
Required dependencies include git and a build environment.

#### Installing dependencies ####

Please refer to [the ns-3 wiki](https://www.nsnam.org/wiki/Installation) for instructions on how to set up your system to install ns-3.

Please check the [releases](https://github.com/westbenj2020/ns-3-pso-app/releases) for further information about dependencies.

#### Downloading #####

First, clone the main ns-3 repository:

```bash
git clone https://gitlab.com/nsnam/ns-3-dev ns-3-dev
```

Then, clone the vr-app module:
```bash
git clone https://github.com/westbenj2020/ns-3-pso-app ns-3-dev/contrib/pso-mobility
```

### Compilation ###

Configure and build ns-3 from the `ns-3-dev` folder:

```bash
./waf configure --enable-tests --enable-examples
./waf build
```

This module does not provide Python bindings at the moment.

### Documentation ###

To compile the documentation, please follow the instructions from the [ns-3 manual](https://www.nsnam.org/docs/manual/html/documentation.html).

Basic steps:

1. Install the documentation-specific dependencies as described in the [ns-3 installation guide](https://www.nsnam.org/wiki/Installation)
1. You might need to fix the ImageMagick permissions for ghostscript files
