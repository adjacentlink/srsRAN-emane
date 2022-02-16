srsRAN-emane

srsRAN-emane is a derivative project of
[srsRAN](https://github.com/srsRAN). In conjunction with the [EMANE LTE Model](https://github.com/adjacentlink/emane-model-lte.git),
srsRAN-emane enables emulating an LTE network in
[EMANE](https://github.com/adjacentlink/emane.git) on a laptop,
desktop or other dedicated computer resources. Software Defined Radio
(SDR) hardware is not required.

The srsRAN LTE applications, `srsenb`, `srsue`, `srsepc` and
`srsmbms`, are adapted to send RF traffic through the EMANE emulation
environment. The EMANE version of the applications are renamed
`srsenb-emane`, `srsue-emane`, `srsepc-emane` and
`srsepc-emane`. `srsenb-emane` and `srsue-emane` contain an embedded
EMANE instance to replace lower levels of the LTE radio stack required
to emulate the over the air communication effects. The new
applications are also instrumented for data extraction using
[OpenStatistic](https://github.com/adjacentlink/openstatistic). Internal
statistics can be queried manually via the `ostatistic` application,
or collected automatically using
[OpenTestPoint](https://github.com/adjacentlink/opentestpoint) and the
[OpenTestPoint LTE Probe](https://github.com/adjacentlink/opentestpoint-probe-lte).

srsRAN-emane is released under the AGPLv3 license. The current stable
version is 21.10, the first release based on srsRAN 21.10.

---
## Build Instructions
The srsRAN suite includes:
  * srsUE - a full-stack SDR 4G/5G-NSA UE application (5G-SA coming soon)
  * srsENB - a full-stack SDR 4G/5G-NSA eNodeB application (5G-SA coming soon)
  * srsEPC - a light-weight 4G core network implementation with MME, HSS and S/P-GW

1. Install the latest [pre-built EMANE bundle](https://github.com/adjacentlink/emane/wiki/Install). EMANE version 1.2.3 or later is **required**.

2. Build and install the [EMANE LTE Model](https://github.com/adjacentlink/emane-model-lte.git).

3. Build and install srsRAN-emane:
   * [Centos 7](#centos-7)
   * [Fedora 35](#fedora-35)
   * [Ubuntu 20.04](#ubuntu-2004)

### Centos 7

Centos 7 requires an additional step of installing and using
devtoolset-9 for c++17 support.

```
sudo yum install cmake fftw3-devel polarssl-devel lksctp-tools-devel libconfig-devel boost-devel redhat-lsb-core

sudo yum install centos-release-scl
sudo yum install devtoolset-9

git clone https://github.com/adjacentlink/srsRAN-emane.git
cd srsRAN-emane
mkdir build
cd build

# enable devtoolset-9 for build
scl enable devtoolset-9 "cmake .. && make"
make package
sudo yum install srsran-emane-*-x86_64.rpm
```

### Fedora 35

```
sudo dnf install cmake fftw3-devel polarssl-devel lksctp-tools-devel libconfig-devel boost-devel redhat-lsb-core
git clone https://github.com/adjacentlink/srsRAN-emane.git
cd srsRAN-emane
mkdir build
cd build
cmake ..
make package
sudo dnf install srsran-emane-*-x86_64.rpm
```

### Ubuntu 20.04

```
sudo apt-get install cmake libfftw3-dev libmbedtls-dev libboost-program-options-dev libconfig++-dev libsctp-dev lsb-release
git clone https://github.com/adjacentlink/srsRAN-emane.git
cd srsRAN-emane
mkdir build
cd build
cmake ..
make package
sudo dpkg -i srsran-emane-*-x86_64.deb; sudo apt-get install -f
```

---
## Demonstration

[EMANE LTE Model](https://github.com/adjacentlink/emane-model-lte.git) contains a demonstration for running
a small network with two UEs and one ENB.

---
## Configuration

Each of the emane version srsRAN applications take an input
configuration file, identical to the one used by the regular srsRAN
applications, with the following additional parameters.


```
[runtime]
daemonize = 1     # 0 - foreground, 1 - run as daemon

[mhal]
#statistic_service_endpoint=0.0.0.0:47100
#emane_configfile=emanelte.xml
```

The new `runtime` configuration section contains a `daemonize` parameter
that controls whether the application runs in the foreground or as a daemon.

The new `mhal` section contains a `statistic_service_endpoint`
parameter to set the OpenStatistic query address and port (all
applications), and the `emane_configfile` parameter takes the
name of the the configuration file fed to the embedded EMANE instance
(`srsenb-emane` and `srsue-emane` only). Default values shown.
