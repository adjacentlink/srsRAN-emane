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
version is 23.04.

The srsRAN suite includes:
  * srsue - a full-stack SDR 4G/5G UE application
  * srsenb - a full-stack SDR 4G/5G e(g)NodeB application
  * srsepc - a light-weight 4G core network implementation with MME, HSS and S/P-GW


---
## Install Instructions

The easiest way to get running with the EMANE LTE model is to install packages
directly from the latest [pre-built EMANE bundle](https://github.com/adjacentlink/emane/wiki/Install).


## Build Instructions

Building srsRAN-emane requires prior installation of [EMANE](https://github.com/adjacentlink/emane),
[OpenTestPoint](https://github.com/adjacentlink/opentestpoint),
[OpenStatistic](https://github.com/adjacentlink/openstatistic), and
 the [EMANE LTE Model](https://github.com/adjacentlink/emane-model-lte.git).

You may build and install those project directly or install the pre-build EMANE bundle
as linked above.

   * [Rocky Linux 8](#rocky-linux-8)
   * [Rocky Linux 9](#rocky-linux-8)
   * [Fedora 41](#fedora-35)
   * [Ubuntu 22.04](#ubuntu-2204)
   * [Ubuntu 24.04](#ubuntu-2204)


### Rocky Linux 8

```
sudo dnf install epel-release dnf-plugins-core
sudo dnf config-manager --set-enabled powertools
sudo dnf install git gcc-c++ make rpm-build \
                 libxml2-devel libpcap-devel pcre-devel libuuid-devel \
                 cmake fftw3-devel mbedtls-devel lksctp-tools-devel libconfig-devel boost-devel redhat-lsb-core

git clone https://github.com/adjacentlink/srsRAN-emane.git
cd srsRAN-emane
mkdir build
cd build
cmake ..
make && make package
sudo dnf install srsran-emane-*-x86_64.rpm
```

### Rocky Linux 9

```
sudo dnf install epel-release dnf-plugins-core
sudo dnf config-manager --enable crb
sudo dnf install git gcc-c++ make rpm-build \
                 libxml2-devel libpcap-devel pcre-devel libuuid-devel \
                 cmake fftw3-devel mbedtls-devel lksctp-tools-devel libconfig-devel boost-devel

git clone https://github.com/adjacentlink/srsRAN-emane.git
cd srsRAN-emane
mkdir build
cd build
cmake ..
make && make package
sudo dnf install srsran-emane-*-x86_64.rpm
```

### Fedora 41

```
sudo dnf install git gcc-c++ make rpm-build libxml2-devel libpcap-devel pcre-devel libuuid-devel \
                 cmake fftw3-devel mbedtls-devel lksctp-tools-devel libconfig-devel boost-devel redhat-lsb-core

git clone https://github.com/adjacentlink/srsRAN-emane.git
cd srsRAN-emane
mkdir build
cd build
cmake ..
make && make package
sudo dnf install srsran-emane-*-x86_64.rpm
```

### Ubuntu 22.04

```
sudo apt-get install git gcc g++ debhelper dh-python pkg-config python3-setuptools \
                     protobuf-compiler libprotobuf-dev python3-protobuf libxml2-dev libpcap-dev libpcre3-dev uuid-dev \
                     cmake libfftw3-dev libmbedtls-dev libboost-program-options-dev libconfig++-dev libsctp-dev lsb-release
git clone https://github.com/adjacentlink/srsRAN-emane.git
cd srsRAN-emane
mkdir build
cd build
cmake ..
make && make package
sudo dpkg -i srsran-emane-*-x86_64.deb; sudo apt-get install -f
```

### Ubuntu 24.04

```
sudo apt-get install git gcc g++ debhelper dh-python pkg-config python3-setuptools \
                     protobuf-compiler libprotobuf-dev python3-protobuf libxml2-dev libpcap-dev libpcre3-dev uuid-dev \
                     cmake libfftw3-dev libmbedtls-dev libboost-program-options-dev libconfig++-dev libsctp-dev lsb-release
git clone https://github.com/adjacentlink/srsRAN-emane.git
cd srsRAN-emane
mkdir build
cd build
cmake ..
make && make package
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
