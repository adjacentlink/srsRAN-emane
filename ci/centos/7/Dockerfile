FROM centos:centos7

RUN mkdir -p /opt/built

RUN yum -y install epel-release
RUN yum -y update

# Copy and install packages from the latest opentestpoint build
WORKDIR /opt
RUN mkdir -p dependencies
COPY --from=gcr.io/automated-builds-303920/centos7.emane /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/centos7.opentestpoint /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/centos7.openstatistic /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/centos7.emane-model-lte /opt/built/* dependencies/
RUN yum -y install dependencies/*\.rpm

# Build tools
RUN yum -y install git gcc-c++ make rpm-build
RUN yum -y install libxml2-devel libpcap-devel pcre-devel libuuid-devel
RUN yum -y install cmake fftw3-devel polarssl-devel lksctp-tools-devel libconfig-devel boost-devel redhat-lsb-core

# Install devtoolset-9 to support c++17 compatibility
RUN yum -y install centos-release-scl
RUN yum -y install devtoolset-9

RUN git clone https://github.com/adjacentlink/srsRAN-emane -b develop
WORKDIR srsRAN-emane
RUN mkdir build
WORKDIR build

# Using devtoolset-9
RUN scl enable devtoolset-9 "cmake .. && make && make package"
RUN cp *\.rpm /opt/built
RUN yum -y install /opt/built/*

RUN yum clean all

RUN echo 'complete'

