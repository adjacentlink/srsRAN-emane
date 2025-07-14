FROM fedora:41

RUN mkdir -p /opt/built

RUN dnf -y update

# Copy and install packages from the latest upsteam dependencies
WORKDIR /opt
RUN mkdir -p dependencies
COPY --from=gcr.io/automated-builds-303920/fedora41.emane /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/fedora41.opentestpoint /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/fedora41.openstatistic /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/fedora41.emane-model-lte /opt/built/* dependencies/
RUN dnf -y install dependencies/*\.rpm

# Build tools and depends
RUN dnf -y install git gcc-c++ make rpm-build
RUN dnf -y install libxml2-devel libpcap-devel pcre-devel libuuid-devel
RUN dnf -y install cmake fftw3-devel mbedtls-devel lksctp-tools-devel libconfig-devel boost-devel redhat-lsb-core

RUN git clone https://github.com/adjacentlink/srsRAN-emane -b develop

WORKDIR srsRAN-emane
RUN mkdir build
WORKDIR build
RUN cmake .. && make && make package
RUN cp *\.rpm /opt/built
RUN dnf -y install /opt/built/*

RUN dnf clean all

RUN echo 'complete'

