FROM ubuntu:24.04

RUN mkdir -p /opt/built

# prevent failures due to interactive apt transactions
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update
RUN apt-get -y upgrade --no-install-recommends

# Copy and install packages from the latest opentestpoint build
WORKDIR /opt
RUN mkdir -p dependencies
COPY --from=gcr.io/automated-builds-303920/ubuntu24.04.emane /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/ubuntu24.04.opentestpoint /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/ubuntu24.04.openstatistic /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/ubuntu24.04.emane-model-lte /opt/built/* dependencies/
RUN dpkg -i dependencies/*; apt-get -y -f install

# Build tools and dependencies
RUN apt-get -y install git gcc g++ debhelper dh-python pkg-config python3-setuptools \
                       protobuf-compiler libprotobuf-dev python3-protobuf
RUN apt-get -y install libxml2-dev libpcap-dev libpcre3-dev uuid-dev
RUN apt-get -y install cmake libfftw3-dev libmbedtls-dev libboost-program-options-dev libconfig++-dev libsctp-dev lsb-release

RUN git clone https://github.com/adjacentlink/srsRAN-emane -b develop

WORKDIR srsRAN-emane
RUN mkdir build
WORKDIR build
RUN cmake .. && make && make package
RUN cp *\.deb /opt/built
RUN dpkg -i /opt/built/*; apt-get -y -f install

RUN echo 'complete'

