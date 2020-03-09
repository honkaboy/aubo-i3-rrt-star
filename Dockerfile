FROM ubuntu:bionic as prereqs

#install prerequisite libraries
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  gnupg2 \
  software-properties-common

WORKDIR /
ADD ./dependencies/rapid-tp-eigen_0-1_amd64.deb /

RUN dpkg -i *.deb

##############################################################################
## build
## Adds prerequisites for building and builds the code
FROM prereqs as build

WORKDIR /

ADD ./source /usr/src/app

RUN apt-get -qq update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    cmake \
    make \
    g++ \
    git

WORKDIR /usr/src
RUN mkdir build
WORKDIR /usr/src/build
RUN cmake ../app
RUN make -j8

################################################################################
FROM build as dev

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  nano \
  vim
WORKDIR /usr/src/app
CMD ["/bin/bash"]
################################################################################
