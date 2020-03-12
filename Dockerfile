# The following commands should be run from this directory
# To build: docker build -t rapid_planner .
# To run: docker run rapid_planner
# To access the docker environment: docker run -it -v source:/usr/src/app rapid_planner bash
# (you will be dropped into a shell in the docker image)
FROM ubuntu:bionic as prereqs

# Install prerequisite libraries
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  gnupg2 \
  software-properties-common \
  cmake \
  make \
  g++ \
  git \
  nano \
  vim

# Install eigen
ADD ./dependencies/rapid-tp-eigen_0-1_amd64.deb /
RUN dpkg -i /rapid-tp-eigen_0-1_amd64.deb


ADD ./source /usr/src/app
RUN mkdir /usr/src/build
WORKDIR /usr/src/build
RUN cmake ../app
RUN make -j8

WORKDIR /usr/src/app
CMD ["/usr/src/build/path_planner_test"]
