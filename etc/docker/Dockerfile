FROM is-skeletons-grouper/dev AS build

ADD . /project
WORKDIR /project

RUN sudo bash build.sh
RUN mkdir -v -p /tmp/deploy                                             \
 && libs=`find build/ -type f -name 'service.bin' -exec ldd {} \;       \
  | cut -d '(' -f 1 | cut -d '>' -f 2 | sort | uniq`                    \
 && for lib in $libs; do                                                \
      cp --verbose --parents $lib /tmp/deploy;                          \
      libdir=`dirname $lib`;                                            \
    done                                                                \
  && cp --verbose `find build/ -type f -name 'service.bin'` /tmp/deploy \
  && cp --verbose etc/conf/options.json /tmp/deploy                     \
  && sudo rm -rf build/

# Deployment container
FROM ubuntu:16.04
WORKDIR /opt/is
COPY --from=build /tmp/deploy /
RUN mv /service.bin /opt/is
RUN mv /options.json /opt/is