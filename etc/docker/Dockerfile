FROM labviros/is-wire-base:1.1.4-gcc54 as build

ADD conanfile.py .
RUN conan install . -s compiler.libcxx=libstdc++11 --build=missing

ADD . /project
RUN cd /project                                                  \
 && ./build.sh                                                   \
 && mkdir -v -p /tmp/deploy                                      \
 && libs=`find build/ -type f -name '*.bin' -exec ldd {} \;      \
    | cut -d '(' -f 1 | cut -d '>' -f 2 | sort | uniq`           \
 && for lib in $libs; do                                         \
      cp --verbose --parents $lib /tmp/deploy;                   \
      libdir=`dirname $lib`;                                     \
    done                                                         \
 && cp --verbose `find build/ -type f -name '*.bin'` /tmp/deploy \
 && cp --verbose etc/conf/options.json /tmp/deploy               \
 && sudo rm -rf build/

# Deployment container
FROM ubuntu:16.04
WORKDIR /opt/is
COPY --from=build /tmp/deploy /
COPY --from=build /project/etc/conf/options.json /opt/is
RUN mv /service.bin /opt/is
RUN mv /rpc.bin /opt/is