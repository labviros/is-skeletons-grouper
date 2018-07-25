#!/bin/bash
set -e

image_dev_tag='is-skeletons-grouper/dev'
image_count=`docker images --filter="reference=${image_dev_tag}" -q | wc -l`
if [[ $image_count == 0 ]]; then
    docker build . -f Dockerfile.dev -t ${image_dev_tag} --no-cache --network=host
fi

docker_user="viros"
image_tag="${docker_user}/is-skeletons-grouper:2.1"
docker build . -f Dockerfile -t ${image_tag} --no-cache --network=host

read -r -p "Do you want to push image ${image_tag}? [y/N] " response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]; then
    echo "Log-in as '${docker_user}' at Docker registry:"
    docker login -u ${docker_user}
    docker push ${image_tag}
fi