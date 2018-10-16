#!/bin/bash
set -e

function docker::has_image {
    image_count=`docker images --filter="reference=$1" -q | wc -l`
    if [[ ${image_count} == 0 ]]; then
        echo "!! Image '$1' not found"
        return 1
    fi
    return 0
}

function docker::build_local {
    tag=$1
    dockerfile=$2
    folder=$3
    echo "!! Building '${tag}'"
    sleep 2
    docker build ${folder} -f ${dockerfile} -t ${tag} --no-cache --network=host
}

function docker::push_image {
    local_tag=$1
    remote_tag=${docker_user}/$2
    read -r -p "?? Do you want to push image ${remote_tag}? [y/N] " response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]; then
        docker tag ${local_tag} ${remote_tag}
        echo "!! Log-in as '${docker_user}' at Docker registry:"
        docker login -u ${docker_user}
        docker push ${remote_tag}
    fi
}

function docker::rebuild_image {
    read -r -p "?? Image '$1' already existis, do you want to rebuild it? [y/N] " response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]; then
        echo "!! Ruilding '$1'"
        return 0
    fi
    return 1
}

image_dev='is-skeletons-grouper/dev'
docker_user="viros"
remote_tag='is-skeletons-grouper:0.0.2'

if ! docker::has_image ${image_dev}; then
    docker::build_local ${image_dev} Dockerfile.dev ../../
elif docker::rebuild_image ${image_dev}; then
    docker::build_local ${image_dev} Dockerfile.dev ../../
else
    echo "!! Alreary have image '$image_dev'"
fi

docker::build_local sks_grouper Dockerfile ../../
docker::push_image sks_grouper ${remote_tag}