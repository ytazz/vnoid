# First of All
- git clone https://github.com/ytazz/vnoid.git

## with NVIDIA video-card
At vnoid directory (not at vnoid/docker directory)

You should install nvidia-docker https://github.com/NVIDIA/nvidia-docker
- docker build -f docker/Dockerfile.with_choreonoid -t vnoid_with_choreonoid .
- docker/run_docker.sh

## without NVIDIA video-card
At vnoid directory (not at vnoid/docker directory)
- docker build -f docker/Dockerfile.with_choreonoid -t vnoid_with_choreonoid_wo_nvidia --build-arg BASE_IMAGE=ubuntu:18.04 .
- docker/run_docker_wo_nvidia.sh

## with vnc (docker-compose)
- docker pull devrt/xserver
- git clone https://github.com/YoheiKakiuchi/misc_docker.git -b xserver_nvidia build_vnc
- (cd build_vnc; ./wrap_for_docker_xserver.sh vnoid_with_choreonoid_wo_nvidia --supervisor --default-command 'choreonoid /usr/local/share/choreonoid-1.8/project/vnoid_sample_project.cnoid' --user-directory /userdir )
- // you can share /userdir (local directory) with docker ( /userdir in docker )
- // install docker-compose (read build_vnc/README.md)
- docker-compose -f build_vnc/docker-compose.yaml -p vnoid up
- // open ppp.xxx.yyy.zzz:3000 with browser (ppp.xxx.yyy.zzz is IP address of PC on which you executed docker-compose)
