# rbe550_racing_line
Find the optimal path around a racing circuit using motion planning methods, considering the vehicle, the circuit and any dynamically varying obstacles on the track

## Installing Docker
Docker will be used to maintain a consistant development environment across teammates. If Docker is not already installed, follow the instructions on installing Docker Engineer on Ubuntu (https://docs.docker.com/engine/install/ubuntu/)
To summarize, execute the following commands:

Add Docker's official GPG key:
```
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```
Add the repository to Apt sources:
```
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```
Install the latest version
```
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
## Starting the program
The next step is to build the docker image that is detailed by the Dockerfile. This docker image uses the ros:noetic-desktop-full image as the base image. You will need aproximately 3.52GB of space in your system.

Listed below are the steps to build the image and start a container.

1. Build the docker image. This command will take a while to execute for the first time.
```
docker build -t racing_line_img .
```
You will not need to run this command again once the image is built unless the Dockerfile is updated. Docker runs every command in the Dockerfile inside its own shell and it stores them in its cache, so only the changed/new commands will be rebuilt with a new docker build command, meaning it will be faster than building the image from scratch.
2. Create and enter a new docker container. Ensure this command is run from the project's root directory
```
./start_docker.sh
```
This bash script will either start a new docker container called RACING_LINE, or attach itself to an existing container if it finds one running. The catkin_ws directory on your local machine is mounted to the catkin_ws directory inside the docker container, so any changes to that directory on your local machine will be reflected inside the docker container.
