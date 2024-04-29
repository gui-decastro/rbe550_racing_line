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
## Building and Starting Docker
The next step is to build the docker image that is detailed by the Dockerfile. This docker image uses the ros:noetic-desktop-full image as the base image. You will need aproximately 3.52GB of space in your system.

Listed below are the steps to build the image and start a container.

1. Build the docker image. This command will take a while to execute for the first time.
```
# Start at the project's root folder
cd docker
./docker_build.sh
```
This creates a docker image called "racing_line_img". You will not need to run this command again once the image is built unless the Dockerfile is updated. Docker runs every command in the Dockerfile inside its own shell and it stores them in its cache, so only the changed/new commands will be rebuilt with a new docker build command, meaning it will be faster than building the image from scratch.

2. Create and enter a new docker container. Ensure this command is run from the project's root directory
```
./start_docker.sh
```
This bash script will either start a new docker container called RACING_LINE, or attach itself to an existing container if it finds one running. The catkin_ws directory on your local machine is mounted to the catkin_ws directory inside the docker container, so any changes to that directory on your local machine will be reflected inside the docker container.

## Running the Racing Line Simulation
Once inside the docker container, launch the program's main launch file which generates the racetrack map which is used for all path planning. Additionally, this launcher will start up RViz, bring up the Audibot robot, and initialize the move_base node which will handle all navigation, including the global and local planners. This launcher is well documented and so are the yaml files containing parameters configuration. The very bottom of the launcher has a few RViz instances that can be chosen from depending on the use case, including a configuration for just the maps, the planners, or the entire simulation. Be mindful of the the RViz Display checkboxes, and toggle them as needed.
```
roslaunch map_generator map_generator.launch
```
Now a path can be generated by just selecting "2D Nav Goal" and selecting a spot on the map to set the goal pose to. The global path will generate almost immediately, but the local path will take some more time.

## Complimentary Tools
Use the velocity_plotter to visualize velocity and acceleration of the generated local path
```
cd /catkin_ws/src/local_planning/src
python3 velocity_plotter.py
```

## Running Individual Components
These files do not need to be ran in isolation, but if there is need to troubleshoot one specific portion of the codebase, follow these steps:

## Running Program
### Generate an occupancy grid
Uses a top down image that is located inside the *racetracks* folder, and converts it into a binary occupancy grid. The output is placed in the *occupancy_grids* folder
```
cd /catkin_ws/src/map_generator/src/
python3 generate_map_yaml.py
```

### Generate waypoints
Interactively select the desired waypoints for a racetrack. Outputs coordinate pairs of a point on the inside contour and a point on the outside contour.
```
cd /catkin_ws/src/map_generator/src/
python3 generate_waypoints.py
```

