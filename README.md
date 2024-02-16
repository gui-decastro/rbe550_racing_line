# rbe550_racing_line
Find the optimal path around a racing circuit using motion planning methods, considering the vehicle, the circuit and any dynamically varying obstacles on the track

## Docker
Docker will be used to maintain a consistant development environment across teammates. If Docker is not already installed, follow the instructions on installing Docker Engineer on Ubuntu (https://docs.docker.com/engine/install/ubuntu/)
To summarize, execute the following commands:
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install the latest version
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
