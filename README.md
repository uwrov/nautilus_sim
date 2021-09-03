# nautluis_sim
ROS Packages for Nautilus Simulation

# Usage
Make sure you have docker installed and have access to [nautilus_surface](https://github.com/uwrov/nautilus_surface).

## Container Setup
### Build the Base Simulation Image Locally (Optional)
Only need to do this if you're making changes to the core set of packages being used in the simulator. Note that this can take up to an hour to finish.
```
docker build -t nautilus_sim_base -f sim.Dockerfile .
```

### Build the Working Image
```Bash
docker build -t nautilus_sim .
```

### Run the Container
```Bash
docker run -p 8080:8080 -it nautilus_sim
```

## Container Usage
### Run Gazebo (empty world)
```Bash
Xvfb :1 -screen 0 1600x1200x16 & export DISPLAY=:1.0  # Start a virtual display (makes rendering easier)
cd gzweb
gzserver --verbose & npm start                        # Start gazebo server and the gzweb interface
```
### Run Gazebo (underwater world)
Terminal 1
```Bash
Xvfb :1 -screen 0 1600x1200x16 & export DISPLAY=:1.0  # Start a virtual display (makes rendering easier)
cd nautilus_worlds/worlds
source /usr/share/gazebo/setup.sh                     # Get our models in gazebo
gzserver --verbose underwater.world                   # Start gazebo server 
```

Terminal 2
```Bash
cd gzweb
npm start                                             # Start the web interface
```