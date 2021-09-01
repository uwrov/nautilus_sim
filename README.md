# nautluis_sim
ROS Packages for Nautilus Simulation

# Usage
Make sure you have docker installed and have access to [nautilus_surface](https://github.com/uwrov/nautilus_surface).

## Container Setup
### Build the Image
```
docker build -t nautilus_sim .
```

### Run the Container
```
docker run -p 8080:8080 -it nautilus_sim
```

## Container Usage
### Run Gazebo (empty world)
```
cd /gzweb
gzserver --verbose & npm start
```