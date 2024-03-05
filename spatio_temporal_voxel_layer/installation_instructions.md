
Clone the repository from GitHub:
```bash
$ git clone https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git
```

Install the dependencies: First initialize rosdep if you haven't already, then install the dependencies.
```bash
$ sudo rosdep init && rosdep update
```

Then install the dependencies:
```bash
$ rosdep install --from-paths . --ignore-src -r -y

$ sudo apt install libtbb-dev

$ sudo apt update
```