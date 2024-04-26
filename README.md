This repository is the implementation of the paper "MGCBS: An Optimal and Efficient Algorithm for Solving Multi-Goal Multi-Agent Path Finding Problem"

This repository is based on code from [libMultiRobotPlanning](https://github.com/whoenig/libMultiRobotPlanning).
## Dataset
The full dataset can be downloaded [here](https://hkustconnect-my.sharepoint.com/:f:/g/personal/mtangag_connect_ust_hk/EtxXB80hrA5GsjkjXmE4jbQBnbDSU04MDIXDPLBUz1jopg?e=SWpLiY). 

## Dependency
The dependency includes:
- yaml-cpp
- Boost
- PkgConfig

A small dataset is given in ./dataset
## Compile
To build the project, using the following instructions
```shell
mkdir build
cd build
cmake ..
make
```

## Execute
The executable is generated in `./build/main`

To run the executable, the input file and the output file should be given.

**Go to the root directory of the repository.**
```
cd ..
```
Use the following instruction to run the executable.
```
./build/main -a <algorithm> -i <input_file> -o <output_file>
```
The `<algorithm>` can be:
- MGCBS (Corresponding th 'MGCBS (A3)' in the paper)
- MGCBS_w1 (Corresponding th 'MGCBS without TIS Forest (A2)' in the paper)
- MGCBS_w2 (Corresponding th 'HCBS (A1)' in the paper)
- OptimalCBS (Corresponding th 'CBS + A* (A4)' in the paper)

For example
```
./build/main -a MGCBS -i dataset_example/lak303d/2_4_0.yaml -o output.yaml
```

In the name of input data file "x_y_z.yaml", 'x' is the agent number, 'y' is the goal number and 'z' is the instance id.

