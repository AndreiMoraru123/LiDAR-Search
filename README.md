# LiDAR Object Detection - Region ikd-Trees & RANSAC

![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white) ![Windows](https://img.shields.io/badge/Windows-0078D6?style=for-the-badge&logo=windows&logoColor=white) ![CMake](https://img.shields.io/badge/CMake-%23008FBA.svg?style=for-the-badge&logo=cmake&logoColor=white) ![Visual Studio](https://img.shields.io/badge/Visual%20Studio-5C2D91.svg?style=for-the-badge&logo=visual-studio&logoColor=white) ![CLion](https://img.shields.io/badge/CLion-black?style=for-the-badge&logo=clion&logoColor=white) ![Udacity](https://img.shields.io/badge/Udacity-white?style=for-the-badge&logo=udacity&logoColor=15B8E6) 	

## Raw data

![raw](https://user-images.githubusercontent.com/81184255/196661247-74e1f4b8-5e99-4aad-a631-5779ff6fa62a.gif)

## Radius tree search

> **Note**
> As you can see below, the tree building time is still the bottleneck for my approach, as I am not making full use of the incremental functionality. Any idea that comes as either an issue or pull request in this regard is highly appreciated.  

![radius](https://user-images.githubusercontent.com/81184255/196661465-795fb331-f0dd-436b-b8a8-97bd616e75e1.gif)

## Box tree search

![box](https://user-images.githubusercontent.com/81184255/196661565-ff70f58f-b5eb-47af-b229-496d0da9daa3.gif)

## How does a K dimensional tree work?

### Building / Insertion

Assuming k = 2 dimensions, below is a short demo

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/203546378-50605a04-55f4-4b26-b7ed-b15d72f9e03a.gif" width="500"/>
</p>

1. The first inserted node, (3,6) will be the root. 
2. To insert (17,15), we have to compare it along the X axis with (3,6). Since 17 > 3, we insert it to the `RIGHT`
3. To insert (13,16), we have to compare it along the X axis with (3,6), as well as the Y axis of (17,15). Since 13 > 3 and 16 > 15, we insert it to the `RIGHT` of (17,15)
4. To insert (6,12), we have to compare it along the X axis with (3,6), as well as Y of (17,15). Since 6 > 3 and 6 < 15, we insert it to the `LEFT` of (17,15)
5. To insert (9,1), we compare it with X of (3,6), Y of (17,15) and X of (6,12). Since 9 > 3 and 1 < 15 and 9 > 6, it goes to the `RIGHT` of (6,2)
6. To insert (10,19) we compare it with X of (3,6), Y of (17,15) and X of (13,15). Since 10 > 3 and 19 > 15 and 10 < 13, it goes to the `LEFT` of (13,15)
7. Lastly, to insert (2,7), we compare it to the X of (3,6), and since 2 < 7, we put it to the `LEFT` of the root node.

### Searching

Since at every step we get rid of one dimension (either X or Y), we make twice the progress we would make by iterating linearly through the three, which means the search is performed in logarithmic time (in this example a log of base 2), so this speeds up things while marking all the clusters of points and then telling them apart via Euclidian distance between neighboring points

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/203548868-b9aaabee-0c89-4f31-b4a0-9df0c1630296.gif" width="500"/>
</p>

To look up (10,19) for instance, we:

1. Eliminate the `LEFT` side (2,7) of the tree from the root since 10 > 3, and proceed to only search to the `RIGHT`
2. Eliminate the `LEFT` side (6,12) -> (9,1) of the subtree having the root (17,15) since 19 > 15, and proceed to only search to the `RIGHT`
3. Find (10,19) to the `LEFT` side the subtree having the root (13,15)

[Here is a more in depth example using the same numbers](https://www.geeksforgeeks.org/k-dimensional-tree/)

## How does RANSAC work?

Again taking a two dimensional example, one can clearly see there is a linear trend in the data below.

However, applying linear regression here would be shifted by the numerous outliers, so to find the ___best___ line (and not necesarily the one that accomodates all the points), random sample consensus is used

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/203555603-7a62ee39-9bb1-472f-b062-074636eae5db.gif" width="500"/>
</p>

The algorithm for this is the following:
1. Randomly select two points to get a line
2. Compute the line equation via: 
```math
ax + by + c = 0
```
3. Get the distance between the formed line and the rest of points using:
```math
d = \frac{|ax + by + c|}{\sqrt{a^2 + b^2}}
```
4. For each point, if its distance from the line is within the chosen threshold, consider the distanced point an ___inlier___
5. Repeat 1-4 for a chosen number of iterations

This way, only the line which accomodates the most inliers is the one considered.

This is extremly useful for segmenting out the drivable plane/road in three dimensions.

## Examples:

> **Note**
> The biggest selling point of the region constrained search is that you can choose where you want to look, which makes a lot of sense for some applications.

![track (1)](https://user-images.githubusercontent.com/81184255/196662011-316004ef-6554-490d-9110-ff68d6422783.gif)

![trackFPS](https://user-images.githubusercontent.com/81184255/196663177-86203a3d-8bd7-43cb-bf86-57653f0c4ec0.gif)

## ___Painless___ installation of the Point Cloud Library on Windows [![Vcpkg](https://skillicons.dev/icons?i=visualstudio)](https://github.com/microsoft/vcpkg) 
### (if you want to try out this code):

* Install vcpkg, Microsoft's unofficial C++ package manager

```bash
> git clone https://github.com/microsoft/vcpkg
> .\vcpkg\bootstrap-vcpkg.bat
```

There is a lot more info to be found at the original [repo](https://github.com/microsoft/vcpkg), but really, this is all you need

* Install PCL x64

```bash
> vcpkg install pcl[vtk]:x64-windows --featurepackages --recurse
```

> **Warning**
> If you do not explicitly declare an x64 build, vcpkg will default it to x86 and cmake will probably fail.

* Next you need to specify the toolchain path to the cmake file

```
-DCMAKE_TOOLCHAIN_FILE=C:\Path\vcpkg.cmake
```

* If you are on a Windows machine, vcpckg will also default your toolchain to MSVC, so you will have to download Visual Studio and use it as the compiler

It looks like this for me (CLion):

![image](https://user-images.githubusercontent.com/81184255/197364009-78660d22-a0e9-4105-8327-9405d300993e.png)


#### Based on the original paper:

```bibtex
@misc{https://doi.org/10.48550/arxiv.2102.10808,
  doi = {10.48550/ARXIV.2102.10808}, 
  url = {https://arxiv.org/abs/2102.10808},
  author = {Cai, Yixi and Xu, Wei and Zhang, Fu},
  keywords = {Robotics (cs.RO), FOS: Computer and information sciences, FOS: Computer and information sciences},
  title = {ikd-Tree: An Incremental K-D Tree for Robotic Applications},
  publisher = {arXiv},
  year = {2021},
  copyright = {Creative Commons Attribution 4.0 International}
}
```

#### and original repos:

[ikd-Tree](https://github.com/hku-mars/ikd-Tree)

[LiDAR](https://github.com/awbrown90/SFND_Lidar_Obstacle_Detection)
