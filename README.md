# LiDAR object detection based on Region K-D Trees and RANSAC drivable plane segmentation


## Raw data

![raw](https://user-images.githubusercontent.com/81184255/196661247-74e1f4b8-5e99-4aad-a631-5779ff6fa62a.gif)

## Radius tree search

![radius](https://user-images.githubusercontent.com/81184255/196661465-795fb331-f0dd-436b-b8a8-97bd616e75e1.gif)


## Box tree search

![box](https://user-images.githubusercontent.com/81184255/196661565-ff70f58f-b5eb-47af-b229-496d0da9daa3.gif)


#### The true utility of the region constrained search is that you can choose where you want to look, which makes a lot of sense for some applications.

## Examples:

![track (1)](https://user-images.githubusercontent.com/81184255/196662011-316004ef-6554-490d-9110-ff68d6422783.gif)

![trackFPS](https://user-images.githubusercontent.com/81184255/196663177-86203a3d-8bd7-43cb-bf86-57653f0c4ec0.gif)

## ___Painless___ installation of the PCL library on Windows:

* Install vcpkg, the Microsoft unofficial C++ package manager

```bash
> git clone https://github.com/microsoft/vcpkg
> .\vcpkg\bootstrap-vcpkg.bat
```

There is a lot more info to be found at the original [repo](https://github.com/microsoft/vcpkg), but really, this is all you need

* Install PCL x64

```bash
> vcpkg install pcl[vtk]:x64-windows --featurepackages --recurse
```

If you do not explicitly declare an x64 build, vcpkg will default it to x86 and cmake will probably fail. Remember that Microsoft works hard to make sure their products are unusable

* Next you need to specify the toolchain file

```
-DCMAKE_TOOLCHAIN_FILE=C:\Users\Andrei\vcpkg\scripts\buildsystems\vcpkg.cmake
```

* If you are on a Windows machine, vcpckg will also default your toolchain to MSVC, so you will have to download Visual Studio and use it as the compiler

It looks like this for me (CLion):

![image](https://user-images.githubusercontent.com/81184255/197364009-78660d22-a0e9-4105-8327-9405d300993e.png)


#### Based on the original paper:

```
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

#### and original IKD-Tree repo:

https://github.com/hku-mars/ikd-Tree


