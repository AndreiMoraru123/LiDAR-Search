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


