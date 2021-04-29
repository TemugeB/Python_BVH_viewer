# Python BVH viewer
Simple python only BVH file viewer. This viewer is based on the BVH file parser written by 20tab: [https://github.com/20tab/bvh-python](https://github.com/20tab/bvh-python).

![Example_figure](Fig/Example.png?raw=true "Example")


**Features**:  
Local space and world space coordinates are calculated frame by frame and plotted in 3D using matplotlib. A dictionary containing local and world space coordinates are calculated in the Draw_bvh function. If you need these coordinates only, simply save the contents of these variables to file. 

**Requirements**:  
Python 3.7  
Matplotlib

**Usage**:  
Simply call as
```
python view_bvh.py /path/to/file.bvh
```

**Note**:  
By default, the figure only shows the bvh file in local space. If you want to see the motion in world space, uncomment the world space plotting code in Draw_bvh functions. Additionally, you may need to change the limits of the viewing window. This can be done by setting figure_limit to some large value in Draw_bvh function.
