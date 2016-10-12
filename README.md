# Segmentation

This ros segmentation package is mainly used for **cylinder segmentation** and **box segmentation**. It's able to segment these elements and publish their **pose** information.
For cylinder element, it will publish the coordinates of cylinder center(point) and central axis(vector) in the world coordinate system, and the radius of the cylinder.
For box element, it will publish the length, width , height, center coordinates, and the angle between the x axis(world coordinate) and longer edge of the box's bottom face.
For sphere element, it will publish the center coordinates, and the radius.

This package can be used together with object recognition program like [Faster RCNN](https://github.com/rbgirshick/py-faster-rcnn) so that the robot can first recognize the object, obtain its pose, and finally grasp it.

## How to use this package

Before you ran the segmentation package, you should have calibrated your kinect. Refer to <a href="https://github.com/CTTC/Kinect-ASUS-Xtion-Pro-Live-Calibration-Tutorials">kinect calibration tutorial</a> if you haven't done so. If you are using kinect v2, then refer to <a href="https://github.com/CTTC/Kinect-v2-Tutorial">kinect v2 calibration tutorial</a>
### Cylinder segmentation

Launch the program by:
```bash
roslaunch segmentation cylinder_seg.launch cv:=true viewDisplay:=true cmdOutput:=true
```
Note: `cv`, `viewDisplay`, `cmdOutput` are optional.

`cv`: if set to true (recommend), you can manually select the region of interest that contains the target cylinder in an opencv image window. In this way, you can deal with the multiple cylinders scenario. Otherwise, the program will try to find a cylinder on its own. It then will not work when multiple cylinders exist.

`viewDisplay`: if set to true, the point cloud viewer will pop out so that you can see the current point clouds.

`cmdOutput`: if set to true, some extra information will show up in the terminal when you launch the program.


To see the cylinder's pose, run the following command in the terminal:
```bash
 rostopic echo /processed_cylinder_data
```

An example output of processed_cylinder_data will be like this:
```bash
---
center: [-0.023846288493437895, 0.07857439498370788, 0.06118096087277025]
central_axis: [0.01558587687020927, 0.07155435404303766, -0.9973149226095387]
radius: 0.0417184047401
```
Note: unit of these parameters is meter.


### Box segmentation

Launch the program by:
```bash
roslaunch segmentation cylinder_seg.launch cv:=true viewDisplay:=true cmdOutput:=true
```
Note: `cv`, `viewDisplay`, `cmdOutput` are optional.

`cv`: if set to true (recommend), you can manually select the region of interest that contains the target box in an opencv image window. In this way, you can deal with the multiple boxes scenario. Otherwise, the program will try to find a box on its own. It then will not work when multiple boxes exist.

`viewDisplay`: if set to true, the point cloud viewer will pop out so that you can see the current point clouds.

`cmdOutput`: if set to true, some extra information will show up in the terminal when you launch the program.


To see the box's pose, run the following command in the terminal:
```bash
 rostopic echo /processed_box_data
```

An example output of processed_box_data will be like this:
```bash
---
center: [0.12652302070941762, 0.041305340052075645, 0.039217906201511776]
length: 0.352367773652
width: 0.154265999794
height: 0.0758980557858
angle: 158.434934444
```
Note: Unit of all parameters here except the angle is meter. Angle is measured in degree.

### Sphere segmentation

Launch the program by:
```bash
roslaunch segmentation sphere_seg.launch viewDisplay:=true cmdOutput:=true
```
Note:  `viewDisplay`, `cmdOutput` are optional.

`viewDisplay`: if set to true, the point cloud viewer will pop out so that you can see the current point clouds.

`cmdOutput`: if set to true, some extra information will show up in the terminal when you launch the program.


To see the sphere's pose, run the following command in the terminal:
```bash
 rostopic echo /processed_sphere_data
```

An example output of processed_sphere_data will be like this:
```bash
---
center: [0.14070103838381948, 0.10160453099038769, 0.01098050713685561]
radius: 0.0591128543019
```
Note: Unit of all parameters here except the angle is meter



## Example:
Here is a testing example for this package.
Kinect is calibrated. And the world coordinate system is set to be located at the bottom left crossing point in the chessboard. Just like the following figure.

<img src="https://github.com/CTTC/Segmentation/blob/master/world_coord.png" alt="world_coord" width="639" height="508" class="aligncenter size-full wp-image-2493" />

### Cylinder Segmentation

First draw a rectangle which contains the cylinder object on the image window.

<img src="https://github.com/CTTC/Segmentation/blob/master/cylinder_cv.png" alt="cylinder_cv" width="639" height="508" class="aligncenter size-full wp-image-2490" />

The segmented cylinder point cloud:

<img src="https://github.com/CTTC/Segmentation/blob/master/cylinder_pcl.png" alt="cylinder_pcl" width="394" height="313" class="aligncenter size-full wp-image-2491" />

Example output of processed_cylinder_data will be like this:
```bash
---
center: [-0.023846288493437895, 0.07857439498370788, 0.06118096087277025]
central_axis: [0.01558587687020927, 0.07155435404303766, -0.9973149226095387]
radius: 0.0417184047401
```
### Box Segmentation

First draw a rectangle which contains the box object on the image window.

<img src="https://github.com/CTTC/Segmentation/blob/master/box_cv.png" alt="box_cv" width="636" height="504" class="aligncenter size-full wp-image-2488" />

The segmented box point cloud:

<img src="https://github.com/CTTC/Segmentation/blob/master/box_pcl.png" alt="box_pcl" width="657" height="363" class="aligncenter size-full wp-image-2489" />

Example output of processed_box_data will be like this:
```bash
---
center: [0.12652302070941762, 0.041305340052075645, 0.039217906201511776]
length: 0.352367773652
width: 0.154265999794
height: 0.0758980557858
angle: 158.434934444
```


### Sphere Segmentation
First draw a rectangle which contains the sphere object on the image window.
<img src="https://github.com/CTTC/Segmentation/blob/master/sphere_cv.png" alt="sphere_cv" width="636" height="477" class="aligncenter size-full wp-image-2556" />
The segmented sphere point cloud:
<img src="https://github.com/CTTC/Segmentation/blob/master/sphere_pcl.png" alt="sphere_pcl" width="530" height="356" class="aligncenter size-full wp-image-2557" />
Example output of processed_sphere_data will be like this:
```bash
---
center: [0.14070103838381948, 0.10160453099038769, 0.01098050713685561]
radius: 0.0591128543019
```