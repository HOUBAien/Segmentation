# Segmentation

This ros segmentation package is mainly used for **cylinder segmentation**, **box segmentation**, and **sphere segmentation**. It's able to segment these elements and publish their **pose** information.

For cylinder element, it will publish the coordinates of cylinder center(point) and central axis(vector) in the world coordinate system, and the radius of the cylinder.

For box (2D) element, it will publish the length, width , height, center coordinates, and the angle between the x axis(world coordinate) and longer edge of the box's bottom face.

For box (3D) element, it will publish the center coordinates, edges length (longest edge length, median edge length, shortest edge length) and corresponding edges directions (represented as vector in world space)

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


### Box segmentation (2D)

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


### Box segmentation (3D)

Launch the program by:
```bash
roslaunch segmentation box_seg_3D.launch viewDisplay:=true cmdOutput:=true
```
Note: `viewDisplay`, `cmdOutput` are optional.

`viewDisplay`: if set to true, the point cloud viewer will pop out so that you can see the current point clouds.
`cmdOutput`: if set to true, some extra information will show up in the terminal when you launch the program.


To see the box's pose, run the following command in the terminal:
```bash
 rostopic echo /processed_box_3D_data
```

An example output of processed_box_3D_data will be like this:
```bash
---
center: [0.16653619377903012, 0.051877501860137065, 0.08140872173126434]
longestEdgeLen: 0.343752086163
medianEdgeLen: 0.162613391876
shortestEdgeLen: 0.0871307402849
longestEdgeDir: [0.5491512947381777, 0.7872976496531994, -0.2803484730436944]
medianEdgeDir: [-0.8466069263542995, 0.5237629179351927, -0.094494010629978]
shortestEdgeDir: [-0.13417326893003623, -0.2524111482444952, -0.9582724801154021]
```
Note: Unit of all parameters here except the angle is meter. This program can estimate the pose of a box placed randomly in space. It deals with the following three cases:
* the box is placed horizontally on the table
* kinect sees three faces of the box
* kinect only sees two faces of the box

The first case is not isolated from the latter two cases. It's just that the program can use simpler algorithm to estimate the box's pose when one face is parallel to the table.



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

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/world_coord.png" alt="world_coord" width="639" height="508" class="aligncenter size-full wp-image-2493" />

### Cylinder Segmentation

First draw a rectangle which contains the cylinder object on the image window.

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/cylinder_cv.png" alt="cylinder_cv" width="639" height="508" class="aligncenter size-full wp-image-2490" />

The segmented cylinder point cloud:

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/cylinder_pcl.png" alt="cylinder_pcl" width="394" height="313" class="aligncenter size-full wp-image-2491" />

Example output of processed_cylinder_data will be like this:
```bash
---
center: [-0.023846288493437895, 0.07857439498370788, 0.06118096087277025]
central_axis: [0.01558587687020927, 0.07155435404303766, -0.9973149226095387]
radius: 0.0417184047401
```
### Box Segmentation

First draw a rectangle which contains the box object on the image window.

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/box_cv.png" alt="box_cv" width="636" height="504" class="aligncenter size-full wp-image-2488" />

The segmented box point cloud:

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/box_pcl.png" alt="box_pcl" width="657" height="363" class="aligncenter size-full wp-image-2489" />

Example output of processed_box_data will be like this:
```bash
---
center: [0.12652302070941762, 0.041305340052075645, 0.039217906201511776]
length: 0.352367773652
width: 0.154265999794
height: 0.0758980557858
angle: 158.434934444
```

### Box Segmentation (3D)

* First case: the box is placed horizontally on the table
In this case, it is pretty much the same as **Box Segmentation (2D)**
First draw a rectangle which contains the box object on the image window.

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/horizontal_cv.png" alt="horizontal_cv" width="630" height="479" class="aligncenter size-full wp-image-2577" />

Then the segmented box point cloud looks like this:

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/horizontal_pcl1.png" alt="horizontal_pcl1" width="455" height="388" class="aligncenter size-full wp-image-2578" />

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/horizontal_pcl2.png" alt="horizontal_pcl2" width="569" height="206" class="aligncenter size-full wp-image-2579" />

Example output of processed_box_3D_data will be like this:
```bash
---
center: [0.15881596472925671, 0.08577798547509508, 0.04179396809914904]
longestEdgeLen: 0.348132386804
medianEdgeLen: 0.158469796181
shortestEdgeLen: 0.0812002420425
longestEdgeDir: [-0.8460249376328047, -0.5330809538510344, -0.00815484792502631]
medianEdgeDir: [0.5330865121979369, -0.8460591003942157, 0.0016640771524550496]
shortestEdgeDir: [-0.007786559068384082, -0.00293940025209827, 0.9999653641121938]
```

* Second case: kinect sees three faces of the box

Draw a rectangle which contains the box object on the image window.

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/3face_cv.png" alt="3face_cv" width="635" height="477" class="aligncenter size-full wp-image-2574" />


Then the segmented box point cloud looks like this:

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/3face_pcl1.png" alt="3face_pcl1" width="803" height="578" class="aligncenter size-full wp-image-2575" />

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/3face_pcl2.png" alt="3face_pcl2" width="677" height="423" class="aligncenter size-full wp-image-2576" />

Example output of processed_box_3D_data will be like this:
```bash
---
center: [0.16653619377903012, 0.051877501860137065, 0.08140872173126434]
longestEdgeLen: 0.343752086163
medianEdgeLen: 0.162613391876
shortestEdgeLen: 0.0871307402849
longestEdgeDir: [0.5491512947381777, 0.7872976496531994, -0.2803484730436944]
medianEdgeDir: [-0.8466069263542995, 0.5237629179351927, -0.094494010629978]
shortestEdgeDir: [-0.13417326893003623, -0.2524111482444952, -0.9582724801154021]
```

* Third case:kinect only sees two faces of the box
Draw a rectangle which contains the box object on the image window.

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/2face_cv.png" alt="2face_cv" width="641" height="483" class="aligncenter size-full wp-image-2571" />

Then the segmented box point cloud looks like this:

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/2face_pcl2.png" alt="2face_pcl2" width="631" height="404" class="aligncenter size-full wp-image-2573" />

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/2face_pcl1.png" alt="2face_pcl1" width="594" height="333" class="aligncenter size-full wp-image-2572" />

Example output of processed_box_3D_data will be like this:
```bash
---
center: [0.12593763719872755, 0.034503789657448825, 0.09440309433414645]
longestEdgeLen: 0.335469901562
medianEdgeLen: 0.158436074853
shortestEdgeLen: 0.0799739807844
longestEdgeDir: [0.10318452483361243, 0.9512752283048365, -0.29056564464582335]
medianEdgeDir: [0.9946621486234158, -0.09856396630986955, 0.030534482820521267]
shortestEdgeDir: [-0.0017048003530623173, -0.31157605455935394, -0.9502196882200361]
```



### Sphere Segmentation

First draw a rectangle which contains the sphere object on the image window.

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/sphere_cv.png" alt="sphere_cv" width="636" height="477" class="aligncenter size-full wp-image-2556" />

The segmented sphere point cloud:

<img src="https://github.com/CTTC/Segmentation/blob/master/figures/sphere_pcl.png" alt="sphere_pcl" width="530" height="356" class="aligncenter size-full wp-image-2557" />

Example output of processed_sphere_data will be like this:
```bash
---
center: [0.14070103838381948, 0.10160453099038769, 0.01098050713685561]
radius: 0.0591128543019
```
