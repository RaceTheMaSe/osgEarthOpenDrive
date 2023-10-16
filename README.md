Experiments with OpenDrive (xodr) map files in the context of osgEarth to blend terrain height data with measured road datasets

Slightly modified versions of [COVISE](https://github.com/hlrs-vis/covise) and
[esmini](https://github.com/esmini/esmini) provide the road meshing and [osgEarth](https://github.com/gwaldron/osgearth) is the terrain engine

Changes to COVISE: mostly in Road.h/.cpp and RoadSystem.h/.cpp - see dedicated [README](src/covise/README.md) for details  
Changes to esmini: mostly in roadgeom.hpp/.cpp and around resource path lookup

Files in folder "oe" are derived and combined from osgEarth examples. osgEarthStuff.cpp contains the main part of this experiment

Pros:
- Elevation blending works with OpenDrive data providing the elevation data the terrain aligns to
- Mesh cutout works
- osgEarth terrain LODing works well with road outlines
- Positioning road scene graph so no floating point precision issues in a double precision (64bit) world coordinate system with rendering doing its thing in single precision (32bit)

Cons:
- Elevation blending is jagged due to the grid structure in the terrain mesh - a better approach would be to extend the road cross sections with additional "blend points" of the batter / shoulder / road bed
- Road LODs not good, so performance is bad on large data sets and if many road segments are in view
- Vertical datums probably not implemented correctly
- No bridges or roads crossing each other
- Texturing of roads not working / considered

Prerequisites:
- OSG 3.6.x installed
- osgEarth 3.x installed
- osgEarth source
- esMini source / resource dir
- covise installed / resource dir

Don't expect something working out-of-the-box.  
Check osgEarthStuff.cpp line 190 and enable elevation source url or adjust to another source  
You need covise and esmini resource folders if you want to experiment with more textures, traffic signs, guard rails, etc. and adjust paths ... this project was initially located in a subfolder of esmini.  
There are different ways of geo-referencing, the coordinate transformations might be off on your data set, vertical datums can be different, different map providers use different tag names for the same feature, e.g. delineators instead of pole.  
I was mostly using a dataset "Testfeld A9" provided by 3D Mapping GmbH in the offset variant. YMMW  
