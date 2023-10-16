These files originate from the COVISE package and are kept unmodified as far as possible.

There are minor adjustments made:
- Restructure folders
- Removed irrelevant defines
- Reimplement coVRConfig and coVRFileManager in a minimal variant
- RoadTerrain rather unused - just here so fasi compiles and in case someone may find this more appropiate than a complete globe like osgEarth  
!!! WARNING !!!
- RoadSystem additionally contains: laneSectionOutlines vectors, initOsg function and calls into the convertCoords functions to map open drive cartesian coordinates to world aligned coordinates.  
- Roads additionally contains: convertCoords calls, default road with objects create
!!! THIS CONVERSION IS INCOMPLETE AND FAULTY!!!!
The geometries that are converted may be positioned correctly, but orientation, normals, tangent, uvs ARE NOT converted. So consider it a hack!!!
