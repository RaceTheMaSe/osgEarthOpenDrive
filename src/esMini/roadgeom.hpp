/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#ifndef ROADGEOM_HPP_
#define ROADGEOM_HPP_

#include <osg/Material>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osg/Group>
#include <osg/Geometry>
#include "RoadManager.hpp"

class RoadGeom
{
public:
    osg::ref_ptr<osg::Group> root_;
    osg::ref_ptr<osg::Group> road_group_;
    osg::ref_ptr<osg::Group> rm_group_;
    osg::ref_ptr<osg::MatrixTransform> base_;
    RoadGeom(roadmanager::OpenDrive* odr, osg::Matrixd originGeocentric);
    int  AddRoadMarks(roadmanager::Lane* lane, osg::Group* group);
    void AddRoadMarkGeom(osg::ref_ptr<osg::Vec3Array> vertices, osg::ref_ptr<osg::DrawElementsUInt> indices, roadmanager::RoadMarkColor color);
    void AddRoadOutline(int road_id, osg::ref_ptr<osg::Vec3dArray> vertices);
    osg::ref_ptr<osg::Texture2D> ReadTexture(std::string filename);

    std::map<long long, osg::ref_ptr<osg::Vec3dArray>> outlines;  // lane section outlines, encode road_id << 8, lane_id

    // create a 2d list of positions for vertices, nr_of_lanes x nr_of_s-values
    typedef struct
    {
        double x;
        double y;
        double z;
        double h;
        double slope;
        double s;
    } GeomPoint;

private:
    void initMaterials();
    void laneSectionOutline(std::vector<std::vector<GeomPoint>>& geom_point_list, unsigned int numLanes, int road_id, int lane_id);
    void roadAndMarkingsMesh(std::vector<std::vector<GeomPoint>>& geom_point_list,
                             roadmanager::LaneSection*            lsec,
                             roadmanager::Lane*                   lane,
                             roadmanager::Position&               pos);
    void createLaneSectionSlice(std::vector<std::vector<GeomPoint>>& geom_point_list,
                                std::vector<double>&                 s_list,
                                roadmanager::Position&               pos,
                                roadmanager::LaneSection*            lsec,
                                roadmanager::Lane*                   lane,
                                int                                  road_id);
    void convertGeometryPointListToMapCoords(std::vector<std::vector<GeomPoint>>& geom_point_list);

    // share materials and color arrays where possible
    std::vector<osg::ref_ptr<osg::Material>>  materialsRoadmark_;
    std::vector<osg::ref_ptr<osg::Vec4Array>> materials_colors_;

    osg::ref_ptr<osg::Texture2D> tex_asphalt;
    osg::ref_ptr<osg::Texture2D> tex_grass;

    osg::ref_ptr<osg::Vec4Array> color_asphalt;
    osg::ref_ptr<osg::Vec4Array> color_concrete;
    osg::ref_ptr<osg::Vec4Array> color_border_inner;
    osg::ref_ptr<osg::Vec4Array> color_grass;

    osg::ref_ptr<osg::Material> materialAsphalt_;
    osg::ref_ptr<osg::Material> materialGrass_;
    osg::ref_ptr<osg::Material> materialConcrete_;
    osg::ref_ptr<osg::Material> materialBorderInner_;
};

#endif  // ROADGEOM_HPP_
