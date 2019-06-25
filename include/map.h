#ifndef MAP_H
#define MAP_H

#include "common_headers.h"
#include "rgbdframe.h"
#include "map_point.h"

namespace rgbd_tutor
{
class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long, MapPoint::Ptr >  map_points_;        // all landmarks
    unordered_map<unsigned long, RGBDFrame::Ptr >     keyframes_;         // all key-frames

    Map() {}
    
    void insertKeyFrame( RGBDFrame::Ptr frame );
    void insertMapPoint( MapPoint::Ptr map_point );
};
}

#endif // MAP_Hs