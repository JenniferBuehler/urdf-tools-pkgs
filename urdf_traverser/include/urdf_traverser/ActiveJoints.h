#ifndef URDF_TRAVERSER_ACTIVEJOINTS_H
#define URDF_TRAVERSER_ACTIVEJOINTS_H

#include <string>

namespace urdf_traverser
{
    class UrdfTraverser;
    
    /** 
     * returns true if there are any fixed joints down from from_link
     * Only joints *after* the given link are returned.
     */
    bool hasFixedJoints(UrdfTraverser& traverser, const std::string& fromLink);
}

#endif  // URDF_TRAVERSER_ACTIVEJOINTS_H
