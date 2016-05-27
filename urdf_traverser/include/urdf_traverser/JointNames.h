#ifndef URDF_TRAVERSER_JOINTNAMES_H
#define URDF_TRAVERSER_JOINTNAMES_H

#include <string>

namespace urdf_traverser
{
class UrdfTraverser;

/**
 * Returns all joint names in depth-frist search order starting from \e fromLink (or from root if
 * \e fromLink is empty). Only joints *after* the given link are returned.
 */
bool getJointNames(UrdfTraverser& traverser, const std::string& fromLink,
                   const bool skipFixed, std::vector<std::string>& result);
}

#endif  // URDF_TRAVERSER_JOINTNAMES_H
