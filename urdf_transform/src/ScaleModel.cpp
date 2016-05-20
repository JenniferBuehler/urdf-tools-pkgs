#include <urdf_transform/ScaleModel.h>
#include <urdf_traverser/UrdfTraverser.h>
#include <urdf_traverser/Functions.h>
#include <ros/ros.h>

using urdf_traverser::UrdfTraverser;

/**
 * Function used for recursion by scaleModel().
 */
int scaleModelFunc(urdf_traverser::RecursionParamsPtr& p)
{
    urdf_traverser::FactorRecursionParamsPtr param =
        baselib_binding_ns::dynamic_pointer_cast<urdf_traverser::FactorRecursionParams>(p);
    if (!param)
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    urdf_traverser::LinkPtr link = param->getLink();
    if (!link)
    {
        ROS_ERROR("Recursion parameter must have initialised link!");
        return -1;
    }
    urdf_traverser::scaleTranslation(link, param->factor);
    urdf_traverser::JointPtr pjoint = link->parent_joint;
    if (pjoint)
    {
        urdf_traverser::scaleTranslation(pjoint, param->factor);
    }
    return 1;
}

bool scale_model::scaleModel(UrdfTraverser& traverser, const std::string& fromLink, double scale_factor)
{
    // do one call of scaleModel(RecursionParams) for the root link
    urdf_traverser::RecursionParamsPtr p(new urdf_traverser::FactorRecursionParams(scale_factor));
    return traverser.traverseTreeTopDown(fromLink, boost::bind(&scaleModelFunc, _1), p, true) == 1;
}

bool scale_model::scaleModel(UrdfTraverser& traverser, double scale_factor)
{
    std::string root_link = traverser.getRootLinkName();
    return scaleModel(traverser,root_link,scale_factor);
}


