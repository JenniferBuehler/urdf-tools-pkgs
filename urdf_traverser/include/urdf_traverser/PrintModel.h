#ifndef PRINT_MODEL_H
#define PRINT_MODEL_H

namespace urdf_traverser
{
    class UrdfTraverser;
}

namespace print_model
{

extern bool printModel(urdf_traverser::UrdfTraverser& traverser, const std::string& fromLink, bool verbose);

extern bool printModel(urdf_traverser::UrdfTraverser& traverser, bool verbose);

}

#endif  // PRINT_MODEL_H
