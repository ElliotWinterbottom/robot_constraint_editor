#include <dqrobotics_extensions/robot_constraint_editor/robot_constraint_editor.hpp>
#include <yaml-cpp/yaml.h>

namespace DQ_robotics_extensions
{

class RobotConstraintEditor::Impl
{
public:
    YAML::Node config_;
    std::string config_file_;
    int vfi_file_version_ = 2; // default value
    bool zero_indexed_ = true; // default value
    Impl()
    {

    };
};

RobotConstraintEditor::RobotConstraintEditor() {}

}
