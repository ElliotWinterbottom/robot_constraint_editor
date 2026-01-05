#include <dqrobotics_extensions/robot_constraint_editor/robot_constraint_editor.hpp>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
#include <filesystem>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <map>
#include <fstream>

namespace DQ_robotics_extensions
{

class RobotConstraintEditor::Impl
{
public:
    enum class VFI_TYPE{
        ENVIRONMENT_TO_ROBOT,
        ROBOT_TO_ROBOT
    };
public:
    YAML::Node config_;
    //Sstd::string config_file_;
    int vfi_file_version_ = 2; // default value
    bool zero_indexed_ = true; // default value
    std::shared_ptr<VFIConfigurationFileYaml> vfi_config_file_yaml_;

    std::map<std::string, VFIConfigurationFile::RawData> yaml_raw_data_map_;

    VFI_TYPE _get_vfi_type(const VFIConfigurationFile::RawData& data)
    {
        if (std::holds_alternative<VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_RAW_DATA>(data))
            return VFI_TYPE::ENVIRONMENT_TO_ROBOT;
        else if (std::holds_alternative<VFIConfigurationFile::ROBOT_TO_ROBOT_RAW_DATA>(data))
            return  VFI_TYPE::ROBOT_TO_ROBOT;
        else
            throw std::runtime_error("Unsupported VFI TYPE!");
    }

    std::vector<VFIConfigurationFile::RawData> _load_data_from_yaml_file(const std::string& config_file)
    {
        if (!vfi_config_file_yaml_)
            vfi_config_file_yaml_ = std::make_shared<VFIConfigurationFileYaml>(config_file);
        return vfi_config_file_yaml_->get_raw_data();
    }

    std::string _extract_tag(const VFIConfigurationFile::RawData& raw_data) {
        return std::visit([](auto&& arg) -> std::string {
            return arg.tag;
        }, raw_data);
    }

    bool is_tag_in_map(const std::string& tag)
    {
        return (yaml_raw_data_map_.find(tag) == yaml_raw_data_map_.end()) ? false : true;
    }

    Impl()
    {

    };
};

RobotConstraintEditor::RobotConstraintEditor() {
    impl_ = std::make_shared<RobotConstraintEditor::Impl>();
}



void  RobotConstraintEditor::load_data(const std::string& config_file)
{
    std::vector<VFIConfigurationFile::RawData> raw_data = impl_->_load_data_from_yaml_file(config_file);
    impl_->yaml_raw_data_map_.clear();
    for (const auto& data : raw_data) {
        std::string tag = impl_->_extract_tag(data);
        impl_->yaml_raw_data_map_.try_emplace(tag, data);
    }
}

void RobotConstraintEditor::replace_data(const std::string& tag, const VFIConfigurationFile::RawData& data)
{
    if (impl_->vfi_config_file_yaml_)
    {
        try{
            // Check if tag exists
            if (!impl_->is_tag_in_map(tag))
                throw std::runtime_error("Tag '" + tag + "' not found!");

            auto current_data = impl_->yaml_raw_data_map_.at(tag);
            if (impl_->_get_vfi_type(current_data) != impl_->_get_vfi_type(data))
                throw std::runtime_error("The new data has a different VFI type from the current one!");

            remove_data(tag);
            add_data(data);
        } catch (const std::runtime_error& e) {
            std::cerr<<e.what()<<std::endl;
            throw std::runtime_error("RobotConstraintEditor::edit_data: Fail to update the VFI data!");
        }

    }
}

void RobotConstraintEditor::add_data(const VFIConfigurationFile::RawData& data)
{
    const std::string tag = impl_->_extract_tag(data);
    if (impl_->is_tag_in_map(tag))
        throw std::runtime_error("Tag '" + tag + "' is being used!");
    impl_->yaml_raw_data_map_.try_emplace(tag, data);
}

void RobotConstraintEditor::remove_data(const std::string& tag)
{
    if (!impl_->is_tag_in_map(tag))
        throw std::runtime_error("Tag '" + tag + "' not found!");
    impl_->yaml_raw_data_map_.erase(tag);
}

void RobotConstraintEditor::save_data(const std::string& path_config_file,
                                      const int &vfi_file_version,
                                      const bool &zero_indexed)
{
    try {
        if (path_config_file.empty())
            throw std::runtime_error("path_config_file path cannot be empty!");

        // Create directory if it doesn't exist
        std::filesystem::path file_path(path_config_file);
        std::filesystem::path directory = file_path.parent_path();

        if (!directory.empty() && !std::filesystem::exists(directory)) {
            std::cout << "Creating directory: " << directory << std::endl;
            std::filesystem::create_directories(directory);
        }

        std::ofstream file(path_config_file);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file for writing: " + path_config_file);
        }

        // Write header
        file << "vfi_file_version: " << vfi_file_version << "\n";
        file << "zero_indexed: " << (zero_indexed ? "true" : "false") << "\n";
        file << "vfi_array:\n";

        for (const auto& [tag, raw_data] : impl_->yaml_raw_data_map_) {

            file << "  -\n";
            std::visit([&file](auto&& arg) {
                using T = std::decay_t<decltype(arg)>;

                if constexpr (std::is_same_v<T, VFIConfigurationFile::ENVIRONMENT_TO_ROBOT_RAW_DATA>) {
                    file << "    vfi_type: \"" << arg.vfi_type << "\"\n";


                    file << "    cs_entity_environment: [";
                    for (size_t i = 0; i < arg.cs_entity_environment.size(); ++i) {
                        file << "\"" << arg.cs_entity_environment[i] << "\"";
                        if (i < arg.cs_entity_environment.size() - 1) file << ", ";
                    }
                    file << "]\n";


                    file << "    cs_entity_robot: [";
                    for (size_t i = 0; i < arg.cs_entity_robot.size(); ++i) {
                        file << "\"" << arg.cs_entity_robot[i] << "\"";
                        if (i < arg.cs_entity_robot.size() - 1) file << ", ";
                    }
                    file << "]\n";

                    file << "    entity_environment_primitive_type: \""
                         << arg.entity_environment_primitive_type << "\"\n";
                    file << "    entity_robot_primitive_type: \""
                         << arg.entity_robot_primitive_type << "\"\n";
                    file << "    robot_index: " << arg.robot_index << "\n";
                    file << "    joint_index: " << arg.joint_index << "\n";
                    file << "    safe_distance: " << arg.safe_distance << "\n";

                    // vfi_gain with .0 for integers
                    file << "    vfi_gain: ";
                    if (arg.vfi_gain == static_cast<int>(arg.vfi_gain)) {
                        file << arg.vfi_gain << ".0";
                    } else {
                        file << arg.vfi_gain;
                    }
                    file << "\n";

                    file << "    direction: \"" << arg.direction << "\"\n";
                    file << "    tag: \"" << arg.tag << "\"\n";

                } else if constexpr (std::is_same_v<T, VFIConfigurationFile::ROBOT_TO_ROBOT_RAW_DATA>) {
                    file << "    vfi_type: \"" << arg.vfi_type << "\"\n";

                    // cs_entity_one
                    file << "    cs_entity_one: [";
                    for (size_t i = 0; i < arg.cs_entity_one.size(); ++i) {
                        file << "\"" << arg.cs_entity_one[i] << "\"";
                        if (i < arg.cs_entity_one.size() - 1) file << ", ";
                    }
                    file << "]\n";

                    // cs_entity_two
                    file << "    cs_entity_two: [";
                    for (size_t i = 0; i < arg.cs_entity_two.size(); ++i) {
                        file << "\"" << arg.cs_entity_two[i] << "\"";
                        if (i < arg.cs_entity_two.size() - 1) file << ", ";
                    }
                    file << "]\n";

                    file << "    entity_one_primitive_type: \""
                         << arg.entity_one_primitive_type << "\"\n";
                    file << "    entity_two_primitive_type: \""
                         << arg.entity_two_primitive_type << "\"\n";
                    file << "    robot_index_one: " << arg.robot_index_one << "\n";
                    file << "    robot_index_two: " << arg.robot_index_two << "\n";
                    file << "    joint_index_one: " << arg.joint_index_one << "\n";
                    file << "    joint_index_two: " << arg.joint_index_two << "\n";
                    file << "    safe_distance: " << arg.safe_distance << "\n";

                    // vfi_gain with .0 for integers
                    file << "    vfi_gain: ";
                    if (arg.vfi_gain == static_cast<int>(arg.vfi_gain)) {
                        file << arg.vfi_gain << ".0";
                    } else {
                        file << arg.vfi_gain;
                    }
                    file << "\n";

                    file << "    direction: \"" << arg.direction << "\"\n";
                    file << "    tag: \"" << arg.tag << "\"\n";
                }
            }, raw_data);
        }

        file.close();

        std::cout << "Successfully saved " << impl_->yaml_raw_data_map_.size()
                  << " VFI entries to: " << path_config_file << std::endl;

    } catch (const std::filesystem::filesystem_error& e) {
        throw std::runtime_error("Filesystem error: " + std::string(e.what()));
    } catch (const std::exception& e) {
        throw std::runtime_error("Error in save_data: " + std::string(e.what()));
    }
}
std::vector<VFIConfigurationFile::RawData> RobotConstraintEditor::get_raw_data()
{
    std::vector<VFIConfigurationFile::RawData> raw_data;
    raw_data.reserve(impl_->yaml_raw_data_map_.size());
    for (auto& pair : impl_->yaml_raw_data_map_)
        raw_data.push_back(pair.second);
    return raw_data;
}

}
