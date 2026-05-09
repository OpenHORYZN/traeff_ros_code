#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class MapExporter : public rclcpp::Node {
public:
    MapExporter() : Node("map_exporter") {
        // Subscribe to the map topic
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/box_map", 
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(), 
            std::bind(&MapExporter::map_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Map Exporter Node initialized. Listening to /map...");
    }

    /**
     * This function performs the actual file writing. 
     * It mimics the behavior of nav2_map_server by creating a .yaml and a .pgm file.
     */
    void save_map_to_disk(const std::string & base_filename) {
        if (!latest_map_) {
            RCLCPP_WARN(this->get_logger(), "No map data received yet. Save aborted.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting manual map save to: %s", base_filename.c_str());

        try {
            std::string yaml_path = base_filename + ".yaml";
            std::string pgm_path = base_filename + ".pgm";

            // 1. Write the YAML Metadata file
            std::ofstream yaml(yaml_path);
            if (!yaml.is_open()) throw std::runtime_error("Could not open YAML file for writing.");
            
            yaml << "image: " << base_filename.substr(base_filename.find_last_of("/\\") + 1) << ".pgm\n";
            yaml << "resolution: " << latest_map_->info.resolution << "\n";
            yaml << "origin: [" << latest_map_->info.origin.position.x << ", " 
                 << latest_map_->info.origin.position.y << ", 0.0]\n";
            yaml << "negate: 0\n";
            yaml << "occupied_thresh: 0.65\n";
            yaml << "free_thresh: 0.25\n";
            yaml.close();

            // 2. Write the PGM image (P5 Binary format)
            std::ofstream pgm(pgm_path, std::ios::binary);
            if (!pgm.is_open()) throw std::runtime_error("Could not open PGM file for writing.");

            // PGM Header: P5, Width, Height, Max Gray Value
            pgm << "P5\n" << latest_map_->info.width << " " << latest_map_->info.height << "\n255\n";

            // Map data is row-major, starting from (0,0) bottom-left. 
            // In PGM, we write top-down, but most map loaders handle the flip via the YAML origin.
            for (int8_t cell : latest_map_->data) {
                if (cell == 100) {
                    pgm.put(static_cast<unsigned char>(0));     // Black (Occupied)
                } else if (cell == 0) {
                    pgm.put(static_cast<unsigned char>(255));   // White (Free)
                } else {
                    pgm.put(static_cast<unsigned char>(205));   // Gray (Unknown / -1)
                }
            }
            pgm.close();

            RCLCPP_INFO(this->get_logger(), "Map successfully saved to %s.yaml and %s.pgm", 
                        base_filename.c_str(), base_filename.c_str());

        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", e.what());
        }
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_map_ = msg;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapExporter>();

    // Spin until the node is interrupted (e.g., RViz closes or Ctrl+C)
    rclcpp::spin(node);

    node->save_map_to_disk("/ros2_ws/exported_map");

    rclcpp::shutdown();
    return 0;
}