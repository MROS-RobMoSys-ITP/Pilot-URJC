#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class LaserFilter : public rclcpp::Node
{
public:
	LaserFilter(): Node("laser_filter")
	{
        sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
            std::bind(&LaserFilter::laserCallback, this, _1));
      //  pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered",10);
				pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered",10);

	}

	void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
	{
    //auto output = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
		sensor_msgs::msg::LaserScan output_;

		//auto output = std::make_shared<sensor_msgs::msg::LaserScan>();

    for (auto& range : output_.ranges)
    {
      if (range < 0.3) range = std::numeric_limits<float>::infinity();
    }

    pub_->publish(output_);

	}

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;


};

int main(int argc, char **argv)
{

   rclcpp::init(argc, argv);

   auto node = std::make_shared<LaserFilter>();

   rclcpp::spin(node);

   rclcpp::shutdown();
   return 0;

 }
