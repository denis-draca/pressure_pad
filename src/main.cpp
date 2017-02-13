//#include "ros/ros.h"
#include "pressure_pad/decompresser.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wallpusher_listener");
    ros::NodeHandle n;

    std::shared_ptr<DECOMPRESSER> gc(new DECOMPRESSER (n));

    std::thread left(&DECOMPRESSER::left_reader ,gc);
    std::thread right(&DECOMPRESSER::right_reader ,gc);
    std::thread left_rivets(&DECOMPRESSER::left_rivet_detector ,gc);
    std::thread right_rivets(&DECOMPRESSER::right_rivet_detector ,gc);

    ros::spin();

    ros::shutdown();

    gc->wake_con();

    left.join();
    right.join();
    left_rivets.join();
    right_rivets.join();



    return 0;
}
