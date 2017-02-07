//#include "ros/ros.h"
#include "pressure_pad/decompresser.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wallpusher_listener");
    ros::NodeHandle n;

    std::shared_ptr<DECOMPRESSER> gc(new DECOMPRESSER (n));
    std::thread left(&DECOMPRESSER::left_reader ,gc);
    std::thread right(&DECOMPRESSER::right_reader ,gc);

    ros::spin();

    ros::shutdown();

    std::thread t2(&DECOMPRESSER::wake_con ,gc);
    left.join();
    right.join();
    t2.join();


    return 0;
}
