#include "ros/ros.h"
#include "cyborg_simon_says_challenge.h"
#include "cyborg_simon_says_game.h"

#include <memory>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cyborg_simon_says");

    ros::NodeHandle node;

    std::vector<std::shared_ptr<cyborg::simon_says::Challenge>> challenges;
    cyborg::simon_says::Game game(challenges);

    ROS_INFO_NAMED("cyborg_simon_says", "cyborg_simon_says ready to serve!");

    ros::spin();
}

