#include <iostream>

#include "cyborg_simon_says/ros_output.h"

cyborg::simon_says::RosOutput::RosOutput(
    ros::Publisher publisher)
    : publisher_(publisher)
{
}

void cyborg::simon_says::RosOutput::say(const std::string& text)
{
    std::cout << "RosOutput -> " << text << std::endl;
}

