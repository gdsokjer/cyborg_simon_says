#ifndef CYBORG_SIMON_SAYS_ROS_OUTPUT_H
#define CYBORG_SIMON_SAYS_ROS_OUTPUT_H

#include "ros/ros.h"
#include "cyborg_simon_says/output.h"

namespace cyborg
{
    namespace simon_says
    {
        class RosOutput : public Output
        {
            public:
                RosOutput(ros::Publisher);

                virtual void say(const std::string&);

            private:
                ros::Publisher publisher_;
        };
    }
}

#endif // CYBORG_SIMON_SAYS_ROS_OUTPUT_H

