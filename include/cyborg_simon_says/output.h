#ifndef CYBORG_SIMON_SAYS_OUTPUT_H
#define CYBORG_SIMON_SAYS_OUTPUT_H

#include <string>

namespace cyborg
{
    namespace simon_says
    {
        class Output
        {
            public:
                virtual ~Output() {}
                virtual void say(const std::string&) = 0;
        };
    }
}

#endif // CYBORG_SIMON_SAYS_OUTPUT_H

