#ifndef CYBORG_SIMON_SAYS_CHALLENGE_H
#define CYBORG_SIMON_SAYS_CHALLENGE_H

namespace cyborg
{
    namespace simon_says
    {
        class Challenge
        {
            public:
                virtual ~Challenge() {}
                virtual void start()      = 0;
                virtual bool isComplete() = 0;
                virtual void end()        = 0;
        };
    }
}

#endif // CYBORG_SIMON_SAYS_CHALLENGE_H

