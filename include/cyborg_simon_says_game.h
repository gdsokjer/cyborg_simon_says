#ifndef CYBORG_SIMON_SAYS_GAME_H
#define CYBORG_SIMON_SAYS_GAME_H

#include "cyborg_simon_says_challenge.h"

#include <memory>
#include <vector>

namespace cyborg
{
    namespace simon_says
    {
        class Game
        {
            public:
                enum class State
                {
                    notStarted, beginChallenge
                };

                Game(const std::vector<std::shared_ptr<Challenge>> challenges)
                    : state_(State::notStarted),
                      challenges_(challenges)
                {
                }

            private:
                State state_;
                std::vector<std::shared_ptr<Challenge>> library_;
                std::vector<std::shared_ptr<Challenge>> challenges_;
        };
    }
}

#endif // CYBORG_SIMON_SAYS_GAME_H

