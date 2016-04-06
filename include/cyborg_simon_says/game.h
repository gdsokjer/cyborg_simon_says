#ifndef CYBORG_SIMON_SAYS_GAME_H
#define CYBORG_SIMON_SAYS_GAME_H

#include "cyborg_simon_says/event.h"
#include "cyborg_simon_says/output.h"

#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace cyborg
{
    namespace simon_says
    {
        class Game
        {
            public:
                typedef std::chrono::monotonic_clock   Clock;
                typedef std::chrono::time_point<Clock> TimePoint;

                enum class Number
                {
                    beginEventDelay,
                    gameRestartDelay,
                    challengeCount
                };

                enum class State
                {
                    none,
                    initialize,
                    awaitStartEvent,
                    announceEventStart,
                    awaitEventCompleted,
                    gameCompleted,
                    gameOver
                };

                enum class String
                {
                    gameStart,
                    gameFailed,
                    gameCompleted,
                    eventCompleted,
                    gameTimeout
                };

                Game(std::shared_ptr<Event>              startEvent,
                     std::vector<std::shared_ptr<Event>> eventLibrary,
                     std::shared_ptr<Output>             output);

                void process();
                void reset();

                double getNumberValue(Number) const;
                void   setNumberValue(Number, double);

                std::string getStringValue(String) const;
                void        setStringValue(String, const std::string&);

            private:
                bool hasStateTimeoutTimerExpired(TimePoint now) const;
                bool hasStateTransitionTimerExpired(TimePoint now) const;

                void setState(State);

                void setState(State,
                              TimePoint now,
                              double stateTransitionDelay = 0.0,
                              double stateTimeoutDelay    = 0.0);

                State                               lastState_;
                State                               state_;
                std::shared_ptr<Event>              startEvent_;
                std::vector<std::shared_ptr<Event>> eventLibrary_;
                std::shared_ptr<Output>             output_;

                std::deque<std::shared_ptr<Event>>  events_;
                std::shared_ptr<Event>              currentEvent_;

                bool isStateTransitionTimerSet_;
                bool isStateTimeoutTimerSet_;

                TimePoint stateTransitionTimerValue_;
                TimePoint stateTimeoutTimerValue_;

                std::map<Number, double>      numberValues_;
                std::map<String, std::string> stringValues_;

                std::default_random_engine rng_;
        };
    }
}

#endif // CYBORG_SIMON_SAYS_GAME_H

