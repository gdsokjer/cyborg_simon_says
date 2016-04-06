#include "cyborg_simon_says/game.h"

#include <algorithm>
#include <iostream>

namespace
{
    namespace local
    {
        const char* getStateName(const cyborg::simon_says::Game::State state)
        {
            switch (state)
            {
            }
        }
    }
}

cyborg::simon_says::Game::Game(
    std::shared_ptr<Event>              startEvent,
    std::vector<std::shared_ptr<Event>> eventLibrary,
    std::shared_ptr<Output>             output)
    : state_(State::initialize),
      startEvent_(startEvent),
      eventLibrary_(eventLibrary),
      output_(output)
{
    reset();
}

double
cyborg::simon_says::Game::getNumberValue(const Number id) const
{
    const auto value = numberValues_.find(id);

    if (value != numberValues_.end())
    {
        return value->second;
    }
    else
    {
        return 0.0;
    }
}

std::string
cyborg::simon_says::Game::getStringValue(const String id) const
{
    const auto value = stringValues_.find(id);

    if (value != stringValues_.end())
    {
        return value->second;
    }
    else
    {
        return "";
    }
}

bool
cyborg::simon_says::Game::hasStateTimeoutTimerExpired(
    const TimePoint now) const
{
    if (isStateTimeoutTimerSet_)
    {
        return now >= stateTimeoutTimerValue_;
    }
    else
    {
        return false;
    }
}

bool
cyborg::simon_says::Game::hasStateTransitionTimerExpired(
    const TimePoint now) const
{
    if (isStateTransitionTimerSet_)
    {
        return now >= stateTransitionTimerValue_;
    }
    else
    {
        return true;
    }
}

void
cyborg::simon_says::Game::process()
{
    const auto now = Clock::now();

    if (state_ != lastState)
    {
        ROS_NAMED_INFO("cyborg_simon_says", "Entered state " << local::getStateName(state_));
        lastState_ = state_;
    }

    if (hasStateTransitionTimerExpired(now))
    {
        switch (state_)
        {
            case State::initialize:
            {
                reset();
                startEvent_->start();
                setState(State::awaitStartEvent);

                break;
            }
            case State::awaitStartEvent:
            {
                if (startEvent_->completed())
                {
                    output_->say(getStringValue(String::gameStart));

                    std::shuffle(eventLibrary_.begin(),
                                 eventLibrary_.end(),
                                 rng_);

                    events_.clear();

                    const auto challengeCount = static_cast<int>(
                        getNumberValue(Number::challengeCount));

                    for (auto i = 0; i < challengeCount; i++)
                    {
                        events_.push_back(eventLibrary_[i]);
                    }

                    setState(State::announceEventStart,
                             now, getNumberValue(Number::beginEventDelay));
                }

                break;
            }
            case State::announceEventStart:
            {
                if (!events_.empty())
                {
                    currentEvent_ = events_.front();
                    events_.pop_front();

                    currentEvent_->start();
                    output_->say(currentEvent_->announcement());

                    setState(State::awaitEventCompleted,
                             now, 0.0, currentEvent_->timeout());
                }
                else
                {
                    output_->say(getStringValue(String::gameCompleted));
                    setState(State::gameOver);
                }

                break;
            }
            case State::awaitEventCompleted:
            {
                if (hasStateTimeoutTimerExpired(now))
                {
                    output_->say(getStringValue(String::gameTimeout));
                    setState(State::gameOver);
                }
                else if (currentEvent_->completed())
                {
                    currentEvent_->stop();
                    output_->say(getStringValue(String::eventCompleted));

                    setState(State::announceEventStart,
                             now, getNumberValue(Number::beginEventDelay));
                }
            }
            case State::gameOver:
            {
                startEvent_->stop();

                setState(State::initialize,
                         now, getNumberValue(Number::gameRestartDelay));
            }
        }
    }
}

void
cyborg::simon_says::Game::reset()
{
    lastState_                 = State::none;
    state_                     = State::initialize;
    isStateTransitionTimerSet_ = false;
    isStateTimeoutTimerSet_    = false;

    rng_ = std::default_random_engine(std::random_device()());

    currentEvent_.reset();
    events_.clear();
}

void
cyborg::simon_says::Game::setNumberValue(const Number id, const double value)
{
    numberValues_[id] = value;
}


void
cyborg::simon_says::Game::setState(
    const State state)
{
    state_                     = state;
    isStateTransitionTimerSet_ = false;
    isStateTimeoutTimerSet_    = false;
}

void
cyborg::simon_says::Game::setState(
    const State     state,
    const TimePoint now,
    const double    stateTransitionDelay,
    const double    stateTimeoutDelay)
{
    if (isStateTransitionTimerSet_ = stateTransitionDelay != 0.0)
    {
        stateTransitionTimerValue_ = now +
            std::chrono::milliseconds(
                static_cast<long>(1000.0 * stateTransitionDelay));
    }

    if (isStateTimeoutTimerSet_ = stateTimeoutDelay != 0.0)
    {
        stateTimeoutTimerValue_ = now +
            std::chrono::milliseconds(
                static_cast<long>(1000.0 * stateTimeoutDelay));
    }

    state_ = state;
}

void
cyborg::simon_says::Game::setStringValue(const String id, const std::string& value)
{
    stringValues_[id] = value;
}

