# Invisibot

Invisibot is Rocket League bot that has invisiblity powers. 

It is powered by Kamael and RLUtilities.

# How does it work?

The bot is hidden when the ball or opponent are not near.

The game packet is manipulated to convince the "core" bot
that the car is still on the field.
Then controls are used to simulate where the car
would move to. This is repeated till the car is close
and can reappear.

RLUtilities is used for simulating
jumps/dodges/aerials while simple ground logic is
used for simulating driving and turning.

`invisibot.py` contains the core logic. It can be easily used to
give other bots invisibility powers. Here are the requirements:

- The bot shoudl be implemented in Python
- The bot should only use `get_output` as the source of game packet
- The bot currently can't use rlutilities (this should be fixed in the future)

# Status

The bot works, however it makes Kamael significantly worse. The biggest
issue seems to be that the flip simulation is not behaving normally.


# To Dos:

- Boost tracking
- Update additional car fields  liked jumped and `wheel_on_ground`
- Improve flip/dodge. Either the above issues are causing issues or we are over-correcting orientation on landing.
