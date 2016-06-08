# CSE190-FreezeTag

Freeze Tag
I implemented a model of how freeze tag should be played in an acyclic environment.  Properties include, variable number of runners, using energy when taking measurements, and stealing energy as you freeze or unfreeze robots.

Rewards:
No penalty for walls
Robot uses an energy unit for each type of sensor
Energy used from sensing gets dispersed throughout the map
Freezer steals 10% of runner’s energy on freeze
Runner steals 3% of energy from the Freezer
Runner being unfrozen steals an additional 1% of energy from the Freezer

Strategy:
Use MDP to decide whether going in the direction (or running away) from a robot is good, or collecting a high amount of energy in a particular grid unit
