string name
uint32 base_speed # how many 'units' the agent can move in one step
uint32 max_speed
uint32 base_load # how much volume the agent may carry
uint32 max_load
uint32 base_battery # the agent's battery size
uint32 max_battery
uint32 base_skill # how fast the agent is at gathering/building/dismantling
uint32 max_skill
uint32 base_vision # how far the agent can perceive (in m)
uint32 max_vision
string[] roads # TODO not yet sent by the server, which roads the agent can navigate (currently 'road' for all roads and 'air' for travelling linear distances between two points)
