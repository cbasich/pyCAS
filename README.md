# pyCAS
A python library for competence-aware systems.

# To Run
In order to run this, you will need to run ```python run_CDB.py``` in the directory src\scripts.

Flags are:

-m : the map file to use, defaults to small_campus.txt

-n : the number of episodes to run

-u : whether to perform model updates (0/1)

-i : whether feedback should be automated (0) or interactive with user (1)

-l : whether all information from execution should be logged to file (0/1)

-v : whether executions sims should print all statements (0/1)

-s : optional specified start room

-e : optional specified end room

The following default command should run the program with default parameters, random start and end states, on the small campus map.