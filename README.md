# 1/s
python3 randomTrips.py -n osm.net.xml -b 0 -e $DUR \--period 1.0 --seed 42 --validate \--trip-attributes 'departLane="best" departPos="random" departSpeed="max"' \-o trips_dens_1ps.rou.xml

# 1.5/s
python3 randomTrips.py -n osm.net.xml -b 0 -e $DUR \--period 0.6666667 --seed 42 --validate \--trip-attributes 'departLane="best" departPos="random" departSpeed="max"' \-o trips_dens_1.5ps.rou.xml


# 2/s
python3 randomTrips.py -n osm.net.xml -b 0 -e $DUR \--period 0.5 --seed 42 --validate \--trip-attributes 'departLane="best" departPos="random" departSpeed="max"' \-o trips_dens_1.5ps.rou.xml


# 2.5/s
python3 randomTrips.py -n osm.net.xml -b 0 -e $DUR \--period 0.4 --seed 42 --validate \--trip-attributes 'departLane="best" departPos="random" departSpeed="max"' \-o trips_dens_1.5ps.rou.xml
