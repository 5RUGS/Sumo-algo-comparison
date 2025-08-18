python3 randomTrips.py -n osm.net.xml.gz -b 0 --period 1.2  --seed 42 --validate --trip-attributes 'departLane="best" departSpeed="max" departPos="random"' -o trips_high.rou.xml
python3 randomTrips.py -n osm.net.xml.gz -b 0 --period 1.8  --seed 42 --validate --trip-attributes 'departLane="best" departSpeed="max" departPos="random"' -o trips_high.rou.xml
python3 randomTrips.py -n osm.net.xml.gz -b 0 --period 3  --seed 42 --validate --trip-attributes 'departLane="best" departSpeed="max" departPos="random"' -o trips_high.rou.xml
