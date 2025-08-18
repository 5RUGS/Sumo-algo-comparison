# --------- TH BANGKOK ----------
cd ../Fukko daimae

python $SUMO_HOME/tools/randomTrips.py -n osm.net.xml -b 0 -e $DUR --period 3   --seed 42 --validate --trip-attributes 'departLane="best" departSpeed="max" departPos="random"' -o trips_low.rou.xml
python $SUMO_HOME/tools/randomTrips.py -n osm.net.xml -b 0 -e $DUR --period 1.8 --seed 42 --validate --trip-attributes 'departLane="best" departSpeed="max" departPos="random"' -o trips_med.rou.xml
python $SUMO_HOME/tools/randomTrips.py -n osm.net.xml -b 0 -e $DUR --period 1.2 --seed 42 --validate --trip-attributes 'departLane="best" departSpeed="max" departPos="random"' -o trips_high.rou.xml


# --------- TH BANGKOK ----------
cd ../Chiand Mai

python $SUMO_HOME/tools/randomTrips.py -n osm.net.xml -b 0 -e $DUR --period 3   --seed 42 --validate --trip-attributes 'departLane="best" departSpeed="max" departPos="random"' -o trips_low.rou.xml
python $SUMO_HOME/tools/randomTrips.py -n osm.net.xml -b 0 -e $DUR --period 1.8 --seed 42 --validate --trip-attributes 'departLane="best" departSpeed="max" departPos="random"' -o trips_med.rou.xml
python $SUMO_HOME/tools/randomTrips.py -n osm.net.xml -b 0 -e $DUR --period 1.2 --seed 42 --validate --trip-attributes 'departLane="best" departSpeed="max" departPos="random"' -o trips_high.rou.xml

# --------- TH BANGKOK ----------
cd ../Singapore

python $SUMO_HOME/tools/randomTrips.py -n osm.net.xml -b 0 -e $DUR --period 3   --seed 42 --validate --trip-attributes 'departLane="best" departSpeed="max" departPos="random"' -o trips_low.rou.xml
python $SUMO_HOME/tools/randomTrips.py -n osm.net.xml -b 0 -e $DUR --period 1.8 --seed 42 --validate --trip-attributes 'departLane="best" departSpeed="max" departPos="random"' -o trips_med.rou.xml
python $SUMO_HOME/tools/randomTrips.py -n osm.net.xml -b 0 -e $DUR --period 1.2 --seed 42 --validate --trip-attributes 'departLane="best" departSpeed="max" departPos="random"' -o trips_high.rou.xml
