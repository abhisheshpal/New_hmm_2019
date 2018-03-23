modify the yaml then:

roscd rasberry_gazebo/
chmod a+x ./scripts/generate_world.py
./scripts/generate_world.py --file ./config/thorvald_world_AB.yaml 
roslaunch rasberry_gazebo thorvald_world_AB.launch
