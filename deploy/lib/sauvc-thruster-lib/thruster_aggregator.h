# ifndef SAUVC2020_THRUSTER_AGGREGATOR_H
# define SAUVC2020_THRUSTER_AGGREGATOR_H
# include "thruster.h"
# include <Arduino.h>
# include <map>
# include <string>

class ThrusterAggregator {
private:
    std::map <int, Thruster> thruster_id_to_instance_map;
    std::map <std::string, std::map<int, int>> motion_to_thruster_id_to_esc_input_map;
    std::map<int, int> thruster_id_to_stabilised_speed_map;
    std::map<int, int> thruster_id_to_actual_speed_map;
    /**
     * A function to add thrusters to the thruster aggregator.
     * @param thrusters_id_and_pin_to_add {std::map<int, byte>} represents thrusters' id and pin to add
     * @return {bool} whether the addition was successful or not
     */
    bool add_thrusters(const std::map<int, byte>& thrusters_id_and_pin_to_add);

    /**
     * A function to store the mapping of a motion type to thrusters' id and esc input to run.
     * @return {bool} whether the storing was successful or not
     */
    bool store_motion_to_thruster_id_and_esc_input_map();

    /**
     * A function to load pre-defined thrusters' motion.
     * @return {std::map<int,int>} the motion to thruster id to esc input map
     */
    std::map<std::string, std::map<int, int>> static load_predefined_motion();

public:
    /**
    * A function to setup the thruster aggregator
    * @return {bool} whether the setup was successful or not
    */
    bool setup();

    /**
     * A function to move thrusters according to a motion.
     * @param motion {string} indicates the motion to for thrusters to produce
     * @return {bool} whether the execution of movement was successful or not
     */
    bool move(const std::string& motion);

    /**
     * A function to stop all thrusters.
     * @return {bool} whether the stopping was successful or not.
     */
    bool stop();

    /**
     * A function to stabilise thrusters.
     * @return {bool} whether the stabilisation was successful or not
     */
    bool stabilise();

    /**
     * A function to update thrusters' stabilised speed.
     * @param new_thruster_id_to_stabilised_speed_map {std::map <int, int>} thrusters' stabilised speed.
     */
    void update_stabilised_speed(std::map<int, int> new_thruster_id_to_stabilised_speed_map);

    /**
     * A function to get the actual thrusters' speed.
     * @return {std::map<int, int>} actual thrusters' speed
     */
    std::map <int, int> get_actual_thrusters_speed();
};

#endif //SAUVC2020_THRUSTER_AGGREGATOR_H
