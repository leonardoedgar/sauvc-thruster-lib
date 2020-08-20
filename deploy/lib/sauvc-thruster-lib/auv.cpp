# include "auv.h"
# include "thruster_aggregator.h"
# include <Arduino.h>
/**
 * The implementation of the function to setup the AUV.
 * @return {bool} whether the setup was successful or not.
 */
bool AUV::setup() {
    this->thruster_aggregator = ThrusterAggregator();
    return this->thruster_aggregator.setup();
}

/**
 * A function to move the AUV according to a pre-defined motion.
 * @param motion {std::string} movement to perform
 * @return {bool} whether the execution of the movement was successful or not
 */
bool AUV::move(std::string motion) {
    if (this->motion == motion and motion == "forward") {
        return this->thruster_aggregator.stabilise();
    }
    else {
        this->motion = motion;
	this->thruster_aggregator.update_stabilised_speed({});
        return this->thruster_aggregator.move(motion);
    }
}

/**
 * A function to stop the AUV.
 * @return {bool} indicates whether the AUV stops successfully or not
 */
bool AUV::stop() {
    this->motion = "stop";
    return this->thruster_aggregator.stop();
}

/**
 * The implementation of the function to get AUV motion.
 * @return {std::string} AUV motion
 */
std::string AUV::get_motion() const {
    return this->motion;
};

/**
 * The implementation of the function to update thrusters' stabilised speed.
 * @param new_thruster_id_to_stabilised_speed_map {std::map <int, int>} thrusters' stabilised speed.
 */
void AUV::update_stabilised_speed(std::map<int, int> new_thruster_id_to_stabilised_speed_map) {
    this->thruster_aggregator.update_stabilised_speed(new_thruster_id_to_stabilised_speed_map);
}

/**
 * The implementation of the function to get the actual thrusters' speed.
 * @return {std::map<int, int>} actual thrusters' speed
 */
std::map <int, int> AUV::get_actual_thrusters_speed() {
    return this->thruster_aggregator.get_actual_thrusters_speed();
}
