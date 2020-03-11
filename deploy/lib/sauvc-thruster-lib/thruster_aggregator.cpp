# include "thruster_aggregator.h"
# include "thruster.h"
# include <string>
# include <map>
# include <Arduino.h>
# include "config/config.h"
/***
 * The implementation of the function to setup the thruster aggregator
 * @return {bool} whether the setup was successful or not
 */
bool ThrusterAggregator::setup() {
    bool addition_success = this->add_thrusters({THRUSTER_ID_TO_ARDUINO_PIN});
    bool store_success = this->store_motion_to_thruster_id_and_esc_input_map();
    delay(1000);
    std::string setup_success {"false"};
    if (addition_success and store_success) {
        setup_success = "true";
    }
    else {
        setup_success = "false";
    }
    Serial.println("Thruster aggregator setup's success: " + String(setup_success.c_str()));
    return addition_success and store_success;
}

/**
 * The implementation of the function to add thrusters to thruster aggregator.
 * @param thrusters_id_and_pin_to_add {std::map<int, byte>} represents thrusters' id and pin to add
 * @return {bool} whether the addition was successful or not
 */
bool ThrusterAggregator::add_thrusters(const std::map<int, byte>& thrusters_id_and_pin_to_add) {
    for (const auto &[thruster_id, thruster_pin]: thrusters_id_and_pin_to_add) {
        thruster_id_to_instance_map.insert(std::pair<int, Thruster>(thruster_id, Thruster(thruster_id, thruster_pin)));
    }
    return thruster_id_to_instance_map.size() == thrusters_id_and_pin_to_add.size();
}

/**
 * The implementation of the function to store the mapping of motion type to run
 * @return {bool} indicates whether the storing was successful or not
 */
bool ThrusterAggregator::store_motion_to_thruster_id_and_esc_input_map() {
    std::map<std::string, std::map<int, int>> loaded_predefined_motion =
            ThrusterAggregator::load_predefined_motion();
    for ( const auto &[motion, motor_id_to_esc_input]: loaded_predefined_motion ) {
        motion_to_thruster_id_to_esc_input_map.insert(std::pair<std::string,
                std::map<int, int>>(motion, motor_id_to_esc_input));
    }
    return motion_to_thruster_id_to_esc_input_map.size() == loaded_predefined_motion.size();
}

/**
 * The implementation of the function to load pre-defined thrusters' motion.
 * @return {std::map<int,int>} the motion to thruster id to esc input map
 */
std::map<std::string, std::map<int, int>> ThrusterAggregator::load_predefined_motion() {
    return {
            {"forward", {THRUSTER_ID_TO_ESC_INPUT_FOR_FORWARD}},
            {"backward", {THRUSTER_ID_TO_ESC_INPUT_FOR_BACKWARD}},
            {"submerge", {THRUSTER_ID_TO_ESC_INPUT_FOR_SUBMERGE}},
            {"surface", {THRUSTER_ID_TO_ESC_INPUT_FOR_SURFACE}},
            {"rotate-left", {THRUSTER_ID_TO_ESC_INPUT_FOR_ROTATE_LEFT}},
            {"rotate-right", {THRUSTER_ID_TO_ESC_INPUT_FOR_ROTATE_RIGHT}},
            {"translate-left", {THRUSTER_ID_TO_ESC_INPUT_FOR_TRANSLATE_LEFT}},
            {"translate-right", {THRUSTER_ID_TO_ESC_INPUT_FOR_TRANSLATE_RIGHT}},
            {"roll-left", {THRUSTER_ID_TO_ESC_INPUT_FOR_ROLL_LEFT}},
            {"roll-right", {THRUSTER_ID_TO_ESC_INPUT_FOR_ROLL_RIGHT}},
            {"pitch-forward", {THRUSTER_ID_TO_ESC_INPUT_FOR_PITCH_FORWARD}},
            {"pitch-backward", {THRUSTER_ID_TO_ESC_INPUT_FOR_PITCH_BACKWARD}}
    };
}


/**
 * The implementation of the function to move thrusters according to a motion
 * @param motion {string} the motion for thrusters to produce
 * @return {bool} whether the execution of movement was successful or not
 */
bool ThrusterAggregator::move(const std::string& motion) {
    if (this->motion_to_thruster_id_to_esc_input_map.find(motion) != this->motion_to_thruster_id_to_esc_input_map.end()) {
        for (const auto &[thruster_id_to_run, esc_input]: this->motion_to_thruster_id_to_esc_input_map[motion]) {
            this->thruster_id_to_instance_map[thruster_id_to_run].run(esc_input);
            this->thruster_id_to_actual_speed_map[thruster_id_to_run] = Thruster::get_safe_esc_input(esc_input);
        }
        return true;
    }
    else {
        return false;
    }
}

/**
 * The implementation of the function to stop all thrusters.
 * @return {bool} whether the stopping was successful or not
 */
bool ThrusterAggregator::stop() {
    for (const auto &[id, thruster]: thruster_id_to_instance_map) {
        thruster.stop();
        this->thruster_id_to_actual_speed_map[id] = int(ESC_INPUT_FOR_STOP_SIGNAL);
    }
    return true;
}

/**
 * The implementation of the function to stabilise thrusters.
 * @return {bool} whether the stabilisation was successful or not
 */
bool ThrusterAggregator::stabilise() {
    for (const auto &[id, thruster]: this->thruster_id_to_instance_map) {
        if(this->thruster_id_to_stabilised_speed_map.find(id) == this->thruster_id_to_stabilised_speed_map.end()) {
            return false;
        }
    }
    for (const auto &[id, stabilised_esc_input]: this->thruster_id_to_stabilised_speed_map) {
        this->thruster_id_to_instance_map[id].run(stabilised_esc_input);
        this->thruster_id_to_actual_speed_map[id] = Thruster::get_safe_esc_input(stabilised_esc_input);
    }
    return true;
}

/**
 * A function to update thrusters' stabilised speed.
 * @param new_thruster_id_to_stabilised_speed_map {std::map <int, int>} thrusters' stabilised speed.
 */
void ThrusterAggregator::update_stabilised_speed(std::map<int, int> new_thruster_id_to_stabilised_speed_map) {
    for (const auto &[thruster_id, stabilised_speed]: new_thruster_id_to_stabilised_speed_map) {
        this->thruster_id_to_stabilised_speed_map[thruster_id] = new_thruster_id_to_stabilised_speed_map[thruster_id];
    }
}

/**
 * The implementation of the function to get the actual thrusters' speed.
 * @return {std::map<int, int>} actual thrusters' speed
 */
std::map <int, int> ThrusterAggregator::get_actual_thrusters_speed() {
    return this->thruster_id_to_actual_speed_map;
}
