# include "thruster.h"
# include <iostream>
# include "../config/config.h"
Thruster::Thruster(int id, int pin): id{id}, pin{pin}{

}

Thruster::Thruster() {
    std::cout << "Thruster constructor without args is called\n.";
}

/**
 * The implementation of the function to get the pin of the thruster.
 * @return {int} the pin of the thruster
 */
int Thruster::get_pin() const {
    return this->pin;
}

/**
 * The implementation of the function to get the id of the thruster.
 * @return {int} the id of the thruster
 */
int Thruster::get_id() const {
    return this->id;
}

/**
 * The implementation of the function to run the thruster at a certain ESC input value.
 * @param esc_input {int} the desired ESC input value
 * @return {bool} whether the thruster runs successfully or not
 */
bool Thruster::run(int esc_input) const {
    std::cout << std::fixed;
    std::cout << "Thruster with id: " << this->id << " with pin: " << this->pin << " is running with ESC input: " << \
        this->get_safe_esc_input(esc_input) << ".\n";
    return true;
}

/**
 * The implementation of the function to stop the motor.
 * @return {bool} indicates whether the stopping was successful or not
 */
bool Thruster::stop() const {
    this->run(ESC_INPUT_FOR_STOP_SIGNAL);
    return true;
}

/**
 * The implementation of the function to get a safe esc input value
 * @param esc_input_value {int} indicates the desired esc input
 * @return {int} represents the safe esc input
 */
int Thruster::get_safe_esc_input(int esc_input) const {
    if (esc_input > MAX_ESC_INPUT) {
        return MAX_ESC_INPUT;
    }
    else if (esc_input < MIN_ESC_INPUT){
        return MIN_ESC_INPUT;
    }
    return esc_input;
}
