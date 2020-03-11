# ifndef SAUVC_THRUSTER_LIB_AUV_H
# define SAUVC_THRUSTER_LIB_AUV_H
# include <thruster_aggregator.h>
# include <string>

class AUV {
private:
    ThrusterAggregator thruster_aggregator;
    std::string motion {"stop"};
public:
    /**
     * A function to setup the AUV.
     * @return {bool} whether the setup was successful or not.
     */
    bool setup();
    /**
     * A function to move the AUV according to a pre-defined motion.
     * @param motion {std::string} movement to perform
     * @return {bool} whether the execution of the movement was successful or not
     */
    bool move(std::string motion);

    /**
     * A function to stop the AUV.
     * @return {bool} indicates whether the AUV stops successfully or not
     */
    bool stop();

    /**
     * A function to get AUV motion.
     * @return {std::string} AUV motion
     */
    std::string get_motion() const;

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


#endif //SAUVC_THRUSTER_LIB_AUV_H
