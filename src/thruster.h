# ifndef SAUVC2020_THRUSTER_H
# define SAUVC2020_THRUSTER_H
# include <string>
class Thruster {
private:
    int pin{};
    int id{};

public:
    Thruster();
    explicit Thruster(int id, int pin);

    /**
     * A function to get the pin of the thruster.
     * @return {int} the pin of the thruster
     */
    int get_pin() const;
    /**
     * A function to get the id of the thruster
     * @return {int} the id of the thruster
     */
    int get_id() const;
    /**
     * The function to run a thruster at a certain ESC input.
     * @param esc_input {int} the desired ESC input
     * @return {bool} whether the thruster runs successfully or not
     */
    bool run(int esc_input) const;

    /**
     * A function to stop the thruster.
     * @return {bool} whether the stopping was successful or not
     */
    bool stop() const;

    /**
    * A function to get a safe esc input value
    * @param esc_input {int} the desired esc input
    * @return {int} the safe esc input
    */
    int static get_safe_esc_input(int esc_input) ;
};

#endif //SAUVC2020_THRUSTER_H
