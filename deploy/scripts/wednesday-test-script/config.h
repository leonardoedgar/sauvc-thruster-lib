# ifndef SAUVC2020_CONFIG_H
# define SAUVC2020_CONFIG_H

// Define e-stop pin
# define ESTOP_PIN byte(10)

// Define timer config
typedef unsigned long ulong;
# define SUBMERGE_TIMEOUT ulong(3000) 
# define MOVE_TIMEOUT ulong(3000)
# define SURFACE_TIMEOUT ulong(3000)

# endif //SAUVC2020_CONFIG_H
