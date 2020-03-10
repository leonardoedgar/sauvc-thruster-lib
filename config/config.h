# ifndef SAUVC2020_CONFIG_H
# define SAUVC2020_CONFIG_H
/**
 * Motor Id to Arduino Pin Connection.
 */
# define THRUSTER_ID_1_PIN int(3)
# define THRUSTER_ID_2_PIN int(4)
# define THRUSTER_ID_3_PIN int(5)
# define THRUSTER_ID_4_PIN int(6)
# define THRUSTER_ID_5_PIN int(7)
# define THRUSTER_ID_6_PIN int(8)
# define THRUSTER_ID_7_PIN int(9)
# define THRUSTER_ID_8_PIN int(10)
# define THRUSTER_ID_TO_ARDUINO_PIN {1, THRUSTER_ID_1_PIN}, \
                                    {2, THRUSTER_ID_2_PIN}, \
                                    {3, THRUSTER_ID_3_PIN}, \
                                    {4, THRUSTER_ID_4_PIN}, \
                                    {5, THRUSTER_ID_5_PIN}, \
                                    {6, THRUSTER_ID_6_PIN}, \
                                    {7, THRUSTER_ID_7_PIN}, \
                                    {8, THRUSTER_ID_8_PIN}

/**
 * ESC Input Value Safety Limit
 */
# define MAX_ESC_INPUT int(1700)
# define MIN_ESC_INPUT int(1300)

/**
 * ESC Input Value for Stop Signal
 */
# define ESC_INPUT_FOR_STOP_SIGNAL int(1500)

/**
 * ESC Input Value of motors for each motion.
 */
# define THRUSTER_ID_TO_ESC_INPUT_FOR_FORWARD {1, 1600}, \
                                              {2, 1600}, \
                                              {3, 1600}, \
                                              {4, 1600}, \
                                              {5, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                              {6, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                              {7, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                              {8, ESC_INPUT_FOR_STOP_SIGNAL}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_BACKWARD {1, 1400}, \
                                               {2, 1400}, \
                                               {3, 1400}, \
                                               {4, 1400}, \
                                               {5, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {6, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {7, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {8, ESC_INPUT_FOR_STOP_SIGNAL}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_SUBMERGE {1, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {2, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {3, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {4, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {5, 1400}, \
                                               {6, 1600}, \
                                               {7, 1600}, \
                                               {8, 1400}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_SURFACE {1, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                              {2, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                              {3, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                              {4, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                              {5, 1600}, \
                                              {6, 1400}, \
                                              {7, 1400}, \
                                              {8, 1600}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_ROTATE_LEFT {1, 1600}, \
                                                  {2, 1400}, \
                                                  {3, 1600}, \
                                                  {4, 1400}, \
                                                  {5, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                  {6, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                  {7, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                  {8, ESC_INPUT_FOR_STOP_SIGNAL}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_ROTATE_RIGHT {1, 1400}, \
                                                   {2, 1600}, \
                                                   {3, 1400}, \
                                                   {4, 1600}, \
                                                   {5, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                   {6, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                   {7, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                   {8, ESC_INPUT_FOR_STOP_SIGNAL}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_TRANSLATE_LEFT {1, 1600}, \
                                                     {2, 1400}, \
                                                     {3, 1400}, \
                                                     {4, 1600}, \
                                                     {5, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                     {6, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                     {7, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                     {8, ESC_INPUT_FOR_STOP_SIGNAL}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_TRANSLATE_RIGHT {1, 1400}, \
                                                      {2, 1600}, \
                                                      {3, 1600}, \
                                                      {4, 1400}, \
                                                      {5, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                      {6, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                      {7, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                      {8, ESC_INPUT_FOR_STOP_SIGNAL}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_ROLL_LEFT {1, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {2, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {3, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {4, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {5, 1400}, \
                                                {6, 1400}, \
                                                {7, 1600}, \
                                                {8, 1600}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_ROLL_RIGHT {1, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                 {2, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                 {3, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                 {4, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                 {5, 1600}, \
                                                 {6, 1600}, \
                                                 {7, 1400}, \
                                                 {8, 1400}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_PITCH_FORWARD {1, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                    {2, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                    {3, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                    {4, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                    {5, 1400}, \
                                                    {6, 1600}, \
                                                    {7, 1400}, \
                                                    {8, 1600}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_PITCH_BACKWARD {1, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                     {2, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                     {3, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                     {4, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                     {5, 1600}, \
                                                     {6, 1400}, \
                                                     {7, 1600}, \
                                                     {8, 1400}
# endif
