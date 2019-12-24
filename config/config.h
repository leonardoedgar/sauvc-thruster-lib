# ifndef SAUVC2020_CONFIG_H
# define SAUVC2020_CONFIG_H
/**
 * Motor to Arduino Pin Connection.
 */
# define MOTOR_NO_1_PIN int(1)
# define MOTOR_NO_2_PIN int(2)
# define MOTOR_NO_3_PIN int(3)
# define MOTOR_NO_4_PIN int(4)
# define MOTOR_NO_5_PIN int(5)
# define MOTOR_NO_6_PIN int(6)
# define MOTOR_NO_7_PIN int(7)
# define MOTOR_NO_8_PIN int(8)
# define MOTOR_PINS_TO_REGISTER {MOTOR_NO_1_PIN, MOTOR_NO_2_PIN, MOTOR_NO_3_PIN, MOTOR_NO_4_PIN, \
                                 MOTOR_NO_5_PIN, MOTOR_NO_6_PIN, MOTOR_NO_7_PIN, MOTOR_NO_8_PIN}

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
# define MOTOR_AND_ESC_INPUT_FOR_FORWARD {MOTOR_NO_1_PIN, 1600}, \
                                         {MOTOR_NO_2_PIN, 1600}, \
                                         {MOTOR_NO_3_PIN, 1600}, \
                                         {MOTOR_NO_4_PIN, 1600}, \
                                         {MOTOR_NO_5_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                         {MOTOR_NO_6_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                         {MOTOR_NO_7_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                         {MOTOR_NO_8_PIN, ESC_INPUT_FOR_STOP_SIGNAL}
# define MOTOR_AND_ESC_INPUT_FOR_BACKWARD {MOTOR_NO_1_PIN, 1400}, \
                                          {MOTOR_NO_2_PIN, 1400}, \
                                          {MOTOR_NO_3_PIN, 1400}, \
                                          {MOTOR_NO_4_PIN, 1400}, \
                                          {MOTOR_NO_5_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                          {MOTOR_NO_6_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                          {MOTOR_NO_7_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                          {MOTOR_NO_8_PIN, ESC_INPUT_FOR_STOP_SIGNAL}
# define MOTOR_AND_ESC_INPUT_FOR_SUBMERGE {MOTOR_NO_1_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                          {MOTOR_NO_2_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                          {MOTOR_NO_3_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                          {MOTOR_NO_4_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                          {MOTOR_NO_5_PIN, 1400}, \
                                          {MOTOR_NO_6_PIN, 1600}, \
                                          {MOTOR_NO_7_PIN, 1600}, \
                                          {MOTOR_NO_8_PIN, 1400}
# define MOTOR_AND_ESC_INPUT_FOR_SURFACE {MOTOR_NO_1_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                         {MOTOR_NO_2_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                         {MOTOR_NO_3_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                         {MOTOR_NO_4_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                         {MOTOR_NO_5_PIN, 1600}, \
                                         {MOTOR_NO_6_PIN, 1400}, \
                                         {MOTOR_NO_7_PIN, 1400}, \
                                         {MOTOR_NO_8_PIN, 1600}
# define MOTOR_AND_ESC_INPUT_FOR_ROTATE_LEFT {MOTOR_NO_1_PIN, 1600}, \
                                             {MOTOR_NO_2_PIN, 1400}, \
                                             {MOTOR_NO_3_PIN, 1600}, \
                                             {MOTOR_NO_4_PIN, 1400}, \
                                             {MOTOR_NO_5_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                             {MOTOR_NO_6_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                             {MOTOR_NO_7_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                             {MOTOR_NO_8_PIN, ESC_INPUT_FOR_STOP_SIGNAL}
# define MOTOR_AND_ESC_INPUT_FOR_ROTATE_RIGHT {MOTOR_NO_1_PIN, 1400}, \
                                              {MOTOR_NO_2_PIN, 1600}, \
                                              {MOTOR_NO_3_PIN, 1400}, \
                                              {MOTOR_NO_4_PIN, 1600}, \
                                              {MOTOR_NO_5_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                              {MOTOR_NO_6_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                              {MOTOR_NO_7_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                              {MOTOR_NO_8_PIN, ESC_INPUT_FOR_STOP_SIGNAL}
# define MOTOR_AND_ESC_INPUT_FOR_TRANSLATE_LEFT {MOTOR_NO_1_PIN, 1600}, \
                                                {MOTOR_NO_2_PIN, 1400}, \
                                                {MOTOR_NO_3_PIN, 1400}, \
                                                {MOTOR_NO_4_PIN, 1600}, \
                                                {MOTOR_NO_5_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_6_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_7_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_8_PIN, ESC_INPUT_FOR_STOP_SIGNAL}
# define MOTOR_AND_ESC_INPUT_FOR_TRANSLATE_RIGHT {MOTOR_NO_1_PIN, 1400}, \
                                                 {MOTOR_NO_2_PIN, 1600}, \
                                                 {MOTOR_NO_3_PIN, 1600}, \
                                                 {MOTOR_NO_4_PIN, 1400}, \
                                                 {MOTOR_NO_5_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                 {MOTOR_NO_6_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                 {MOTOR_NO_7_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                 {MOTOR_NO_8_PIN, ESC_INPUT_FOR_STOP_SIGNAL}
# define MOTOR_AND_ESC_INPUT_FOR_ROLL_LEFT {MOTOR_NO_1_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                           {MOTOR_NO_2_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                           {MOTOR_NO_3_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                           {MOTOR_NO_4_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                           {MOTOR_NO_5_PIN, 1400}, \
                                           {MOTOR_NO_6_PIN, 1400}, \
                                           {MOTOR_NO_7_PIN, 1600}, \
                                           {MOTOR_NO_8_PIN, 1600}
# define MOTOR_AND_ESC_INPUT_FOR_ROLL_RIGHT {MOTOR_NO_1_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                            {MOTOR_NO_2_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                            {MOTOR_NO_3_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                            {MOTOR_NO_4_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                            {MOTOR_NO_5_PIN, 1600}, \
                                            {MOTOR_NO_6_PIN, 1600}, \
                                            {MOTOR_NO_7_PIN, 1400}, \
                                            {MOTOR_NO_8_PIN, 1400}
# define MOTOR_AND_ESC_INPUT_FOR_PITCH_FORWARD {MOTOR_NO_1_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {MOTOR_NO_2_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {MOTOR_NO_3_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {MOTOR_NO_4_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                               {MOTOR_NO_5_PIN, 1400}, \
                                               {MOTOR_NO_6_PIN, 1600}, \
                                               {MOTOR_NO_7_PIN, 1400}, \
                                               {MOTOR_NO_8_PIN, 1600}
# define MOTOR_AND_ESC_INPUT_FOR_PITCH_BACKWARD {MOTOR_NO_1_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_2_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_3_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_4_PIN, ESC_INPUT_FOR_STOP_SIGNAL}, \
                                                {MOTOR_NO_5_PIN, 1600}, \
                                                {MOTOR_NO_6_PIN, 1400}, \
                                                {MOTOR_NO_7_PIN, 1600}, \
                                                {MOTOR_NO_8_PIN, 1400}
# endif
