#ifndef OBSTACLE_H_
#define OBSTACLE_H_

uint8_t get_obstacle(void);
uint16_t get_front_sensor_value(void);
//start the obstacle  thread
void obstacle_start(void);

#endif /* OBSTACLE_H_ */
