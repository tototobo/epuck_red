/*

File    : main.c
Author  : Thomas Bonnaud & Louis Rosset
Date    : 10 may 2020

Capture and analyze an image and returns the position of a red object to "move" module.

*/
#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

/**
 * @brief returns the position of a red object to "move" module
 *
 */
uint16_t get_line_position(void);

//start the process image thread
void process_image_start(void);


#endif /* PROCESS_IMAGE_H */
