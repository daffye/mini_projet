#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define RED 1
#define BLUE 2
#define YELLOW 3
#define THRESH_INTERSECT 300  //à changer en fct des tailles

float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);
uint8_t color_detection(void);
uint8_t intersect_detec(uint16_t line_width);
uint8_t get_target_color(void);
uint8_t get_intersection_bool(void);

#endif /* PROCESS_IMAGE_H */
