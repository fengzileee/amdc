#ifndef debris_thresholds__H
#define debris_thresholds__H

const float open_door_box_x1 = 0.4;
const float open_door_box_y1 = 0.4;
const float open_door_box_x2 = 0.6;
const float open_door_box_y2 = 0.6;
const float open_door_box_w = open_door_box_x2 - open_door_box_x1;
const float open_door_box_h = open_door_box_y2 - open_door_box_y1;

const float close_door_box_x1 = 0.1;
const float close_door_box_y1 = 0.8;
const float close_door_box_x2 = 0.9;
const float close_door_box_y2 = 1.0;
const float close_door_box_w = close_door_box_x2 - close_door_box_x1;
const float close_door_box_h = close_door_box_y2 - close_door_box_y1;

const float stay_opened_box_x1 = 0.20;
const float stay_opened_box_y1 = 0.2;
const float stay_opened_box_x2 = 0.8;
const float stay_opened_box_y2 = 1.0;
const float stay_opened_box_w = stay_opened_box_x2 - stay_opened_box_x1;
const float stay_opened_box_h = stay_opened_box_y2 - stay_opened_box_y1;

#define inside_box(x, y, x1, y1, x2, y2) \
(x > x1 && x < x2 && y > y1 && y < y2)

#define in_open_door_box(C)     \
(inside_box(C(0),               \
            C(1),               \
            open_door_box_x1,   \
            open_door_box_y1,   \
            open_door_box_x2,   \
            open_door_box_y2))

#define in_close_door_box(C)     \
(inside_box(C(0),               \
            C(1),               \
            close_door_box_x1,   \
            close_door_box_y1,   \
            close_door_box_x2,   \
            close_door_box_y2))

#define in_stay_opened_box(C)   \
(inside_box(C(0),               \
            C(1),               \
            stay_opened_box_x1,   \
            stay_opened_box_y1,   \
            stay_opened_box_x2,   \
            stay_opened_box_y2))

#endif
