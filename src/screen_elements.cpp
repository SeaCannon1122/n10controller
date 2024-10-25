#include "gui/menu.h"
#include "gui/pixel_char.h"
#include "controller_node.h"
#include "platform.h"

#include <stdio.h>
#include <math.h>

#define M_PI 3.14159265

struct n10controller_label {
    int menu_item_type;
	int z;
	int x;
	int y;
	int alignment_x;
	int alignment_y;
	int text_alignment_x;
	int text_alignment_y;
	int max_width;
	int max_rows;
	int selectable;
	int hoverable;
	int text_size;
	struct pixel_char text[64];
};

const int vel_offset = -250;

struct n10controller_label title_label = {
      MENU_ITEM_LABEL,
      1,
      0,
      30,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_TOP,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_TOP,
      9999999,
      9999999,
      0,
      0, 
      4,
      {0}
};

struct n10controller_label lin_x_label = {
      MENU_ITEM_LABEL,
      1,
      vel_offset,
      200,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      9999999,
      9999999,
      0,
      0, 
      2,
      {0}
};

struct n10controller_label lin_y_label = {
      MENU_ITEM_LABEL,
      1,
      vel_offset,
      240,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      9999999,
      9999999,
      0,
      0, 
      2,
      {0}
};

struct n10controller_label ang_z_label = {
      MENU_ITEM_LABEL,
      1,
      vel_offset,
      280,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      9999999,
      9999999,
      0,
      0, 
      2,
      {0}
};

struct n10controller_label vel_0_label = {
      MENU_ITEM_LABEL,
      1,
      -105+vel_offset,
      -152,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      9999999,
      9999999,
      0,
      0, 
      2,
      {0}
};

struct n10controller_label vel_1_label = {
      MENU_ITEM_LABEL,
      1,
      105+vel_offset,
      -152,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      9999999,
      9999999,
      0,
      0, 
      2,
      {0}
};

struct n10controller_label vel_2_label = {
      MENU_ITEM_LABEL,
      1,
      -105+vel_offset,
      0,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      9999999,
      9999999,
      0,
      0, 
      2,
      {0}
};
struct n10controller_label vel_3_label = {
      MENU_ITEM_LABEL,
      1,
      105+vel_offset,
      0,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      9999999,
      9999999,
      0,
      0, 
      2,
      {0}
};
struct n10controller_label vel_4_label = {
      MENU_ITEM_LABEL,
      1,
      -105+vel_offset,
      152,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      9999999,
      9999999,
      0,
      0, 
      2,
      {0}
};
struct n10controller_label vel_5_label = {
      MENU_ITEM_LABEL,
      1,
      105+vel_offset,
      152,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      ALIGNMENT_MIDDLE,
      9999999,
      9999999,
      0,
      0, 
      2,
      {0}
};

struct menu_line vel_arrow_0 = {
    -105+vel_offset,
    -152,
    -105+vel_offset,
    -152,
    2.f,
    0xff000000,
    ALIGNMENT_MIDDLE,
    ALIGNMENT_MIDDLE
};

struct menu_line vel_arrow_1 = {
    105+vel_offset,
    -152,
    105+vel_offset,
    -152,
    2.f,
    0xff000000,
    ALIGNMENT_MIDDLE,
    ALIGNMENT_MIDDLE
};

struct menu_line vel_arrow_2 = {
    -105+vel_offset,
    0,
    -105+vel_offset,
    0,
    2.f,
    0xff000000,
    ALIGNMENT_MIDDLE,
    ALIGNMENT_MIDDLE
};

struct menu_line vel_arrow_3 = {
    105+vel_offset,
    0,
    105+vel_offset,
    0,
    2.f,
    0xff000000,
    ALIGNMENT_MIDDLE,
    ALIGNMENT_MIDDLE
};

struct menu_line vel_arrow_4 = {
    -105+vel_offset,
    152,
    -105+vel_offset,
    152,
    2.f,
    0xff000000,
    ALIGNMENT_MIDDLE,
    ALIGNMENT_MIDDLE
};

struct menu_line vel_arrow_5 = {
    105+vel_offset,
    152,
    105+vel_offset,
    152,
    2.f,
    0xff000000,
    ALIGNMENT_MIDDLE,
    ALIGNMENT_MIDDLE
};

struct menu_line base_line_1 = {
    -105-20+vel_offset,
    152,
    105+20+vel_offset,
    152,
    3.f,
    0xff9f9f9f,
    ALIGNMENT_MIDDLE,
    ALIGNMENT_MIDDLE
};

  struct menu_line base_line_2 = {
    -105-20+vel_offset,
    0,
    105+20+vel_offset,
    0,
    3.f,
    0xff9f9f9f,
    ALIGNMENT_MIDDLE,
    ALIGNMENT_MIDDLE
};

  struct menu_line base_line_3 = {
    -105-20+vel_offset,
    -152,
    105+20+vel_offset,
    -152,
    3.f,
    0xff9f9f9f,
    ALIGNMENT_MIDDLE,
    ALIGNMENT_MIDDLE
};

struct menu_line base_line_4 = {
    -105+vel_offset,
    -152-20,
    -105+vel_offset,
    152+20,
    3.f,
    0xff9f9f9f,
    ALIGNMENT_MIDDLE,
    ALIGNMENT_MIDDLE
};

  struct menu_line base_line_5 = {
    105+vel_offset,
    -152-20,
    105+vel_offset,
    152+20,
    3.f,
    0xff9f9f9f,
    ALIGNMENT_MIDDLE,
    ALIGNMENT_MIDDLE
};

int old_width = 1;
int old_height = 1;
struct controller_node_feedback_data old_node_data;

struct pixel_font* font;
  

void screen_elements_init() {
    pixel_char_convert_string_in(title_label.text, "N10 Controller Panel", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(lin_x_label.text, "linear x:  0.00", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(lin_y_label.text, "linear y:  0.00", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(ang_z_label.text, "angular z:  0.00", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(vel_0_label.text, " 00.00 m/s", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(vel_1_label.text, " 00.00 m/s", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(vel_2_label.text, " 00.00 m/s", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(vel_3_label.text, " 00.00 m/s", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(vel_4_label.text, " 00.00 m/s", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(vel_5_label.text, " 00.00 m/s", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);



    font = load_pixel_font("default.pixelfont");

    if(font == NULL) printf("failed to load font\n");
}

int draw_screen_elements(struct controller_node_feedback_data* node_data, unsigned int* pixels, int width, int height) {
    int change_bit = 0;

    if(old_width != width || old_height != height) {
      old_width = width;
      old_height = height;
      change_bit = 1;
    }

    if(old_node_data.lin_x != node_data->lin_x || old_node_data.lin_y != node_data->lin_y || old_node_data.ang_z != node_data->ang_z) {
      change_bit = 1;

      old_node_data.lin_x = node_data->lin_x;
      old_node_data.lin_y = node_data->lin_y;
      old_node_data.ang_z = node_data->ang_z;

      if(old_node_data.lin_x < 0) lin_x_label.text[10].value = '-';
      else lin_x_label.text[10].value = ' ';
      lin_x_label.text[11].value = (int)abs(old_node_data.lin_x) + '0';
      lin_x_label.text[13].value = (int)abs(old_node_data.lin_x * 10) % 10 + '0';
      lin_x_label.text[14].value = (int)abs(old_node_data.lin_x * 100) % 10 + '0';

      if(old_node_data.lin_y < 0) lin_y_label.text[10].value = '-';
      else lin_y_label.text[10].value = ' ';
      lin_y_label.text[11].value = (int)abs(old_node_data.lin_y) + '0';
      lin_y_label.text[13].value = (int)abs(old_node_data.lin_y * 10) % 10 + '0';
      lin_y_label.text[14].value = (int)abs(old_node_data.lin_y * 100) % 10 + '0';

      if(old_node_data.ang_z < 0) ang_z_label.text[11].value = '-';
      else ang_z_label.text[11].value = ' ';
      ang_z_label.text[12].value = (int)abs(old_node_data.ang_z) + '0';
      ang_z_label.text[14].value = (int)abs(old_node_data.ang_z * 10) % 10 + '0';
      ang_z_label.text[15].value = (int)abs(old_node_data.ang_z * 100) % 10 + '0';

    }

    if(
      old_node_data.wheel_speed_0 != node_data->wheel_speed_0 || 
      old_node_data.wheel_speed_1 != node_data->wheel_speed_1 ||
      old_node_data.wheel_speed_2 != node_data->wheel_speed_2 || 
      old_node_data.wheel_speed_3 != node_data->wheel_speed_3 || 
      old_node_data.wheel_speed_4 != node_data->wheel_speed_4 || 
      old_node_data.wheel_speed_5 != node_data->wheel_speed_5 ||

      old_node_data.wheel_angle_0 != node_data->wheel_angle_0 ||
      old_node_data.wheel_angle_1 != node_data->wheel_angle_1 ||
      old_node_data.wheel_angle_2 != node_data->wheel_angle_2 ||
      old_node_data.wheel_angle_3 != node_data->wheel_angle_3 ||
      old_node_data.wheel_angle_4 != node_data->wheel_angle_4 ||
      old_node_data.wheel_angle_5 != node_data->wheel_angle_5
    ) {

      change_bit = 1;

      old_node_data.wheel_speed_0 = node_data->wheel_speed_0;
      old_node_data.wheel_speed_1 = node_data->wheel_speed_1;
      old_node_data.wheel_speed_2 = node_data->wheel_speed_2;
      old_node_data.wheel_speed_3 = node_data->wheel_speed_3;
      old_node_data.wheel_speed_4 = node_data->wheel_speed_4;
      old_node_data.wheel_speed_5 = node_data->wheel_speed_5;

      old_node_data.wheel_angle_0 = node_data->wheel_angle_0;
      old_node_data.wheel_angle_1 = node_data->wheel_angle_1;
      old_node_data.wheel_angle_2 = node_data->wheel_angle_2;
      old_node_data.wheel_angle_3 = node_data->wheel_angle_3;
      old_node_data.wheel_angle_4 = node_data->wheel_angle_4;
      old_node_data.wheel_angle_5 = node_data->wheel_angle_5;

      double magnitude_0 = node_data->wheel_speed_0 / 60 * 2 * M_PI * 0.05;

      if(magnitude_0 < 0) vel_0_label.text[0].value = '-';
      else vel_0_label.text[0].value = ' ';
      vel_0_label.text[1].value = ((int)abs(magnitude_0) / 10) % 10 + '0';
      vel_0_label.text[2].value = ((int)abs(magnitude_0)) % 10 + '0';
      vel_0_label.text[4].value = ((int)abs(magnitude_0) * 10) % 10 + '0';
      vel_0_label.text[5].value = ((int)abs(magnitude_0) * 100) % 10 + '0';

      vel_arrow_0.x1 = vel_arrow_0.x0 - magnitude_0 * sin(old_node_data.wheel_angle_0) * 40.f;      
      vel_arrow_0.y1 = vel_arrow_0.y0 - magnitude_0 * cos(old_node_data.wheel_angle_0) * 40.f;

      double magnitude_1 = node_data->wheel_speed_1 / 60 * 2 * M_PI * 0.05;

      if(magnitude_1 < 0) vel_1_label.text[0].value = '-';
      else vel_1_label.text[0].value = ' ';
      vel_1_label.text[1].value = ((int)abs(magnitude_1) / 10) % 10 + '0';
      vel_1_label.text[2].value = ((int)abs(magnitude_1)) % 10 + '0';
      vel_1_label.text[4].value = ((int)abs(magnitude_1 * 10)) % 10 + '0';
      vel_1_label.text[5].value = ((int)abs(magnitude_1 * 100)) % 10 + '0';

      vel_arrow_1.x1 = vel_arrow_1.x0 - magnitude_1 * sin(old_node_data.wheel_angle_1) * 40.f;      
      vel_arrow_1.y1 = vel_arrow_1.y0 - magnitude_1 * cos(old_node_data.wheel_angle_1) * 40.f;

      double magnitude_2 = node_data->wheel_speed_2 / 60.f * 2.f * M_PI * 0.05;

      if(magnitude_2 < 0) vel_2_label.text[0].value = '-';
      else vel_2_label.text[0].value = ' ';
      vel_2_label.text[1].value = ((int)abs(magnitude_2) / 10) % 10 + '0';
      vel_2_label.text[2].value = ((int)abs(magnitude_2)) % 10 + '0';
      vel_2_label.text[4].value = ((int)abs(magnitude_2 * 10)) % 10 + '0';
      vel_2_label.text[5].value = ((int)abs(magnitude_2 * 100)) % 10 + '0';

      vel_arrow_2.x1 = vel_arrow_2.x0 - magnitude_2 * sin(old_node_data.wheel_angle_2) * 40.f;      
      vel_arrow_2.y1 = vel_arrow_2.y0 - magnitude_2 * cos(old_node_data.wheel_angle_2) * 40.f;

      double magnitude_3 = node_data->wheel_speed_3 / 60.f * 2.f * M_PI * 0.05;

      if(magnitude_0 < 0) vel_0_label.text[0].value = '-';
      else vel_0_label.text[0].value = ' ';
      vel_0_label.text[1].value = ((int)abs(magnitude_0) / 10) % 10 + '0';
      vel_0_label.text[2].value = ((int)abs(magnitude_0)) % 10 + '0';
      vel_0_label.text[4].value = ((int)abs(magnitude_0 * 10)) % 10 + '0';
      vel_0_label.text[5].value = ((int)abs(magnitude_0 * 100)) % 10 + '0';

      vel_arrow_3.x1 = vel_arrow_3.x0 - magnitude_3 * sin(old_node_data.wheel_angle_3) * 40.f;      
      vel_arrow_3.y1 = vel_arrow_3.y0 - magnitude_3 * cos(old_node_data.wheel_angle_3) * 40.f;

      double magnitude_4 = node_data->wheel_speed_4 / 60 * 2 * M_PI * 0.05;

      vel_arrow_4.x1 = vel_arrow_4.x0 - magnitude_4 * sin(old_node_data.wheel_angle_4) * 40.f;      
      vel_arrow_4.y1 = vel_arrow_4.y0 - magnitude_4 * cos(old_node_data.wheel_angle_4) * 40.f;

      double magnitude_5 = node_data->wheel_speed_5 / 60 * 2 * M_PI * 0.05;

      vel_arrow_5.x1 = vel_arrow_5.x0 - magnitude_5 * sin(old_node_data.wheel_angle_5) * 40.f;      
      vel_arrow_5.y1 = vel_arrow_5.y0 - magnitude_5 * cos(old_node_data.wheel_angle_5) * 40.f;

    }

    if(change_bit) {

        for(int i = 0; i < width * height; i++) pixels[i] = 0xff5f5fdf;

        menu_scene_draw_line(&base_line_1, pixels, width, height, 1);
        menu_scene_draw_line(&base_line_2, pixels, width, height, 1);
        menu_scene_draw_line(&base_line_3, pixels, width, height, 1);
        menu_scene_draw_line(&base_line_4, pixels, width, height, 1);
        menu_scene_draw_line(&base_line_5, pixels, width, height, 1);

        menu_scene_draw_arrow(&vel_arrow_0, pixels, width, height, 1);
        menu_scene_draw_arrow(&vel_arrow_1, pixels, width, height, 1);
        menu_scene_draw_arrow(&vel_arrow_2, pixels, width, height, 1);
        menu_scene_draw_arrow(&vel_arrow_3, pixels, width, height, 1);
        menu_scene_draw_arrow(&vel_arrow_4, pixels, width, height, 1);
        menu_scene_draw_arrow(&vel_arrow_5, pixels, width, height, 1);

        menu_scene_draw_label((struct menu_label*)&lin_x_label, pixels, width, height, 1, (const void**)&font);
        menu_scene_draw_label((struct menu_label*)&lin_y_label, pixels, width, height, 1, (const void**)&font);
        menu_scene_draw_label((struct menu_label*)&ang_z_label, pixels, width, height, 1, (const void**)&font);

        menu_scene_draw_label((struct menu_label*)&vel_0_label, pixels, width, height, 1, (const void**)&font);
        menu_scene_draw_label((struct menu_label*)&vel_1_label, pixels, width, height, 1, (const void**)&font);
        menu_scene_draw_label((struct menu_label*)&vel_2_label, pixels, width, height, 1, (const void**)&font);
        menu_scene_draw_label((struct menu_label*)&vel_3_label, pixels, width, height, 1, (const void**)&font);
        menu_scene_draw_label((struct menu_label*)&vel_4_label, pixels, width, height, 1, (const void**)&font);
        menu_scene_draw_label((struct menu_label*)&vel_5_label, pixels, width, height, 1, (const void**)&font);

        menu_scene_draw_label((struct menu_label*)&title_label, pixels, width, height, 1, (const void**)&font);


        return 1;
    }

    return 0;
}
