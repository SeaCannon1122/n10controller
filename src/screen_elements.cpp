#include "gui/menu.h"
#include "gui/pixel_char.h"
#include "controller_node.h"
#include "platform.h"

#include <stdio.h>

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
struct controller_node_data old_note_data;

struct pixel_font* font;
  

void screen_elements_init() {
    pixel_char_convert_string_in(title_label.text, "N10 Controller Panel", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(vel_0_label.text, "00.0 m/s", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(vel_1_label.text, "00.0 m/s", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(vel_2_label.text, "00.0 m/s", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(vel_3_label.text, "00.0 m/s", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(vel_4_label.text, "00.0 m/s", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);
    pixel_char_convert_string_in(vel_5_label.text, "00.0 m/s", 0xffffffff, 0xff000000, PIXEL_CHAR_SHADOW_MASK);



    font = load_pixel_font("default.pixelfont");

    if(font == NULL) printf("failed to load font\n");
}

int draw_screen_elements(struct controller_node_data* node_data, unsigned int* pixels, int width, int height) {
    int change = 0;

    if(old_width != width || old_height != height) {
      old_width = width;
      old_height = height;
      change = 1;
      
    }

    if(change) {

        for(int i = 0; i < width * height; i++) pixels[i] = 0xff5f5fdf;

        menu_scene_draw_line(&base_line_1, pixels, width, height, 1);
        menu_scene_draw_line(&base_line_2, pixels, width, height, 1);
        menu_scene_draw_line(&base_line_3, pixels, width, height, 1);
        menu_scene_draw_line(&base_line_4, pixels, width, height, 1);
        menu_scene_draw_line(&base_line_5, pixels, width, height, 1);

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
